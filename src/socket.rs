use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::{cmp, thread};

use super::LORA_MTU;

struct Packet([u8; LORA_MTU], usize);

impl Packet {
    fn copy_to_buffer(&self, buffer: &mut [u8]) -> usize {
        let size = cmp::min(self.size(), buffer.len());
        for (target, origin) in buffer[0..size].iter_mut().zip(self.0.iter()) {
            *target = *origin;
        }
        size
    }

    fn size(&self) -> usize {
        self.1
    }
}

impl From<&[u8]> for Packet {
    fn from(buffer: &[u8]) -> Packet {
        let mut target = [0; LORA_MTU];
        let size = cmp::min(LORA_MTU, buffer.len());
        for (target, origin) in target[0..size].iter_mut().zip(buffer.iter()) {
            *target = *origin;
        }
        Packet(target, size)
    }
}

impl<'a> Into<&'a [u8]> for &'a Packet {
    fn into(self) -> &'a [u8] {
        &self.0[0..self.1]
    }
}

#[derive(Clone)]
pub struct LoRa<T>(Arc<LoRaBody<T>>);

struct LoRaBody<T> {
    inq: Mutex<VecDeque<Packet>>,
    outq: (Mutex<VecDeque<Packet>>, Condvar),
    link: T,
}

impl<T> LoRa<T>
where
    T: 'static + Link + Send + Sync,
{
    fn init(&self) {
        // Thread for polling for packets received on the link.
        let body = Arc::clone(&self.0);
        thread::spawn(move || {
            let mut buffer = [0; LORA_MTU];
            loop {
                match body.link.receive(&mut buffer) {
                    Ok(size) => body.inq.lock().unwrap().push_back(buffer[0..size].into()),
                    Err(()) => continue,
                }
            }
        });
        // Thread for transmitting buffered packets over the link.
        let body = Arc::clone(&self.0);
        thread::spawn(move || loop {
            let &(ref lock, ref cvar) = &body.outq;
            let mut waiting = lock.lock().unwrap();
            while waiting.is_empty() {
                waiting = cvar.wait(waiting).unwrap();
            }
            // TODO we want collision avoidance here
            if let Some(packet) = waiting.front() {
                match body.link.transmit(packet.into()) {
                    // When queueing the transmission we reported that all bytes were transitted,
                    // so we need to retry send until we can send the whole packet.
                    Ok(n) if n == packet.size() => waiting.pop_front(),
                    _ => continue,
                };
            }
        });
    }

    pub fn receive(&self, buffer: &mut [u8]) -> Result<usize, ()> {
        if let Some(packet) = self.0.inq.lock().unwrap().pop_front() {
            Ok(packet.copy_to_buffer(buffer))
        } else {
            Err(())
        }
    }

    pub fn transmit(&self, buffer: &[u8]) -> Result<usize, ()> {
        let packet: Packet = buffer.into();
        let size = packet.size();
        {
            let &(ref lock, ref cvar) = &self.0.outq;
            lock.lock().unwrap().push_back(packet);
            cvar.notify_one();
        }
        Ok(size)
    }
}

impl<T> From<T> for LoRa<T>
where
    T: 'static + Link + Send + Sync,
{
    fn from(t: T) -> Self {
        let lora = LoRa(Arc::new(LoRaBody {
            inq: Mutex::new(VecDeque::new()),
            outq: (Mutex::new(VecDeque::new()), Condvar::new()),
            link: t,
        }));
        lora.init();
        lora
    }
}

pub trait Link {
    fn receive(&self, buffer: &mut [u8]) -> Result<usize, ()>;

    fn transmit(&self, buffer: &[u8]) -> Result<usize, ()>;
}
