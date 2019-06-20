use std::sync::Arc;
use std::{cmp, thread};

use crossbeam::queue::ArrayQueue;

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
    inq: ArrayQueue<Packet>,
    outq: ArrayQueue<Packet>,
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
                    Ok(size) => while let Err(_) = body.inq.push(buffer[0..size].into()) {},
                    Err(()) => continue,
                };
            }
        });
        // Thread for transmitting buffered packets over the link.
        let body = Arc::clone(&self.0);
        thread::spawn(move || loop {
            while body.outq.is_empty() {}
            // TODO we want collision avoidance here.
            if let Ok(packet) = body.outq.pop() {
                let buffer = (&packet).into();
                // Re-try physical transmission until we get through.
                while let Err(_) = body.link.transmit(buffer) {}
            }
        });
    }

    pub fn receive(&self, buffer: &mut [u8]) -> Result<usize, ()> {
        if let Ok(packet) = self.0.inq.pop() {
            Ok(packet.copy_to_buffer(buffer))
        } else {
            Err(())
        }
    }

    pub fn transmit(&self, buffer: &[u8]) -> Result<usize, ()> {
        let packet: Packet = buffer.into();
        let size = packet.size();
        match self.0.outq.push(packet) {
            Ok(_) => Ok(size),
            _ => Err(()),
        }
    }
}

impl<T> From<T> for LoRa<T>
where
    T: 'static + Link + Send + Sync,
{
    fn from(t: T) -> Self {
        let lora = LoRa(Arc::new(LoRaBody {
            inq: ArrayQueue::new(1024),
            outq: ArrayQueue::new(1024),
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
