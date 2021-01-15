use std::sync::Arc;
use std::{cmp, fmt, thread, time};

use crossbeam::queue::ArrayQueue;

use rand::Rng;

use super::LORA_MTU;

const BUFFER_SIZE: usize = 1024;

struct Packet {
    size: usize,
    buffer: [u8; LORA_MTU],
    rssi: i32,
    snr: f64,
}

pub struct PacketMetadata {
    size: usize,
    rssi: i32,
    snr: f64,
}

impl Packet {
    fn copy_to_buffer(&self, buffer: &mut [u8]) -> usize {
        let size = cmp::min(self.size, buffer.len());
        for (target, origin) in buffer[0..size].iter_mut().zip(self.buffer.iter()) {
            *target = *origin;
        }
        size
    }
}

impl From<&[u8]> for Packet {
    fn from(buffer: &[u8]) -> Packet {
        let mut target = [0; LORA_MTU];
        let size = cmp::min(LORA_MTU, buffer.len());
        for (target, origin) in target[0..size].iter_mut().zip(buffer.iter()) {
            *target = *origin;
        }
        Packet {
            size,
            buffer: target,
            rssi: 0,
            snr: 0f64,
        }
    }
}

impl PacketMetadata {
    pub fn size(&self) -> usize {
        self.size
    }

    pub fn rssi(&self) -> i32 {
        self.rssi
    }

    pub fn snr(&self) -> f64 {
        self.snr
    }
}

impl From<Packet> for PacketMetadata {
    fn from(packet: Packet) -> PacketMetadata {
        PacketMetadata {
            size: packet.size,
            rssi: packet.rssi,
            snr: packet.snr,
        }
    }
}

impl<'a> Into<&'a [u8]> for &'a Packet {
    fn into(self) -> &'a [u8] {
        &self.buffer[0..self.size]
    }
}

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
                    Ok(size) => {
                        let rssi = body.link.get_last_rssi();
                        let snr = body.link.get_last_snr();
                        let size = cmp::min(size, LORA_MTU);
                        while let Err(_) = body.inq.push(Packet {
                            size,
                            buffer: buffer.clone(),
                            rssi,
                            snr,
                        }) {}
                    }
                    _ => continue,
                };
            }
        });
        // Thread for transmitting buffered packets over the link.
        let body = Arc::clone(&self.0);
        thread::spawn(move || {
            let mut rng = rand::thread_rng();
            loop {
                while body.outq.is_empty() {}
                if let Some(packet) = body.outq.pop() {
                    let mut retries = 0;
                    let buffer = (&packet).into();
                    // Re-try physical transmission until we get through. (BEB until retry count 5.)
                    while let Err(_) = body.link.transmit(buffer) {
                        retries = cmp::min(retries + 1, 4);
                        let backoff = rng.gen_range(0..(1 << retries));
                        thread::sleep(time::Duration::from_millis(500) * backoff);
                    }
                }
            }
        });
    }

    pub fn receive(&self, buffer: &mut [u8]) -> Result<PacketMetadata, ()> {
        if let Some(packet) = self.0.inq.pop() {
            let copied = packet.copy_to_buffer(buffer);
            let mut metadata: PacketMetadata = packet.into();
            metadata.size = copied;
            Ok(metadata)
        } else {
            Err(())
        }
    }

    pub fn transmit(&self, buffer: &[u8]) -> Result<usize, ()> {
        let packet: Packet = buffer.into();
        let size = packet.size;
        match self.0.outq.push(packet) {
            Ok(_) => Ok(size),
            _ => Err(()),
        }
    }

    pub fn configure_link<F>(&self, f: F)
    where
        F: Fn(&dyn Link),
    {
        f(&self.0.link)
    }
}

impl<T> From<T> for LoRa<T>
where
    T: 'static + Link + Send + Sync,
{
    fn from(t: T) -> Self {
        let lora = LoRa(Arc::new(LoRaBody {
            inq: ArrayQueue::new(BUFFER_SIZE),
            outq: ArrayQueue::new(BUFFER_SIZE),
            link: t,
        }));
        lora.init();
        lora
    }
}

pub trait Link {
    fn receive(&self, buffer: &mut [u8]) -> Result<usize, ()>;

    fn transmit(&self, buffer: &[u8]) -> Result<usize, ()>;

    fn get_last_rssi(&self) -> i32;

    fn get_last_snr(&self) -> f64;
}

impl<T> fmt::Debug for LoRa<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "LoRa")
    }
}

impl<T> Clone for LoRa<T> {
    fn clone(&self) -> Self {
        LoRa(Arc::clone(&self.0))
    }
}
