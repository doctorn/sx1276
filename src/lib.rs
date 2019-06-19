//! Crate for driving the [SEMTECH SX1276](https://www.mouser.com/ds/2/761/sx1276-1278113.pdf) in
//! LoRa mode.
//!
//! All examples here assume the use of a Raspberry Pi Model 3B+ with the Dragino SX1276 LoRa HAT;
//! however, it should be possible to use the crate on any hardware implementing the necessary
//! traits from the `embedded_hal` crate.
//!
//! # Examples
//! ## Simple Receive
//! ```no_run
//! extern crate embedded_hal;
//! extern crate linux_embedded_hal as hal;
//!
//! use hal::spidev::{self, SpidevOptions};
//! use hal::sysfs_gpio::Direction;
//! use hal::{Pin, Spidev};
//!
//! const NSS_PIN: u64 = 25;
//! const RESET_PIN: u64 = 17;
//! const FREQUENCY: u64 = 868;
//!
//! fn main() {
//!     let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
//!     let options = SpidevOptions::new()
//!         .bits_per_word(8)
//!         .max_speed_hz(1_000_000)
//!         .mode(spidev::SPI_MODE_0)
//!         .build();
//!     spi.configure(&options).unwrap();
//!
//!     let cs = Pin::new(NSS_PIN);
//!     cs.export().unwrap();
//!     cs.set_direction(Direction::Out).unwrap();
//!
//!     let reset = Pin::new(RESET_PIN);
//!     reset.export().unwrap();
//!     reset.set_direction(Direction::Out).unwrap();
//!
//!     let sx1276 = sx1276::SX1276::new(spi, cs, reset, FREQUENCY)
//!         .expect("Failed to communicate with radio module!");
//!
//!     let mut buffer = [0; sx1276::LORA_MTU];
//!     loop {
//!         if let Ok(size) = sx1276.receive(&mut buffer) {
//!             for c in buffer[0..size].iter() {
//!                 print!("{}", *c as char)
//!             }
//!             println!();
//!         }
//!     }
//! }
//! ```
//!
//! ## Simple Transmit
//! ```no_run
//! extern crate embedded_hal;
//! extern crate linux_embedded_hal as hal;
//!
//! use hal::spidev::{self, SpidevOptions};
//! use hal::sysfs_gpio::Direction;
//! use hal::{Pin, Spidev};
//!
//! const NSS_PIN: u64 = 25;
//! const RESET_PIN: u64 = 17;
//! const FREQUENCY: u64 = 868;
//!
//! fn main() {
//!     let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
//!     let options = SpidevOptions::new()
//!         .bits_per_word(8)
//!         .max_speed_hz(1_000_000)
//!         .mode(spidev::SPI_MODE_0)
//!         .build();
//!     spi.configure(&options).unwrap();
//!
//!     let cs = Pin::new(NSS_PIN);
//!     cs.export().unwrap();
//!     cs.set_direction(Direction::Out).unwrap();
//!
//!     let reset = Pin::new(RESET_PIN);
//!     reset.export().unwrap();
//!     reset.set_direction(Direction::Out).unwrap();
//!
//!     let sx1276 = sx1276::SX1276::new(spi, cs, reset, FREQUENCY)
//!         .expect("Failed to communicate with radio module!");
//!     sx1276.set_transmission_power(17);
//!
//!     let mut buffer = [0; sx1276::LORA_MTU];
//!     for (b, c) in buffer.iter_mut().zip("HELLO".chars()) {
//!         *b = c as u8;
//!     }
//!     loop {
//!         sx1276.transmit(&buffer);
//!     }
//! }
//! ```
#![crate_type = "lib"]
#![crate_name = "sx1276"]

extern crate bit_field;
extern crate embedded_hal;

use std::{cmp, thread, time};

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

/// The version number for the SX1276.
pub const SX1276_VERSION: u8 = 0x12;

/// The maximum transmission unit for LoRa payloads.
pub const LORA_MTU: usize = 0xFF;

mod registers;
mod selected_spi;
pub mod socket;

use registers::{Mode, Reg, RegisterFile, IRQ};
use selected_spi::SelectedSPI;

/// An immutable abstraction of the SX1276 chip. Can be safely used between threads. Creating
/// multiple instances of this `struct` using the same underlying hardware is a logic error.
///
/// For information about the specific behaviour of any of the methods implemented for this
/// `struct` with respect to LoRa modulation, please refer to the
/// [datasheet](https://www.mouser.com/ds/2/761/sx1276-1278113.pdf) for the chip.
pub struct SX1276<SPI, NSS, RESET> {
    spi: SelectedSPI<SPI, NSS>,
    reset: RESET,
}

impl<SPI, NSS, RESET, E> SX1276<SPI, NSS, RESET>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    NSS: OutputPin,
    RESET: OutputPin,
{
    /// Build a new instance of the chip using the given SPI channel, slave select pin and reset
    /// pin.
    ///
    /// Building multiple instances of this `struct` using the same underlying hardware is a logic
    /// error.
    ///
    /// In order to share the same hardware between multiple owners please use [`Rc`](std::rc::Rc)
    /// or [`Arc`](std::sync::Arc).
    pub fn new(spi: SPI, nss: NSS, reset: RESET, freq: u64) -> Result<Self, u8> {
        let spi = SelectedSPI::new(spi, nss);
        let mut sx1276 = SX1276 { spi, reset };
        sx1276.reset();
        match sx1276.get_version() {
            SX1276_VERSION => {
                sx1276.reset();
                {
                    let mut spi = sx1276.spi.select();
                    spi.set_mode(Mode::Sleep);
                    spi.set_frequency(freq);
                    spi.write_reg(Reg::FifoRxBaseAddr, 0x00);
                    spi.write_reg(Reg::FifoTxBaseAddr, 0x00);
                    spi.set_mode(Mode::Stdby);
                    let lna = spi.read_reg(Reg::Lna);
                    // Set boost to 150% TODO check if this is necessary
                    spi.write_reg(Reg::Lna, lna | 0x03);
                    spi.write_reg(Reg::ModemConfig3, 0x04);
                }
                Ok(sx1276)
            }
            n => Err(n),
        }
    }

    /// Get the value of the version register of the chip. If the SPI channel is working correctly
    /// this should always return [`SX1276_VERSION`](constant.SX1276_VERSION.html).
    pub fn get_version(&self) -> u8 {
        self.spi.select().get_version()
    }

    /// Check if the chip is in implicit header mode (default is `false`).
    pub fn in_implicit_header_mode(&self) -> bool {
        self.spi.select().in_explicit_header_mode()
    }

    /// Check if the chip is in explicit header mode (default is `true`).
    pub fn in_explicit_header_mode(&self) -> bool {
        !self.spi.select().in_explicit_header_mode()
    }

    /// Set the chip into implicit header mode.
    ///
    /// In implicit header mode transmissions consist only of the preamble and payload (with its
    /// optional 16-bit CRC).
    pub fn set_implicit_header_mode(&self) {
        self.spi.select().set_implicit_header_mode()
    }

    /// Set the chip into explicit header mode.
    ///
    /// In explicit header mode, a short header containing the payload length; the forward error
    /// correction code rate; whether the payload contains a CRC; and a header CRC is included in
    /// transmissions between the preamble and the payload.
    pub fn set_explicit_header_mode(&self) {
        self.spi.select().set_explicit_header_mode()
    }

    /// Check if CRCs are enabled.
    pub fn crc_enabled(&self) -> bool {
        self.spi.select().crc_enabled()
    }

    /// Enable CRCs for incoming and outgoing packets.
    pub fn enable_crc(&self) {
        self.spi.select().enable_crc()
    }

    /// Disable CRCs for incoming and outgoing packets.
    pub fn disable_crc(&self) {
        self.spi.select().disable_crc()
    }

    /// Set the operating for the antenna (in Hz).
    pub fn set_frequency(&self, freq: u64) {
        self.spi.select().set_frequency(freq)
    }

    /// Get the RSSI for the last successfully received packet.
    pub fn get_last_rssi(&self) -> i32 {
        self.spi.select().get_last_rssi()
    }

    /// Get the signal to noise ration for the last successfully received packet.
    pub fn get_last_snr(&self) -> f64 {
        self.spi.select().get_last_snr()
    }

    /// Set the transmission power for the antenna.
    ///
    /// The minimum value that can be used is 2 and the maxiumum value is 17. Any values outside of
    /// this range are brought within.
    pub fn set_transmission_power(&self, power: u8) {
        self.spi.select().set_transmission_power(power)
    }

    // TODO OCP

    // TODO frequency

    // TODO coding rate

    // TODO preamble

    // TODO invert IQ

    // TODO spreading factor

    // TODO signal bandwidth

    // TODO frequency error

    // TODO LDO flag

    // TODO collision restarts

    // TODO collision restart threshold

    fn reset(&mut self) {
        self.reset.set_low().ok().unwrap();
        thread::sleep(time::Duration::from_millis(10));
        self.reset.set_high().ok().unwrap();
        thread::sleep(time::Duration::from_millis(10));
    }
}

impl<SPI, NSS, RESET, E> socket::Link for SX1276<SPI, NSS, RESET>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    NSS: OutputPin,
    RESET: OutputPin,
{
    /// Poll for an incoming packet.
    ///
    /// If a packet was successfully received, returns `Ok(n)` where `n` is the number of bytes
    /// written into the buffer. Otherwise, returns `Err(())`.
    ///
    /// The first call to `receive` after a reset or the successful reset of a packet should always
    /// return `Err(())` and set the chip into the mode `RxSingle`. Subsequent calls to `receive`
    /// will then poll for packet receive interrupts and place the packet contents in the buffer
    /// provided.
    ///
    /// The maxiumum packet length that can be received is 255 and therefore at least 255 bytes
    /// should be allocated to the buffer for packets to be received in their entirety.
    ///
    /// (255 not 256 because the register representing the number of bytes in a received packet is
    /// 8-bits and counts from 0.)
    fn receive(&self, buffer: &mut [u8]) -> Result<usize, ()> {
        let mut spi = self.spi.select();
        if !spi.in_mode(Mode::RxSingle) {
            spi.set_mode(Mode::RxSingle);
            Err(())
        } else if !spi.received_irq(IRQ::RxDone) {
            Err(())
        } else {
            spi.clear_irq(IRQ::RxDone);
            let size = cmp::min(spi.read_reg(Reg::RxNbBytes) as usize, buffer.len());
            let fifo_addr = spi.read_reg(Reg::FifoRxCurrentAddr);
            spi.write_reg(Reg::FifoAddrPtr, fifo_addr);
            for byte in buffer[0..size].iter_mut() {
                *byte = spi.read_reg(Reg::Fifo);
            }
            spi.write_reg(Reg::FifoAddrPtr, 0);
            Ok(size)
        }
    }

    /// Transmit the contents of a buffer as an outgoing packet.
    ///
    /// If a packet was successfully transmitted, returns `Ok(n)` where `n` is the number of bytes
    /// successfully transmitted. Otherwise, returns `Err(())` (this will occur if the chip is
    /// already in mode `Tx`, i.e. a transmission is already taking place).
    ///
    /// The maximum transmission unit is 255 bytes and therefore any bytes after the first 255 will
    /// not be transmitted.
    ///
    /// (255 not 256 because the register representing the number of bytes in a received packet is
    /// 8-bits and counts from 0.)
    fn transmit(&self, buffer: &[u8]) -> Result<usize, ()> {
        let mut spi = self.spi.select();
        if spi.in_mode(Mode::Tx) {
            Err(())
        } else {
            let size = cmp::min(LORA_MTU, buffer.len());
            spi.set_mode(Mode::Stdby);
            spi.write_reg(Reg::FifoAddrPtr, 0);
            spi.write_reg(Reg::PayloadLength, 0);
            for byte in buffer[0..size].iter() {
                spi.write_reg(Reg::Fifo, *byte);
            }
            spi.write_reg(Reg::PayloadLength, size as u8);
            spi.set_mode(Mode::Tx);
            while !spi.received_irq(IRQ::TxDone) {}
            // Section 4.1.3 states we automatically return to Stdby here
            spi.clear_irq(IRQ::TxDone);
            Ok(size)
        }
    }
}
