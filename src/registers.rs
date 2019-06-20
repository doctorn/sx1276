use bit_field::BitField;
use std::cmp;

use embedded_hal::blocking::spi::{Transfer, Write};

pub trait RegisterFile {
    fn read_reg(&mut self, reg: Reg) -> u8;

    fn write_reg(&mut self, reg: Reg, byte: u8);

    fn get_version(&mut self) -> u8 {
        self.read_reg(Reg::Version)
    }

    fn in_implicit_header_mode(&mut self) -> bool {
        self.read_reg(Reg::ModemConfig1).get_bit(0)
    }

    fn in_explicit_header_mode(&mut self) -> bool {
        !self.read_reg(Reg::ModemConfig1).get_bit(0)
    }

    fn set_implicit_header_mode(&mut self) {
        let config = self.read_reg(Reg::ModemConfig1);
        self.write_reg(Reg::ModemConfig1, config | 0x01);
    }

    fn set_explicit_header_mode(&mut self) {
        let config = self.read_reg(Reg::ModemConfig1);
        self.write_reg(Reg::ModemConfig1, config & 0xFE);
    }

    fn crc_enabled(&mut self) -> bool {
        self.read_reg(Reg::ModemConfig2).get_bit(2)
    }

    fn enable_crc(&mut self) {
        let config = self.read_reg(Reg::ModemConfig2);
        self.write_reg(Reg::ModemConfig2, config | 0x04);
    }

    fn disable_crc(&mut self) {
        let config = self.read_reg(Reg::ModemConfig2);
        self.write_reg(Reg::ModemConfig2, config & 0xFB);
    }

    fn set_frequency(&mut self, freq: u64) {
        // Section 4.1.4.
        let frf = (freq * (1 << 19)) / 32;
        self.write_reg(Reg::FrfMsb, ((frf & 0x00FF_0000) >> 16) as u8);
        self.write_reg(Reg::FrfMsb, ((frf & 0x0000_FF00) >> 8) as u8);
        self.write_reg(Reg::FrfMsb, (frf & 0x0000_00FF) as u8);
    }

    fn get_last_rssi(&mut self) -> i32 {
        // Section 5.5.5.
        i32::from(self.read_reg(Reg::PktRssiValue)) - 157
    }

    fn get_last_snr(&mut self) -> f64 {
        // Section 6.4.
        f64::from(self.read_reg(Reg::PktSnrValue)) / 4f64
    }

    fn set_transmission_power(&mut self, power: u8) {
        let power = cmp::min(power, 17);
        let power = cmp::max(power, 2);
        self.write_reg(Reg::PaConfig, 0x80 | (power - 2));
    }

    fn set_mode(&mut self, mode: Mode) {
        self.write_reg(Reg::OpMode, Mode::LoRa as u8 | mode as u8);
    }

    fn received_irq(&mut self, irq: Irq) -> bool {
        self.read_reg(Reg::IrqFlags).get_bit(irq as usize)
    }

    fn clear_irq(&mut self) {
        self.write_reg(Reg::IrqFlags, 0xFF);
    }
}

impl<SPI, E> RegisterFile for SPI
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
{
    fn read_reg(&mut self, reg: Reg) -> u8 {
        let mut buffer = [reg as u8 & 0x7f, 0];
        self.transfer(&mut buffer).ok().unwrap();
        buffer[1]
    }

    fn write_reg(&mut self, reg: Reg, byte: u8) {
        let mut buffer = [reg as u8 | 0x80, byte];
        self.write(&mut buffer).ok().unwrap();
    }
}

#[derive(Copy, Clone)]
#[allow(unused)]
pub enum Reg {
    Fifo = 0x00,
    OpMode = 0x01,
    FrfMsb = 0x06,
    FrfMid = 0x07,
    FrfLsb = 0x08,
    PaConfig = 0x09,
    Ocp = 0x0b,
    Lna = 0x0c,
    FifoAddrPtr = 0x0d,
    FifoTxBaseAddr = 0x0e,
    FifoRxBaseAddr = 0x0f,
    FifoRxCurrentAddr = 0x10,
    IrqFlags = 0x12,
    RxNbBytes = 0x13,
    PktSnrValue = 0x19,
    PktRssiValue = 0x1a,
    ModemConfig1 = 0x1d,
    ModemConfig2 = 0x1e,
    PreambleMsb = 0x20,
    PreambleLsb = 0x21,
    PayloadLength = 0x22,
    ModemConfig3 = 0x26,
    FreqErrorMsb = 0x28,
    FreqErrorMid = 0x29,
    FreqErrorLsb = 0x2a,
    RssiWideband = 0x2c,
    DetectionOptimize = 0x31,
    Invertiq = 0x33,
    DetectionThreshold = 0x37,
    SyncWord = 0x39,
    Invertiq2 = 0x3b,
    DioMapping1 = 0x40,
    Version = 0x42,
    PaDac = 0x4d,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[allow(unused)]
pub enum Mode {
    LoRa = 0x80,
    Sleep = 0x00,
    Stdby = 0x01,
    Tx = 0x03,
    RxContinuous = 0x05,
    RxSingle = 0x06,
}

#[derive(Copy, Clone)]
#[allow(unused)]
pub enum Irq {
    RxTimeout = 0x07,
    RxDone = 0x06,
    PayloadCrcError = 0x05,
    ValidHeader = 0x04,
    TxDone = 0x03,
    CadDone = 0x02,
    FhssChangeChannel = 0x01,
    CadDetected = 0x00,
}
