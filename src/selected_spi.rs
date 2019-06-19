use std::ops;
use std::sync::{Mutex, MutexGuard};

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

pub struct SelectedSPI<SPI, NSS>(Mutex<(SPI, NSS)>);

impl<SPI, NSS, E> SelectedSPI<SPI, NSS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    NSS: OutputPin,
{
    pub fn new(spi: SPI, mut nss: NSS) -> SelectedSPI<SPI, NSS> {
        nss.set_high().ok().unwrap();
        SelectedSPI(Mutex::new((spi, nss)))
    }

    pub fn select(&self) -> SPIGuard<SPI, NSS> {
        SPIGuard(self.0.lock().unwrap())
    }
}

pub struct SPIGuard<'a, SPI, NSS>(MutexGuard<'a, (SPI, NSS)>)
where
    NSS: OutputPin;

impl<'a, SPI, NSS> ops::Drop for SPIGuard<'a, SPI, NSS>
where
    NSS: OutputPin,
{
    fn drop(&mut self) {
        let (_, ref mut nss) = *self.0;
        nss.set_high().ok().unwrap();
    }
}

impl<'a, SPI, NSS, E> ops::Deref for SPIGuard<'a, SPI, NSS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    NSS: OutputPin,
{
    type Target = SPI;

    fn deref(&self) -> &Self::Target {
        let (ref spi, _) = *self.0;
        spi
    }
}

impl<'a, SPI, NSS, E> ops::DerefMut for SPIGuard<'a, SPI, NSS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    NSS: OutputPin,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        let (ref mut spi, ref mut nss) = *self.0;
        nss.set_high().ok().unwrap();
        nss.set_low().ok().unwrap();
        spi
    }
}
