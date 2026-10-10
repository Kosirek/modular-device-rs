//! 25LC1024 SPI EEPROM driver, ported from `25lc1024.c` / `25lc1024.h`.
//!
//! Talks to any [`SpiDevice`] (the device wrapper drives chip-select around
//! each transaction), so it can be exercised in tests with an
//! `embedded-hal-mock` SPI device that records the exact command/address bytes.

use crate::hal::{EepromConfig, HalError, HalResult, SpiDevice, SpiOperation};

// Instruction set.
pub const CMD_READ: u8 = 0x03;
pub const CMD_WRITE: u8 = 0x02;
pub const CMD_WREN: u8 = 0x06;
pub const CMD_WRDI: u8 = 0x04;
pub const CMD_RDSR: u8 = 0x05;
pub const CMD_WRSR: u8 = 0x01;
pub const CMD_ERASE_PAGE: u8 = 0x42;
pub const CMD_ERASE_SECTOR: u8 = 0xD8;
pub const CMD_ERASE_CHIP: u8 = 0xC7;

pub const PAGE_SIZE: u32 = 256;
pub const SECTOR_SIZE: u32 = 32_768;
pub const CHIP_SIZE: u32 = 131_072;

/// 25LC1024 driver bound to an SPI device.
pub struct Eeprom<S: SpiDevice> {
    spi: S,
    config: EepromConfig,
}

impl<S: SpiDevice> Eeprom<S> {
    pub fn new(spi: S) -> Self {
        Self::with_config(spi, EepromConfig::default())
    }

    /// Create an EEPROM driver with explicit configuration.
    pub fn with_config(spi: S, config: EepromConfig) -> Self {
        Self { spi, config }
    }

    /// Access the device configuration.
    pub fn config(&self) -> &EepromConfig {
        &self.config
    }

    /// Build the 4-byte command + 24-bit address header (MSB first), matching
    /// the C `write_flash_8` / `read_flash_8` address sequence.
    fn cmd_addr(cmd: u8, addr: u32) -> [u8; 4] {
        [cmd, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8]
    }

    /// Read the status register (`read_flash_status`).
    pub fn read_status(&mut self) -> HalResult<u8> {
        let mut buf = [CMD_RDSR, 0xFF];
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(|_| HalError::Bus)?;
        Ok(buf[1])
    }

    /// Write the status register (`write_flash_status`).
    pub fn write_status(&mut self, value: u8) -> HalResult<()> {
        self.write_enable()?;
        self.spi
            .write(&[CMD_WRSR, value])
            .map_err(|_| HalError::Bus)
    }

    /// `true` while a write/erase is in progress (`is_flash_busy`).
    pub fn is_busy(&mut self) -> HalResult<bool> {
        Ok((self.read_status()? & 0x01) != 0)
    }

    /// Set the write-enable latch (`WREN`).
    pub fn write_enable(&mut self) -> HalResult<()> {
        self.spi.write(&[CMD_WREN]).map_err(|_| HalError::Bus)
    }

    /// Clear the write-enable latch (`WRDI`).
    pub fn write_disable(&mut self) -> HalResult<()> {
        self.spi.write(&[CMD_WRDI]).map_err(|_| HalError::Bus)
    }

    /// Read `buf.len()` bytes starting at `addr` (`read_flash_8`).
    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> HalResult<()> {
        let header = Self::cmd_addr(CMD_READ, addr);
        self.spi
            .transaction(&mut [SpiOperation::Write(&header), SpiOperation::Read(buf)])
            .map_err(|_| HalError::Bus)
    }

    /// Write `data` starting at `addr`. The write must stay within a single
    /// page (`write_flash_page`). Sets WREN first.
    /// Returns `Err(HalError::Bus)` if the write would cross a page boundary.
    pub fn write(&mut self, addr: u32, data: &[u8]) -> HalResult<()> {
        let page_size = self.config.page_size;
        let page_offset = (addr % page_size) as usize;
        if page_offset + data.len() > page_size as usize {
            return Err(HalError::Bus);
        }
        self.write_enable()?;
        let header = Self::cmd_addr(CMD_WRITE, addr);
        self.spi
            .transaction(&mut [SpiOperation::Write(&header), SpiOperation::Write(data)])
            .map_err(|_| HalError::Bus)
    }

    /// Poll status register until write/erase completes (`WIP == 0`), or max_spins expires.
    pub fn wait_ready(&mut self, max_spins: u32) -> HalResult<()> {
        for _ in 0..max_spins {
            if !self.is_busy()? {
                return Ok(());
            }
        }
        Err(HalError::Timeout)
    }

    /// Write arbitrary length `data` starting at `addr`, crossing page
    /// boundaries automatically and waiting for each page write to complete.
    pub fn write_all(&mut self, mut addr: u32, mut data: &[u8]) -> HalResult<()> {
        let page_size = self.config.page_size;
        while !data.is_empty() {
            let page_offset = (addr % page_size) as usize;
            let bytes_in_page = (page_size as usize - page_offset).min(data.len());
            self.write(addr, &data[..bytes_in_page])?;
            self.wait_ready(100_000)?;
            addr += bytes_in_page as u32;
            data = &data[bytes_in_page..];
        }
        Ok(())
    }

    /// Erase the 256-byte page containing `addr` (`erase_flash_page`).
    pub fn erase_page(&mut self, addr: u32) -> HalResult<()> {
        self.erase(CMD_ERASE_PAGE, addr)
    }

    /// Erase the 32 KiB sector containing `addr` (`erase_flash_sector`).
    pub fn erase_sector(&mut self, addr: u32) -> HalResult<()> {
        self.erase(CMD_ERASE_SECTOR, addr)
    }

    /// Erase the whole chip (`erase_flash_chip`).
    ///
    /// Note: the C version used the *sector* opcode here by mistake; this port
    /// uses the correct `ERASE_CHIP` (0xC7) instruction.
    pub fn erase_chip(&mut self) -> HalResult<()> {
        self.write_enable()?;
        self.spi.write(&[CMD_ERASE_CHIP]).map_err(|_| HalError::Bus)
    }

    fn erase(&mut self, cmd: u8, addr: u32) -> HalResult<()> {
        self.write_enable()?;
        let header = Self::cmd_addr(cmd, addr);
        self.spi.write(&header).map_err(|_| HalError::Bus)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as Tx};

    #[test]
    fn read_status_sends_rdsr_then_clocks_byte() {
        let expectations = [
            Tx::transaction_start(),
            Tx::transfer_in_place(vec![CMD_RDSR, 0xFF], vec![0x00, 0x03]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        assert_eq!(eeprom.read_status().unwrap(), 0x03);
        spi.done();
    }

    #[test]
    fn is_busy_reflects_wip_bit() {
        let expectations = [
            Tx::transaction_start(),
            Tx::transfer_in_place(vec![CMD_RDSR, 0xFF], vec![0x00, 0x01]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        assert!(eeprom.is_busy().unwrap());
        spi.done();
    }

    #[test]
    fn read_emits_command_and_msb_first_address() {
        // 0x012345 -> 0x01, 0x23, 0x45; two data bytes read back as 0xAB.
        let expectations = [
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_READ, 0x01, 0x23, 0x45]),
            Tx::read_vec(vec![0xAB, 0xAB]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        let mut buf = [0u8; 2];
        eeprom.read(0x012345, &mut buf).unwrap();
        assert_eq!(buf, [0xAB, 0xAB]);
        spi.done();
    }

    #[test]
    fn write_sets_wren_then_writes_payload() {
        let expectations = [
            // write_enable: a one-byte WREN transaction.
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_WREN]),
            Tx::transaction_end(),
            // write: command+address header, then the payload.
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_WRITE, 0x00, 0x01, 0x00]),
            Tx::write_vec(vec![0xDE, 0xAD]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        eeprom.write(0x000100, &[0xDE, 0xAD]).unwrap();
        spi.done();
    }

    #[test]
    fn erase_chip_uses_correct_opcode() {
        let expectations = [
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_WREN]),
            Tx::transaction_end(),
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_ERASE_CHIP]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        eeprom.erase_chip().unwrap();
        spi.done();
    }

    #[test]
    fn erase_sector_sends_address() {
        let expectations = [
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_WREN]),
            Tx::transaction_end(),
            Tx::transaction_start(),
            Tx::write_vec(vec![CMD_ERASE_SECTOR, 0x00, 0x80, 0x00]),
            Tx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut eeprom = Eeprom::new(spi.clone());
        eeprom.erase_sector(0x8000).unwrap();
        spi.done();
    }

    #[test]
    fn write_rejects_crossing_page_boundary() {
        let mut spi = SpiMock::new(&[]);
        let mut eeprom = Eeprom::new(spi.clone());
        // Page ends at 256. Starting at 255 with 2 bytes crosses the page boundary.
        assert_eq!(eeprom.write(255, &[1, 2]), Err(HalError::Bus));
        spi.done();
    }
}
