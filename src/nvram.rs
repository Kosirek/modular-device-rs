//! Non-volatile configuration and crash dump storage in SPI EEPROM (25LC1024).
//!
//! Stores device network settings, sensor sampling intervals, boot counter,
//! and post-mortem crash records with CRC16-CCITT integrity protection.
//!
//! If EEPROM is uninitialised (all `0xFF`) or data corruption is detected,
//! it automatically falls back to safe default parameters.

use crate::eeprom::{CHIP_SIZE, PAGE_SIZE};
use crate::hal::{HalError, HalResult, SpiDevice};
use crate::rtc::TimeStruct;
use alloc::string::String;

pub const CONFIG_MAGIC: &[u8; 8] = b"UMCFG01\0";
pub const PANIC_MAGIC: &[u8; 8] = b"UMPANIC1";
pub const TELEMETRY_MAGIC: &[u8; 8] = b"UMTEL01\0";

pub const CONFIG_EEPROM_ADDR: u32 = 0x0000;
pub const PANIC_EEPROM_ADDR: u32 = 0x0100;

/// First page available for emergency offline telemetry buffering.
pub const OFFLINE_START_PAGE: u32 = 2;
/// Total number of 256-byte pages allocated for the circular telemetry buffer.
pub const OFFLINE_PAGE_COUNT: u32 = 510;

/// Standard CRC16-CCITT (polynomial 0x1021, init 0xFFFF).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Errors related to non-volatile configuration parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NvramError {
    /// Magic header does not match expected signature.
    InvalidMagic,
    /// CRC16 checksum mismatch (corrupted data).
    ChecksumMismatch,
    /// Storage bus or hardware communication fault.
    Storage(HalError),
}

impl From<HalError> for NvramError {
    fn from(err: HalError) -> Self {
        NvramError::Storage(err)
    }
}

/// Persistent device configuration saved in non-volatile memory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NvramConfig {
    pub dhcp_enabled: bool,
    pub ip: [u8; 4],
    pub subnet: [u8; 4],
    pub gateway: [u8; 4],
    pub mac: [u8; 6],
    pub local_port: u16,
    pub remote_port: u16,
    pub command_port: u16,
    pub acc_interval: u64,
    pub led_interval: u64,
    pub temp_interval: u64,
    pub pressure_interval: u64,
    pub file_sync_interval: u64,
    pub default_power_mask: u8,
    pub boot_count: u32,
    pub crash_count: u32,
}

impl Default for NvramConfig {
    fn default() -> Self {
        Self::default_board()
    }
}

impl NvramConfig {
    /// Default factory configuration matching `mainPrg.c` and board defaults.
    pub fn default_board() -> Self {
        Self {
            dhcp_enabled: false,
            ip: [192, 168, 2, 101],
            subnet: [255, 255, 255, 0],
            gateway: [192, 168, 2, 1],
            mac: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            local_port: 22223,
            remote_port: 22223,
            command_port: 22223,
            acc_interval: 1000,
            led_interval: 5000,
            temp_interval: 10000,
            pressure_interval: 5000,
            file_sync_interval: 50000,
            default_power_mask: crate::power::DEFAULT_BOARD_POWER_MASK,
            boot_count: 0,
            crash_count: 0,
        }
    }

    /// Serialize into a fixed 256-byte page buffer with magic header and CRC16.
    pub fn serialize(&self) -> [u8; PAGE_SIZE as usize] {
        let mut buf = [0u8; PAGE_SIZE as usize];

        // 0..8: Magic
        buf[0..8].copy_from_slice(CONFIG_MAGIC);

        // 8: DHCP enabled flag
        buf[8] = if self.dhcp_enabled { 1 } else { 0 };

        // 9..13: IP
        buf[9..13].copy_from_slice(&self.ip);
        // 13..17: Subnet
        buf[13..17].copy_from_slice(&self.subnet);
        // 17..21: Gateway
        buf[17..21].copy_from_slice(&self.gateway);
        // 21..27: MAC
        buf[21..27].copy_from_slice(&self.mac);

        // 27..29: local_port
        buf[27..29].copy_from_slice(&self.local_port.to_be_bytes());
        // 29..31: remote_port
        buf[29..31].copy_from_slice(&self.remote_port.to_be_bytes());
        // 31..33: command_port
        buf[31..33].copy_from_slice(&self.command_port.to_be_bytes());

        // 33..41: acc_interval
        buf[33..41].copy_from_slice(&self.acc_interval.to_be_bytes());
        // 41..49: led_interval
        buf[41..49].copy_from_slice(&self.led_interval.to_be_bytes());
        // 49..57: temp_interval
        buf[49..57].copy_from_slice(&self.temp_interval.to_be_bytes());
        // 57..65: pressure_interval
        buf[57..65].copy_from_slice(&self.pressure_interval.to_be_bytes());
        // 65..73: file_sync_interval
        buf[65..73].copy_from_slice(&self.file_sync_interval.to_be_bytes());

        // 73: default_power_mask
        buf[73] = self.default_power_mask;

        // 74..78: boot_count
        buf[74..78].copy_from_slice(&self.boot_count.to_be_bytes());
        // 78..82: crash_count
        buf[78..82].copy_from_slice(&self.crash_count.to_be_bytes());

        // 254..256: CRC16 of bytes 0..254
        let crc = crc16(&buf[0..254]);
        buf[254..256].copy_from_slice(&crc.to_be_bytes());

        buf
    }

    /// Deserialize a 256-byte page buffer, validating magic and CRC16.
    pub fn deserialize(buf: &[u8; PAGE_SIZE as usize]) -> Result<Self, NvramError> {
        if &buf[0..8] != CONFIG_MAGIC {
            return Err(NvramError::InvalidMagic);
        }

        let expected_crc = u16::from_be_bytes([buf[254], buf[255]]);
        let actual_crc = crc16(&buf[0..254]);
        if expected_crc != actual_crc {
            return Err(NvramError::ChecksumMismatch);
        }

        let dhcp_enabled = buf[8] != 0;
        let mut ip = [0u8; 4];
        ip.copy_from_slice(&buf[9..13]);
        let mut subnet = [0u8; 4];
        subnet.copy_from_slice(&buf[13..17]);
        let mut gateway = [0u8; 4];
        gateway.copy_from_slice(&buf[17..21]);
        let mut mac = [0u8; 6];
        mac.copy_from_slice(&buf[21..27]);

        let local_port = u16::from_be_bytes([buf[27], buf[28]]);
        let remote_port = u16::from_be_bytes([buf[29], buf[30]]);
        let command_port = u16::from_be_bytes([buf[31], buf[32]]);

        let mut b8 = [0u8; 8];
        b8.copy_from_slice(&buf[33..41]);
        let acc_interval = u64::from_be_bytes(b8);
        b8.copy_from_slice(&buf[41..49]);
        let led_interval = u64::from_be_bytes(b8);
        b8.copy_from_slice(&buf[49..57]);
        let temp_interval = u64::from_be_bytes(b8);
        b8.copy_from_slice(&buf[57..65]);
        let pressure_interval = u64::from_be_bytes(b8);
        b8.copy_from_slice(&buf[65..73]);
        let file_sync_interval = u64::from_be_bytes(b8);

        let default_power_mask = buf[73];

        let mut b4 = [0u8; 4];
        b4.copy_from_slice(&buf[74..78]);
        let boot_count = u32::from_be_bytes(b4);
        b4.copy_from_slice(&buf[78..82]);
        let crash_count = u32::from_be_bytes(b4);

        Ok(Self {
            dhcp_enabled,
            ip,
            subnet,
            gateway,
            mac,
            local_port,
            remote_port,
            command_port,
            acc_interval,
            led_interval,
            temp_interval,
            pressure_interval,
            file_sync_interval,
            default_power_mask,
            boot_count,
            crash_count,
        })
    }
}

/// Post-mortem crash report stored in EEPROM.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PanicRecord {
    pub message: String,
    pub timestamp: Option<TimeStruct>,
}

impl PanicRecord {
    pub fn serialize(&self) -> [u8; PAGE_SIZE as usize] {
        let mut buf = [0u8; PAGE_SIZE as usize];
        buf[0..8].copy_from_slice(PANIC_MAGIC);

        if let Some(ref t) = self.timestamp {
            buf[8] = 1;
            buf[9..11].copy_from_slice(&t.year.to_be_bytes());
            buf[11] = t.month;
            buf[12] = t.day_of_month;
            buf[13] = t.hour;
            buf[14] = t.minute;
            buf[15] = t.second;
        } else {
            buf[8] = 0;
        }

        let msg_bytes = self.message.as_bytes();
        let len = msg_bytes.len().min(200);
        buf[16] = len as u8;
        buf[17..17 + len].copy_from_slice(&msg_bytes[..len]);

        let crc = crc16(&buf[0..254]);
        buf[254..256].copy_from_slice(&crc.to_be_bytes());
        buf
    }

    pub fn deserialize(buf: &[u8; PAGE_SIZE as usize]) -> Result<Self, NvramError> {
        if &buf[0..8] != PANIC_MAGIC {
            return Err(NvramError::InvalidMagic);
        }
        let expected_crc = u16::from_be_bytes([buf[254], buf[255]]);
        let actual_crc = crc16(&buf[0..254]);
        if expected_crc != actual_crc {
            return Err(NvramError::ChecksumMismatch);
        }

        let timestamp = if buf[8] == 1 {
            let year = u16::from_be_bytes([buf[9], buf[10]]);
            Some(TimeStruct {
                year,
                month: buf[11],
                day_of_month: buf[12],
                day_of_week: 0,
                hour: buf[13],
                minute: buf[14],
                second: buf[15],
                day_of_year: 0,
            })
        } else {
            None
        };

        let len = buf[16] as usize;
        let msg_slice = &buf[17..17 + len.min(200)];
        let message = String::from_utf8_lossy(msg_slice).into();

        Ok(Self { message, timestamp })
    }
}

/// Compact telemetry record saved to offline emergency EEPROM buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OfflineRecord {
    pub sequence: u32,
    pub timestamp: Option<TimeStruct>,
    pub data: String,
}

impl OfflineRecord {
    pub fn serialize(&self) -> [u8; PAGE_SIZE as usize] {
        let mut buf = [0u8; PAGE_SIZE as usize];
        buf[0..8].copy_from_slice(TELEMETRY_MAGIC);

        buf[8..12].copy_from_slice(&self.sequence.to_be_bytes());

        if let Some(ref t) = self.timestamp {
            buf[12] = 1;
            buf[13..15].copy_from_slice(&t.year.to_be_bytes());
            buf[15] = t.month;
            buf[16] = t.day_of_month;
            buf[17] = t.hour;
            buf[18] = t.minute;
            buf[19] = t.second;
        } else {
            buf[12] = 0;
        }

        let d_bytes = self.data.as_bytes();
        let len = d_bytes.len().min(230);
        buf[20..22].copy_from_slice(&(len as u16).to_be_bytes());
        buf[22..22 + len].copy_from_slice(&d_bytes[..len]);

        let crc = crc16(&buf[0..254]);
        buf[254..256].copy_from_slice(&crc.to_be_bytes());
        buf
    }

    pub fn deserialize(buf: &[u8; PAGE_SIZE as usize]) -> Result<Self, NvramError> {
        if &buf[0..8] != TELEMETRY_MAGIC {
            return Err(NvramError::InvalidMagic);
        }
        let expected_crc = u16::from_be_bytes([buf[254], buf[255]]);
        let actual_crc = crc16(&buf[0..254]);
        if expected_crc != actual_crc {
            return Err(NvramError::ChecksumMismatch);
        }

        let sequence = u32::from_be_bytes([buf[8], buf[9], buf[10], buf[11]]);

        let timestamp = if buf[12] == 1 {
            let year = u16::from_be_bytes([buf[13], buf[14]]);
            Some(TimeStruct {
                year,
                month: buf[15],
                day_of_month: buf[16],
                day_of_week: 0,
                hour: buf[17],
                minute: buf[18],
                second: buf[19],
                day_of_year: 0,
            })
        } else {
            None
        };

        let len = u16::from_be_bytes([buf[20], buf[21]]) as usize;
        let data_slice = &buf[22..22 + len.min(230)];
        let data = String::from_utf8_lossy(data_slice).into();

        Ok(Self {
            sequence,
            timestamp,
            data,
        })
    }
}

/// Abstract non-volatile memory device interface.
pub trait NvramDevice {
    fn read_bytes(&mut self, addr: u32, buf: &mut [u8]) -> HalResult<()>;
    fn write_bytes(&mut self, addr: u32, data: &[u8]) -> HalResult<()>;
}

impl<D: NvramDevice + ?Sized> NvramDevice for &mut D {
    fn read_bytes(&mut self, addr: u32, buf: &mut [u8]) -> HalResult<()> {
        (**self).read_bytes(addr, buf)
    }

    fn write_bytes(&mut self, addr: u32, data: &[u8]) -> HalResult<()> {
        (**self).write_bytes(addr, data)
    }
}

impl<S: SpiDevice> NvramDevice for crate::eeprom::Eeprom<S> {
    fn read_bytes(&mut self, addr: u32, buf: &mut [u8]) -> HalResult<()> {
        self.read(addr, buf)
    }

    fn write_bytes(&mut self, addr: u32, data: &[u8]) -> HalResult<()> {
        self.write_all(addr, data)
    }
}

/// In-memory simulated NVRAM block (128 KB) for tests and host demo.
pub struct InMemoryNvram {
    pub memory: [u8; CHIP_SIZE as usize],
}

impl Default for InMemoryNvram {
    fn default() -> Self {
        Self {
            memory: [0xFF; CHIP_SIZE as usize],
        }
    }
}

impl NvramDevice for InMemoryNvram {
    fn read_bytes(&mut self, addr: u32, buf: &mut [u8]) -> HalResult<()> {
        let start = addr as usize;
        let end = start + buf.len();
        if end > self.memory.len() {
            return Err(HalError::Bus);
        }
        buf.copy_from_slice(&self.memory[start..end]);
        Ok(())
    }

    fn write_bytes(&mut self, addr: u32, data: &[u8]) -> HalResult<()> {
        let start = addr as usize;
        let end = start + data.len();
        if end > self.memory.len() {
            return Err(HalError::Bus);
        }
        self.memory[start..end].copy_from_slice(data);
        Ok(())
    }
}

/// High-level manager for non-volatile configuration and crash dumps.
pub struct NvramManager<D: NvramDevice> {
    device: D,
}

impl<D: NvramDevice> NvramManager<D> {
    pub fn new(device: D) -> Self {
        Self { device }
    }

    /// Access the underlying NVRAM device.
    pub fn device(&self) -> &D {
        &self.device
    }

    /// Mutably access the underlying NVRAM device.
    pub fn device_mut(&mut self) -> &mut D {
        &mut self.device
    }

    /// Load device configuration from EEPROM.
    ///
    /// If uninitialised or corrupted, returns [`NvramConfig::default_board`].
    pub fn load_config(&mut self) -> NvramConfig {
        let mut buf = [0u8; PAGE_SIZE as usize];
        if self.device.read_bytes(CONFIG_EEPROM_ADDR, &mut buf).is_ok() {
            if let Ok(cfg) = NvramConfig::deserialize(&buf) {
                return cfg;
            }
        }
        NvramConfig::default_board()
    }

    /// Save configuration to EEPROM with updated CRC16.
    pub fn save_config(&mut self, cfg: &NvramConfig) -> HalResult<()> {
        let buf = cfg.serialize();
        self.device.write_bytes(CONFIG_EEPROM_ADDR, &buf)
    }

    /// Increment the boot counter in NVRAM and return the new count.
    pub fn increment_boot_count(&mut self) -> HalResult<u32> {
        let mut cfg = self.load_config();
        cfg.boot_count = cfg.boot_count.wrapping_add(1);
        self.save_config(&cfg)?;
        Ok(cfg.boot_count)
    }

    /// Record a crash/panic event in Page 1 and increment the crash counter.
    pub fn record_crash(&mut self, message: &str, rtc_time: Option<&TimeStruct>) -> HalResult<()> {
        let mut cfg = self.load_config();
        cfg.crash_count = cfg.crash_count.wrapping_add(1);
        let _ = self.save_config(&cfg);

        let panic = PanicRecord {
            message: message.into(),
            timestamp: rtc_time.cloned(),
        };
        let buf = panic.serialize();
        self.device.write_bytes(PANIC_EEPROM_ADDR, &buf)
    }

    /// Read the last crash report from Page 1, if valid.
    pub fn read_last_crash(&mut self) -> Option<PanicRecord> {
        let mut buf = [0u8; PAGE_SIZE as usize];
        if self.device.read_bytes(PANIC_EEPROM_ADDR, &mut buf).is_ok() {
            PanicRecord::deserialize(&buf).ok()
        } else {
            None
        }
    }

    /// Write a telemetry record into the offline emergency circular buffer in EEPROM.
    ///
    /// Stores the serialized payload in page `OFFLINE_START_PAGE + (sequence % OFFLINE_PAGE_COUNT)`.
    pub fn log_offline_telemetry(
        &mut self,
        sequence: u32,
        data: &str,
        time: Option<&TimeStruct>,
    ) -> HalResult<u32> {
        let slot = sequence % OFFLINE_PAGE_COUNT;
        let page_addr = (OFFLINE_START_PAGE + slot) * PAGE_SIZE;

        let record = OfflineRecord {
            sequence,
            timestamp: time.cloned(),
            data: data.into(),
        };
        let buf = record.serialize();
        self.device.write_bytes(page_addr, &buf)?;
        Ok(slot)
    }

    /// Read an offline telemetry record from the specified slot (0..509).
    pub fn read_offline_telemetry(&mut self, slot: u32) -> Result<OfflineRecord, NvramError> {
        let page_addr = (OFFLINE_START_PAGE + (slot % OFFLINE_PAGE_COUNT)) * PAGE_SIZE;
        let mut buf = [0u8; PAGE_SIZE as usize];
        self.device.read_bytes(page_addr, &mut buf)?;
        OfflineRecord::deserialize(&buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_computes_deterministic_checksum() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn config_serialization_roundtrip() {
        let cfg = NvramConfig {
            dhcp_enabled: true,
            ip: [10, 0, 0, 42],
            subnet: [255, 0, 0, 0],
            gateway: [10, 0, 0, 1],
            mac: [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC],
            local_port: 3000,
            remote_port: 3001,
            command_port: 3002,
            acc_interval: 200,
            led_interval: 400,
            temp_interval: 600,
            pressure_interval: 800,
            file_sync_interval: 1000,
            default_power_mask: 0x55,
            boot_count: 15,
            crash_count: 2,
        };

        let bytes = cfg.serialize();
        let decoded = NvramConfig::deserialize(&bytes).unwrap();
        assert_eq!(cfg, decoded);
    }

    #[test]
    fn invalid_magic_or_corrupted_crc_rejected() {
        let mut bytes = NvramConfig::default_board().serialize();

        // Corrupt magic
        bytes[0] = b'X';
        assert_eq!(
            NvramConfig::deserialize(&bytes),
            Err(NvramError::InvalidMagic)
        );

        // Restore magic and corrupt payload
        bytes[0] = CONFIG_MAGIC[0];
        bytes[10] ^= 0xFF;
        assert_eq!(
            NvramConfig::deserialize(&bytes),
            Err(NvramError::ChecksumMismatch)
        );
    }

    #[test]
    fn manager_load_uninitialised_returns_defaults() {
        let mut nvram = InMemoryNvram::default();
        let mut mgr = NvramManager::new(&mut nvram);

        let cfg = mgr.load_config();
        assert_eq!(cfg, NvramConfig::default_board());
        assert_eq!(cfg.boot_count, 0);
    }

    #[test]
    fn manager_increments_boot_count_and_persists() {
        let mut nvram = InMemoryNvram::default();
        let mut mgr = NvramManager::new(&mut nvram);

        assert_eq!(mgr.increment_boot_count().unwrap(), 1);
        assert_eq!(mgr.increment_boot_count().unwrap(), 2);
        assert_eq!(mgr.increment_boot_count().unwrap(), 3);

        let loaded = mgr.load_config();
        assert_eq!(loaded.boot_count, 3);
    }

    #[test]
    fn manager_records_and_reads_crash_dump() {
        let mut nvram = InMemoryNvram::default();
        let mut mgr = NvramManager::new(&mut nvram);

        assert_eq!(mgr.read_last_crash(), None);

        let rtc = TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 14,
            day_of_week: 6,
            hour: 21,
            minute: 30,
            second: 0,
            day_of_year: 319,
        };

        mgr.record_crash("HardFault: Bus error at 0x2009C000", Some(&rtc))
            .unwrap();

        let cfg = mgr.load_config();
        assert_eq!(cfg.crash_count, 1);

        let crash = mgr.read_last_crash().unwrap();
        assert_eq!(crash.message, "HardFault: Bus error at 0x2009C000");
        assert_eq!(crash.timestamp.unwrap().year, 2024);
    }

    #[test]
    fn offline_telemetry_buffering_and_ring_wrapping() {
        let mut nvram = InMemoryNvram::default();
        let mut mgr = NvramManager::new(&mut nvram);

        let rtc = TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 20,
            hour: 18,
            minute: 0,
            second: 10,
            ..Default::default()
        };

        // 1. Log first telemetry record
        let slot0 = mgr
            .log_offline_telemetry(0, "Accelerometer-X10,Y-20,Z30*", Some(&rtc))
            .unwrap();
        assert_eq!(slot0, 0);

        // 2. Read back
        let rec0 = mgr.read_offline_telemetry(0).unwrap();
        assert_eq!(rec0.sequence, 0);
        assert_eq!(rec0.data, "Accelerometer-X10,Y-20,Z30*");
        assert_eq!(rec0.timestamp.unwrap().hour, 18);

        // 3. Test ring buffer wrapping: sequence 510 should map to slot 0
        let slot_wrap = mgr
            .log_offline_telemetry(510, "GPS_time-180010*", None)
            .unwrap();
        assert_eq!(slot_wrap, 0);

        let rec_wrap = mgr.read_offline_telemetry(0).unwrap();
        assert_eq!(rec_wrap.sequence, 510);
        assert_eq!(rec_wrap.data, "GPS_time-180010*");
        assert!(rec_wrap.timestamp.is_none());
    }
}
