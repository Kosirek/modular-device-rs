//! SD card + FAT filesystem storage, built on the [`embedded-sdmmc`] crate.
//!
//! This replaces the hand-written `sdcard` (a port of `mmcbb.c`) and `fat16`
//! (a port of `ff.c`) modules with the maintained `embedded-sdmmc` stack: an
//! SD-over-SPI block driver plus a FAT16/FAT32 filesystem. The original C
//! firmware did the same thing with ChaN's `mmcbb.c` + FatFs (`ff.c`).
//!
//! The firmware only needs to append timestamped lines to a single
//! root-directory file (`DEBUG.LOG`); [`SdLogSink`] provides that on top of a
//! [`VolumeManager`] and implements [`LogSink`] so [`crate::logs::Logger`] can
//! write straight to the card.
//!
//! As in the original, the card must already be FAT-formatted with a partition
//! table (the user formats it on a PC) — `embedded-sdmmc` mounts, it does not
//! format.
//!
//! On real hardware you construct the `VolumeManager` from
//! `embedded_sdmmc::SdCard::new(spi_device, delay)` where `spi_device`
//! implements [`crate::hal::SpiDevice`] and `delay` implements
//! [`crate::hal::DelayNs`]. Tests use an in-RAM block device.

use crate::hal::{HalError, HalResult, LogSink};
use embedded_sdmmc::{
    BlockDevice, Mode, RawDirectory, RawVolume, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};

#[cfg(not(test))]
use alloc::{string::String, vec, vec::Vec};

/// A [`TimeSource`] that reports a fixed epoch.
///
/// FatFs file timestamps are not used by the firmware's logic, so a constant is
/// sufficient; swap in an RTC-backed implementation if real timestamps matter.
#[derive(Debug, Default, Clone, Copy)]
pub struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Append-only log file on a FAT volume — the equivalent of `logs.c` writing
/// `debug.log` through FatFs.
///
/// Wraps an `embedded-sdmmc` [`VolumeManager`]. Each line opens the file in
/// append mode, writes, and closes again, so no filesystem handles are held
/// between calls (mirrors the open/seek-end/write/close the C code performs).
pub struct SdLogSink<D, T>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    vm: VolumeManager<D, T>,
    path: String,
    volume_idx: usize,
}

impl<D, T> SdLogSink<D, T>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    /// Wrap a `VolumeManager` and target `path` (e.g. `"DEBUG.LOG"`).
    pub fn new(vm: VolumeManager<D, T>, path: &str) -> Self {
        Self {
            vm,
            path: String::from(path),
            volume_idx: 0,
        }
    }

    /// Wrap a `VolumeManager` configured according to [`crate::hal::StorageConfig`].
    pub fn with_config(vm: VolumeManager<D, T>, config: &crate::hal::StorageConfig) -> Self {
        Self {
            vm,
            path: String::from(config.log_filename),
            volume_idx: config.volume_idx,
        }
    }

    /// Open configured volume and its root directory, returning both raw handles.
    fn open_root(&self) -> HalResult<(RawVolume, RawDirectory)> {
        let volume = self
            .vm
            .open_raw_volume(VolumeIdx(self.volume_idx))
            .map_err(|_| HalError::Bus)?;
        let root = self.vm.open_root_dir(volume).map_err(|_| HalError::Bus)?;
        Ok((volume, root))
    }

    /// Append raw bytes to the log file, creating it if necessary.
    pub fn append(&self, data: &[u8]) -> HalResult<()> {
        self.append_slices(&[data])
    }

    /// Append multiple slices sequentially without intermediate heap allocations.
    pub fn append_slices(&self, slices: &[&[u8]]) -> HalResult<()> {
        let (volume, root) = self.open_root()?;
        let result = (|| -> HalResult<()> {
            let file = self
                .vm
                .open_file_in_dir(root, self.path.as_str(), Mode::ReadWriteCreateOrAppend)
                .map_err(|_| HalError::Bus)?;

            let mut write_err = false;
            for slice in slices {
                if self.vm.write(file, slice).is_err() {
                    write_err = true;
                    break;
                }
            }

            // Always close file handle, preserving any write error
            let close_res = self.vm.close_file(file).map_err(|_| HalError::Bus);
            if write_err {
                return Err(HalError::Bus);
            }
            close_res
        })();
        // Always release the directory/volume handles, even on error.
        let _ = self.vm.close_dir(root);
        let _ = self.vm.close_volume(volume);
        result
    }

    /// Read the whole log file back (used by tests / diagnostics).
    pub fn read_all(&self) -> HalResult<Vec<u8>> {
        let (volume, root) = self.open_root()?;
        let result = (|| -> HalResult<Vec<u8>> {
            let file = self
                .vm
                .open_file_in_dir(root, self.path.as_str(), Mode::ReadOnly)
                .map_err(|_| HalError::Bus)?;
            let len = self.vm.file_length(file).map_err(|_| HalError::Bus)? as usize;
            let mut out = vec![0u8; len];
            let mut got = 0;
            while got < len {
                let n = self
                    .vm
                    .read(file, &mut out[got..])
                    .map_err(|_| HalError::Bus)?;
                if n == 0 {
                    break;
                }
                got += n;
            }
            out.truncate(got);
            self.vm.close_file(file).map_err(|_| HalError::Bus)?;
            Ok(out)
        })();
        let _ = self.vm.close_dir(root);
        let _ = self.vm.close_volume(volume);
        result
    }

    /// Consume the sink and return the underlying `VolumeManager`.
    pub fn into_inner(self) -> VolumeManager<D, T> {
        self.vm
    }
}

impl<D, T> LogSink for SdLogSink<D, T>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    fn write_line(&mut self, line: &str) -> HalResult<()> {
        self.append_slices(&[line.as_bytes(), b"\n"])
    }

    fn flush(&mut self) -> HalResult<()> {
        // Each append closes the file, committing the FAT + directory entry, so
        // there is nothing buffered to flush.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;
    use embedded_sdmmc::{Block, BlockCount, BlockIdx};

    const SECTOR: usize = 512;
    const PART_START_LBA: u32 = 2048;
    const PART_SECTORS: u32 = 98_304; // 48 MiB -> comfortably FAT16
    const TOTAL_SECTORS: u32 = PART_START_LBA + PART_SECTORS;

    /// In-RAM block device for `embedded-sdmmc`. `read`/`write` take `&self`,
    /// so the backing buffer lives behind a `RefCell`.
    struct RamDisk {
        data: RefCell<Vec<u8>>,
    }

    impl RamDisk {
        fn new(image: Vec<u8>) -> Self {
            Self {
                data: RefCell::new(image),
            }
        }
    }

    impl BlockDevice for RamDisk {
        type Error = core::convert::Infallible;

        fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Self::Error> {
            let data = self.data.borrow();
            for (i, block) in blocks.iter_mut().enumerate() {
                let off = (start.0 as usize + i) * SECTOR;
                block.contents.copy_from_slice(&data[off..off + SECTOR]);
            }
            Ok(())
        }

        fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Self::Error> {
            let mut data = self.data.borrow_mut();
            for (i, block) in blocks.iter().enumerate() {
                let off = (start.0 as usize + i) * SECTOR;
                data[off..off + SECTOR].copy_from_slice(&block.contents);
            }
            Ok(())
        }

        fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
            Ok(BlockCount((self.data.borrow().len() / SECTOR) as u32))
        }
    }

    /// Build a disk image: an MBR with one FAT16 partition entry, followed by a
    /// partition formatted with the `fatfs` crate (embedded-sdmmc can't
    /// format, so we prepare a realistic pre-formatted card here).
    fn formatted_image() -> Vec<u8> {
        // Format the partition region in isolation; a fresh Cursor sits at 0,
        // which `format_volume` requires.
        let mut part = vec![0u8; PART_SECTORS as usize * SECTOR];
        fatfs::format_volume(
            std::io::Cursor::new(&mut part[..]),
            fatfs::FormatVolumeOptions::new().fat_type(fatfs::FatType::Fat16),
        )
        .expect("format fat16");

        // Assemble the full image and write a minimal MBR partition entry 1.
        let mut image = vec![0u8; TOTAL_SECTORS as usize * SECTOR];
        let p = 446; // first partition table entry
        image[p] = 0x00; // not bootable
        image[p + 4] = 0x06; // partition type: FAT16
        image[p + 8..p + 12].copy_from_slice(&PART_START_LBA.to_le_bytes());
        image[p + 12..p + 16].copy_from_slice(&PART_SECTORS.to_le_bytes());
        image[510] = 0x55;
        image[511] = 0xAA;
        let start = PART_START_LBA as usize * SECTOR;
        image[start..start + part.len()].copy_from_slice(&part);
        image
    }

    fn make_sink() -> SdLogSink<RamDisk, FixedTime> {
        let disk = RamDisk::new(formatted_image());
        let vm = VolumeManager::new(disk, FixedTime);
        SdLogSink::new(vm, "DEBUG.LOG")
    }

    #[test]
    fn appends_and_reads_back_lines() {
        let mut sink = make_sink();
        sink.write_line("first").unwrap();
        sink.write_line("second").unwrap();
        assert_eq!(sink.read_all().unwrap(), b"first\nsecond\n");
    }

    #[test]
    fn data_survives_new_sink_on_same_disk() {
        let mut sink = make_sink();
        sink.write_line("persisted").unwrap();
        // Re-wrap the same volume manager: the file is still there.
        let vm = sink.into_inner();
        let sink2 = SdLogSink::new(vm, "DEBUG.LOG");
        assert_eq!(sink2.read_all().unwrap(), b"persisted\n");
    }

    #[test]
    fn integrates_with_logger() {
        use crate::logs::Logger;
        use crate::rtc::TimeStruct;

        let sink = make_sink();
        let mut logger = Logger::new(sink);
        logger.init();
        logger.open().unwrap();
        let t = TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 7,
            hour: 1,
            minute: 2,
            second: 3,
            ..Default::default()
        };
        logger.add("event", &t, 99).unwrap();

        let sink = logger.into_sink();
        let text = String::from_utf8(sink.read_all().unwrap()).unwrap();
        assert!(text.starts_with("Start logging ..."));
        assert!(text.contains("#99 :: event"));
    }

    #[test]
    fn with_config_uses_custom_filename() {
        let config = crate::hal::StorageConfig {
            log_filename: "CUSTOM.LOG",
            volume_idx: 0,
            ..crate::hal::StorageConfig::default()
        };
        let disk = RamDisk::new(formatted_image());
        let vm = VolumeManager::new(disk, FixedTime);
        let mut sink = SdLogSink::with_config(vm, &config);
        sink.write_line("custom line").unwrap();
        assert_eq!(sink.read_all().unwrap(), b"custom line\n");
    }
}
