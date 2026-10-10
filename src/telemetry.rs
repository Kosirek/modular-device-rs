//! Sensor telemetry formatting and structured data logging (`DATA.LOG`).
//!
//! Port of the telemetry formatting and logging loops from `mainPrg.c`.
//! Handles formatting sensor readings (accelerometer, electronic compass,
//! GPS time/coordinates, temperature/voltage) into standard strings for:
//! 1. Appending to the SD card `data.log` file via [`DataLogger`].
//! 2. Transmitting as UDP broadcast payloads via [`crate::net_stack::NetworkStack`].

use crate::accelerometer::AccSample;
use crate::compass::Orientation;
use crate::gps::GpsData;
use crate::hal::{HalResult, LogSink};
use crate::rtc::TimeStruct;

#[cfg(not(test))]
use alloc::{format, string::String, vec::Vec};

/// Format an accelerometer sample matching `mainPrg.c`:
/// `"Accelerometer-X<X>,Y<Y>,Z<Z>*"`.
pub fn format_accelerometer(sample: &AccSample) -> String {
    format!("Accelerometer-X{},Y{},Z{}*", sample.x, sample.y, sample.z)
}

/// Format compass orientation angles matching `mainPrg.c`:
/// `"Compass-<heading>,<pitch>,<roll>*"`.
pub fn format_compass(orientation: &Orientation) -> String {
    format!(
        "Compass-{:.2},{:.2},{:.2}*",
        orientation.heading, orientation.pitch, orientation.roll
    )
}

/// Format GPS timestamp and coordinates matching `mainPrg.c`:
/// `"GPS_time-<hour>:<min>,<year>/<month>/<day>,<lon>,<lat>*"`.
pub fn format_gps(gps: &GpsData) -> String {
    format!(
        "GPS_time-{}:{},{}/{}/{},{},{}*",
        gps.time.hour,
        gps.time.minute,
        gps.date.year,
        gps.date.month,
        gps.date.day,
        gps.coords.longitude,
        gps.coords.latitude
    )
}

/// Format 3 temperature sensor readings and 1 power supply voltage reading:
/// `"Temperature-<T1>,<T2>,<T3>,<V>*"`.
pub fn format_temperature(temp: &[f32; 3], power_volt: f32) -> String {
    format!(
        "Temperature-{:.1},{:.1},{:.1},{:.1}*",
        temp[0], temp[1], temp[2], power_volt
    )
}

/// Format RTC date and time prefix matching `mainPrg.c`:
/// `"YYYY-MM-DD HH:MM:SS  "`.
pub fn format_rtc_timestamp(rtc: &TimeStruct) -> String {
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}  ",
        rtc.year, rtc.month, rtc.day_of_month, rtc.hour, rtc.minute, rtc.second
    )
}

/// Strongly-typed sensor telemetry record.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryRecord {
    /// 3-axis accelerometer sample.
    Accelerometer(AccSample),
    /// Electronic compass heading, pitch, and roll.
    Compass(Orientation),
    /// Decoded GPS timestamp and coordinates.
    Gps(GpsData),
    /// Temperatures and supply voltage.
    Temperature {
        temp: [f32; 3],
        power_volt: f32,
    },
    /// Custom / arbitrary telemetry string.
    Custom(String),
}

impl TelemetryRecord {
    /// Format the record into the standard firmware protocol string.
    pub fn to_telemetry_string(&self) -> String {
        match self {
            Self::Accelerometer(acc) => format_accelerometer(acc),
            Self::Compass(comp) => format_compass(comp),
            Self::Gps(gps) => format_gps(gps),
            Self::Temperature { temp, power_volt } => format_temperature(temp, *power_volt),
            Self::Custom(msg) => msg.clone(),
        }
    }
}

/// Reusable buffer for batching log lines before writing to the sink.
pub struct DataLogger<S: LogSink> {
    sink: S,
    enabled: bool,
    opened: bool,
    buffered_lines: Vec<String>,
    batch_threshold: usize,
}

impl<S: LogSink> DataLogger<S> {
    /// Create a new `DataLogger` writing directly to `sink`.
    pub fn new(sink: S) -> Self {
        Self::with_batching(sink, 1)
    }

    /// Create a `DataLogger` that batches up to `batch_threshold` lines in memory
    /// before committing them to the backing sink (reduces SPI/FAT transaction churn).
    pub fn with_batching(sink: S, batch_threshold: usize) -> Self {
        Self {
            sink,
            enabled: true,
            opened: false,
            buffered_lines: Vec::new(),
            batch_threshold: batch_threshold.max(1),
        }
    }

    /// Enable or disable logging to the backing store (controlled by e.g. SW1/SW2).
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// `true` if logging is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Open the log file, writing the startup RTC timestamp and banner.
    pub fn open(&mut self, rtc: &TimeStruct) -> HalResult<()> {
        if self.enabled {
            let ts = format_rtc_timestamp(rtc);
            self.sink.write_line(&format!("{}Start logging ...", ts))?;
            self.opened = true;
        }
        Ok(())
    }

    /// Write an RTC timestamp line into the log (called every `FILE_RELOAD_TIME`).
    pub fn log_timestamp(&mut self, rtc: &TimeStruct) -> HalResult<()> {
        if self.enabled && self.opened {
            let ts = format_rtc_timestamp(rtc);
            self.write_line_internal(&ts)?;
        }
        Ok(())
    }

    /// Log a strongly-typed telemetry record.
    pub fn log_record(&mut self, record: &TelemetryRecord) -> HalResult<()> {
        if self.enabled && self.opened {
            let line = record.to_telemetry_string();
            self.write_line_internal(&line)?;
        }
        Ok(())
    }

    /// Log a raw pre-formatted line (e.g. from an existing buffer).
    pub fn log_raw(&mut self, line: &str) -> HalResult<()> {
        if self.enabled && self.opened {
            self.write_line_internal(line)?;
        }
        Ok(())
    }

    fn write_line_internal(&mut self, line: &str) -> HalResult<()> {
        if self.batch_threshold <= 1 {
            self.sink.write_line(line)
        } else {
            self.buffered_lines.push(String::from(line));
            if self.buffered_lines.len() >= self.batch_threshold {
                self.flush_buffer()?;
            }
            Ok(())
        }
    }

    fn flush_buffer(&mut self) -> HalResult<()> {
        for line in self.buffered_lines.drain(..) {
            self.sink.write_line(&line)?;
        }
        Ok(())
    }

    /// Flush any pending buffered lines and sync the backing storage (`f_sync`).
    pub fn sync(&mut self) -> HalResult<()> {
        if self.opened {
            self.flush_buffer()?;
            self.sink.flush()?;
        }
        Ok(())
    }

    /// Consume the logger and return the underlying sink.
    pub fn into_sink(mut self) -> HalResult<S> {
        let _ = self.flush_buffer();
        Ok(self.sink)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gps::{Coordinates, GpsDate, GpsTime};
    use std::cell::RefCell;
    use std::rc::Rc;

    #[derive(Clone, Default)]
    struct MockSink {
        lines: Rc<RefCell<Vec<String>>>,
        flushed: Rc<RefCell<usize>>,
    }

    impl LogSink for MockSink {
        fn write_line(&mut self, line: &str) -> HalResult<()> {
            self.lines.borrow_mut().push(line.to_string());
            Ok(())
        }

        fn flush(&mut self) -> HalResult<()> {
            *self.flushed.borrow_mut() += 1;
            Ok(())
        }
    }

    fn sample_rtc() -> TimeStruct {
        TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 7,
            hour: 14,
            minute: 30,
            second: 15,
            ..Default::default()
        }
    }

    #[test]
    fn format_accelerometer_matches_c_pattern() {
        let acc = AccSample {
            x: 12,
            y: -5,
            z: 128,
        };
        assert_eq!(format_accelerometer(&acc), "Accelerometer-X12,Y-5,Z128*");
    }

    #[test]
    fn format_compass_matches_c_pattern() {
        let comp = Orientation {
            heading: 180.25,
            pitch: -12.50,
            roll: 5.75,
        };
        assert_eq!(format_compass(&comp), "Compass-180.25,-12.50,5.75*");
    }

    #[test]
    fn format_gps_matches_c_pattern() {
        let gps = GpsData {
            time: GpsTime {
                hour: 12,
                minute: 35,
                second: 0,
            },
            date: GpsDate {
                year: 2024,
                month: 6,
                day: 7,
            },
            coords: Coordinates::new("4807.038", 'N', "01131.000", 'E'),
            ..Default::default()
        };
        assert_eq!(
            format_gps(&gps),
            "GPS_time-12:35,2024/6/7,01131.000,4807.038*"
        );
    }

    #[test]
    fn format_temperature_matches_c_pattern() {
        let temp = [21.5, 22.0, 23.5];
        let pwr = 12.4;
        assert_eq!(
            format_temperature(&temp, pwr),
            "Temperature-21.5,22.0,23.5,12.4*"
        );
    }

    #[test]
    fn format_rtc_timestamp_matches_c_pattern() {
        let rtc = sample_rtc();
        assert_eq!(format_rtc_timestamp(&rtc), "2024-11-07 14:30:15  ");
    }

    #[test]
    fn data_logger_open_and_logs_records() {
        let sink = MockSink::default();
        let mut logger = DataLogger::new(sink.clone());

        let rtc = sample_rtc();
        logger.open(&rtc).unwrap();

        let acc = AccSample { x: 1, y: 2, z: 3 };
        logger.log_record(&TelemetryRecord::Accelerometer(acc)).unwrap();

        let lines = sink.lines.borrow();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0], "2024-11-07 14:30:15  Start logging ...");
        assert_eq!(lines[1], "Accelerometer-X1,Y2,Z3*");
    }

    #[test]
    fn data_logger_disabled_does_not_log() {
        let sink = MockSink::default();
        let mut logger = DataLogger::new(sink.clone());
        logger.open(&sample_rtc()).unwrap();

        logger.set_enabled(false);
        logger
            .log_record(&TelemetryRecord::Accelerometer(AccSample { x: 0, y: 0, z: 0 }))
            .unwrap();

        assert_eq!(sink.lines.borrow().len(), 1); // only the open line
    }

    #[test]
    fn data_logger_batching_reduces_sink_writes_until_sync() {
        let sink = MockSink::default();
        let mut logger = DataLogger::with_batching(sink.clone(), 3);
        logger.open(&sample_rtc()).unwrap(); // open writes directly

        // Write 2 records (< threshold 3) -> should not write to sink yet
        logger.log_raw("line 1").unwrap();
        logger.log_raw("line 2").unwrap();
        assert_eq!(sink.lines.borrow().len(), 1); // only open line

        // Sync flushes pending batch
        logger.sync().unwrap();
        assert_eq!(sink.lines.borrow().len(), 3);
        assert_eq!(*sink.flushed.borrow(), 1);
    }
}
