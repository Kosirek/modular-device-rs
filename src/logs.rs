//! Timestamped logging, ported from `logs.c`.
//!
//! The firmware writes timestamped lines to a FatFs file. Here the storage is
//! abstracted behind [`LogSink`], so tests use an in-memory sink and assert on
//! the exact formatting. The timestamp comes from an [`rtc::TimeStruct`] plus a
//! monotonic tick, matching `add_log`.

use crate::hal::{HalResult, LogSink};
use crate::rtc::TimeStruct;

#[cfg(not(test))]
use alloc::format;
#[cfg(not(test))]
use alloc::string::String;

/// Logger that prefixes each message with `YYYY-MM-DD HH:MM:SS #<ticks> :: `.
pub struct Logger<S: LogSink> {
    sink: S,
    enabled: bool,
    opened: bool,
}

impl<S: LogSink> Logger<S> {
    pub fn new(sink: S) -> Self {
        Self {
            sink,
            enabled: false,
            opened: false,
        }
    }

    /// Enable logging (`init_logs`).
    pub fn init(&mut self) {
        self.enabled = true;
    }

    /// Open the log (`open_log_file`): writes the start banner.
    pub fn open(&mut self) -> HalResult<()> {
        if self.enabled {
            self.sink.write_line("Start logging ...")?;
            self.opened = true;
        }
        Ok(())
    }

    /// Close the log (`close_log_file`): writes the closing banner and flushes.
    pub fn close(&mut self) -> HalResult<()> {
        if self.opened {
            self.sink.write_line("Closing Log file.")?;
            self.sink.flush()?;
            self.opened = false;
        }
        Ok(())
    }

    /// Format prefix into a writer without allocating a String.
    pub fn format_prefix_into<W: core::fmt::Write>(
        w: &mut W,
        time: &TimeStruct,
        ticks: u64,
    ) -> core::fmt::Result {
        write!(
            w,
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02} #{} :: ",
            time.year, time.month, time.day_of_month, time.hour, time.minute, time.second, ticks
        )
    }

    /// Format the timestamp prefix used by `add_log`.
    pub fn format_prefix(time: &TimeStruct, ticks: u64) -> String {
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02} #{} :: ",
            time.year, time.month, time.day_of_month, time.hour, time.minute, time.second, ticks
        )
    }

    /// Append a timestamped message (`add_log`). No-op until opened.
    pub fn add(&mut self, message: &str, time: &TimeStruct, ticks: u64) -> HalResult<()> {
        if self.opened {
            let line = format!(
                "{:04}-{:02}-{:02} {:02}:{:02}:{:02} #{} :: {}",
                time.year,
                time.month,
                time.day_of_month,
                time.hour,
                time.minute,
                time.second,
                ticks,
                message
            );
            self.sink.write_line(&line)?;
        }
        Ok(())
    }

    /// Append and immediately flush (`save_log`).
    pub fn save(&mut self, message: &str, time: &TimeStruct, ticks: u64) -> HalResult<()> {
        self.add(message, time, ticks)?;
        if self.opened {
            self.sink.flush()?;
        }
        Ok(())
    }

    /// Consume the logger and return the underlying sink.
    pub fn into_sink(self) -> S {
        self.sink
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    /// Simple in-memory log sink that records every line and flush.
    #[derive(Clone, Default)]
    struct MemSink {
        lines: Rc<RefCell<Vec<String>>>,
        flushes: Rc<RefCell<usize>>,
    }

    impl LogSink for MemSink {
        fn write_line(&mut self, line: &str) -> HalResult<()> {
            self.lines.borrow_mut().push(line.to_string());
            Ok(())
        }
        fn flush(&mut self) -> HalResult<()> {
            *self.flushes.borrow_mut() += 1;
            Ok(())
        }
    }

    fn sample_time() -> TimeStruct {
        TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 7,
            hour: 16,
            minute: 5,
            second: 9,
            ..Default::default()
        }
    }

    #[test]
    fn no_logging_before_open() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        logger.init();
        // add before open -> nothing written
        logger.add("hello", &sample_time(), 42).unwrap();
        assert!(sink.lines.borrow().is_empty());
    }

    #[test]
    fn open_writes_banner_and_enables_logging() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        logger.init();
        logger.open().unwrap();
        assert_eq!(sink.lines.borrow()[0], "Start logging ...");
    }

    #[test]
    fn add_writes_timestamped_message() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        logger.init();
        logger.open().unwrap();
        logger.add("event happened", &sample_time(), 12345).unwrap();
        let lines = sink.lines.borrow();
        let last = lines.last().unwrap();
        assert!(last.contains("2024"));
        assert!(last.contains("#12345 :: event happened"), "got: {last}");
    }

    #[test]
    fn save_flushes() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        logger.init();
        logger.open().unwrap();
        logger.save("x", &sample_time(), 1).unwrap();
        assert_eq!(*sink.flushes.borrow(), 1);
    }

    #[test]
    fn close_writes_banner_and_flushes() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        logger.init();
        logger.open().unwrap();
        logger.close().unwrap();
        assert_eq!(sink.lines.borrow().last().unwrap(), "Closing Log file.");
        assert_eq!(*sink.flushes.borrow(), 1);
    }

    #[test]
    fn disabled_logger_open_is_noop() {
        let sink = MemSink::default();
        let mut logger = Logger::new(sink.clone());
        // init() not called -> disabled
        logger.open().unwrap();
        assert!(sink.lines.borrow().is_empty());
    }
}
