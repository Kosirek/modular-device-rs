//! Watchdog timer, ported from `wtd.c`.
//!
//! The C driver computes a timeout reload value from the core clock and only
//! feeds the watchdog when it is enabled. The reload computation is the
//! testable arithmetic; the actual feed sequence (0xAA, 0x55) is exposed so a
//! mock/spy can verify it.

/// The two-byte feed sequence the LPC17xx watchdog expects.
pub const WDT_FEED_1: u8 = 0xAA;
pub const WDT_FEED_2: u8 = 0x55;
/// Watchdog clock prescale used by the C code (`SystemCoreClock / 8`).
pub const WDT_CLOCK_DIV: u32 = 8;

/// Compute the watchdog timer constant (`WDTC`) for a given timeout.
///
/// Port of the `WatchdogConfiguration` expression with correct lower bound:
/// `(SystemCoreClock/8) * (ms/1000)`, clamped to minimum `0xFF`.
pub fn timeout_to_wdtc(core_clock_hz: u32, timeout_ms: u32) -> u32 {
    let ticks = ((core_clock_hz as u64 / WDT_CLOCK_DIV as u64) * timeout_ms as u64 / 1000) as u32;
    ticks.max(0xFF)
}

/// Watchdog state (`gWatchDogOn`) plus a feed counter so callers can prove the
/// feed happened. Generic over a feed sink so tests can record the sequence.
pub struct Watchdog<F: FnMut(u8)> {
    enabled: bool,
    feed: F,
}

impl<F: FnMut(u8)> Watchdog<F> {
    pub fn new(feed: F) -> Self {
        Self {
            enabled: false,
            feed,
        }
    }

    /// Enable / disable the watchdog. While disabled, [`Self::feed`] is a no-op,
    /// matching the C `gWatchDogOn` guard.
    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Create a watchdog with explicit configuration applied immediately.
    pub fn with_config(feed: F, config: &crate::hal::WatchdogConfig) -> (Self, Option<u32>) {
        let mut wdt = Self::new(feed);
        let reload = wdt.configure_with(config);
        (wdt, reload)
    }

    /// Configure the watchdog according to [`crate::hal::WatchdogConfig`].
    pub fn configure_with(&mut self, config: &crate::hal::WatchdogConfig) -> Option<u32> {
        self.set_enabled(config.enabled);
        self.configure(config.core_clock_hz, config.timeout_ms)
    }

    /// Configure the watchdog. Returns the computed `WDTC` value, or `None` if
    /// the watchdog is disabled or the timeout is zero (the C early-out cases).
    pub fn configure(&mut self, core_clock_hz: u32, timeout_ms: u32) -> Option<u32> {
        if !self.enabled || timeout_ms == 0 {
            return None;
        }
        let wdtc = timeout_to_wdtc(core_clock_hz, timeout_ms);
        self.feed_now();
        Some(wdtc)
    }

    /// Feed the watchdog, but only while enabled (`FeedWatchdog`).
    pub fn feed(&mut self) -> bool {
        if !self.enabled {
            return false;
        }
        self.feed_now();
        true
    }

    fn feed_now(&mut self) {
        (self.feed)(WDT_FEED_1);
        (self.feed)(WDT_FEED_2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;
    use std::cell::RefCell;
    use std::rc::Rc;

    #[rstest]
    // 96 MHz core, 1000 ms -> 96e6/8 = 12_000_000
    #[case(96_000_000, 1000, 12_000_000)]
    // 100 ms at 96 MHz -> 1_200_000
    #[case(96_000_000, 100, 1_200_000)]
    // Small timeout clamps to minimum 0xFF
    #[case(1_000, 1, 0xFF)]
    fn wdtc_matches_formula(#[case] clk: u32, #[case] ms: u32, #[case] expected: u32) {
        assert_eq!(timeout_to_wdtc(clk, ms), expected);
    }

    #[test]
    fn disabled_watchdog_does_not_feed() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let l = log.clone();
        let mut wdt = Watchdog::new(move |b| l.borrow_mut().push(b));
        assert!(!wdt.feed());
        assert!(log.borrow().is_empty());
    }

    #[test]
    fn enabled_feed_emits_sequence() {
        let log = Rc::new(RefCell::new(Vec::new()));
        let l = log.clone();
        let mut wdt = Watchdog::new(move |b| l.borrow_mut().push(b));
        wdt.set_enabled(true);
        assert!(wdt.feed());
        assert_eq!(*log.borrow(), vec![WDT_FEED_1, WDT_FEED_2]);
    }

    #[test]
    fn configure_returns_none_when_disabled_or_zero() {
        let mut wdt = Watchdog::new(|_| {});
        assert_eq!(wdt.configure(96_000_000, 1000), None); // disabled
        wdt.set_enabled(true);
        assert_eq!(wdt.configure(96_000_000, 0), None); // zero timeout
        assert!(wdt.configure(96_000_000, 1000).is_some());
    }

    #[test]
    fn configure_with_applies_watchdog_config() {
        let cfg = crate::hal::WatchdogConfig {
            core_clock_hz: 96_000_000,
            timeout_ms: 500,
            enabled: true,
        };
        let (wdt, reload) = Watchdog::with_config(|_| {}, &cfg);
        assert!(wdt.is_enabled());
        assert_eq!(reload, Some(6_000_000));
    }
}
