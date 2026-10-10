//! System timer / delay logic, ported from `timer0.c`.
//!
//! On the LPC17xx a hardware match interrupt increments a 64-bit tick counter
//! at a fixed resolution (default 1 µs). The interesting, bug-prone part is the
//! *delay tick rounding*, so we model the counter as a plain struct and test the
//! rounding maths directly — no peripheral needed.

use embedded_hal::delay::DelayNs;

/// Monotonic tick counter with a configurable microsecond resolution.
#[derive(Debug, Clone, Copy)]
pub struct SystemTimer {
    ticks: u64,
    resolution_us: u64,
}

impl SystemTimer {
    /// Create a timer. `resolution_us == 0` is coerced to 1 µs, exactly like
    /// `timer_init()`.
    pub fn new(resolution_us: u64) -> Self {
        Self {
            ticks: 0,
            resolution_us: resolution_us.max(1),
        }
    }

    /// Create a timer using explicit [`crate::hal::TimerConfig`].
    pub fn with_config(config: &crate::hal::TimerConfig) -> Self {
        Self::new(config.resolution_us)
    }

    /// Advance by one tick (called from the match-interrupt on hardware).
    pub fn tick(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
    }

    /// Current tick count (`get_sys_time()`).
    pub fn now(&self) -> u64 {
        self.ticks
    }

    /// Microsecond resolution per tick.
    pub fn resolution_us(&self) -> u64 {
        self.resolution_us
    }

    /// Number of ticks to wait for a `us` microsecond delay.
    ///
    /// Port of the rounding in `delay_us()`: always round **up** so we never
    /// wait less than requested, and never return 0.
    pub fn ticks_for_delay(&self, us: u64) -> u64 {
        if us == 0 {
            return 1;
        }
        (us.saturating_add(self.resolution_us - 1) / self.resolution_us).max(1)
    }

    /// Busy-wait helper for tests / simulation: advances the counter to satisfy
    /// `delay_us(us)` and returns the absolute target tick.
    pub fn simulate_delay_us(&mut self, us: u64) -> u64 {
        let stop = self.ticks + self.ticks_for_delay(us);
        self.ticks = stop;
        stop
    }
}

/// Adapter that implements [`DelayNs`] using a function/closure that queries the current tick.
pub struct TimerDelay<T> {
    now_fn: T,
    resolution_us: u64,
}

impl<T: FnMut() -> u64> TimerDelay<T> {
    pub fn new(resolution_us: u64, now_fn: T) -> Self {
        Self {
            now_fn,
            resolution_us: resolution_us.max(1),
        }
    }

    /// Create a delay adapter using explicit [`crate::hal::TimerConfig`].
    pub fn with_config(config: &crate::hal::TimerConfig, now_fn: T) -> Self {
        Self::new(config.resolution_us, now_fn)
    }
}

impl<T: FnMut() -> u64> DelayNs for TimerDelay<T> {
    fn delay_ns(&mut self, ns: u32) {
        let us = (ns as u64).div_ceil(1000);
        let ticks = if us == 0 {
            1
        } else {
            us.div_ceil(self.resolution_us)
        };
        let start = (self.now_fn)();
        while (self.now_fn)().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    fn delay_us(&mut self, us: u32) {
        let ticks = if us == 0 {
            1
        } else {
            (us as u64).div_ceil(self.resolution_us)
        };
        let start = (self.now_fn)();
        while (self.now_fn)().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[test]
    fn zero_resolution_becomes_one() {
        let t = SystemTimer::new(0);
        assert_eq!(t.resolution_us(), 1);
    }

    #[test]
    fn tick_increments_now() {
        let mut t = SystemTimer::new(1);
        assert_eq!(t.now(), 0);
        t.tick();
        t.tick();
        assert_eq!(t.now(), 2);
    }

    #[rstest]
    // resolution 1us: exact multiples, plus the "never zero" rule.
    #[case(1, 0, 1)]
    #[case(1, 1, 1)]
    #[case(1, 1000, 1000)]
    // resolution 10us: rounding up on remainder.
    #[case(10, 0, 1)]
    #[case(10, 5, 1)]
    #[case(10, 10, 1)]
    #[case(10, 11, 2)]
    #[case(10, 25, 3)]
    fn delay_rounds_up(#[case] res: u64, #[case] us: u64, #[case] expected: u64) {
        let t = SystemTimer::new(res);
        assert_eq!(t.ticks_for_delay(us), expected);
    }

    #[test]
    fn simulate_delay_advances_counter() {
        let mut t = SystemTimer::new(100);
        let target = t.simulate_delay_us(250); // -> 3 ticks
        assert_eq!(target, 3);
        assert_eq!(t.now(), 3);
    }

    #[test]
    fn timer_delay_busy_waits() {
        let mut ticks = 0u64;
        let mut delay = TimerDelay::new(10, || {
            ticks += 1;
            ticks
        });
        delay.delay_us(25); // needs (25 + 9) / 10 = 3 ticks
        assert!(ticks >= 3);
    }
}
