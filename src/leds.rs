//! LED status indicators and signaling patterns, ported from `dev_conf.h` and `mainPrg.c`.
//!
//! The LPC1768 board has 4 user LEDs:
//! * `LED1`: P3.25
//! * `LED2`: P3.26 (heartbeat / telemetry activity indicator)
//! * `LED3`: P4.28 (signalization / error blink indicator)
//! * `LED4`: P4.29
//!
//! In `mainPrg.c`, `led_sig(flash_count, fast_blink)` emits blink sequences on `LED3`
//! to communicate startup state or fatal error codes to the user without a screen.

use crate::hal::{DelayNs, HalResult};

/// Frequency for fast signaling blinks (200 ms) matching `LED_SIGNALIZATION_FAST`.
pub const LED_SIGNAL_FAST_MS: u32 = 200;

/// Frequency for slow signaling blinks (500 ms) matching `LED_SIGNALIZATION_SLOW`.
pub const LED_SIGNAL_SLOW_MS: u32 = 500;

/// The 4 on-board status LEDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Led {
    /// LED 1 (P3.25).
    Led1 = 1,
    /// LED 2 (P3.26) - heartbeat indicator.
    Led2 = 2,
    /// LED 3 (P4.28) - signalization indicator.
    Led3 = 3,
    /// LED 4 (P4.29).
    Led4 = 4,
}

/// Hardware trait for controlling the 4 on-board status LEDs.
pub trait LedDriver {
    /// Set the logical state of an LED (true = on, false = off).
    fn set_led(&mut self, led: Led, on: bool) -> HalResult<()>;
    /// Check the current logical state of an LED.
    fn is_led_on(&self, led: Led) -> bool;
}

/// In-memory mock implementation of [`LedDriver`] for unit tests and simulation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct InMemoryLedDriver {
    pub led1: bool,
    pub led2: bool,
    pub led3: bool,
    pub led4: bool,
}

impl LedDriver for InMemoryLedDriver {
    fn set_led(&mut self, led: Led, on: bool) -> HalResult<()> {
        match led {
            Led::Led1 => self.led1 = on,
            Led::Led2 => self.led2 = on,
            Led::Led3 => self.led3 = on,
            Led::Led4 => self.led4 = on,
        }
        Ok(())
    }

    fn is_led_on(&self, led: Led) -> bool {
        match led {
            Led::Led1 => self.led1,
            Led::Led2 => self.led2,
            Led::Led3 => self.led3,
            Led::Led4 => self.led4,
        }
    }
}

/// High-level manager coordinating LEDs and blink sequences.
pub struct LedManager<D: LedDriver> {
    driver: D,
}

impl<D: LedDriver> LedManager<D> {
    /// Create a new LED manager wrapping a hardware driver.
    pub fn new(driver: D) -> Self {
        Self { driver }
    }

    /// Access the underlying driver.
    pub fn driver(&self) -> &D {
        &self.driver
    }

    /// Mutably access the underlying driver.
    pub fn driver_mut(&mut self) -> &mut D {
        &mut self.driver
    }

    /// Turn on a specific LED.
    pub fn on(&mut self, led: Led) -> HalResult<()> {
        self.driver.set_led(led, true)
    }

    /// Turn off a specific LED.
    pub fn off(&mut self, led: Led) -> HalResult<()> {
        self.driver.set_led(led, false)
    }

    /// Set state for a specific LED.
    pub fn set(&mut self, led: Led, on: bool) -> HalResult<()> {
        self.driver.set_led(led, on)
    }

    /// Check if a specific LED is illuminated.
    pub fn is_on(&self, led: Led) -> bool {
        self.driver.is_led_on(led)
    }

    /// Toggle the state of a specific LED, returning its new state.
    pub fn toggle(&mut self, led: Led) -> HalResult<bool> {
        let new_state = !self.is_on(led);
        self.driver.set_led(led, new_state)?;
        Ok(new_state)
    }

    /// Turn off all 4 status LEDs.
    pub fn all_off(&mut self) -> HalResult<()> {
        self.off(Led::Led1)?;
        self.off(Led::Led2)?;
        self.off(Led::Led3)?;
        self.off(Led::Led4)?;
        Ok(())
    }

    /// Execute one signaling cycle on LED3 matching `led_sig` in `mainPrg.c`:
    /// * Clears LED4 and LED3.
    /// * Flashes LED3 `flash_count` times with `blink_freq` ms period.
    /// * Followed by an inter-burst pause of `blink_freq * 4` ms.
    pub fn signal_cycle<DEL: DelayNs>(
        &mut self,
        flash_count: u8,
        fast_blink: bool,
        delay: &mut DEL,
    ) -> HalResult<()> {
        let count = flash_count.max(1);
        let freq_ms = if fast_blink {
            LED_SIGNAL_FAST_MS
        } else {
            LED_SIGNAL_SLOW_MS
        };

        self.off(Led::Led4)?;
        self.off(Led::Led3)?;

        for _ in 0..count {
            delay.delay_ms(freq_ms);
            self.on(Led::Led3)?;
            delay.delay_ms(freq_ms);
            self.off(Led::Led3)?;
        }

        delay.delay_ms(freq_ms * 4);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockDelay {
        total_delayed_ms: u32,
    }

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.total_delayed_ms += ns / 1_000_000;
        }

        fn delay_ms(&mut self, ms: u32) {
            self.total_delayed_ms += ms;
        }
    }

    #[test]
    fn on_off_and_toggle_work() {
        let driver = InMemoryLedDriver::default();
        let mut mgr = LedManager::new(driver);

        assert!(!mgr.is_on(Led::Led1));
        mgr.on(Led::Led1).unwrap();
        assert!(mgr.is_on(Led::Led1));

        let new_state = mgr.toggle(Led::Led1).unwrap();
        assert!(!new_state);
        assert!(!mgr.is_on(Led::Led1));

        mgr.on(Led::Led2).unwrap();
        mgr.on(Led::Led3).unwrap();
        mgr.all_off().unwrap();
        assert!(!mgr.is_on(Led::Led2));
        assert!(!mgr.is_on(Led::Led3));
    }

    #[test]
    fn signal_cycle_flashes_correct_number_of_times_and_delays() {
        let driver = InMemoryLedDriver::default();
        let mut mgr = LedManager::new(driver);
        let mut delay = MockDelay {
            total_delayed_ms: 0,
        };

        // 3 fast flashes (200ms on, 200ms off each = 400ms * 3 = 1200ms + pause 800ms = 2000ms)
        mgr.signal_cycle(3, true, &mut delay).unwrap();

        assert_eq!(delay.total_delayed_ms, (200 * 2 * 3) + (200 * 4));
        assert!(!mgr.is_on(Led::Led3));
        assert!(!mgr.is_on(Led::Led4));
    }
}
