//! Key / switch scanning, ported from `get_key()` in `mainPrg.c`.
//!
//! Five switches spread across three GPIO ports are combined into one bitmask.
//! We abstract each switch behind an [`InputPin`] so the combining logic is
//! testable without hardware.

use crate::hal::{HalError, HalResult, InputPin, KeypadConfig};

// Output bitmask positions (`KEY_SW*_MASK`).
pub const KEY_SW1: u8 = 1;
pub const KEY_SW2: u8 = 2;
pub const KEY_SW3: u8 = 4;
pub const KEY_SW4: u8 = 8;
pub const KEY_SW5: u8 = 16;

/// Build the key bitmask from the five switch input levels.
///
/// A `true` (logic high) on a switch sets its bit, exactly like `get_key()`.
pub fn scan_keys(sw1: bool, sw2: bool, sw3: bool, sw4: bool, sw5: bool) -> u8 {
    let mut key = 0u8;
    if sw1 {
        key |= KEY_SW1;
    }
    if sw2 {
        key |= KEY_SW2;
    }
    if sw3 {
        key |= KEY_SW3;
    }
    if sw4 {
        key |= KEY_SW4;
    }
    if sw5 {
        key |= KEY_SW5;
    }
    key
}

/// Keypad bound to five input pins. Generic over each pin type so switches can
/// reside on different GPIO ports without pin type-erasure.
pub struct Keypad<
    P1: InputPin,
    P2: InputPin = P1,
    P3: InputPin = P1,
    P4: InputPin = P1,
    P5: InputPin = P1,
> {
    pub sw1: P1,
    pub sw2: P2,
    pub sw3: P3,
    pub sw4: P4,
    pub sw5: P5,
    pub config: KeypadConfig,
}

impl<P1: InputPin, P2: InputPin, P3: InputPin, P4: InputPin, P5: InputPin>
    Keypad<P1, P2, P3, P4, P5>
{
    pub fn new(sw1: P1, sw2: P2, sw3: P3, sw4: P4, sw5: P5) -> Self {
        Self::with_config(sw1, sw2, sw3, sw4, sw5, KeypadConfig::default())
    }

    /// Create a keypad with explicit configuration (e.g. active-low switches).
    pub fn with_config(sw1: P1, sw2: P2, sw3: P3, sw4: P4, sw5: P5, config: KeypadConfig) -> Self {
        Self {
            sw1,
            sw2,
            sw3,
            sw4,
            sw5,
            config,
        }
    }

    /// Read all switches and return the combined bitmask (`get_key`),
    /// taking polarity into account from the configuration.
    pub fn read(&mut self) -> HalResult<u8> {
        let active_high = self.config.active_high;
        let s1 = self.sw1.is_high().map_err(|_| HalError::Bus)? == active_high;
        let s2 = self.sw2.is_high().map_err(|_| HalError::Bus)? == active_high;
        let s3 = self.sw3.is_high().map_err(|_| HalError::Bus)? == active_high;
        let s4 = self.sw4.is_high().map_err(|_| HalError::Bus)? == active_high;
        let s5 = self.sw5.is_high().map_err(|_| HalError::Bus)? == active_high;
        Ok(scan_keys(s1, s2, s3, s4, s5))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as Tx};
    use rstest::rstest;

    #[rstest]
    #[case(false, false, false, false, false, 0)]
    #[case(true, false, false, false, false, KEY_SW1)]
    #[case(false, true, false, false, false, KEY_SW2)]
    #[case(false, false, true, false, false, KEY_SW3)]
    #[case(false, false, false, true, false, KEY_SW4)]
    #[case(false, false, false, false, true, KEY_SW5)]
    #[case(true, true, true, true, true, 0b11111)]
    #[case(true, false, true, false, true, KEY_SW1 | KEY_SW3 | KEY_SW5)]
    fn scan_combines_bits(
        #[case] s1: bool,
        #[case] s2: bool,
        #[case] s3: bool,
        #[case] s4: bool,
        #[case] s5: bool,
        #[case] expected: u8,
    ) {
        assert_eq!(scan_keys(s1, s2, s3, s4, s5), expected);
    }

    /// A digital input mock that, when polled once, reports `level`.
    fn pin(level: bool) -> PinMock {
        let state = if level { State::High } else { State::Low };
        PinMock::new(&[Tx::get(state)])
    }

    #[test]
    fn keypad_reads_pins() {
        let pins = [pin(true), pin(false), pin(true), pin(false), pin(false)];
        let mut kp = Keypad::new(
            pins[0].clone(),
            pins[1].clone(),
            pins[2].clone(),
            pins[3].clone(),
            pins[4].clone(),
        );
        assert_eq!(kp.read().unwrap(), KEY_SW1 | KEY_SW3);
        for mut p in pins {
            p.done();
        }
    }

    #[test]
    fn keypad_active_low_config_inverts_polarity() {
        // With active_low: false (active_high = false), pin Low means pressed.
        let pins = [pin(false), pin(true), pin(true), pin(true), pin(false)];
        let config = KeypadConfig {
            active_high: false,
            switch_count: 5,
            debounce_ms: 20,
        };
        let mut kp = Keypad::with_config(
            pins[0].clone(),
            pins[1].clone(),
            pins[2].clone(),
            pins[3].clone(),
            pins[4].clone(),
            config,
        );
        // Pin 0 and Pin 4 are Low -> pressed!
        assert_eq!(kp.read().unwrap(), KEY_SW1 | KEY_SW5);
        for mut p in pins {
            p.done();
        }
    }
}
