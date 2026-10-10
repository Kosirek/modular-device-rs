//! Electronic compass driver, ported from `kompas.c` / `kompas.h`.
//!
//! The C code reads 6 bytes over I2C and converts three big-endian 16-bit words
//! into heading (`kurs`), pitch (`przeglebienie`) and roll (`przechyl`) in
//! tenths of a degree. Heading is unsigned; pitch/roll are signed. That decode
//! is the testable core; the I2C transport is abstracted behind [`I2cBus`].

use crate::hal::{CompassConfig, HalError, HalResult, I2c};

/// 7-bit I2C address (the C code used 0x32/0x33 as the 8-bit write/read forms).
pub const COMPASS_ADDR: u8 = 0x32 >> 1;
/// "Get data" command register.
pub const GET_DATA: u8 = 0x31;

/// Decoded orientation, all in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Orientation {
    /// Heading 0..360 (`kurs`).
    pub heading: f32,
    /// Pitch (`przeglebienie`).
    pub pitch: f32,
    /// Roll (`przechyl`).
    pub roll: f32,
}

/// Decode the 6 raw bytes into an [`Orientation`].
///
/// Port of the math in `read_compass_data`:
/// * heading = unsigned(b0,b1) / 10
/// * pitch   = signed 16-bit two's complement(b2,b3) / 10
/// * roll    = signed 16-bit two's complement(b4,b5) / 10
pub fn decode(raw: &[u8; 6]) -> Orientation {
    let heading = u16::from_be_bytes([raw[0], raw[1]]) as f32 / 10.0;
    let pitch = i16::from_be_bytes([raw[2], raw[3]]) as f32 / 10.0;
    let roll = i16::from_be_bytes([raw[4], raw[5]]) as f32 / 10.0;
    Orientation {
        heading,
        pitch,
        roll,
    }
}

/// I2C compass device.
pub struct Compass<I: I2c> {
    i2c: I,
    config: CompassConfig,
}

impl<I: I2c> Compass<I> {
    pub fn new(i2c: I) -> Self {
        Self::with_config(i2c, CompassConfig::default())
    }

    /// Create a compass device with explicit configuration parameters.
    pub fn with_config(i2c: I, config: CompassConfig) -> Self {
        Self { i2c, config }
    }

    /// Access device configuration.
    pub fn config(&self) -> &CompassConfig {
        &self.config
    }

    /// Trigger a measurement and read the 6 result bytes, then decode.
    ///
    /// Port of `read_compass_data` (write the GET_DATA command, then read 6
    /// bytes).
    pub fn read(&mut self) -> HalResult<Orientation> {
        let addr = self.config.i2c_addr;
        self.i2c
            .write(addr, &[GET_DATA])
            .map_err(|_| HalError::Bus)?;
        let mut raw = [0u8; 6];
        self.i2c.read(addr, &mut raw).map_err(|_| HalError::Bus)?;
        Ok(decode(&raw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as Tx};
    use rstest::rstest;

    #[rstest]
    // heading 123.4°, pitch +5.0°, roll -2.7° (0xFFE5 is -27 in two's complement -> -2.7°)
    #[case([0x04, 0xD2, 0x00, 0x32, 0xFF, 0xE5], 123.4, 5.0, -2.7)]
    // zeroes
    #[case([0, 0, 0, 0, 0, 0], 0.0, 0.0, 0.0)]
    // edge cases: -0.1° (0xFFFF = -1), +3276.7° (0x7FFF = 32767), -3276.8° (0x8000 = -32768)
    #[case([0x00, 0x01, 0xFF, 0xFF, 0x7F, 0xFF], 0.1, -0.1, 3276.7)]
    #[case([0x00, 0x00, 0x80, 0x00, 0x80, 0x00], 0.0, -3276.8, -3276.8)]
    fn decode_cases(
        #[case] raw: [u8; 6],
        #[case] heading: f32,
        #[case] pitch: f32,
        #[case] roll: f32,
    ) {
        let o = decode(&raw);
        assert!((o.heading - heading).abs() < 0.05, "heading {}", o.heading);
        assert!((o.pitch - pitch).abs() < 0.05, "pitch {}", o.pitch);
        assert!((o.roll - roll).abs() < 0.05, "roll {}", o.roll);
    }

    #[test]
    fn read_triggers_command_then_reads_six_bytes() {
        let expectations = [
            Tx::write(COMPASS_ADDR, vec![GET_DATA]),
            Tx::read(COMPASS_ADDR, vec![0x04, 0xD2, 0x00, 0x32, 0xFF, 0xE5]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut compass = Compass::new(i2c.clone());
        let o = compass.read().unwrap();
        assert!((o.heading - 123.4).abs() < 0.05);
        i2c.done();
    }

    #[test]
    fn read_uses_custom_address_from_config() {
        let custom_addr = 0x22;
        let expectations = [
            Tx::write(custom_addr, vec![GET_DATA]),
            Tx::read(custom_addr, vec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let config = CompassConfig {
            i2c_addr: custom_addr,
            ..CompassConfig::default()
        };
        let mut compass = Compass::with_config(i2c.clone(), config);
        let o = compass.read().unwrap();
        assert_eq!(o.heading, 0.0);
        i2c.done();
    }
}
