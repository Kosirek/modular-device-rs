//! Accelerometer driver, ported from `akcelerometr.c` / `akcelerometr.h`.
//!
//! Two LIS3-style accelerometers share one SPI bus, each with its own
//! chip-select. We model a single device parameterised by its CS pin; the demo
//! creates two instances. The interesting, testable part is the signed-byte
//! conversion in `get_acc_data`.

use crate::hal::{AccelerometerConfig, HalError, HalResult, SpiDevice};

// Register map (subset used by the firmware).
pub const CTRL_REG1: u8 = 0x20;
pub const CTRL_REG2: u8 = 0x21;
pub const STATUS_REG: u8 = 0x27;
pub const OUT_X: u8 = 0x29;
pub const OUT_Y: u8 = 0x2B;
pub const OUT_Z: u8 = 0x2D;
/// OR'd into a register address to request a read.
pub const READ_CONF: u8 = 0x80;

/// One acceleration sample. Mirrors the C `_acc` struct but uses real signed
/// values (the C code stored these in `char`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AccSample {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Convert one raw output byte the way `get_acc_data` does:
/// values >= 128 are treated as negative (two's complement in 8 bits), then a
/// fixed `+128` bias is applied so the result is an unsigned-ish 0..255 range.
pub fn convert_axis(raw: u8) -> i16 {
    let mut v = raw as i16;
    if v >= 128 {
        v -= 256;
    }
    v + 128
}

/// SPI accelerometer driven through an [`SpiDevice`] (the device wrapper
/// handles chip-select; the demo creates one instance per physical sensor).
pub struct Accelerometer<S: SpiDevice> {
    spi: S,
}

impl<S: SpiDevice> Accelerometer<S> {
    pub fn new(spi: S) -> Self {
        Self { spi }
    }

    fn read_register(&mut self, reg: u8) -> HalResult<u8> {
        let mut buf = [READ_CONF | reg, 0xFF];
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(|_| HalError::Bus)?;
        Ok(buf[1])
    }

    fn write_register(&mut self, reg: u8, value: u8) -> HalResult<()> {
        self.spi.write(&[reg, value]).map_err(|_| HalError::Bus)
    }

    /// Configure the device using a specific [`AccelerometerConfig`].
    ///
    /// Sets power-down / active mode, Output Data Rate (ODR), enables all 3 axes,
    /// and configures measurement scale (e.g. ±2g or ±8g).
    pub fn configure_with(&mut self, config: &AccelerometerConfig) -> HalResult<()> {
        let current = self.read_register(CTRL_REG1)?;
        let pwr = if config.power_down { 0 } else { 1 << 6 };
        let odr = if config.odr_hz >= 400 { 1 << 7 } else { 0 };
        let ctrl1 = odr | pwr | 0x07;

        self.write_register(CTRL_REG1, current | (1 << 6))?;
        self.write_register(CTRL_REG1, ctrl1)?;

        let fs = if config.full_scale_g >= 8 { 1 << 5 } else { 0 };
        let ctrl2 = 0x12 | fs;
        self.write_register(CTRL_REG2, ctrl2)
    }

    /// Configure the device: enable power-down bit and set ODR, like
    /// `configure_acc` with default board settings.
    pub fn configure(&mut self) -> HalResult<()> {
        self.configure_with(&AccelerometerConfig {
            odr_hz: 400,
            full_scale_g: 2,
            power_down: false,
            ..AccelerometerConfig::default()
        })
    }

    /// `true` when a fresh sample is available (`is_data_ready`, status bit 4).
    pub fn is_data_ready(&mut self) -> HalResult<bool> {
        Ok((self.read_register(STATUS_REG)? & 0x10) != 0)
    }

    /// Read X/Y/Z and apply the same conversion as `get_acc_data`.
    pub fn read_sample(&mut self) -> HalResult<AccSample> {
        let x = self.read_register(OUT_X)?;
        let y = self.read_register(OUT_Y)?;
        let z = self.read_register(OUT_Z)?;
        Ok(AccSample {
            x: convert_axis(x),
            y: convert_axis(y),
            z: convert_axis(z),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as Tx};
    use rstest::rstest;

    #[rstest]
    #[case(0, 128)] // 0 -> +128 bias
    #[case(1, 129)]
    #[case(127, 255)] // max positive before sign flip
    #[case(128, 0)] // -128 + 128
    #[case(255, 127)] // -1 + 128
    #[case(200, 72)] // (200-256)+128 = 72
    fn axis_conversion(#[case] raw: u8, #[case] expected: i16) {
        assert_eq!(convert_axis(raw), expected);
    }

    /// One register read is a single-transaction `transfer_in_place` of
    /// `[READ_CONF|reg, 0xFF]` whose second byte is the returned data.
    fn read_reg(reg: u8, value: u8) -> [Tx<u8>; 3] {
        [
            Tx::transaction_start(),
            Tx::transfer_in_place(vec![READ_CONF | reg, 0xFF], vec![0x00, value]),
            Tx::transaction_end(),
        ]
    }

    #[test]
    fn read_sample_reads_three_axes() {
        let mut expectations = Vec::new();
        // Each axis returns 128, which converts to 0.
        expectations.extend(read_reg(OUT_X, 128));
        expectations.extend(read_reg(OUT_Y, 128));
        expectations.extend(read_reg(OUT_Z, 128));
        let mut spi = SpiMock::new(&expectations);
        let mut acc = Accelerometer::new(spi.clone());
        let s = acc.read_sample().unwrap();
        assert_eq!(s, AccSample { x: 0, y: 0, z: 0 });
        spi.done();
    }

    #[test]
    fn data_ready_checks_status_bit() {
        let expectations = read_reg(STATUS_REG, 0x10);
        let mut spi = SpiMock::new(&expectations);
        let mut acc = Accelerometer::new(spi.clone());
        assert!(acc.is_data_ready().unwrap());
        spi.done();
    }

    #[test]
    fn configure_with_sets_custom_odr_and_scale() {
        let mut expectations = Vec::new();
        // read CTRL_REG1 -> 0x00
        expectations.extend(read_reg(CTRL_REG1, 0x00));
        // write current | (1 << 6) -> 0x40
        expectations.push(Tx::transaction_start());
        expectations.push(Tx::write_vec(vec![CTRL_REG1, 0x40]));
        expectations.push(Tx::transaction_end());
        // write custom ctrl1: 400Hz (0x80) | active (0x40) | XYZ (0x07) = 0xC7
        expectations.push(Tx::transaction_start());
        expectations.push(Tx::write_vec(vec![CTRL_REG1, 0xC7]));
        expectations.push(Tx::transaction_end());
        // write custom ctrl2: 0x12 | scale 8g (0x20) = 0x32
        expectations.push(Tx::transaction_start());
        expectations.push(Tx::write_vec(vec![CTRL_REG2, 0x32]));
        expectations.push(Tx::transaction_end());

        let mut spi = SpiMock::new(&expectations);
        let mut acc = Accelerometer::new(spi.clone());
        let config = AccelerometerConfig {
            odr_hz: 400,
            full_scale_g: 8,
            power_down: false,
            ..AccelerometerConfig::default()
        };
        acc.configure_with(&config).unwrap();
        spi.done();
    }
}
