//! Barometric pressure / temperature sensor (MS5534-class), ported from
//! `higro.c`.
//!
//! The sensor exposes calibration words read once at start-up; pressure (`pr`)
//! and temperature (`tm`) raw conversions are then turned into engineering
//! units with a fixed polynomial. That polynomial (`get_data`) is the valuable,
//! bug-prone part, so we port it as a pure function over a calibration struct
//! and verify it numerically.

use crate::hal::{HalError, HalResult, I2c};

/// 7-bit I2C address for the MS5534 calibration EEPROM (`0xA1 >> 1`).
pub const MS5534_EEPROM_ADDR: u8 = 0xA1 >> 1;

/// 7-bit I2C address for the MS5534 ADC (`0xEE >> 1`).
pub const MS5534_ADC_ADDR: u8 = 0xEE >> 1;

/// Factory calibration words `c1..c7` plus the extra `a,b,c,d` coefficients the
/// firmware reads from the device EEPROM.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PressureCalibration {
    pub c1: u16,
    pub c2: u16,
    pub c3: u16,
    pub c4: u16,
    pub c5: u16,
    pub c6: u16,
    pub c7: u16,
    pub a: u16,
    pub b: u16,
    pub c: u16,
    pub d: u16,
}

/// Result of a conversion.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PressureReading {
    /// Temperature in tenths of a degree (as produced by the C formula).
    pub temperature: f32,
    /// Pressure in the sensor's native units (tenths of mbar).
    pub pressure: f32,
}

/// Convert raw pressure (`pr`) and temperature (`tm`) words to engineering
/// units, exactly mirroring `get_data()`.
pub fn convert(cal: &PressureCalibration, pr: u16, tm: u16) -> PressureReading {
    let tm_f = tm as f32;
    let pr_f = pr as f32;
    let c5 = cal.c5 as f32;

    // Temperature difference term, with the C branch on (pr >= c5).
    let dt_base = tm_f - c5;
    let scaled = (dt_base / 128.0) * (dt_base / 128.0);
    let dut = if pr >= cal.c5 {
        dt_base - scaled * cal.a as f32 / (1u32 << cal.c) as f32
    } else {
        dt_base - scaled * cal.b as f32 / (1u32 << cal.c) as f32
    };

    let off = (cal.c2 as f32 + (cal.c4 as f32 - 1024.0) * dut / 16384.0) * 4.0;
    let sens = cal.c1 as f32 + cal.c3 as f32 * dut / 1024.0;
    let x = sens * (pr_f - 7168.0) / 16384.0 - off;

    let pressure = x * 10.0 / 32.0 + cal.c7 as f32;
    let temperature = 250.0 + dut * cal.c6 as f32 / 65536.0 - dut / (1u32 << cal.d) as f32;

    PressureReading {
        temperature,
        pressure,
    }
}

/// Read calibration words from MS5534 EEPROM via I2C matching `higro.c`.
pub fn read_calibration<I: I2c>(i2c: &mut I) -> HalResult<PressureCalibration> {
    let mut words = [0u8; 14];
    i2c.read(MS5534_EEPROM_ADDR, &mut words)
        .map_err(|_| HalError::Bus)?;

    let c1 = u16::from_be_bytes([words[0], words[1]]);
    let c2 = u16::from_be_bytes([words[2], words[3]]);
    let c3 = u16::from_be_bytes([words[4], words[5]]);
    let c4 = u16::from_be_bytes([words[6], words[7]]);
    let c5 = u16::from_be_bytes([words[8], words[9]]);
    let c6 = u16::from_be_bytes([words[10], words[11]]);
    let c7 = u16::from_be_bytes([words[12], words[13]]);

    let mut read_coeff = |reg: u8| -> HalResult<u16> {
        let mut buf = [0u8; 1];
        i2c.write_read(MS5534_EEPROM_ADDR, &[reg], &mut buf)
            .map_err(|_| HalError::Bus)?;
        Ok(buf[0] as u16)
    };

    let a = read_coeff(30)?;
    let b = read_coeff(31)?;
    let c = read_coeff(32)?;
    let d = read_coeff(33)?;

    Ok(PressureCalibration {
        c1,
        c2,
        c3,
        c4,
        c5,
        c6,
        c7,
        a,
        b,
        c,
        d,
    })
}

/// Hardware driver for MS5534 pressure and temperature sensor communicating via I2C.
pub struct PressureSensor<I: I2c> {
    i2c: I,
    cal: PressureCalibration,
}

impl<I: I2c> PressureSensor<I> {
    /// Initialize sensor by reading factory calibration over I2C.
    pub fn new(mut i2c: I) -> HalResult<Self> {
        let cal = read_calibration(&mut i2c)?;
        Ok(Self { i2c, cal })
    }

    /// Construct sensor with known/mocked calibration coefficients.
    pub fn with_calibration(i2c: I, cal: PressureCalibration) -> Self {
        Self { i2c, cal }
    }

    /// Access calibration coefficients.
    pub fn calibration(&self) -> &PressureCalibration {
        &self.cal
    }

    /// Start pressure ADC conversion and read the 16-bit raw ADC code.
    pub fn read_raw_pressure(&mut self) -> HalResult<u16> {
        // Start pressure conversion: write [0xFF, 0xF0] to ADC
        self.i2c
            .write(MS5534_ADC_ADDR, &[0xFF, 0xF0])
            .map_err(|_| HalError::Bus)?;

        // Read ADC: write [0xFD], then read 2 bytes big-endian
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MS5534_ADC_ADDR, &[0xFD], &mut buf)
            .map_err(|_| HalError::Bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Start temperature ADC conversion and read the 16-bit raw ADC code.
    pub fn read_raw_temperature(&mut self) -> HalResult<u16> {
        // Start temp conversion: write [0xFF, 0xD0] to ADC
        self.i2c
            .write(MS5534_ADC_ADDR, &[0xFF, 0xD0])
            .map_err(|_| HalError::Bus)?;

        // Read ADC: write [0xFD], then read 2 bytes big-endian
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(MS5534_ADC_ADDR, &[0xFD], &mut buf)
            .map_err(|_| HalError::Bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Perform a full measurement cycle: read raw pressure and temperature, then convert.
    pub fn read(&mut self) -> HalResult<PressureReading> {
        let pr = self.read_raw_pressure()?;
        let tm = self.read_raw_temperature()?;
        Ok(convert(&self.cal, pr, tm))
    }
}

/// Sequencer matching the C `TIMER1_IRQHandler` state machine that alternates
/// between starting/reading pressure and temperature conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PressureStep {
    StartPressure,
    ReadPressure,
    StartTemperature,
    ReadTemperature,
}

impl PressureStep {
    /// Advance to the next step in the round-robin cycle.
    pub fn next(self) -> Self {
        match self {
            PressureStep::StartPressure => PressureStep::ReadPressure,
            PressureStep::ReadPressure => PressureStep::StartTemperature,
            PressureStep::ReadTemperature => PressureStep::StartPressure,
            PressureStep::StartTemperature => PressureStep::ReadTemperature,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as Tx};

    /// Representative MS5534 calibration set.
    fn sample_cal() -> PressureCalibration {
        PressureCalibration {
            c1: 24000,
            c2: 4000,
            c3: 350,
            c4: 400,
            c5: 7000,
            c6: 4500,
            c7: 2500,
            a: 100,
            b: 110,
            c: 8,
            d: 8,
        }
    }

    #[test]
    fn convert_is_deterministic_and_finite() {
        let cal = sample_cal();
        let r = convert(&cal, 9000, 7200);
        assert!(r.pressure.is_finite());
        assert!(r.temperature.is_finite());
    }

    #[test]
    fn higher_raw_pressure_increases_output_pressure() {
        let cal = sample_cal();
        let low = convert(&cal, 8000, 7000).pressure;
        let high = convert(&cal, 12000, 7000).pressure;
        assert!(high > low, "expected {high} > {low}");
    }

    #[test]
    fn branch_selects_a_or_b_coefficient() {
        // When pr < c5 the 'b' coefficient is used, when pr >= c5 the 'a' one.
        let mut cal = sample_cal();
        cal.a = 0;
        cal.b = 1000; // big difference so the branch is observable
        let with_a = convert(&cal, cal.c5 + 100, 9000); // pr >= c5 -> uses a (=0)
        let with_b = convert(&cal, cal.c5 - 100, 9000); // pr < c5 -> uses b
        assert_ne!(with_a.temperature, with_b.temperature);
    }

    #[test]
    fn step_cycle_round_robins() {
        let mut s = PressureStep::StartPressure;
        s = s.next();
        assert_eq!(s, PressureStep::ReadPressure);
        s = s.next();
        assert_eq!(s, PressureStep::StartTemperature);
        s = s.next();
        assert_eq!(s, PressureStep::ReadTemperature);
        s = s.next();
        assert_eq!(s, PressureStep::StartPressure);
    }

    #[test]
    fn pressure_sensor_read_cal_and_read_data() {
        let cal_bytes: [u8; 14] = [
            (24000u16 >> 8) as u8,
            24000u16 as u8, // c1
            (4000u16 >> 8) as u8,
            4000u16 as u8, // c2
            (350u16 >> 8) as u8,
            350u16 as u8, // c3
            (400u16 >> 8) as u8,
            400u16 as u8, // c4
            (7000u16 >> 8) as u8,
            7000u16 as u8, // c5
            (4500u16 >> 8) as u8,
            4500u16 as u8, // c6
            (2500u16 >> 8) as u8,
            2500u16 as u8, // c7
        ];

        let expectations = [
            Tx::read(MS5534_EEPROM_ADDR, cal_bytes.to_vec()),
            Tx::write_read(MS5534_EEPROM_ADDR, vec![30], vec![100]), // a
            Tx::write_read(MS5534_EEPROM_ADDR, vec![31], vec![110]), // b
            Tx::write_read(MS5534_EEPROM_ADDR, vec![32], vec![8]),   // c
            Tx::write_read(MS5534_EEPROM_ADDR, vec![33], vec![8]),   // d
            // Raw pressure read
            Tx::write(MS5534_ADC_ADDR, vec![0xFF, 0xF0]),
            Tx::write_read(MS5534_ADC_ADDR, vec![0xFD], vec![0x23, 0x28]), // 9000
            // Raw temp read
            Tx::write(MS5534_ADC_ADDR, vec![0xFF, 0xD0]),
            Tx::write_read(MS5534_ADC_ADDR, vec![0xFD], vec![0x1C, 0x20]), // 7200
        ];

        let mut i2c = I2cMock::new(&expectations);
        let mut sensor = PressureSensor::new(i2c.clone()).unwrap();

        assert_eq!(sensor.calibration().c1, 24000);
        assert_eq!(sensor.calibration().a, 100);

        let reading = sensor.read().unwrap();
        assert!(reading.pressure.is_finite());
        assert!(reading.temperature.is_finite());

        i2c.done();
    }
}
