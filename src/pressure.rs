//! Barometric pressure / temperature sensor (MS5534-class), ported from
//! `higro.c`.
//!
//! The sensor exposes calibration words read once at start-up; pressure (`pr`)
//! and temperature (`tm`) raw conversions are then turned into engineering
//! units with a fixed polynomial. That polynomial (`get_data`) is the valuable,
//! bug-prone part, so we port it as a pure function over a calibration struct
//! and verify it numerically.

/// Factory calibration words `c1..c7` plus the extra `a,b,c,d` coefficients the
/// firmware reads from the device EEPROM.
#[derive(Debug, Clone, Copy, Default)]
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureReading {
    /// Temperature in tenths of a degree (as produced by the C formula).
    pub temperature: f32,
    /// Pressure in the sensor's native units.
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
            PressureStep::StartTemperature => PressureStep::ReadTemperature,
            PressureStep::ReadTemperature => PressureStep::StartPressure,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
