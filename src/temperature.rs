//! Temperature / supply-voltage ADC driver, ported from `termometr.c`.
//!
//! The firmware samples four 12-bit ADC channels and applies a first-order IIR
//! filter (`new = 0.1*sample + 0.9*old`). Channels 0..2 are temperatures, channel
//! 3 is the supply voltage. We keep the filter state as plain fields and convert
//! the raw 12-bit code with the same scale, all host-testable.

use crate::hal::{Adc, TemperatureConfig};

/// Number of temperature channels.
pub const TEMP_CHANNELS: usize = 3;
/// IIR smoothing factor applied to each new sample.
pub const IIR_ALPHA: f32 = 0.1;
/// 12-bit full scale.
pub const ADC_FULL_SCALE: f32 = 4095.0;

/// Convert a raw 12-bit ADC code to engineering units using the C scale
/// (`code * 330 / 4095`), i.e. hundredths of a volt / °C depending on channel.
pub fn raw_to_value(raw_code: u16) -> f32 {
    (raw_code & 0x0FFF) as f32 * 330.0 / ADC_FULL_SCALE
}

/// Convert a raw ADC code using explicit configuration parameters.
pub fn raw_to_value_with_config(raw_code: u16, config: &TemperatureConfig) -> f32 {
    let fs = if config.full_scale == 0 {
        ADC_FULL_SCALE
    } else {
        config.full_scale as f32
    };
    (raw_code & 0x0FFF) as f32 * (config.vref_mv as f32 / 10.0) / fs
}

/// Apply the one-pole IIR filter: `alpha*new + (1.0 - alpha)*old`.
pub fn iir_with_alpha(old: f32, sample: f32, alpha: f32) -> f32 {
    alpha * sample + (1.0 - alpha) * old
}

/// Apply the one-pole IIR filter: `0.1*new + 0.9*old`.
pub fn iir(old: f32, sample: f32) -> f32 {
    iir_with_alpha(old, sample, IIR_ALPHA)
}

/// Holds the filtered readings (the C globals `temp[3]` and `power_volt`).
#[derive(Debug, Clone, Copy)]
pub struct TemperatureSensor {
    pub temp: [f32; TEMP_CHANNELS],
    pub power_volt: f32,
    pub initialized: bool,
    pub config: TemperatureConfig,
}

impl Default for TemperatureSensor {
    fn default() -> Self {
        Self {
            temp: [0.0; TEMP_CHANNELS],
            power_volt: 0.0,
            initialized: false,
            config: TemperatureConfig::default(),
        }
    }
}

impl TemperatureSensor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a temperature sensor with explicit hardware configuration.
    pub fn with_config(config: TemperatureConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    /// Reset the filter state so the next `update` seeds directly from incoming samples.
    pub fn reset(&mut self) {
        self.initialized = false;
        self.temp = [0.0; TEMP_CHANNELS];
        self.power_volt = 0.0;
    }

    /// Sample all configured channels through `adc` and update the filtered state,
    /// mirroring one full pass of `get_temp`.
    ///
    /// On the first sample (or after `reset`), seeds the filter directly from the inputs
    /// to avoid waiting 50+ cycles for convergence from zero.
    pub fn update<A: Adc>(&mut self, adc: &mut A) {
        let alpha = self.config.iir_alpha;
        if !self.initialized {
            for (idx, &ch) in self
                .config
                .temp_channels
                .iter()
                .enumerate()
                .take(TEMP_CHANNELS)
            {
                let code = adc.read_channel(ch);
                self.temp[idx] = raw_to_value_with_config(code, &self.config);
            }
            let pwr_code = adc.read_channel(self.config.power_channel);
            self.power_volt = raw_to_value_with_config(pwr_code, &self.config);
            self.initialized = true;
            return;
        }

        for (idx, &ch) in self
            .config
            .temp_channels
            .iter()
            .enumerate()
            .take(TEMP_CHANNELS)
        {
            let value = raw_to_value_with_config(adc.read_channel(ch), &self.config);
            self.temp[idx] = iir_with_alpha(self.temp[idx], value, alpha);
        }
        let v = raw_to_value_with_config(adc.read_channel(self.config.power_channel), &self.config);
        self.power_volt = iir_with_alpha(self.power_volt, v, alpha);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::MockAdc;
    use rstest::rstest;

    #[rstest]
    #[case(0, 0.0)]
    #[case(4095, 330.0)]
    #[case(2048, 165.04)] // ~ half scale
    fn raw_conversion(#[case] code: u16, #[case] expected: f32) {
        assert!(
            (raw_to_value(code) - expected).abs() < 0.1,
            "{}",
            raw_to_value(code)
        );
    }

    #[test]
    fn raw_conversion_masks_to_12_bits() {
        // Upper bits (e.g. the ADC "done"/overrun flags) must be ignored.
        assert_eq!(raw_to_value(0xF000), 0.0);
        assert!((raw_to_value(0xF000 | 4095) - 330.0).abs() < 0.1);
    }

    #[test]
    fn iir_converges_towards_input() {
        let mut v = 0.0f32;
        for _ in 0..200 {
            v = iir(v, 100.0);
        }
        assert!((v - 100.0).abs() < 0.5);
    }

    #[test]
    fn update_filters_all_channels() {
        let mut adc = MockAdc::new();
        // Every channel returns full-scale -> 330.0 raw value.
        adc.expect_read_channel().returning(|_| 4095);

        let mut sensor = TemperatureSensor::new();
        // First update: seeds directly to 330.0 (no cold start delay)
        sensor.update(&mut adc);
        for t in sensor.temp {
            assert!((t - 330.0).abs() < 0.1);
        }
        assert!((sensor.power_volt - 330.0).abs() < 0.1);

        // Reset clears state
        sensor.reset();
        assert!(!sensor.initialized);
    }

    #[test]
    fn custom_config_reads_specified_channels_and_scales_with_vref() {
        let mut adc = MockAdc::new();
        // Return 2048 (half-scale) for channels 4, 5, 6, and 7
        adc.expect_read_channel()
            .with(mockall::predicate::eq(4))
            .returning(|_| 2048);
        adc.expect_read_channel()
            .with(mockall::predicate::eq(5))
            .returning(|_| 2048);
        adc.expect_read_channel()
            .with(mockall::predicate::eq(6))
            .returning(|_| 2048);
        adc.expect_read_channel()
            .with(mockall::predicate::eq(7))
            .returning(|_| 4095);

        let config = TemperatureConfig {
            temp_channels: [4, 5, 6],
            power_channel: 7,
            vref_mv: 5000, // 5.0V full scale -> 500.0 hundredths of V
            full_scale: 4095,
            iir_alpha: 0.2,
        };
        let mut sensor = TemperatureSensor::with_config(config);
        sensor.update(&mut adc);

        // 2048 / 4095 * 500.0 = 250.0
        for t in sensor.temp {
            assert!((t - 250.0).abs() < 1.0);
        }
        // 4095 / 4095 * 500.0 = 500.0
        assert!((sensor.power_volt - 500.0).abs() < 1.0);
    }
}
