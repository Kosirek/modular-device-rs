//! Hardware Abstraction Layer.
//!
//! Drivers are written against the standard [`embedded-hal`](embedded_hal) 1.0
//! traits instead of touching LPC17xx registers directly. On the real target
//! you provide an `embedded-hal` implementation for the MCU peripherals (any
//! LPC HAL crate); in unit tests we substitute
//! [`embedded-hal-mock`](https://docs.rs/embedded-hal-mock) fakes. Storage uses
//! [`embedded-sdmmc`], whose own `BlockDevice`/`TimeSource` traits live in that
//! crate.
//!
//! A couple of capabilities have no `embedded-hal` equivalent — a one-shot ADC
//! read and an append-only log sink — so those stay as small local traits with
//! `#[automock]` for testing.
//!
//! This is the *Dependency Inversion* seam that lets the whole application be
//! tested on a PC.

// Re-export the embedded-hal traits the drivers use, so the rest of the crate
// refers to them through `crate::hal::*` from a single import site.
pub use embedded_hal::delay::DelayNs;
pub use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal::i2c::{I2c, SevenBitAddress};
pub use embedded_hal::spi::{Operation as SpiOperation, SpiDevice};

/// Result alias used by the local traits and by drivers that collapse the
/// (generic) `embedded-hal` bus error into one small concrete type.
pub type HalResult<T> = Result<T, HalError>;

/// Minimal error type for peripheral operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HalError {
    /// A peripheral did not respond / NACKed.
    Nack,
    /// Operation timed out.
    Timeout,
    /// Generic bus fault.
    Bus,
}

/// Single-channel ADC read (temperature / supply voltage sensing).
///
/// `embedded-hal` 1.0 has no blocking ADC trait, so this stays local.
#[cfg_attr(test, mockall::automock)]
pub trait Adc {
    /// Start a conversion on `channel` and return the raw 12-bit value.
    fn read_channel(&mut self, channel: u8) -> u16;
}

/// Append-only log sink (a FatFs file in the firmware, an in-memory buffer or
/// SD-card file in tests). No `embedded-hal` equivalent exists.
#[cfg_attr(test, mockall::automock)]
pub trait LogSink {
    /// Append a line of text to the log.
    fn write_line(&mut self, line: &str) -> HalResult<()>;
    /// Flush buffered data to the backing store.
    fn flush(&mut self) -> HalResult<()>;
}

// ---------------------------------------------------------------------------
// Traits without an embedded-hal equivalent — one per subsystem that could not
// be covered by the re-exports above.
// ---------------------------------------------------------------------------

/// Byte-by-byte serial / UART source for the GPS receiver.
///
/// The GPS module feeds raw bytes into [`crate::gps::GpsReceiver`]. On
/// hardware this is backed by the LPC17xx UART0 FIFO; in tests you can mock it
/// to replay arbitrary byte sequences.
///
/// Returns `None` when the FIFO is empty (non-blocking read).
#[cfg_attr(test, mockall::automock)]
pub trait UartRead {
    /// Read one byte from the receive FIFO, or `None` if no data is available.
    fn read_byte(&mut self) -> Option<u8>;
}

/// Monotonic µs tick counter used by the system timer / delay logic.
///
/// [`crate::timer::TimerDelay`] currently accepts any `FnMut() -> u64` closure.
/// This trait is the named equivalent that firmware code and tests can implement
/// explicitly instead of relying on a raw closure.
#[cfg_attr(test, mockall::automock)]
pub trait MonotonicClock {
    /// Return the current tick count in microseconds since boot (wraps at `u64::MAX`).
    fn now_us(&mut self) -> u64;
}

/// Access to the LPC17xx Real-Time Clock consolidated-time registers.
///
/// The three hardware registers (`CTIME0`, `CTIME1`, `CTIME2`) are decoded by
/// [`crate::rtc::TimeStruct::from_ctime`]. Abstracting the register read here
/// keeps `rtc.rs` a pure decoder and lets you mock the hardware in tests.
#[cfg_attr(test, mockall::automock)]
pub trait RtcRegisters {
    /// Read `(CTIME0, CTIME1, CTIME2)` as a tuple of raw 32-bit words.
    fn read_ctime(&mut self) -> (u32, u32, u32);
}

/// Hardware watchdog feeder.
///
/// The LPC17xx watchdog requires writing `0xAA` then `0x55` to the `WDFEED`
/// register to restart the timer. [`crate::watchdog::Watchdog`] currently
/// accepts any `FnMut(u8)` closure; this trait is the named equivalent for code
/// that prefers explicit trait bounds over anonymous closures.
#[cfg_attr(test, mockall::automock)]
pub trait WatchdogFeed {
    /// Write one byte of the feed sequence to the watchdog peripheral.
    ///
    /// Must be called first with [`crate::watchdog::WDT_FEED_1`] (`0xAA`) and
    /// immediately after with [`crate::watchdog::WDT_FEED_2`] (`0x55`).
    fn write_byte(&mut self, byte: u8);
}

/// Physical Ethernet MAC frame transceiver.
///
/// Backed on hardware by the LPC1768 EMAC peripheral (DMA descriptor rings).
#[cfg_attr(test, mockall::automock)]
pub trait EthernetDevice {
    /// Transmit a raw Ethernet frame onto the wire.
    fn transmit(&mut self, packet: &[u8]) -> HalResult<()>;
    /// Receive a raw Ethernet frame into `buffer`, returning the number of bytes read.
    /// Returns `Ok(None)` if no frame is currently available in the receive ring.
    fn receive(&mut self, buffer: &mut [u8]) -> HalResult<Option<usize>>;
}

/// Hardware interface for reading the MS5534 barometric pressure sensor.
///
/// On hardware, this handles the 3-wire synchronous protocol to read factory calibration
/// words and start/read 16-bit ADC conversions for pressure and temperature.
#[cfg_attr(test, mockall::automock)]
pub trait PressureSensorDevice {
    /// Read factory calibration parameters from the sensor.
    fn read_calibration(&mut self) -> HalResult<crate::pressure::PressureCalibration>;
    /// Start a conversion and read the raw 16-bit ADC conversion result.
    fn read_raw_conversion(&mut self, step: crate::pressure::PressureStep) -> HalResult<u16>;
}

// ---------------------------------------------------------------------------
// Bus Configuration Types
// ---------------------------------------------------------------------------

/// SPI Clock Polarity and Phase mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpiMode {
    /// CPOL = 0, CPHA = 0
    #[default]
    Mode0,
    /// CPOL = 0, CPHA = 1
    Mode1,
    /// CPOL = 1, CPHA = 0
    Mode2,
    /// CPOL = 1, CPHA = 1
    Mode3,
}

/// Bit transmission order for serial buses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BitOrder {
    /// Most Significant Bit first (standard).
    #[default]
    MsbFirst,
    /// Least Significant Bit first.
    LsbFirst,
}

/// Generic SPI bus configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiConfig {
    /// Bus clock frequency in Hertz (e.g. 5_000_000 for 5 MHz).
    pub frequency_hz: u32,
    /// Clock polarity and phase.
    pub mode: SpiMode,
    /// Bit transmission order.
    pub bit_order: BitOrder,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 1_000_000,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
        }
    }
}

/// Standard I2C bus operating speeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum I2cSpeed {
    /// Standard mode (up to 100 kHz).
    #[default]
    Standard100kHz,
    /// Fast mode (up to 400 kHz).
    Fast400kHz,
    /// Fast mode plus (up to 1 MHz).
    FastPlus1MHz,
}

impl I2cSpeed {
    /// Operating frequency in Hertz.
    pub const fn hz(&self) -> u32 {
        match self {
            Self::Standard100kHz => 100_000,
            Self::Fast400kHz => 400_000,
            Self::FastPlus1MHz => 1_000_000,
        }
    }
}

/// Generic I2C bus configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct I2cConfig {
    /// Bus speed setting.
    pub speed: I2cSpeed,
}

/// UART character word length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    #[default]
    Eight,
}

/// UART parity generation and check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Parity {
    #[default]
    None,
    Odd,
    Even,
}

/// UART stop bit count.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StopBits {
    #[default]
    One,
    Two,
}

/// Generic UART / serial port configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartConfig {
    /// Baud rate in bits per second (e.g. 9600, 115200).
    pub baud_rate: u32,
    /// Character length.
    pub data_bits: DataBits,
    /// Parity mode.
    pub parity: Parity,
    /// Number of stop bits.
    pub stop_bits: StopBits,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: 9600,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware Module Configurations
// ---------------------------------------------------------------------------

/// Configuration for the Watchdog module (LPC17xx WDT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogConfig {
    /// Core clock frequency in Hz used to calculate the prescaler (`SystemCoreClock`).
    pub core_clock_hz: u32,
    /// Timeout duration in milliseconds.
    pub timeout_ms: u32,
    /// Whether the watchdog is enabled at startup.
    pub enabled: bool,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            core_clock_hz: 96_000_000,
            timeout_ms: 1000,
            enabled: true,
        }
    }
}

impl WatchdogConfig {
    /// Calculate the LPC17xx `WDTC` reload value according to this configuration.
    pub fn timeout_to_wdtc(&self) -> u32 {
        crate::watchdog::timeout_to_wdtc(self.core_clock_hz, self.timeout_ms)
    }
}

/// Configuration for the System Timer (Timer0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerConfig {
    /// Core / peripheral clock frequency in Hz.
    pub core_clock_hz: u32,
    /// Microsecond resolution per timer tick.
    pub resolution_us: u64,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            core_clock_hz: 96_000_000,
            resolution_us: 1,
        }
    }
}

impl TimerConfig {
    /// Number of ticks required to achieve `us` microseconds delay.
    pub fn ticks_for_us(&self, us: u64) -> u64 {
        if us == 0 {
            1
        } else {
            (us.saturating_add(self.resolution_us - 1) / self.resolution_us).max(1)
        }
    }
}

/// Real-Time Clock clock source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RtcClockSource {
    /// Dedicated 32.768 kHz quartz crystal on RTCX1/RTCX2 pins.
    #[default]
    External32kHz,
    /// Peripheral clock prescaled from core clock.
    InternalPclk,
}

/// Configuration for the LPC17xx Real-Time Clock (RTC).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RtcConfig {
    /// Oscillator / clock input source.
    pub clock_source: RtcClockSource,
    /// Periodic calibration adjustment value (CALVAL register).
    pub calibration_value: u16,
    /// Calibration direction (true = forward/add, false = backward/remove ticks).
    pub calibration_forward: bool,
}

/// Configuration for the GPS receiver UART interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsConfig {
    /// UART communication parameters.
    pub uart: UartConfig,
    /// Receiver ring buffer capacity in bytes.
    pub buffer_capacity: usize,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            uart: UartConfig {
                baud_rate: 9600,
                data_bits: DataBits::Eight,
                parity: Parity::None,
                stop_bits: StopBits::One,
            },
            buffer_capacity: 128,
        }
    }
}

/// Configuration for a 3-axis LIS3 SPI accelerometer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccelerometerConfig {
    /// SPI bus communication parameters.
    pub spi: SpiConfig,
    /// Output data rate in Hz (ODR).
    pub odr_hz: u16,
    /// Full-scale acceleration range in ±g (e.g. 2g).
    pub full_scale_g: u8,
    /// Start device in low-power / power-down state.
    pub power_down: bool,
    /// Chip select identifier (e.g. 0 for sensor 1, 1 for sensor 2).
    pub cs_id: u8,
}

impl Default for AccelerometerConfig {
    fn default() -> Self {
        Self {
            spi: SpiConfig {
                frequency_hz: 5_000_000,
                mode: SpiMode::Mode3,
                bit_order: BitOrder::MsbFirst,
            },
            odr_hz: 100,
            full_scale_g: 2,
            power_down: false,
            cs_id: 0,
        }
    }
}

/// Configuration for the 25LC1024 SPI EEPROM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EepromConfig {
    /// SPI bus communication parameters.
    pub spi: SpiConfig,
    /// Page size in bytes for write operations.
    pub page_size: u32,
    /// Sector erase size in bytes.
    pub sector_size: u32,
    /// Total capacity in bytes.
    pub chip_size: u32,
    /// Hardware write-protect pin active.
    pub write_protect: bool,
}

impl Default for EepromConfig {
    fn default() -> Self {
        Self {
            spi: SpiConfig {
                frequency_hz: 20_000_000,
                mode: SpiMode::Mode0,
                bit_order: BitOrder::MsbFirst,
            },
            page_size: crate::eeprom::PAGE_SIZE,
            sector_size: crate::eeprom::SECTOR_SIZE,
            chip_size: crate::eeprom::CHIP_SIZE,
            write_protect: false,
        }
    }
}

/// Configuration for the electronic compass (I2C).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassConfig {
    /// I2C bus communication settings.
    pub i2c: I2cConfig,
    /// 7-bit device slave address.
    pub i2c_addr: u8,
    /// Expected sample rate in Hz.
    pub update_rate_hz: u8,
}

impl Default for CompassConfig {
    fn default() -> Self {
        Self {
            i2c: I2cConfig {
                speed: I2cSpeed::Standard100kHz,
            },
            i2c_addr: crate::compass::COMPASS_ADDR,
            update_rate_hz: 10,
        }
    }
}

/// Configuration for the ADC temperature and supply voltage monitoring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TemperatureConfig {
    /// ADC channels connected to temperature sensors (default channels 0, 1, 2).
    pub temp_channels: [u8; 3],
    /// ADC channel connected to power supply voltage divider (default channel 3).
    pub power_channel: u8,
    /// ADC reference voltage in millivolts.
    pub vref_mv: u32,
    /// ADC full-scale resolution (12-bit = 4095).
    pub full_scale: u16,
    /// First-order IIR smoothing factor (0.0 .. 1.0).
    pub iir_alpha: f32,
}

impl Default for TemperatureConfig {
    fn default() -> Self {
        Self {
            temp_channels: [0, 1, 2],
            power_channel: 3,
            vref_mv: 3300,
            full_scale: 4095,
            iir_alpha: crate::temperature::IIR_ALPHA,
        }
    }
}

/// Configuration for the barometric pressure and temperature sensor (MS5534).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PressureConfig {
    /// SPI clock frequency used for the 3-wire sensor interface.
    pub spi: SpiConfig,
    /// Sampling cycle interval in milliseconds.
    pub sample_interval_ms: u32,
}

impl Default for PressureConfig {
    fn default() -> Self {
        Self {
            spi: SpiConfig {
                frequency_hz: 500_000,
                mode: SpiMode::Mode0,
                bit_order: BitOrder::MsbFirst,
            },
            sample_interval_ms: 50,
        }
    }
}

/// Configuration for the 5-switch digital input keypad.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeypadConfig {
    /// Switch polarity (true = logic high is pressed, false = active low).
    pub active_high: bool,
    /// Number of switches combined in the bitmask.
    pub switch_count: u8,
    /// Software debounce duration in milliseconds.
    pub debounce_ms: u32,
}

impl Default for KeypadConfig {
    fn default() -> Self {
        Self {
            active_high: true,
            switch_count: 5,
            debounce_ms: 20,
        }
    }
}

/// Configuration for the SD card logging storage subsystem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageConfig {
    /// SPI clock frequency during SD card identification / initialization (≤ 400 kHz).
    pub spi_init_clock_hz: u32,
    /// SPI clock frequency during normal data transfer (up to 20-25 MHz).
    pub spi_data_clock_hz: u32,
    /// Primary log file path in the root directory.
    pub log_filename: &'static str,
    /// Partition / volume index to mount.
    pub volume_idx: usize,
}

impl Default for StorageConfig {
    fn default() -> Self {
        Self {
            spi_init_clock_hz: 400_000,
            spi_data_clock_hz: 20_000_000,
            log_filename: "DEBUG.LOG",
            volume_idx: 0,
        }
    }
}

/// Configuration for the Ethernet / EMAC network controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EthernetConfig {
    /// Local MAC address (6 bytes).
    pub mac_address: [u8; 6],
    /// Static IPv4 address (4 bytes), used before/if DHCP is disabled.
    pub ip_address: [u8; 4],
    /// Subnet mask (4 bytes).
    pub subnet_mask: [u8; 4],
    /// Default gateway IPv4 address (4 bytes).
    pub gateway: [u8; 4],
    /// Enable DHCP client for automatic network configuration.
    pub dhcp_enabled: bool,
    /// PHY transceiver address on the MDIO/MDC bus.
    pub phy_address: u8,
    /// Use Reduced Media Independent Interface (RMII).
    pub rmii: bool,
    /// Link speed in Mbps (10 or 100).
    pub speed_mbps: u16,
    /// Full-duplex mode enabled.
    pub full_duplex: bool,
}

impl Default for EthernetConfig {
    fn default() -> Self {
        Self {
            mac_address: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            ip_address: [192, 168, 2, 101],
            subnet_mask: [255, 255, 255, 0],
            gateway: [192, 168, 2, 1],
            dhcp_enabled: true,
            phy_address: 1,
            rmii: true,
            speed_mbps: 100,
            full_duplex: true,
        }
    }
}

/// Unified top-level hardware configuration for the LPC1768 UM system board.
///
/// Contains the configuration for every hardware peripheral and subsystem on the board.
#[derive(Debug, Clone, PartialEq)]
pub struct HardwareConfig {
    /// Master CPU core clock in Hertz (e.g. 96 MHz on LPC1768).
    pub core_clock_hz: u32,
    /// Watchdog configuration.
    pub watchdog: WatchdogConfig,
    /// System timer configuration.
    pub timer: TimerConfig,
    /// Real-time clock configuration.
    pub rtc: RtcConfig,
    /// GPS UART configuration.
    pub gps: GpsConfig,
    /// Two onboard 3-axis SPI accelerometers (sensor 0 and sensor 1).
    pub accelerometers: [AccelerometerConfig; 2],
    /// 25LC1024 SPI EEPROM configuration.
    pub eeprom: EepromConfig,
    /// Electronic compass (I2C) configuration.
    pub compass: CompassConfig,
    /// ADC temperature and voltage sensing configuration.
    pub temperature: TemperatureConfig,
    /// MS5534 pressure sensor configuration.
    pub pressure: PressureConfig,
    /// 5-switch digital keypad configuration.
    pub keypad: KeypadConfig,
    /// SD card FAT storage configuration.
    pub storage: StorageConfig,
    /// Ethernet EMAC configuration.
    pub ethernet: EthernetConfig,
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self {
            core_clock_hz: 96_000_000,
            watchdog: WatchdogConfig::default(),
            timer: TimerConfig::default(),
            rtc: RtcConfig::default(),
            gps: GpsConfig::default(),
            accelerometers: [
                AccelerometerConfig {
                    cs_id: 0,
                    ..AccelerometerConfig::default()
                },
                AccelerometerConfig {
                    cs_id: 1,
                    ..AccelerometerConfig::default()
                },
            ],
            eeprom: EepromConfig::default(),
            compass: CompassConfig::default(),
            temperature: TemperatureConfig::default(),
            pressure: PressureConfig::default(),
            keypad: KeypadConfig::default(),
            storage: StorageConfig::default(),
            ethernet: EthernetConfig::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_hardware_config_matches_board_specs() {
        let hw = HardwareConfig::default();
        assert_eq!(hw.core_clock_hz, 96_000_000);
        assert_eq!(hw.watchdog.timeout_ms, 1000);
        assert_eq!(hw.timer.resolution_us, 1);
        assert_eq!(hw.gps.uart.baud_rate, 9600);
        assert_eq!(hw.accelerometers[0].cs_id, 0);
        assert_eq!(hw.accelerometers[1].cs_id, 1);
        assert_eq!(hw.compass.i2c_addr, 0x19);
        assert_eq!(hw.temperature.temp_channels, [0, 1, 2]);
        assert_eq!(hw.temperature.power_channel, 3);
        assert_eq!(hw.keypad.switch_count, 5);
        assert_eq!(hw.storage.log_filename, "DEBUG.LOG");
        assert_eq!(hw.ethernet.speed_mbps, 100);
    }

    #[test]
    fn watchdog_config_computes_wdtc() {
        let wdt = WatchdogConfig {
            core_clock_hz: 96_000_000,
            timeout_ms: 1000,
            enabled: true,
        };
        assert_eq!(wdt.timeout_to_wdtc(), 12_000_000);
    }

    #[test]
    fn timer_config_computes_ticks_for_us() {
        let t = TimerConfig {
            core_clock_hz: 96_000_000,
            resolution_us: 10,
        };
        assert_eq!(t.ticks_for_us(0), 1);
        assert_eq!(t.ticks_for_us(5), 1);
        assert_eq!(t.ticks_for_us(10), 1);
        assert_eq!(t.ticks_for_us(25), 3);
    }

    #[test]
    fn i2c_speed_frequency() {
        assert_eq!(I2cSpeed::Standard100kHz.hz(), 100_000);
        assert_eq!(I2cSpeed::Fast400kHz.hz(), 400_000);
        assert_eq!(I2cSpeed::FastPlus1MHz.hz(), 1_000_000);
    }
}
