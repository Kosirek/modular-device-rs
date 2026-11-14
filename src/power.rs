//! Power domain control for on-board peripherals, ported from `dev_conf.h` and `mainPrg.c`.
//!
//! On the LPC1768 board, on-board sensors and peripherals are connected to
//! power switches driven by GPIO2 pins 0..7:
//! * Bit 0: Accelerometer 1 (`ACC1_ON`)
//! * Bit 1: Accelerometer 2 (`ACC2_ON`)
//! * Bit 2: MS5534 Pressure sensor (`PREASURE_ON`)
//! * Bit 3: Internal temperature sensor (`TEMP_ON`)
//! * Bit 4: External temperature sensor (`EXT_TEMP_ON`)
//! * Bit 5: GPS receiver module (`GPS_ON`)
//! * Bit 6: SD card 3.3V supply (`SD_3V3_ON`)
//! * Bit 7: Electronic compass (`COMPASS_ON`)
//!
//! Unless these pins are driven high (active), the respective hardware devices
//! remain unpowered on the physical board.

use crate::hal::HalResult;

/// Identifiers for on-board peripheral power domains (bits 0..7 on GPIO2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerDomain {
    /// Accelerometer 1 power switch (`ACC1_ON`, bit 0).
    Acc1 = 0,
    /// Accelerometer 2 power switch (`ACC2_ON`, bit 1).
    Acc2 = 1,
    /// MS5534 Barometric pressure unit power switch (`PREASURE_ON`, bit 2).
    Pressure = 2,
    /// On-board temperature sensor power switch (`TEMP_ON`, bit 3).
    Temp = 3,
    /// External temperature sensor power switch (`EXT_TEMP_ON`, bit 4).
    ExtTemp = 4,
    /// GPS receiver module power switch (`GPS_ON`, bit 5).
    Gps = 5,
    /// SD card 3.3V power switch (`SD_3V3_ON`, bit 6).
    Sd3V3 = 6,
    /// Electronic compass power switch (`COMPASS_ON`, bit 7).
    Compass = 7,
}

impl PowerDomain {
    /// Bitmask for this power domain.
    pub const fn mask(self) -> u8 {
        1 << (self as u8)
    }
}

pub const ACC1_ON_MASK: u8 = 1 << 0;
pub const ACC2_ON_MASK: u8 = 1 << 1;
pub const PRESSURE_ON_MASK: u8 = 1 << 2;
pub const TEMP_ON_MASK: u8 = 1 << 3;
pub const EXT_TEMP_ON_MASK: u8 = 1 << 4;
pub const GPS_ON_MASK: u8 = 1 << 5;
pub const SD_3V3_ON_MASK: u8 = 1 << 6;
pub const COMPASS_ON_MASK: u8 = 1 << 7;

/// All supported power domains combined into a bitmask (0xFF).
pub const ALL_POWER_DOMAINS_MASK: u8 = 0xFF;

/// Default operational power mask matching `mainPrg.c` lines 167-170:
/// Enables GPS, Compass, Acc2, SD 3V3, Temp, and Pressure; Acc1 remains off.
pub const DEFAULT_BOARD_POWER_MASK: u8 =
    GPS_ON_MASK | COMPASS_ON_MASK | ACC2_ON_MASK | SD_3V3_ON_MASK | TEMP_ON_MASK | PRESSURE_ON_MASK;

/// Interface for controlling the 8-bit power control register (GPIO2 pins 0..7).
pub trait PowerPort {
    /// Write the entire 8-bit power domain bitmask.
    fn write_mask(&mut self, mask: u8) -> HalResult<()>;
    /// Read the current 8-bit power domain bitmask.
    fn read_mask(&self) -> u8;
}

/// In-memory implementation of [`PowerPort`] for unit tests and simulation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct InMemoryPowerPort {
    pub mask: u8,
}

impl InMemoryPowerPort {
    pub fn new(initial_mask: u8) -> Self {
        Self { mask: initial_mask }
    }
}

impl PowerPort for InMemoryPowerPort {
    fn write_mask(&mut self, mask: u8) -> HalResult<()> {
        self.mask = mask;
        Ok(())
    }

    fn read_mask(&self) -> u8 {
        self.mask
    }
}

/// Manager coordinating power state for all on-board peripheral rails.
pub struct PowerManager<P: PowerPort> {
    port: P,
    current_mask: u8,
}

impl<P: PowerPort> PowerManager<P> {
    /// Create a power manager around a hardware [`PowerPort`].
    pub fn new(mut port: P) -> HalResult<Self> {
        let current_mask = port.read_mask();
        port.write_mask(current_mask)?;
        Ok(Self { port, current_mask })
    }

    /// Initialize the power manager and apply the default board configuration
    /// matching `mainPrg.c` (GPS, Compass, Acc2, SD, Temp, Pressure ON; Acc1 OFF).
    pub fn with_default_board_power(mut port: P) -> HalResult<Self> {
        port.write_mask(DEFAULT_BOARD_POWER_MASK)?;
        Ok(Self {
            port,
            current_mask: DEFAULT_BOARD_POWER_MASK,
        })
    }

    /// Access the underlying port.
    pub fn port(&self) -> &P {
        &self.port
    }

    /// Mutably access the underlying port.
    pub fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }

    /// Current 8-bit power mask.
    pub fn current_mask(&self) -> u8 {
        self.current_mask
    }

    /// Check if a specific peripheral power domain is energized.
    pub fn is_enabled(&self, domain: PowerDomain) -> bool {
        (self.current_mask & domain.mask()) != 0
    }

    /// Turn power on for a specific peripheral domain (`PWR_DEV_ON`).
    pub fn enable(&mut self, domain: PowerDomain) -> HalResult<()> {
        self.set(domain, true)
    }

    /// Turn power off for a specific peripheral domain (`PWR_DEV_OFF`).
    pub fn disable(&mut self, domain: PowerDomain) -> HalResult<()> {
        self.set(domain, false)
    }

    /// Set power state for a specific peripheral domain.
    pub fn set(&mut self, domain: PowerDomain, on: bool) -> HalResult<()> {
        let mask = if on {
            self.current_mask | domain.mask()
        } else {
            self.current_mask & !domain.mask()
        };
        self.port.write_mask(mask)?;
        self.current_mask = mask;
        Ok(())
    }

    /// Energize all peripheral domains.
    pub fn enable_all(&mut self) -> HalResult<()> {
        self.port.write_mask(ALL_POWER_DOMAINS_MASK)?;
        self.current_mask = ALL_POWER_DOMAINS_MASK;
        Ok(())
    }

    /// Cut power to all peripheral domains (low power / sleep mode).
    pub fn disable_all(&mut self) -> HalResult<()> {
        self.port.write_mask(0)?;
        self.current_mask = 0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_board_power_matches_mainprg_c() {
        let port = InMemoryPowerPort::default();
        let mgr = PowerManager::with_default_board_power(port).unwrap();

        assert!(mgr.is_enabled(PowerDomain::Gps));
        assert!(mgr.is_enabled(PowerDomain::Compass));
        assert!(mgr.is_enabled(PowerDomain::Acc2));
        assert!(mgr.is_enabled(PowerDomain::Sd3V3));
        assert!(mgr.is_enabled(PowerDomain::Temp));
        assert!(mgr.is_enabled(PowerDomain::Pressure));
        assert!(!mgr.is_enabled(PowerDomain::Acc1));
        assert!(!mgr.is_enabled(PowerDomain::ExtTemp));
    }

    #[test]
    fn enable_and_disable_toggles_domains_cleanly() {
        let port = InMemoryPowerPort::default();
        let mut mgr = PowerManager::new(port).unwrap();

        assert_eq!(mgr.current_mask(), 0);

        mgr.enable(PowerDomain::Gps).unwrap();
        assert!(mgr.is_enabled(PowerDomain::Gps));
        assert_eq!(mgr.current_mask(), GPS_ON_MASK);

        mgr.enable(PowerDomain::Sd3V3).unwrap();
        assert!(mgr.is_enabled(PowerDomain::Sd3V3));
        assert_eq!(mgr.current_mask(), GPS_ON_MASK | SD_3V3_ON_MASK);

        mgr.disable(PowerDomain::Gps).unwrap();
        assert!(!mgr.is_enabled(PowerDomain::Gps));
        assert!(mgr.is_enabled(PowerDomain::Sd3V3));
        assert_eq!(mgr.current_mask(), SD_3V3_ON_MASK);
    }

    #[test]
    fn enable_all_and_disable_all() {
        let port = InMemoryPowerPort::default();
        let mut mgr = PowerManager::new(port).unwrap();

        mgr.enable_all().unwrap();
        assert_eq!(mgr.current_mask(), 0xFF);
        assert!(mgr.is_enabled(PowerDomain::Acc1));
        assert!(mgr.is_enabled(PowerDomain::ExtTemp));

        mgr.disable_all().unwrap();
        assert_eq!(mgr.current_mask(), 0);
        assert!(!mgr.is_enabled(PowerDomain::Acc2));
    }
}
