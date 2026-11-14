//! Driver for the DP83848 10/100 Ethernet Physical Layer Transceiver (PHY).
//!
//! Communicates over the IEEE 802.3u MII/RMII Management Interface (MDIO/MDC)
//! connected to the LPC1768 EMAC peripheral.
//!
//! Provides link monitoring, speed/duplex detection, software reset,
//! and auto-negotiation restart capabilities.

use crate::hal::HalResult;

/// Default DP83848 hardware PHY address on LPC1768 evaluation boards.
pub const DP83848_DEFAULT_ADDR: u8 = 1;

// Standard MII Registers (IEEE 802.3u Clause 22)
pub const REG_BMCR: u8 = 0x00;
pub const REG_BMSR: u8 = 0x01;
pub const REG_PHYIDR1: u8 = 0x02;
pub const REG_PHYIDR2: u8 = 0x03;
pub const REG_ANAR: u8 = 0x04;
pub const REG_ANLPAR: u8 = 0x05;
pub const REG_ANER: u8 = 0x06;

// DP83848 Vendor-Specific Registers
pub const REG_PHYSTS: u8 = 0x10;
pub const REG_MICR: u8 = 0x11;
pub const REG_MISR: u8 = 0x12;

// BMCR (Basic Mode Control Register) bits
pub const BMCR_RESET: u16 = 1 << 15;
pub const BMCR_LOOPBACK: u16 = 1 << 14;
pub const BMCR_SPEED_100: u16 = 1 << 13;
pub const BMCR_AN_ENABLE: u16 = 1 << 12;
pub const BMCR_POWER_DOWN: u16 = 1 << 11;
pub const BMCR_ISOLATE: u16 = 1 << 10;
pub const BMCR_RESTART_AN: u16 = 1 << 9;
pub const BMCR_FULL_DUPLEX: u16 = 1 << 8;
pub const BMCR_COLLISION_TEST: u16 = 1 << 7;

// BMSR (Basic Mode Status Register) bits
pub const BMSR_100_TX_FD: u16 = 1 << 14;
pub const BMSR_100_TX_HD: u16 = 1 << 13;
pub const BMSR_10_FD: u16 = 1 << 12;
pub const BMSR_10_HD: u16 = 1 << 11;
pub const BMSR_AN_COMPLETE: u16 = 1 << 5;
pub const BMSR_REMOTE_FAULT: u16 = 1 << 4;
pub const BMSR_AN_ABILITY: u16 = 1 << 3;
pub const BMSR_LINK_STATUS: u16 = 1 << 2;
pub const BMSR_JABBER_DETECT: u16 = 1 << 1;
pub const BMSR_EXTENDED_CAP: u16 = 1 << 0;

// PHYSTS (DP83848 PHY Status Register) bits
pub const PHYSTS_LINK_STATUS: u16 = 1 << 0;
pub const PHYSTS_SPEED_10: u16 = 1 << 1; // 0 = 100 Mbps, 1 = 10 Mbps
pub const PHYSTS_DUPLEX_FULL: u16 = 1 << 2; // 1 = Full Duplex, 0 = Half Duplex
pub const PHYSTS_LOOPBACK: u16 = 1 << 3;
pub const PHYSTS_AN_COMPLETE: u16 = 1 << 4;

/// Expected DP83848 OUI and model identifier.
pub const DP83848_PHY_ID: u32 = 0x20005C90;
pub const DP83848_PHY_ID_MASK: u32 = 0xFFFFFFF0;

/// Link communication speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkSpeed {
    Speed10Mbps,
    Speed100Mbps,
}

/// Link duplex mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DuplexMode {
    HalfDuplex,
    FullDuplex,
}

/// Consolidated physical Ethernet link status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyLinkStatus {
    /// Cable link detected and active.
    pub link_up: bool,
    /// Negotiated or forced line speed.
    pub speed: LinkSpeed,
    /// Negotiated or forced duplex mode.
    pub duplex: DuplexMode,
    /// Auto-negotiation completed successfully.
    pub autoneg_complete: bool,
}

/// Abstract MII/RMII Management Interface bus (MDIO/MDC).
pub trait MiiBus {
    /// Read a 16-bit register from the PHY at `phy_addr`.
    fn read_phy(&mut self, phy_addr: u8, reg_addr: u8) -> HalResult<u16>;

    /// Write a 16-bit register to the PHY at `phy_addr`.
    fn write_phy(&mut self, phy_addr: u8, reg_addr: u8, value: u16) -> HalResult<()>;
}

impl<M: MiiBus + ?Sized> MiiBus for &mut M {
    fn read_phy(&mut self, phy_addr: u8, reg_addr: u8) -> HalResult<u16> {
        (**self).read_phy(phy_addr, reg_addr)
    }

    fn write_phy(&mut self, phy_addr: u8, reg_addr: u8, value: u16) -> HalResult<()> {
        (**self).write_phy(phy_addr, reg_addr, value)
    }
}

/// Driver for the DP83848 Ethernet PHY transceiver.
pub struct Dp83848<M: MiiBus> {
    bus: M,
    phy_addr: u8,
}

impl<M: MiiBus> Dp83848<M> {
    /// Create a new driver instance for DP83848 at the given PHY address.
    pub fn new(bus: M, phy_addr: u8) -> Self {
        Self { bus, phy_addr }
    }

    /// Access the underlying MII management bus.
    pub fn bus(&self) -> &M {
        &self.bus
    }

    /// Mutably access the underlying MII management bus.
    pub fn bus_mut(&mut self) -> &mut M {
        &mut self.bus
    }

    /// Read the 32-bit PHY identifier (PHYIDR1 + PHYIDR2).
    pub fn read_id(&mut self) -> HalResult<u32> {
        let id1 = self.bus.read_phy(self.phy_addr, REG_PHYIDR1)? as u32;
        let id2 = self.bus.read_phy(self.phy_addr, REG_PHYIDR2)? as u32;
        Ok((id1 << 16) | id2)
    }

    /// Verify whether the connected PHY matches DP83848 signature.
    pub fn is_dp83848(&mut self) -> HalResult<bool> {
        let id = self.read_id()?;
        Ok((id & DP83848_PHY_ID_MASK) == (DP83848_PHY_ID & DP83848_PHY_ID_MASK))
    }

    /// Check if the Ethernet cable link is active (BMSR bit 2).
    ///
    /// Note: BMSR Link Status bit is latching-low, meaning a read clears a previous drop.
    /// To get the real-time link status, two reads or PHYSTS register reading can be used.
    pub fn is_link_up(&mut self) -> HalResult<bool> {
        // Read PHYSTS (real-time DP83848 register)
        let physts = self.bus.read_phy(self.phy_addr, REG_PHYSTS)?;
        Ok((physts & PHYSTS_LINK_STATUS) != 0)
    }

    /// Read full negotiated link status (speed, duplex, auto-negotiation complete).
    pub fn read_link_status(&mut self) -> HalResult<PhyLinkStatus> {
        let physts = self.bus.read_phy(self.phy_addr, REG_PHYSTS)?;
        let link_up = (physts & PHYSTS_LINK_STATUS) != 0;
        let speed = if (physts & PHYSTS_SPEED_10) != 0 {
            LinkSpeed::Speed10Mbps
        } else {
            LinkSpeed::Speed100Mbps
        };
        let duplex = if (physts & PHYSTS_DUPLEX_FULL) != 0 {
            DuplexMode::FullDuplex
        } else {
            DuplexMode::HalfDuplex
        };
        let autoneg_complete = (physts & PHYSTS_AN_COMPLETE) != 0;

        Ok(PhyLinkStatus {
            link_up,
            speed,
            duplex,
            autoneg_complete,
        })
    }

    /// Issue a software reset to the PHY (BMCR bit 15).
    pub fn reset(&mut self) -> HalResult<()> {
        let val = self.bus.read_phy(self.phy_addr, REG_BMCR)?;
        self.bus
            .write_phy(self.phy_addr, REG_BMCR, val | BMCR_RESET)?;
        Ok(())
    }

    /// Restart auto-negotiation process (BMCR bits 12 and 9).
    pub fn restart_autonegotiation(&mut self) -> HalResult<()> {
        let val = self.bus.read_phy(self.phy_addr, REG_BMCR)?;
        let new_val = val | BMCR_AN_ENABLE | BMCR_RESTART_AN;
        self.bus.write_phy(self.phy_addr, REG_BMCR, new_val)
    }

    /// Manually configure speed and duplex without auto-negotiation.
    pub fn configure_manual(&mut self, speed: LinkSpeed, duplex: DuplexMode) -> HalResult<()> {
        let mut bmcr = 0u16;
        if speed == LinkSpeed::Speed100Mbps {
            bmcr |= BMCR_SPEED_100;
        }
        if duplex == DuplexMode::FullDuplex {
            bmcr |= BMCR_FULL_DUPLEX;
        }
        self.bus.write_phy(self.phy_addr, REG_BMCR, bmcr)
    }
}

/// Simulated in-memory MII management bus for unit tests and host simulation.
#[derive(Debug, Clone)]
pub struct InMemoryMiiBus {
    pub registers: [u16; 32],
}

impl Default for InMemoryMiiBus {
    fn default() -> Self {
        let mut registers = [0u16; 32];
        registers[REG_BMCR as usize] = BMCR_AN_ENABLE | BMCR_SPEED_100 | BMCR_FULL_DUPLEX;
        registers[REG_BMSR as usize] =
            BMSR_100_TX_FD | BMSR_100_TX_HD | BMSR_10_FD | BMSR_10_HD | BMSR_AN_ABILITY;
        registers[REG_PHYIDR1 as usize] = (DP83848_PHY_ID >> 16) as u16;
        registers[REG_PHYIDR2 as usize] = (DP83848_PHY_ID & 0xFFFF) as u16;
        registers[REG_PHYSTS as usize] = 0; // Link down initially
        Self { registers }
    }
}

impl InMemoryMiiBus {
    /// Helper to simulate cable connect with negotiated parameters.
    pub fn set_link_state(&mut self, up: bool, speed: LinkSpeed, duplex: DuplexMode) {
        let mut physts = 0u16;
        let mut bmsr = self.registers[REG_BMSR as usize];
        if up {
            physts |= PHYSTS_LINK_STATUS | PHYSTS_AN_COMPLETE;
            bmsr |= BMSR_LINK_STATUS | BMSR_AN_COMPLETE;
            if speed == LinkSpeed::Speed10Mbps {
                physts |= PHYSTS_SPEED_10;
            }
            if duplex == DuplexMode::FullDuplex {
                physts |= PHYSTS_DUPLEX_FULL;
            }
        } else {
            physts = 0;
            bmsr &= !(BMSR_LINK_STATUS | BMSR_AN_COMPLETE);
        }
        self.registers[REG_PHYSTS as usize] = physts;
        self.registers[REG_BMSR as usize] = bmsr;
    }
}

impl MiiBus for InMemoryMiiBus {
    fn read_phy(&mut self, _phy_addr: u8, reg_addr: u8) -> HalResult<u16> {
        let idx = (reg_addr & 0x1F) as usize;
        Ok(self.registers[idx])
    }

    fn write_phy(&mut self, _phy_addr: u8, reg_addr: u8, value: u16) -> HalResult<()> {
        let idx = (reg_addr & 0x1F) as usize;
        self.registers[idx] = value;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phy_identification_succeeds() {
        let bus = InMemoryMiiBus::default();
        let mut phy = Dp83848::new(bus, DP83848_DEFAULT_ADDR);

        let id = phy.read_id().unwrap();
        assert_eq!(id, DP83848_PHY_ID);
        assert!(phy.is_dp83848().unwrap());
    }

    #[test]
    fn link_state_reporting() {
        let mut bus = InMemoryMiiBus::default();
        bus.set_link_state(false, LinkSpeed::Speed100Mbps, DuplexMode::FullDuplex);
        let mut phy = Dp83848::new(bus, DP83848_DEFAULT_ADDR);

        assert!(!phy.is_link_up().unwrap());
        let status = phy.read_link_status().unwrap();
        assert!(!status.link_up);

        // Cable connected: 100M Full Duplex
        phy.bus_mut()
            .set_link_state(true, LinkSpeed::Speed100Mbps, DuplexMode::FullDuplex);
        assert!(phy.is_link_up().unwrap());
        let status = phy.read_link_status().unwrap();
        assert!(status.link_up);
        assert_eq!(status.speed, LinkSpeed::Speed100Mbps);
        assert_eq!(status.duplex, DuplexMode::FullDuplex);
        assert!(status.autoneg_complete);

        // Cable changed: 10M Half Duplex
        phy.bus_mut()
            .set_link_state(true, LinkSpeed::Speed10Mbps, DuplexMode::HalfDuplex);
        let status = phy.read_link_status().unwrap();
        assert!(status.link_up);
        assert_eq!(status.speed, LinkSpeed::Speed10Mbps);
        assert_eq!(status.duplex, DuplexMode::HalfDuplex);
    }

    #[test]
    fn software_reset_and_restart_autoneg() {
        let bus = InMemoryMiiBus::default();
        let mut phy = Dp83848::new(bus, DP83848_DEFAULT_ADDR);

        phy.reset().unwrap();
        let bmcr = phy.bus().registers[REG_BMCR as usize];
        assert_eq!(bmcr & BMCR_RESET, BMCR_RESET);

        phy.restart_autonegotiation().unwrap();
        let bmcr = phy.bus().registers[REG_BMCR as usize];
        assert_eq!(
            bmcr & (BMCR_AN_ENABLE | BMCR_RESTART_AN),
            BMCR_AN_ENABLE | BMCR_RESTART_AN
        );
    }

    #[test]
    fn manual_configuration() {
        let bus = InMemoryMiiBus::default();
        let mut phy = Dp83848::new(bus, DP83848_DEFAULT_ADDR);

        phy.configure_manual(LinkSpeed::Speed100Mbps, DuplexMode::FullDuplex)
            .unwrap();
        let bmcr = phy.bus().registers[REG_BMCR as usize];
        assert_eq!(bmcr, BMCR_SPEED_100 | BMCR_FULL_DUPLEX);

        phy.configure_manual(LinkSpeed::Speed10Mbps, DuplexMode::HalfDuplex)
            .unwrap();
        let bmcr = phy.bus().registers[REG_BMCR as usize];
        assert_eq!(bmcr, 0);
    }
}
