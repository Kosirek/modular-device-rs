//! Remote control command parser and dispatcher over UDP.
//!
//! Ported from the application UDP payload handler hook in `mainPrg.c`.
//! Allows remote monitoring and configuration of the sensor logger via UDP text datagrams.

use alloc::format;
use alloc::string::String;

/// Supported remote control commands sent over UDP.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// Ping request (`PING`). Returns `PONG`.
    Ping,
    /// Status inquiry (`STATUS`). Returns active state (SD, Ethernet, IP, RTC).
    Status,
    /// Enable SD card logging (`LOG START` or `LOG ON`).
    LogStart,
    /// Disable SD card logging (`LOG STOP` or `LOG OFF`).
    LogStop,
    /// Enable Ethernet UDP telemetry broadcasting (`ETH START` or `ETH ON`).
    EthStart,
    /// Disable Ethernet UDP telemetry broadcasting (`ETH STOP` or `ETH OFF`).
    EthStop,
    /// Force synchronization/flush of the SD card log buffer (`SYNC`).
    Sync,
    /// Query peripheral power rail state (`POWER` or `POWER STATUS`).
    PowerStatus,
    /// Toggle specific power domain (`POWER <DOMAIN> <ON|OFF>`).
    PowerSet(crate::power::PowerDomain, bool),
    /// Turn all peripheral power domains on or off (`POWER ALL <ON|OFF>`).
    PowerAll(bool),
    /// Query persistent NVRAM configuration (`CONFIG` or `CONFIG GET`).
    ConfigGet,
    /// Reset persistent NVRAM configuration to board defaults (`CONFIG RESET`).
    ConfigReset,
    /// Save current configuration into persistent NVRAM (`CONFIG SAVE`).
    ConfigSave,
    /// Request software system reset / reboot (`REBOOT`).
    Reboot,
    /// Query available commands (`HELP` or `?`).
    Help,
    /// Unknown or unrecognised command payload.
    Unknown(String),
}

impl Command {
    /// Parse an incoming raw text payload into a [`Command`].
    ///
    /// Ignores leading/trailing whitespace and ASCII case.
    pub fn parse(input: &str) -> Self {
        let trimmed = input.trim();
        if trimmed.is_empty() {
            return Command::Unknown(String::new());
        }

        // Split into tokens (e.g. "POWER" "GPS" "ON")
        let mut tokens = trimmed.split_whitespace();
        let first = tokens.next().unwrap_or("");
        let second = tokens.next().unwrap_or("");
        let third = tokens.next().unwrap_or("");

        if first.eq_ignore_ascii_case("PING") {
            Command::Ping
        } else if first.eq_ignore_ascii_case("STATUS") {
            Command::Status
        } else if first.eq_ignore_ascii_case("SYNC") {
            Command::Sync
        } else if first.eq_ignore_ascii_case("REBOOT")
            || (first.eq_ignore_ascii_case("RESET") && second.is_empty())
        {
            Command::Reboot
        } else if first.eq_ignore_ascii_case("HELP") || first == "?" {
            Command::Help
        } else if first.eq_ignore_ascii_case("LOG") {
            if second.eq_ignore_ascii_case("START") || second.eq_ignore_ascii_case("ON") {
                Command::LogStart
            } else if second.eq_ignore_ascii_case("STOP") || second.eq_ignore_ascii_case("OFF") {
                Command::LogStop
            } else {
                Command::Unknown(trimmed.into())
            }
        } else if first.eq_ignore_ascii_case("ETH") {
            if second.eq_ignore_ascii_case("START") || second.eq_ignore_ascii_case("ON") {
                Command::EthStart
            } else if second.eq_ignore_ascii_case("STOP") || second.eq_ignore_ascii_case("OFF") {
                Command::EthStop
            } else {
                Command::Unknown(trimmed.into())
            }
        } else if first.eq_ignore_ascii_case("POWER") {
            if second.is_empty()
                || second.eq_ignore_ascii_case("STATUS")
                || second.eq_ignore_ascii_case("GET")
            {
                Command::PowerStatus
            } else if second.eq_ignore_ascii_case("ALL") {
                if third.eq_ignore_ascii_case("ON") || third.eq_ignore_ascii_case("1") {
                    Command::PowerAll(true)
                } else if third.eq_ignore_ascii_case("OFF") || third.eq_ignore_ascii_case("0") {
                    Command::PowerAll(false)
                } else {
                    Command::Unknown(trimmed.into())
                }
            } else if let Some(domain) = crate::power::PowerDomain::from_name(second) {
                if third.eq_ignore_ascii_case("ON") || third.eq_ignore_ascii_case("1") {
                    Command::PowerSet(domain, true)
                } else if third.eq_ignore_ascii_case("OFF") || third.eq_ignore_ascii_case("0") {
                    Command::PowerSet(domain, false)
                } else {
                    Command::Unknown(trimmed.into())
                }
            } else {
                Command::Unknown(trimmed.into())
            }
        } else if first.eq_ignore_ascii_case("CONFIG") {
            if second.is_empty()
                || second.eq_ignore_ascii_case("GET")
                || second.eq_ignore_ascii_case("STATUS")
            {
                Command::ConfigGet
            } else if second.eq_ignore_ascii_case("RESET") || second.eq_ignore_ascii_case("DEFAULT")
            {
                Command::ConfigReset
            } else if second.eq_ignore_ascii_case("SAVE") || second.eq_ignore_ascii_case("WRITE") {
                Command::ConfigSave
            } else {
                Command::Unknown(trimmed.into())
            }
        } else {
            Command::Unknown(trimmed.into())
        }
    }
}

/// Interface for executing commands against application state.
pub trait CommandHandler {
    /// Current state of SD card logging.
    fn is_sd_enabled(&self) -> bool;
    /// Enable or disable SD card logging.
    fn set_sd_enabled(&mut self, enabled: bool);

    /// Current state of Ethernet broadcasting.
    fn is_eth_enabled(&self) -> bool;
    /// Enable or disable Ethernet broadcasting.
    fn set_eth_enabled(&mut self, enabled: bool);

    /// Current local IPv4 address.
    fn current_ip(&self) -> [u8; 4];

    /// Formatted current RTC timestamp string, if available.
    fn rtc_string(&self) -> Option<String>;

    /// Force flushing SD card file buffers.
    fn sync_storage(&mut self) -> Result<(), crate::hal::HalError>;

    /// Query the current 8-bit power rail bitmask.
    fn power_mask(&self) -> u8 {
        crate::power::DEFAULT_BOARD_POWER_MASK
    }

    /// Set an individual peripheral power rail state.
    fn set_power_domain(
        &mut self,
        _domain: crate::power::PowerDomain,
        _on: bool,
    ) -> Result<(), crate::hal::HalError> {
        Ok(())
    }

    /// Set all peripheral power rails on or off.
    fn set_all_power(&mut self, _on: bool) -> Result<(), crate::hal::HalError> {
        Ok(())
    }

    /// Formatted NVRAM configuration string.
    fn config_string(&self) -> Option<String> {
        None
    }

    /// Reset NVRAM configuration back to factory board defaults.
    fn reset_config(&mut self) -> Result<(), crate::hal::HalError> {
        Ok(())
    }

    /// Save current active settings to persistent NVRAM.
    fn save_config(&mut self) -> Result<(), crate::hal::HalError> {
        Ok(())
    }

    /// Request a device software reboot.
    fn request_reboot(&mut self) {}

    /// Formatted physical PHY link status (e.g. `UP 100M/FD` or `DOWN`).
    fn link_status_string(&self) -> Option<String> {
        None
    }
}

/// Execute a [`Command`] against a [`CommandHandler`], returning the formatted reply string.
pub fn execute_command<H: CommandHandler>(cmd: &Command, handler: &mut H) -> String {
    match cmd {
        Command::Ping => String::from("PONG"),
        Command::Status => {
            let sd_str = if handler.is_sd_enabled() { "ON" } else { "OFF" };
            let eth_str = if handler.is_eth_enabled() {
                "ON"
            } else {
                "OFF"
            };
            let ip = handler.current_ip();
            let rtc = handler
                .rtc_string()
                .unwrap_or_else(|| String::from("UNKNOWN"));
            let link = handler
                .link_status_string()
                .map(|l| format!(" link={}", l))
                .unwrap_or_default();
            format!(
                "STATUS sd={} eth={} ip={}.{}.{}.{} rtc={}{}",
                sd_str, eth_str, ip[0], ip[1], ip[2], ip[3], rtc, link
            )
        }
        Command::LogStart => {
            handler.set_sd_enabled(true);
            String::from("OK LOG STARTED")
        }
        Command::LogStop => {
            handler.set_sd_enabled(false);
            String::from("OK LOG STOPPED")
        }
        Command::EthStart => {
            handler.set_eth_enabled(true);
            String::from("OK ETH STARTED")
        }
        Command::EthStop => {
            handler.set_eth_enabled(false);
            String::from("OK ETH STOPPED")
        }
        Command::Sync => match handler.sync_storage() {
            Ok(()) => String::from("OK SYNCED"),
            Err(_) => String::from("ERR SYNC FAILED"),
        },
        Command::PowerStatus => {
            let mask = handler.power_mask();
            format!(
                "POWER mask=0x{:02X} gps={} compass={} acc1={} acc2={} temp={} ext_temp={} press={} sd={}",
                mask,
                if (mask & crate::power::GPS_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::COMPASS_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::ACC1_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::ACC2_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::TEMP_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::EXT_TEMP_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::PRESSURE_ON_MASK) != 0 { "ON" } else { "OFF" },
                if (mask & crate::power::SD_3V3_ON_MASK) != 0 { "ON" } else { "OFF" },
            )
        }
        Command::PowerSet(domain, on) => match handler.set_power_domain(*domain, *on) {
            Ok(()) => format!("OK POWER {:?} {}", domain, if *on { "ON" } else { "OFF" }),
            Err(_) => String::from("ERR POWER SET FAILED"),
        },
        Command::PowerAll(on) => match handler.set_all_power(*on) {
            Ok(()) => format!("OK POWER ALL {}", if *on { "ON" } else { "OFF" }),
            Err(_) => String::from("ERR POWER SET FAILED"),
        },
        Command::ConfigGet => match handler.config_string() {
            Some(s) => s,
            None => String::from("ERR CONFIG NOT AVAILABLE"),
        },
        Command::ConfigReset => match handler.reset_config() {
            Ok(()) => String::from("OK CONFIG RESET TO DEFAULTS"),
            Err(_) => String::from("ERR CONFIG RESET FAILED"),
        },
        Command::ConfigSave => match handler.save_config() {
            Ok(()) => String::from("OK CONFIG SAVED"),
            Err(_) => String::from("ERR CONFIG SAVE FAILED"),
        },
        Command::Reboot => {
            handler.request_reboot();
            String::from("OK REBOOTING")
        }
        Command::Help => String::from(
            "COMMANDS: PING, STATUS, LOG <ON|OFF>, ETH <ON|OFF>, SYNC, POWER <STATUS|ALL <ON|OFF>|<DOMAIN> <ON|OFF>>, CONFIG <GET|RESET|SAVE>, REBOOT, HELP",
        ),
        Command::Unknown(raw) => format!("ERR UNKNOWN COMMAND: {}", raw),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::HalError;

    #[derive(Default)]
    struct MockHandler {
        sd_on: bool,
        eth_on: bool,
        ip: [u8; 4],
        rtc: Option<String>,
        sync_ok: bool,
    }

    impl CommandHandler for MockHandler {
        fn is_sd_enabled(&self) -> bool {
            self.sd_on
        }
        fn set_sd_enabled(&mut self, enabled: bool) {
            self.sd_on = enabled;
        }
        fn is_eth_enabled(&self) -> bool {
            self.eth_on
        }
        fn set_eth_enabled(&mut self, enabled: bool) {
            self.eth_on = enabled;
        }
        fn current_ip(&self) -> [u8; 4] {
            self.ip
        }
        fn rtc_string(&self) -> Option<String> {
            self.rtc.clone()
        }
        fn sync_storage(&mut self) -> Result<(), HalError> {
            if self.sync_ok {
                Ok(())
            } else {
                Err(HalError::Bus)
            }
        }
    }

    #[test]
    fn parse_ping_and_help() {
        assert_eq!(Command::parse("PING"), Command::Ping);
        assert_eq!(Command::parse("ping\r\n"), Command::Ping);
        assert_eq!(Command::parse("  Ping  "), Command::Ping);

        assert_eq!(Command::parse("HELP"), Command::Help);
        assert_eq!(Command::parse("?"), Command::Help);
    }

    #[test]
    fn parse_log_controls() {
        assert_eq!(Command::parse("LOG START"), Command::LogStart);
        assert_eq!(Command::parse("log on\r\n"), Command::LogStart);
        assert_eq!(Command::parse("LOG STOP"), Command::LogStop);
        assert_eq!(Command::parse("log off"), Command::LogStop);
        assert_eq!(
            Command::parse("LOG RESTART"),
            Command::Unknown(String::from("LOG RESTART"))
        );
    }

    #[test]
    fn parse_eth_controls() {
        assert_eq!(Command::parse("ETH START"), Command::EthStart);
        assert_eq!(Command::parse("eth on"), Command::EthStart);
        assert_eq!(Command::parse("ETH STOP"), Command::EthStop);
        assert_eq!(Command::parse("eth off\n"), Command::EthStop);
    }

    #[test]
    fn parse_status_and_sync() {
        assert_eq!(Command::parse("STATUS"), Command::Status);
        assert_eq!(Command::parse("status\r\n"), Command::Status);
        assert_eq!(Command::parse("SYNC"), Command::Sync);
        assert_eq!(Command::parse("sync"), Command::Sync);
    }

    #[test]
    fn execute_ping_and_help() {
        let mut h = MockHandler::default();
        assert_eq!(execute_command(&Command::Ping, &mut h), "PONG");
        assert!(execute_command(&Command::Help, &mut h).contains("COMMANDS:"));
    }

    #[test]
    fn execute_log_and_eth_toggles() {
        let mut h = MockHandler {
            sd_on: false,
            eth_on: true,
            ..Default::default()
        };

        assert_eq!(
            execute_command(&Command::LogStart, &mut h),
            "OK LOG STARTED"
        );
        assert!(h.sd_on);

        assert_eq!(execute_command(&Command::LogStop, &mut h), "OK LOG STOPPED");
        assert!(!h.sd_on);

        assert_eq!(execute_command(&Command::EthStop, &mut h), "OK ETH STOPPED");
        assert!(!h.eth_on);

        assert_eq!(
            execute_command(&Command::EthStart, &mut h),
            "OK ETH STARTED"
        );
        assert!(h.eth_on);
    }

    #[test]
    fn execute_status_formatting() {
        let mut h = MockHandler {
            sd_on: true,
            eth_on: false,
            ip: [192, 168, 2, 101],
            rtc: Some(String::from("2024-11-07 14:30:15")),
            sync_ok: true,
        };

        let res = execute_command(&Command::Status, &mut h);
        assert_eq!(
            res,
            "STATUS sd=ON eth=OFF ip=192.168.2.101 rtc=2024-11-07 14:30:15"
        );
    }

    #[test]
    fn execute_sync_success_and_failure() {
        let mut h = MockHandler {
            sync_ok: true,
            ..Default::default()
        };
        assert_eq!(execute_command(&Command::Sync, &mut h), "OK SYNCED");

        h.sync_ok = false;
        assert_eq!(execute_command(&Command::Sync, &mut h), "ERR SYNC FAILED");
    }

    #[test]
    fn execute_unknown() {
        let mut h = MockHandler::default();
        let cmd = Command::parse("FOOBAR");
        assert_eq!(execute_command(&cmd, &mut h), "ERR UNKNOWN COMMAND: FOOBAR");
    }

    #[test]
    fn parse_power_and_config_and_reboot() {
        assert_eq!(Command::parse("POWER"), Command::PowerStatus);
        assert_eq!(Command::parse("POWER STATUS"), Command::PowerStatus);
        assert_eq!(Command::parse("power all on"), Command::PowerAll(true));
        assert_eq!(Command::parse("POWER ALL OFF"), Command::PowerAll(false));
        assert_eq!(
            Command::parse("POWER GPS ON"),
            Command::PowerSet(crate::power::PowerDomain::Gps, true)
        );
        assert_eq!(
            Command::parse("POWER COMPASS OFF"),
            Command::PowerSet(crate::power::PowerDomain::Compass, false)
        );
        assert_eq!(
            Command::parse("POWER ACC1 1"),
            Command::PowerSet(crate::power::PowerDomain::Acc1, true)
        );

        assert_eq!(Command::parse("CONFIG"), Command::ConfigGet);
        assert_eq!(Command::parse("CONFIG GET"), Command::ConfigGet);
        assert_eq!(Command::parse("CONFIG RESET"), Command::ConfigReset);
        assert_eq!(Command::parse("CONFIG SAVE"), Command::ConfigSave);

        assert_eq!(Command::parse("REBOOT"), Command::Reboot);
        assert_eq!(Command::parse("RESET"), Command::Reboot);
    }

    #[test]
    fn execute_power_and_config() {
        struct PowerMock {
            base: MockHandler,
            mask: u8,
            reboot_called: bool,
        }
        impl CommandHandler for PowerMock {
            fn is_sd_enabled(&self) -> bool {
                self.base.is_sd_enabled()
            }
            fn set_sd_enabled(&mut self, enabled: bool) {
                self.base.set_sd_enabled(enabled)
            }
            fn is_eth_enabled(&self) -> bool {
                self.base.is_eth_enabled()
            }
            fn set_eth_enabled(&mut self, enabled: bool) {
                self.base.set_eth_enabled(enabled)
            }
            fn current_ip(&self) -> [u8; 4] {
                self.base.current_ip()
            }
            fn rtc_string(&self) -> Option<String> {
                self.base.rtc_string()
            }
            fn sync_storage(&mut self) -> Result<(), HalError> {
                self.base.sync_storage()
            }
            fn power_mask(&self) -> u8 {
                self.mask
            }
            fn set_power_domain(
                &mut self,
                domain: crate::power::PowerDomain,
                on: bool,
            ) -> Result<(), HalError> {
                if on {
                    self.mask |= domain.mask();
                } else {
                    self.mask &= !domain.mask();
                }
                Ok(())
            }
            fn set_all_power(&mut self, on: bool) -> Result<(), HalError> {
                self.mask = if on { 0xFF } else { 0x00 };
                Ok(())
            }
            fn config_string(&self) -> Option<String> {
                Some(String::from("CONFIG ip=192.168.2.101"))
            }
            fn request_reboot(&mut self) {
                self.reboot_called = true;
            }
        }

        let mut h = PowerMock {
            base: MockHandler::default(),
            mask: crate::power::DEFAULT_BOARD_POWER_MASK,
            reboot_called: false,
        };

        let status = execute_command(&Command::PowerStatus, &mut h);
        assert!(status.contains("gps=ON"));
        assert!(status.contains("acc1=OFF"));

        let res = execute_command(
            &Command::PowerSet(crate::power::PowerDomain::Acc1, true),
            &mut h,
        );
        assert_eq!(res, "OK POWER Acc1 ON");
        assert_eq!(
            h.mask & crate::power::ACC1_ON_MASK,
            crate::power::ACC1_ON_MASK
        );

        let res = execute_command(&Command::PowerAll(false), &mut h);
        assert_eq!(res, "OK POWER ALL OFF");
        assert_eq!(h.mask, 0x00);

        let cfg = execute_command(&Command::ConfigGet, &mut h);
        assert_eq!(cfg, "CONFIG ip=192.168.2.101");

        let reboot = execute_command(&Command::Reboot, &mut h);
        assert_eq!(reboot, "OK REBOOTING");
        assert!(h.reboot_called);
    }
}
