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

        // Split into up to two tokens (e.g. "LOG" and "START")
        let mut tokens = trimmed.split_whitespace();
        let first = tokens.next().unwrap_or("");
        let second = tokens.next().unwrap_or("");

        if first.eq_ignore_ascii_case("PING") {
            Command::Ping
        } else if first.eq_ignore_ascii_case("STATUS") {
            Command::Status
        } else if first.eq_ignore_ascii_case("SYNC") {
            Command::Sync
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
            format!(
                "STATUS sd={} eth={} ip={}.{}.{}.{} rtc={}",
                sd_str, eth_str, ip[0], ip[1], ip[2], ip[3], rtc
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
        Command::Help => {
            String::from("COMMANDS: PING, STATUS, LOG <ON|OFF>, ETH <ON|OFF>, SYNC, HELP")
        }
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
}
