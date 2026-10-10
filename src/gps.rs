//! GPS receiver and aggregate state, ported from `GPS.c`.
//!
//! NMEA-0183 protocol parsing, emission, data structures, and the sentence
//! registry are implemented in the [`crate::nmea`] module. This module defines
//! the aggregate [`GpsData`] state and the [`GpsReceiver`] byte-stream framer
//! that processes incoming UART bytes into NMEA sentences.

// DONE: 1. extract NMEA to separate module, implement methods as struct implementation
// DONE: 2. NMEA messages collection rework to use map where key is message type and
//          value is a box implementing trait with parse and emit methods. This allows
//          adding new message types without changing the core parser logic.
// DONE: 3. some NMEA messages have the same field, structure for these is shared
//          to avoid code duplication (Coordinates, GpsTime, GpsDate).

pub use crate::nmea::*;

/// Decoded GPS state. Mirrors the C `_GPS` struct (field names kept close to
/// the originals so the mapping is obvious).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GpsData {
    pub time: GpsTime,
    pub date: GpsDate,
    /// Position coordinates (latitude, longitude and hemispheres).
    pub coords: Coordinates,
    pub altitude: u32,
    /// Speed, stored in km/h after conversion (`predkosc`).
    pub speed: f32,
    /// Count of sentences we did not understand or checksum failed (`unhandled_messages`).
    pub unhandled_messages: u32,
}

impl GpsData {
    /// Create an empty GPS state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Convenience accessor for latitude string.
    pub fn latitude(&self) -> &str {
        &self.coords.latitude
    }

    /// Convenience accessor for longitude string.
    pub fn longitude(&self) -> &str {
        &self.coords.longitude
    }
}

/// Parse one complete NMEA sentence into `gps` using standard message handlers.
///
/// Unknown / malformed sentences only bump `unhandled_messages`, matching the C
/// behaviour. The leading `$` and trailing CR/LF are tolerated.
pub fn parse(msg: &str, gps: &mut GpsData) {
    let reg = NmeaRegistry::with_default_messages();
    reg.parse(msg, gps);
}

/// Parse an NMEA sentence strictly, verifying its checksum first.
/// If the checksum is missing or does not match, increments `unhandled_messages` and returns false.
pub fn parse_strict(msg: &str, gps: &mut GpsData) -> bool {
    let reg = NmeaRegistry::with_default_messages();
    reg.parse_strict(msg, gps)
}

/// Byte-stream NMEA framer — the host-testable replacement for the C
/// `UART0_IRQHandler` state machine. Uses a fixed stack buffer (zero heap allocation)
/// coupled with a configurable [`NmeaRegistry`].
#[derive(Debug)]
pub struct GpsReceiver {
    buf: [u8; MAX_LEN_NMEA],
    len: usize,
    data: GpsData,
    registry: NmeaRegistry,
}

impl Default for GpsReceiver {
    fn default() -> Self {
        Self {
            buf: [0u8; MAX_LEN_NMEA],
            len: 0,
            data: GpsData::default(),
            registry: NmeaRegistry::with_default_messages(),
        }
    }
}

impl GpsReceiver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a receiver configured according to [`crate::hal::GpsConfig`].
    pub fn with_config(_config: &crate::hal::GpsConfig) -> Self {
        Self::default()
    }

    /// Create a receiver with a custom sentence registry.
    pub fn with_registry(registry: NmeaRegistry) -> Self {
        Self {
            buf: [0u8; MAX_LEN_NMEA],
            len: 0,
            data: GpsData::default(),
            registry,
        }
    }

    /// Drain all available bytes from a [`crate::hal::UartRead`] source and feed them into the receiver.
    ///
    /// Returns the number of bytes processed.
    pub fn poll<U: crate::hal::UartRead>(&mut self, uart: &mut U) -> usize {
        let mut count = 0;
        while let Some(byte) = uart.read_byte() {
            self.feed_byte(byte);
            count += 1;
        }
        count
    }

    /// Access the sentence registry.
    pub fn registry(&self) -> &NmeaRegistry {
        &self.registry
    }

    /// Mutably access the sentence registry to register or override message handlers.
    pub fn registry_mut(&mut self) -> &mut NmeaRegistry {
        &mut self.registry
    }

    /// Register an additional sentence handler directly on the receiver.
    pub fn register_message<M: NmeaMessage + 'static>(&mut self, msg: M) {
        self.registry.register(msg);
    }

    /// Access the most recently decoded GPS state.
    pub fn data(&self) -> &GpsData {
        &self.data
    }

    /// Feed one received byte. Returns `true` when a sentence just completed
    /// (and was parsed).
    pub fn feed_byte(&mut self, byte: u8) -> bool {
        // Start collecting only at '$'; a new '$' restarts the buffer.
        if byte == b'$' {
            self.buf[0] = byte;
            self.len = 1;
            return false;
        }
        if self.len == 0 {
            return false; // waiting for a start delimiter
        }

        // Overflow guard mirrors MAX_LEN_NMEA in the C ISR.
        if self.len >= MAX_LEN_NMEA {
            self.len = 0;
            return false;
        }

        self.buf[self.len] = byte;
        self.len += 1;

        // Complete on CR LF.
        if self.len >= 2 && self.buf[self.len - 2] == b'\r' && self.buf[self.len - 1] == b'\n' {
            if let Ok(sentence) = core::str::from_utf8(&self.buf[..self.len]) {
                self.registry.parse(sentence, &mut self.data);
            }
            self.len = 0;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[test]
    fn parses_gpgga() {
        let mut gps = GpsData::default();
        parse(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,,*47",
            &mut gps,
        );
        assert_eq!(
            gps.time,
            GpsTime {
                hour: 12,
                minute: 35,
                second: 19
            }
        );
        assert_eq!(
            gps.coords,
            Coordinates::new("4807.038", 'N', "01131.000", 'E')
        );
        assert_eq!(gps.altitude, 545);
    }

    #[test]
    fn corrupt_checksum_sentence_is_rejected_by_parse_strict() {
        let mut gps = GpsData::default();
        let ok = parse_strict(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*99",
            &mut gps,
        );
        assert!(!ok);
        assert_eq!(gps.unhandled_messages, 1);
        assert_eq!(gps.coords.latitude, "");

        let ok = parse_strict(
            "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
            &mut gps,
        );
        assert!(ok);
        assert_eq!(gps.coords.latitude, "4807.038");
    }

    #[test]
    fn parses_gpzda() {
        let mut gps = GpsData::default();
        parse("$GPZDA,160012,07,11,2024,,*", &mut gps);
        assert_eq!(
            gps.time,
            GpsTime {
                hour: 16,
                minute: 0,
                second: 12
            }
        );
        assert_eq!(
            gps.date,
            GpsDate {
                year: 2024,
                month: 11,
                day: 7
            }
        );
    }

    #[test]
    fn parses_gprmc_with_speed_and_date() {
        let mut gps = GpsData::default();
        parse(
            "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,*",
            &mut gps,
        );
        assert_eq!(
            gps.time,
            GpsTime {
                hour: 12,
                minute: 35,
                second: 19
            }
        );
        assert_eq!(
            gps.coords,
            Coordinates::new("4807.038", 'N', "01131.000", 'E')
        );
        // 22.4 knots -> km/h
        assert!((gps.speed - 22.4 * 1.852).abs() < 0.01);
        // The C code does `year + 2000`, so a two-digit "94" becomes 2094.
        assert_eq!(
            gps.date,
            GpsDate {
                year: 2094,
                month: 3,
                day: 23
            }
        );
    }

    #[test]
    fn parses_gpvtg_speed() {
        let mut gps = GpsData::default();
        parse("$GPVTG,,,,5.5*48", &mut gps);
        assert!((gps.speed - 5.5).abs() < 0.001);
    }

    #[test]
    fn parses_gpgll() {
        let mut gps = GpsData::default();
        parse("$GPGLL,4916.45,N,12311.12,W,225444,A*31", &mut gps);
        assert_eq!(
            gps.coords,
            Coordinates::new("4916.45", 'N', "12311.12", 'W')
        );
        assert_eq!(
            gps.time,
            GpsTime {
                hour: 22,
                minute: 54,
                second: 44
            }
        );
    }

    #[test]
    fn unknown_sentence_increments_counter() {
        let mut gps = GpsData::default();
        parse("$GPGSV,3,1,11,03,03,111,00*4b", &mut gps);
        parse("$GPGSA,A,3,04,05,,09,12,,,*33", &mut gps);
        assert_eq!(gps.unhandled_messages, 2);
    }

    #[rstest]
    #[case(
        "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68",
        0x68
    )]
    #[case(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
        0x47
    )]
    fn checksum_matches(#[case] sentence: &str, #[case] expected: u8) {
        assert_eq!(nmea_checksum(sentence), expected);
    }

    #[test]
    fn receiver_frames_and_parses() {
        let mut rx = GpsReceiver::new();
        let sentence = b"$GPZDA,160012,07,11,2024,,*\r\n";
        let mut completed = false;
        for &b in sentence {
            completed |= rx.feed_byte(b);
        }
        assert!(completed);
        assert_eq!(
            rx.data().date,
            GpsDate {
                year: 2024,
                month: 11,
                day: 7
            }
        );
    }

    #[test]
    fn receiver_restarts_on_new_dollar() {
        let mut rx = GpsReceiver::new();
        for &b in b"junk$GPZDA,160012,07,11,2024,,*\r\n" {
            rx.feed_byte(b);
        }
        assert_eq!(rx.data().date.year, 2024);
    }

    #[derive(Debug)]
    struct MockPgrme;

    impl NmeaMessage for MockPgrme {
        fn message_type(&self) -> &'static str {
            "PGRME"
        }

        fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
            let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
            if let Ok(alt) = get(1).split('.').next().unwrap_or("0").parse::<u32>() {
                gps.altitude = alt;
            }
            true
        }

        fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
            alloc::format!("PGRME,{}.0,M,2.0,M,3.0,M", gps.altitude)
        }
    }

    #[test]
    fn receiver_with_custom_registered_message() {
        let mut rx = GpsReceiver::new();
        rx.register_message(MockPgrme);

        let sentence = b"$PGRME,42.0,M,2.0,M,3.0,M*22\r\n";
        let mut completed = false;
        for &b in sentence {
            completed |= rx.feed_byte(b);
        }
        assert!(completed);
        assert_eq!(rx.data().altitude, 42);
    }

    #[test]
    fn receiver_poll_drains_uart_read() {
        let sentence = b"$GPZDA,160012,07,11,2024,,*\r\n";
        let mut mock_uart = crate::hal::MockUartRead::new();
        let mut idx = 0;
        mock_uart.expect_read_byte().returning(move || {
            if idx < sentence.len() {
                let b = sentence[idx];
                idx += 1;
                Some(b)
            } else {
                None
            }
        });

        let mut rx = GpsReceiver::with_config(&crate::hal::GpsConfig::default());
        let count = rx.poll(&mut mock_uart);
        assert_eq!(count, sentence.len());
        assert_eq!(rx.data().date.year, 2024);
    }
}
