//! NMEA-0183 protocol parsing, emission, and sentence registry.
//!
//! Ported and restructured from `GPS.c`. Provides:
//! - Position and time-of-day structures ([`Coordinates`], [`GpsTime`], [`GpsDate`])
//! - The extensible [`NmeaMessage`] trait with `parse` and `emit` methods
//! - Concrete handlers for standard sentences ([`Gpgga`], [`Gpzda`], [`Gprmc`], [`Gpvtg`], [`Gpgll`])
//! - [`NmeaRegistry`] for dynamic dispatch without modifying parser core
//! - NMEA checksum computation and validation functions

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use heapless::String as HString;

use crate::gps::GpsData;

/// Largest NMEA sentence we buffer (matches `MAX_LEN_NMEA` in `GPS.h`).
pub const MAX_LEN_NMEA: usize = 80;

/// Time-of-day from a GPS/NMEA fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpsTime {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl GpsTime {
    /// Create a new time-of-day.
    pub fn new(hour: u8, minute: u8, second: u8) -> Self {
        Self {
            hour,
            minute,
            second,
        }
    }

    /// Parse `hhmmss` from the leading 6 characters of an NMEA time field.
    pub fn from_nmea(field: &str) -> Option<Self> {
        let (hour, minute, second) = parse_hms(field)?;
        Some(Self {
            hour,
            minute,
            second,
        })
    }
}

/// Calendar date from a GPS/NMEA fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpsDate {
    pub year: u16,
    pub month: u8,
    pub day: u8,
}

impl GpsDate {
    /// Create a new date.
    pub fn new(year: u16, month: u8, day: u8) -> Self {
        Self { year, month, day }
    }

    /// Parse date from separate day, month, year strings (e.g. from `GPZDA`).
    pub fn from_dmy(day: &str, month: &str, year: &str) -> Self {
        Self {
            day: day.parse().unwrap_or(0),
            month: month.parse().unwrap_or(0),
            year: year.parse().unwrap_or(0),
        }
    }

    /// Parse 6-digit `ddmmyy` date format (e.g. from `GPRMC`).
    pub fn from_ddmmyy(date_str: &str) -> Option<Self> {
        let date = date_str.trim();
        if date.len() >= 6 && date.as_bytes()[..6].iter().all(u8::is_ascii_digit) {
            Some(Self {
                day: date[0..2].parse().ok()?,
                month: date[2..4].parse().ok()?,
                year: date[4..6].parse::<u16>().ok()? + 2000,
            })
        } else {
            None
        }
    }
}

/// Geographical coordinates (latitude and longitude with hemispheres).
///
/// Shared across NMEA sentences that report position (`GPGGA`, `GPRMC`, `GPGLL`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Coordinates {
    /// Latitude string (`szer_geogr`), stack-allocated via heapless.
    pub latitude: HString<16>,
    /// N/S hemisphere (`szer_polkola`), e.g. 'N' or 'S'.
    pub lat_hemisphere: char,
    /// Longitude string (`dl_geogr`), stack-allocated via heapless.
    pub longitude: HString<16>,
    /// E/W hemisphere (`dl_polkola`), e.g. 'E' or 'W'.
    pub lon_hemisphere: char,
}

impl Coordinates {
    /// Create coordinates from string slices and hemisphere characters.
    pub fn new(lat: &str, lat_h: char, lon: &str, lon_h: char) -> Self {
        Self {
            latitude: to_hstring(lat),
            lat_hemisphere: lat_h,
            longitude: to_hstring(lon),
            lon_hemisphere: lon_h,
        }
    }

    /// Parse and construct coordinates from raw NMEA field strings.
    pub fn from_nmea(lat: &str, lat_h: &str, lon: &str, lon_h: &str) -> Self {
        Self {
            latitude: to_hstring(lat),
            lat_hemisphere: first_char(lat_h),
            longitude: to_hstring(lon),
            lon_hemisphere: first_char(lon_h),
        }
    }
}

pub(crate) fn to_hstring(s: &str) -> HString<16> {
    HString::try_from(s).unwrap_or_default()
}

/// Parse `hhmmss` from the leading 6 characters of a time field.
fn parse_hms(field: &str) -> Option<(u8, u8, u8)> {
    let digits: &str = field.split('.').next().unwrap_or(field);
    if digits.len() < 6 || !digits.as_bytes()[..6].iter().all(u8::is_ascii_digit) {
        return None;
    }
    let h = digits[0..2].parse().ok()?;
    let m = digits[2..4].parse().ok()?;
    let s = digits[4..6].parse().ok()?;
    Some((h, m, s))
}

pub(crate) fn first_char(field: &str) -> char {
    field.chars().next().unwrap_or('\0')
}

/// Check whether a sentence contains a valid NMEA checksum suffix `*hh`.
///
/// Returns `true` if the checksum is present and valid, `false` if missing or invalid.
pub fn verify_checksum(msg: &str) -> bool {
    let msg = msg.trim_end_matches(['\r', '\n']);
    if let Some((sentence_part, hex_str)) = msg.split_once('*') {
        let hex_str = hex_str.trim();
        if hex_str.len() == 2 {
            if let Ok(expected) = u8::from_str_radix(hex_str, 16) {
                return nmea_checksum(sentence_part) == expected;
            }
        }
    }
    false
}

/// Compute the NMEA XOR checksum of a sentence (everything between `$` and `*`).
///
/// Port of `nmea_checksum()`.
pub fn nmea_checksum(sentence: &str) -> u8 {
    let bytes = sentence.as_bytes();
    let mut checksum: u8 = 0;
    // C starts at index 1 (after '$') and XORs until '*'.
    let start = if bytes.first() == Some(&b'$') { 1 } else { 0 };
    for &b in &bytes[start..] {
        if b == b'*' {
            break;
        }
        checksum ^= b;
    }
    checksum
}

/// Trait for NMEA sentence handlers capable of parsing and emitting sentences.
///
/// Implements TODO 2: decoupling sentence definitions so new message types can be
/// registered dynamically in an [`NmeaRegistry`] without modifying the core parser.
pub trait NmeaMessage: core::fmt::Debug {
    /// 5-character sentence identifier, e.g. `"GPGGA"`.
    fn message_type(&self) -> &'static str;

    /// Parse comma-separated fields of the sentence into `gps`.
    /// `fields[0]` is the sentence identifier (e.g. `"GPGGA"`), `fields[1..]` are the payload fields.
    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool;

    /// Emit the comma-separated payload for this message from `gps` (without leading '$' or checksum).
    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String;

    /// Emit a full formatted NMEA sentence with leading '$', checksum '*hh', and trailing "\r\n".
    fn emit(&self, gps: &GpsData) -> alloc::string::String {
        let payload = self.emit_payload(gps);
        let cks = nmea_checksum(&payload);
        alloc::format!("${payload}*{cks:02X}\r\n")
    }
}

/// `GPGGA` message handler (Global Positioning System Fix Data).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gpgga;

impl NmeaMessage for Gpgga {
    fn message_type(&self) -> &'static str {
        "GPGGA"
    }

    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
        let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
        if let Some(t) = GpsTime::from_nmea(get(1)) {
            gps.time = t;
        }
        gps.coords = Coordinates::from_nmea(get(2), get(3), get(4), get(5));
        let alt_str = get(9).trim();
        if let Some(int_part) = alt_str.split('.').next() {
            gps.altitude = int_part.parse().unwrap_or(0);
        }
        true
    }

    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
        alloc::format!(
            "GPGGA,{:02}{:02}{:02},{},{},{},{},1,08,0.9,{},M,,",
            gps.time.hour,
            gps.time.minute,
            gps.time.second,
            gps.coords.latitude,
            gps.coords.lat_hemisphere,
            gps.coords.longitude,
            gps.coords.lon_hemisphere,
            gps.altitude
        )
    }
}

/// `GPZDA` message handler (Time & Date - UTC, day, month, year, and local time zone).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gpzda;

impl NmeaMessage for Gpzda {
    fn message_type(&self) -> &'static str {
        "GPZDA"
    }

    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
        let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
        if let Some(t) = GpsTime::from_nmea(get(1)) {
            gps.time = t;
        }
        gps.date = GpsDate::from_dmy(get(2), get(3), get(4));
        true
    }

    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
        alloc::format!(
            "GPZDA,{:02}{:02}{:02},{:02},{:02},{:04},,",
            gps.time.hour,
            gps.time.minute,
            gps.time.second,
            gps.date.day,
            gps.date.month,
            gps.date.year
        )
    }
}

/// `GPRMC` message handler (Recommended Minimum Specific GPS/TRANSIT Data).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gprmc;

impl NmeaMessage for Gprmc {
    fn message_type(&self) -> &'static str {
        "GPRMC"
    }

    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
        let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
        if let Some(t) = GpsTime::from_nmea(get(1)) {
            gps.time = t;
        }
        gps.coords = Coordinates::from_nmea(get(3), get(4), get(5), get(6));
        let knots: f32 = get(7).trim().parse().unwrap_or(0.0);
        gps.speed = knots * 1.852; // knots -> km/h
        if let Some(d) = GpsDate::from_ddmmyy(get(9)) {
            gps.date = d;
        }
        true
    }

    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
        let knots = if gps.speed > 0.0 {
            gps.speed / 1.852
        } else {
            0.0
        };
        let yr = gps.date.year % 100;
        alloc::format!(
            "GPRMC,{:02}{:02}{:02},A,{},{},{},{},{:05.1},000.0,{:02}{:02}{:02},,,",
            gps.time.hour,
            gps.time.minute,
            gps.time.second,
            gps.coords.latitude,
            gps.coords.lat_hemisphere,
            gps.coords.longitude,
            gps.coords.lon_hemisphere,
            knots,
            gps.date.day,
            gps.date.month,
            yr
        )
    }
}

/// `GPVTG` message handler (Course Over Ground and Ground Speed).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gpvtg;

impl NmeaMessage for Gpvtg {
    fn message_type(&self) -> &'static str {
        "GPVTG"
    }

    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
        let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
        gps.speed = get(4).trim().parse().unwrap_or(0.0);
        true
    }

    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
        alloc::format!("GPVTG,,,,{:.1},K", gps.speed)
    }
}

/// `GPGLL` message handler (Geographic Position - Latitude/Longitude).
#[derive(Debug, Clone, Copy, Default)]
pub struct Gpgll;

impl NmeaMessage for Gpgll {
    fn message_type(&self) -> &'static str {
        "GPGLL"
    }

    fn parse(&self, fields: &[&str], gps: &mut GpsData) -> bool {
        let get = |i: usize| -> &str { fields.get(i).copied().unwrap_or("") };
        gps.coords = Coordinates::from_nmea(get(1), get(2), get(3), get(4));
        if let Some(t) = GpsTime::from_nmea(get(5)) {
            gps.time = t;
        }
        true
    }

    fn emit_payload(&self, gps: &GpsData) -> alloc::string::String {
        alloc::format!(
            "GPGLL,{},{},{},{},{:02}{:02}{:02},A",
            gps.coords.latitude,
            gps.coords.lat_hemisphere,
            gps.coords.longitude,
            gps.coords.lon_hemisphere,
            gps.time.hour,
            gps.time.minute,
            gps.time.second
        )
    }
}

/// Registry mapping NMEA sentence identifiers to boxed [`NmeaMessage`] trait implementations.
///
/// Implements TODO 2: allows registering new sentence types without modifying the core parser.
#[derive(Debug)]
pub struct NmeaRegistry {
    handlers: BTreeMap<&'static str, Box<dyn NmeaMessage>>,
}

impl Default for NmeaRegistry {
    fn default() -> Self {
        Self::with_default_messages()
    }
}

impl NmeaRegistry {
    /// Create an empty registry with no handlers registered.
    pub fn new() -> Self {
        Self {
            handlers: BTreeMap::new(),
        }
    }

    /// Create a registry pre-loaded with the 5 firmware sentence types:
    /// `GPGGA`, `GPZDA`, `GPRMC`, `GPVTG`, `GPGLL`.
    pub fn with_default_messages() -> Self {
        let mut reg = Self::new();
        reg.register(Gpgga);
        reg.register(Gpzda);
        reg.register(Gprmc);
        reg.register(Gpvtg);
        reg.register(Gpgll);
        reg
    }

    /// Register a message handler implementing [`NmeaMessage`].
    pub fn register<M: NmeaMessage + 'static>(&mut self, msg: M) {
        self.handlers.insert(msg.message_type(), Box::new(msg));
    }

    /// Register a boxed message handler implementing [`NmeaMessage`].
    pub fn register_boxed(&mut self, msg: Box<dyn NmeaMessage>) {
        self.handlers.insert(msg.message_type(), msg);
    }

    /// Retrieve a reference to a registered handler.
    pub fn get(&self, msg_type: &str) -> Option<&dyn NmeaMessage> {
        self.handlers.get(msg_type).map(|b| b.as_ref())
    }

    /// Number of registered message handlers.
    pub fn len(&self) -> usize {
        self.handlers.len()
    }

    /// Check if the registry has no handlers.
    pub fn is_empty(&self) -> bool {
        self.handlers.is_empty()
    }

    /// Check whether a handler for `msg_type` is registered.
    pub fn contains(&self, msg_type: &str) -> bool {
        self.handlers.contains_key(msg_type)
    }

    /// Parse an NMEA sentence using the registered handlers.
    ///
    /// Returns `true` if a registered handler successfully parsed the sentence.
    /// Unknown or unhandled sentences increment `gps.unhandled_messages` and return `false`.
    pub fn parse(&self, msg: &str, gps: &mut GpsData) -> bool {
        let msg = msg.trim_end_matches(['\r', '\n']);

        // Strip the checksum suffix `*hh` if present, and the leading `$`.
        let body = msg.strip_prefix('$').unwrap_or(msg);
        let body = body.split('*').next().unwrap_or(body);

        let mut fields = [""; 16];
        let mut num_fields = 0;
        for (i, field) in body.split(',').enumerate() {
            if i < 16 {
                fields[i] = field;
                num_fields = i + 1;
            }
        }

        let kind = if num_fields > 0 { fields[0] } else { "" };
        if let Some(handler) = self.handlers.get(kind) {
            if handler.parse(&fields[..num_fields], gps) {
                return true;
            }
        }

        gps.unhandled_messages += 1;
        false
    }

    /// Parse an NMEA sentence strictly, verifying its checksum first.
    pub fn parse_strict(&self, msg: &str, gps: &mut GpsData) -> bool {
        if !verify_checksum(msg) {
            gps.unhandled_messages += 1;
            return false;
        }
        self.parse(msg, gps)
    }

    /// Emit an NMEA sentence with checksum for `msg_type` from `gps`.
    pub fn emit(&self, msg_type: &str, gps: &GpsData) -> Option<alloc::string::String> {
        self.handlers.get(msg_type).map(|h| h.emit(gps))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn registry_supports_custom_message_type_without_modifying_core_parser() {
        let mut reg = NmeaRegistry::with_default_messages();
        assert_eq!(reg.len(), 5);
        assert!(!reg.contains("PGRME"));

        // Register new message type
        reg.register(MockPgrme);
        assert_eq!(reg.len(), 6);
        assert!(reg.contains("PGRME"));

        let mut gps = GpsData::default();
        let parsed = reg.parse("$PGRME,15.5,M,2.0,M,3.0,M*1F\r\n", &mut gps);
        assert!(parsed);
        assert_eq!(gps.altitude, 15);
        assert_eq!(gps.unhandled_messages, 0);

        // Emit custom sentence
        let emitted = reg.emit("PGRME", &gps).unwrap();
        assert!(emitted.starts_with("$PGRME,15.0,M,2.0,M,3.0,M*"));
        assert!(verify_checksum(&emitted));
    }

    #[test]
    fn registry_emit_and_parse_roundtrip_all_builtins() {
        let reg = NmeaRegistry::with_default_messages();

        let gps = GpsData {
            time: GpsTime::new(14, 30, 15),
            date: GpsDate::new(2025, 5, 20),
            coords: Coordinates::new("5214.123", 'N', "02101.456", 'E'),
            altitude: 120,
            speed: 55.56,
            unhandled_messages: 0,
        };

        // GPGGA emit -> parse roundtrip
        let gga_str = reg.emit("GPGGA", &gps).unwrap();
        assert!(verify_checksum(&gga_str));
        let mut parsed_gga = GpsData::default();
        assert!(reg.parse_strict(&gga_str, &mut parsed_gga));
        assert_eq!(parsed_gga.time, gps.time);
        assert_eq!(parsed_gga.coords, gps.coords);
        assert_eq!(parsed_gga.altitude, gps.altitude);

        // GPZDA emit -> parse roundtrip
        let zda_str = reg.emit("GPZDA", &gps).unwrap();
        assert!(verify_checksum(&zda_str));
        let mut parsed_zda = GpsData::default();
        assert!(reg.parse_strict(&zda_str, &mut parsed_zda));
        assert_eq!(parsed_zda.time, gps.time);
        assert_eq!(parsed_zda.date, gps.date);

        // GPGLL emit -> parse roundtrip
        let gll_str = reg.emit("GPGLL", &gps).unwrap();
        assert!(verify_checksum(&gll_str));
        let mut parsed_gll = GpsData::default();
        assert!(reg.parse_strict(&gll_str, &mut parsed_gll));
        assert_eq!(parsed_gll.coords, gps.coords);
        assert_eq!(parsed_gll.time, gps.time);

        // GPVTG emit -> parse roundtrip
        let vtg_str = reg.emit("GPVTG", &gps).unwrap();
        assert!(verify_checksum(&vtg_str));
        let mut parsed_vtg = GpsData::default();
        assert!(reg.parse_strict(&vtg_str, &mut parsed_vtg));
        assert!((parsed_vtg.speed - gps.speed).abs() < 0.2);

        // GPRMC emit -> parse roundtrip
        let rmc_str = reg.emit("GPRMC", &gps).unwrap();
        assert!(verify_checksum(&rmc_str));
        let mut parsed_rmc = GpsData::default();
        assert!(reg.parse_strict(&rmc_str, &mut parsed_rmc));
        assert_eq!(parsed_rmc.coords, gps.coords);
        assert_eq!(parsed_rmc.time, gps.time);
        assert_eq!(parsed_rmc.date, gps.date);
    }

    #[test]
    fn coordinates_construction_and_accessors() {
        let coords = Coordinates::from_nmea("5214.123", "N", "02101.456", "E");
        assert_eq!(coords.latitude, "5214.123");
        assert_eq!(coords.lat_hemisphere, 'N');
        assert_eq!(coords.longitude, "02101.456");
        assert_eq!(coords.lon_hemisphere, 'E');
    }

    #[test]
    fn time_and_date_nmea_parsers() {
        let t = GpsTime::from_nmea("123456.78").unwrap();
        assert_eq!(t, GpsTime::new(12, 34, 56));
        assert!(GpsTime::from_nmea("12").is_none());

        let d = GpsDate::from_ddmmyy("251224").unwrap();
        assert_eq!(d, GpsDate::new(2024, 12, 25));
        assert!(GpsDate::from_ddmmyy("short").is_none());

        let d2 = GpsDate::from_dmy("15", "08", "2023");
        assert_eq!(d2, GpsDate::new(2023, 8, 15));
    }
}
