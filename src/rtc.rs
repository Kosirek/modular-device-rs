//! Real-time clock data model, ported from `rtc.c` / `rtc.h`.
//!
//! On the LPC17xx the current time is read from three "consolidated time"
//! registers (`CTIME0..2`) that pack the fields as bitfields. We keep the exact
//! same decoding here as a *pure function* of those three register words, which
//! makes it trivial to unit-test without any RTC hardware.

/// Calendar/clock value. Mirrors the C `time_struct`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TimeStruct {
    pub year: u16,
    pub day_of_year: u16,
    pub month: u8,
    pub day_of_month: u8,
    pub day_of_week: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl TimeStruct {
    /// Decode the three LPC17xx consolidated-time registers into a `TimeStruct`.
    ///
    /// Bit layout matches `get_rtc_time()`:
    /// * `ctime0`: `[26:24]=DOW`, `[20:16]=HOUR`, `[13:8]=MIN`, `[5:0]=SEC`
    /// * `ctime1`: `[27:16]=YEAR`, `[11:8]=MONTH`, `[4:0]=DOM`
    /// * `ctime2`: `[11:0]=DOY`
    pub fn from_ctime(ctime0: u32, ctime1: u32, ctime2: u32) -> Self {
        Self {
            year: ((ctime1 >> 16) & 0xFFF) as u16,
            month: ((ctime1 >> 8) & 0xF) as u8,
            day_of_month: (ctime1 & 0x1F) as u8,
            day_of_week: ((ctime0 >> 24) & 0x7) as u8,
            hour: ((ctime0 >> 16) & 0x1F) as u8,
            minute: ((ctime0 >> 8) & 0x3F) as u8,
            second: (ctime0 & 0x3F) as u8,
            day_of_year: (ctime2 & 0xFFF) as u16,
        }
    }

    /// Pack into the FatFs `get_fattime()` 32-bit format:
    /// `bit31:25 year-1980 | bit24:21 month | bit20:16 day |
    ///  bit15:11 hour | bit10:5 minute | bit4:0 second/2`.
    pub fn to_fattime(&self) -> u32 {
        let year = (self.year as u32).saturating_sub(1980);
        (year << 25)
            | ((self.month as u32) << 21)
            | ((self.day_of_month as u32) << 16)
            | ((self.hour as u32) << 11)
            | ((self.minute as u32) << 5)
            | ((self.second as u32) / 2)
    }

    /// `"YYYY-MM-DD HH:MM:SS"` formatting used by the logger.
    pub fn to_iso(&self) -> alloc::string::String {
        alloc::format!(
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
            self.year,
            self.month,
            self.day_of_month,
            self.hour,
            self.minute,
            self.second
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[test]
    fn decodes_known_ctime() {
        // DOW=6, HOUR=23, MIN=45, SEC=30
        let ctime0 = (6u32 << 24) | (23u32 << 16) | (45u32 << 8) | 30;
        // YEAR=2024, MONTH=11, DOM=7
        let ctime1 = (2024u32 << 16) | (11u32 << 8) | 7;
        let ctime2 = 312u32; // day-of-year

        let t = TimeStruct::from_ctime(ctime0, ctime1, ctime2);
        assert_eq!(t.day_of_week, 6);
        assert_eq!(t.hour, 23);
        assert_eq!(t.minute, 45);
        assert_eq!(t.second, 30);
        assert_eq!(t.year, 2024);
        assert_eq!(t.month, 11);
        assert_eq!(t.day_of_month, 7);
        assert_eq!(t.day_of_year, 312);
    }

    #[rstest]
    #[case(0x3F, 0)] // SEC field is masked to 6 bits -> 63 -> but max valid 59; mask only
    fn masks_seconds_field(#[case] raw_sec: u32, #[case] _ignored: u32) {
        let t = TimeStruct::from_ctime(raw_sec & 0x3F, 0, 0);
        assert_eq!(t.second, (raw_sec & 0x3F) as u8);
    }

    #[test]
    fn fattime_packs_fields() {
        let t = TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 7,
            hour: 23,
            minute: 45,
            second: 30,
            ..Default::default()
        };
        let f = t.to_fattime();
        assert_eq!((f >> 25) & 0x7F, 2024 - 1980);
        assert_eq!((f >> 21) & 0x0F, 11);
        assert_eq!((f >> 16) & 0x1F, 7);
        assert_eq!((f >> 11) & 0x1F, 23);
        assert_eq!((f >> 5) & 0x3F, 45);
        assert_eq!(f & 0x1F, 30 / 2);
    }

    #[test]
    fn iso_string() {
        let t = TimeStruct {
            year: 2024,
            month: 1,
            day_of_month: 2,
            hour: 3,
            minute: 4,
            second: 5,
            ..Default::default()
        };
        assert_eq!(t.to_iso(), "2024-01-02 03:04:05");
    }
}
