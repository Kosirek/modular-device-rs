//! Pure networking helpers ported from `emac.c`.
//!
//! These are byte-level utilities with no hardware dependency, so they are the
//! easiest things to verify exhaustively with table-driven `rstest` cases.

/// MAC address length in bytes.
pub const MAC_ADDRESS_LENGTH: usize = 6;
/// IPv4 address length in bytes.
pub const IPV4_ADDRESS_LENGTH: usize = 4;

/// Swap the two bytes of a 16-bit word (host <-> network order).
///
/// Port of `byte_sawp_word()`.
#[inline]
pub fn byte_swap_word(word: u16) -> u16 {
    word.rotate_left(8)
}

/// One's-complement 16-bit Internet checksum over `buf` (RFC 1071).
///
/// Handles odd-length buffers by padding with a zero octet on the right (as MSB).
/// Folds the 32-bit accumulated sum until no carry remains.
pub fn chksum16(buf: &[u8]) -> u16 {
    let mut sum: u32 = 0;

    let chunks = buf.chunks_exact(2);
    let remainder = chunks.remainder();

    for pair in chunks {
        let word = u16::from_be_bytes([pair[0], pair[1]]);
        sum += word as u32;
    }

    if let Some(&last_byte) = remainder.first() {
        sum += (last_byte as u32) << 8;
    }

    while (sum >> 16) != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    (!sum & 0xFFFF) as u16
}

/// `true` when two MAC addresses are byte-for-byte identical.
///
/// Port of `is_same_mac()`. Returns `false` if either slice is too short
/// (the C version dereferenced fixed-length arrays; here we guard the length).
pub fn is_same_mac(mac1: &[u8], mac2: &[u8]) -> bool {
    if mac1.len() < MAC_ADDRESS_LENGTH || mac2.len() < MAC_ADDRESS_LENGTH {
        return false;
    }
    mac1[..MAC_ADDRESS_LENGTH] == mac2[..MAC_ADDRESS_LENGTH]
}

/// `true` when two IPv4 addresses are byte-for-byte identical.
///
/// Port of `is_same_ip()`.
pub fn is_same_ip(ip1: &[u8], ip2: &[u8]) -> bool {
    if ip1.len() < IPV4_ADDRESS_LENGTH || ip2.len() < IPV4_ADDRESS_LENGTH {
        return false;
    }
    ip1[..IPV4_ADDRESS_LENGTH] == ip2[..IPV4_ADDRESS_LENGTH]
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    #[rstest]
    #[case(0x0001, 0x0100)]
    #[case(0x1234, 0x3412)]
    #[case(0xABCD, 0xCDAB)]
    #[case(0xFF00, 0x00FF)]
    #[case(0x0000, 0x0000)]
    fn swaps_word_bytes(#[case] input: u16, #[case] expected: u16) {
        assert_eq!(byte_swap_word(input), expected);
        // Swapping twice is the identity.
        assert_eq!(byte_swap_word(byte_swap_word(input)), input);
    }

    #[test]
    fn checksum_of_empty_is_all_ones() {
        assert_eq!(chksum16(&[]), 0xFFFF);
    }

    #[rstest]
    // A well-known IPv4 header checksum example (RFC 1071 style).
    #[case(&[0x45, 0x00, 0x00, 0x73, 0x00, 0x00, 0x40, 0x00,
            0x40, 0x11, 0x00, 0x00, 0xc0, 0xa8, 0x00, 0x01,
            0xc0, 0xa8, 0x00, 0xc7], 0xb861)]
    fn checksum_matches_reference(#[case] data: &[u8], #[case] expected: u16) {
        assert_eq!(chksum16(data), expected);
    }

    #[test]
    fn checksum_validates_to_zero_complement() {
        // Inserting the computed checksum back into the data should make the
        // recomputed sum fold to 0x0000 (the standard verification property).
        let mut header = vec![
            0x45u8, 0x00, 0x00, 0x73, 0x00, 0x00, 0x40, 0x00, 0x40, 0x11, 0x00, 0x00, 0xc0, 0xa8,
            0x00, 0x01, 0xc0, 0xa8, 0x00, 0xc7,
        ];
        let cks = chksum16(&header);
        header[10] = (cks >> 8) as u8;
        header[11] = (cks & 0xff) as u8;
        assert_eq!(chksum16(&header), 0x0000);
    }

    #[test]
    fn checksum_handles_odd_length() {
        // RFC 1071: an odd buffer [.., b] is padded on the right with zero,
        // so [0x01, 0x02, 0x03] is equivalent to [0x01, 0x02, 0x03, 0x00].
        let odd = [0x01u8, 0x02, 0x03];
        let even = [0x01u8, 0x02, 0x03, 0x00];
        assert_eq!(chksum16(&odd), chksum16(&even));
    }

    #[rstest]
    #[case(&[1, 2, 3, 4, 5, 6], &[1, 2, 3, 4, 5, 6], true)]
    #[case(&[1, 2, 3, 4, 5, 6], &[1, 2, 3, 4, 5, 7], false)]
    #[case(&[0, 0, 0, 0, 0, 0], &[0, 0, 0, 0, 0, 0], true)]
    fn compares_mac(#[case] a: &[u8], #[case] b: &[u8], #[case] expected: bool) {
        assert_eq!(is_same_mac(a, b), expected);
    }

    #[test]
    fn short_mac_is_never_equal() {
        assert!(!is_same_mac(&[1, 2, 3], &[1, 2, 3]));
    }

    #[rstest]
    #[case(&[192, 168, 0, 1], &[192, 168, 0, 1], true)]
    #[case(&[192, 168, 0, 1], &[192, 168, 0, 2], false)]
    #[case(&[255, 255, 255, 255], &[255, 255, 255, 255], true)]
    fn compares_ip(#[case] a: &[u8], #[case] b: &[u8], #[case] expected: bool) {
        assert_eq!(is_same_ip(a, b), expected);
    }
}
