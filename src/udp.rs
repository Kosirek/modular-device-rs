//! UDP/IPv4 framing, ported from `udp.c` / `udp.h`.
//!
//! The C code overlays packed C structs on a byte buffer. Here we build and
//! inspect the wire bytes directly, which makes endianness explicit and the
//! result easy to assert in tests.

use crate::net::{chksum16, is_same_ip, is_same_mac, IPV4_ADDRESS_LENGTH, MAC_ADDRESS_LENGTH};

#[cfg(not(test))]
use alloc::{vec, vec::Vec};

/// EtherType for IPv4 (`UDP_FRAME_TYPE`).
pub const UDP_FRAME_TYPE: u16 = 0x0800;

/// Header lengths (packed, matching the C structs).
pub const ETHER_HDR_LEN: usize = 14;
pub const IP_HDR_LEN: usize = 20;
pub const UDP_HDR_LEN: usize = 8;
/// Combined Ethernet + IP + UDP header length (`HdrLen`).
pub const HDR_LEN: usize = ETHER_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN;

/// Maximum frame buffer (`PACKET_BUF_SIZE`).
pub const PACKET_BUF_SIZE: usize = 800;
/// Maximum UDP payload (`PACKET_DATA_SIZE`).
pub const PACKET_DATA_SIZE: usize = PACKET_BUF_SIZE - HDR_LEN;

const CR: u8 = 13;
const LF: u8 = 10;

/// Addressing needed to emit a frame (the C globals `local_mac`, `remote_mac`,
/// `LocalIP`, `RemoteIP`).
#[derive(Debug, Clone, Copy)]
pub struct Endpoint {
    pub local_mac: [u8; MAC_ADDRESS_LENGTH],
    pub remote_mac: [u8; MAC_ADDRESS_LENGTH],
    pub local_ip: [u8; IPV4_ADDRESS_LENGTH],
    pub remote_ip: [u8; IPV4_ADDRESS_LENGTH],
}

/// Write a complete Ethernet/IP/UDP frame carrying `data` followed by CR+LF into `out`.
///
/// Returns the number of bytes written, or `None` if data is invalid or `out` is too small.
pub fn build_udp_frame_into(
    data: &[u8],
    local_port: u16,
    remote_port: u16,
    ep: &Endpoint,
    packet_id: u16,
    out: &mut [u8],
) -> Option<usize> {
    if data.is_empty() || data.len() >= PACKET_DATA_SIZE - 2 {
        return None;
    }

    let payload_length = data.len() + 2; // + CR + LF
    let total_len = HDR_LEN + payload_length;
    if out.len() < total_len {
        return None;
    }

    // ---- Ethernet header ----
    out[0..6].copy_from_slice(&ep.remote_mac);
    out[6..12].copy_from_slice(&ep.local_mac);
    out[12..14].copy_from_slice(&UDP_FRAME_TYPE.to_be_bytes());

    // ---- IP header (offset 14) ----
    let ip = 14;
    let ip_total = (payload_length + IP_HDR_LEN + UDP_HDR_LEN) as u16;
    out[ip] = 0x45; // version 4, IHL 5
    out[ip + 1] = 0x00; // DSCP/ECN
    out[ip + 2..ip + 4].copy_from_slice(&ip_total.to_be_bytes());
    out[ip + 4..ip + 6].copy_from_slice(&packet_id.to_be_bytes()); // Identification
    out[ip + 6..ip + 8].copy_from_slice(&0x0000u16.to_be_bytes()); // Flags/Frag
    out[ip + 8] = 0x80; // TTL
    out[ip + 9] = 0x11; // Protocol = UDP
    out[ip + 10..ip + 12].copy_from_slice(&[0, 0]); // checksum placeholder
    out[ip + 12..ip + 16].copy_from_slice(&ep.local_ip);
    out[ip + 16..ip + 20].copy_from_slice(&ep.remote_ip);
    let ip_cksum = chksum16(&out[ip..ip + IP_HDR_LEN]);
    out[ip + 10..ip + 12].copy_from_slice(&ip_cksum.to_be_bytes());

    // ---- UDP header (offset 34) ----
    let udp = ip + IP_HDR_LEN;
    let udp_len = (payload_length + UDP_HDR_LEN) as u16;
    out[udp..udp + 2].copy_from_slice(&local_port.to_be_bytes());
    out[udp + 2..udp + 4].copy_from_slice(&remote_port.to_be_bytes());
    out[udp + 4..udp + 6].copy_from_slice(&udp_len.to_be_bytes());
    out[udp + 6..udp + 8].copy_from_slice(&[0, 0]); // udp checksum disabled

    // ---- Payload (offset 42) ----
    let pl = udp + UDP_HDR_LEN;
    out[pl..pl + data.len()].copy_from_slice(data);
    out[pl + data.len()] = CR;
    out[pl + data.len() + 1] = LF;

    Some(total_len)
}

/// Build a complete Ethernet/IP/UDP frame carrying `data` followed by CR+LF.
///
/// Port of `EmacSendUDP()`. Returns `None` on the same guards the C used
/// (empty / oversized payload).
pub fn build_udp_frame(
    data: &[u8],
    local_port: u16,
    remote_port: u16,
    ep: &Endpoint,
) -> Option<Vec<u8>> {
    if data.is_empty() || data.len() >= PACKET_DATA_SIZE - 2 {
        return None;
    }

    let payload_length = data.len() + 2;
    let total_len = HDR_LEN + payload_length;
    let mut frame = vec![0u8; total_len];
    build_udp_frame_into(data, local_port, remote_port, ep, 0xD8F4, &mut frame)?;
    Some(frame)
}

/// View over a received frame's header fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UdpView<'a> {
    pub dest_mac: &'a [u8],
    pub dest_ip: &'a [u8],
    pub src_port: u16,
    pub dest_port: u16,
    pub payload: &'a [u8],
}

/// Parse the headers of a received frame. Returns `None` if it is too short or invalid.
///
/// Verifies EtherType (IPv4), IP version, Protocol (UDP), and clips payload to UDP length,
/// ignoring any Ethernet padding bytes.
pub fn parse_frame(frame: &[u8]) -> Option<UdpView<'_>> {
    if frame.len() < HDR_LEN {
        return None;
    }
    // Verify EtherType == 0x0800 (IPv4)
    if u16::from_be_bytes([frame[12], frame[13]]) != UDP_FRAME_TYPE {
        return None;
    }
    // IP header check: IPv4 and IHL >= 20
    let ver = frame[14] >> 4;
    let ihl = (frame[14] & 0x0F) as usize * 4;
    if ver != 4 || ihl < IP_HDR_LEN {
        return None;
    }
    // Protocol must be UDP (0x11 = 17)
    if frame[23] != 0x11 {
        return None;
    }

    let udp = 14 + ihl;
    if frame.len() < udp + UDP_HDR_LEN {
        return None;
    }

    let dest_mac = &frame[0..6];
    let dest_ip = &frame[14 + 16..14 + 20];
    let src_port = u16::from_be_bytes([frame[udp], frame[udp + 1]]);
    let dest_port = u16::from_be_bytes([frame[udp + 2], frame[udp + 3]]);
    let udp_len = u16::from_be_bytes([frame[udp + 4], frame[udp + 5]]) as usize;

    if udp_len < UDP_HDR_LEN {
        return None;
    }
    let data_len = udp_len - UDP_HDR_LEN;
    let payload_start = udp + UDP_HDR_LEN;

    if frame.len() < payload_start + data_len {
        return None;
    }
    // Clip payload strictly to data_len (ignoring Ethernet padding to 60+ bytes)
    let payload = &frame[payload_start..payload_start + data_len];
    Some(UdpView {
        dest_mac,
        dest_ip,
        src_port,
        dest_port,
        payload,
    })
}

/// Accept a frame only if it is addressed to us and matches the port pair.
///
/// Port of `filter_pass()`.
pub fn filter_pass(frame: &[u8], local_mac: &[u8], local_port: u16, remote_port: u16) -> bool {
    match parse_frame(frame) {
        Some(v) => {
            is_same_mac(v.dest_mac, local_mac)
                && v.dest_port == local_port
                && v.src_port == remote_port
        }
        None => false,
    }
}

/// Decide whether an IP packet is destined for us using an explicit subnet mask.
pub fn is_for_us_with_mask(dest_ip: &[u8], local_ip: &[u8], subnet_mask: &[u8]) -> bool {
    let broadcast = [255u8, 255, 255, 255];
    if is_same_ip(dest_ip, &broadcast) || is_same_ip(dest_ip, local_ip) {
        return true;
    }
    if dest_ip.len() >= 4 && local_ip.len() >= 4 && subnet_mask.len() >= 4 {
        let mut subnet_bcast = [0u8; 4];
        for i in 0..4 {
            subnet_bcast[i] = local_ip[i] | !subnet_mask[i];
        }
        if is_same_ip(dest_ip, &subnet_bcast) {
            return true;
        }
    }
    false
}

/// Decide whether an IP packet is destined for us (unicast or one of the two
/// broadcast forms), as in `get_line_from_mac()`. Assumes default /24 mask.
pub fn is_for_us(dest_ip: &[u8], local_ip: &[u8]) -> bool {
    is_for_us_with_mask(dest_ip, local_ip, &[255, 255, 255, 0])
}

/// Trim a payload at the first `CR LF`, returning the inclusive length, exactly
/// like the scan loop in `get_line_from_mac()`. Returns the original length if
/// no terminator is found.
pub fn line_length(payload: &[u8]) -> usize {
    for i in 1..payload.len() {
        if payload[i - 1] == CR && payload[i] == LF {
            return i + 1;
        }
    }
    payload.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::rstest;

    fn ep() -> Endpoint {
        Endpoint {
            local_mac: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            remote_mac: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
            local_ip: [192, 168, 2, 101],
            remote_ip: [192, 168, 2, 255],
        }
    }

    #[test]
    fn builds_well_formed_frame() {
        let frame = build_udp_frame(b"HELLO", 5000, 6000, &ep()).unwrap();
        // total = 42 header + 5 data + 2 (CR/LF)
        assert_eq!(frame.len(), HDR_LEN + 5 + 2);

        let v = parse_frame(&frame).unwrap();
        assert_eq!(v.dest_mac, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        assert_eq!(v.dest_ip, &[192, 168, 2, 255]);
        assert_eq!(v.src_port, 5000);
        assert_eq!(v.dest_port, 6000);
        assert_eq!(&v.payload[..5], b"HELLO");
        assert_eq!(&v.payload[5..], &[CR, LF]);
    }

    #[test]
    fn ethertype_is_ipv4() {
        let frame = build_udp_frame(b"x", 1, 2, &ep()).unwrap();
        assert_eq!(&frame[12..14], &[0x08, 0x00]);
    }

    #[test]
    fn ip_checksum_is_valid() {
        let frame = build_udp_frame(b"data", 1234, 4321, &ep()).unwrap();
        // Recomputing over the IP header (incl. checksum) must fold to zero.
        assert_eq!(chksum16(&frame[14..14 + IP_HDR_LEN]), 0x0000);
    }

    #[rstest]
    #[case(&[], false)]
    fn rejects_empty_payload(#[case] data: &[u8], #[case] _x: bool) {
        assert!(build_udp_frame(data, 1, 2, &ep()).is_none());
    }

    #[test]
    fn filter_matches_addressed_frame() {
        let frame = build_udp_frame(b"hi", 7, 9, &ep()).unwrap();
        // For a frame built for the remote, dest_mac is the remote's; emulate a
        // received frame addressed back to us by swapping addresses.
        let mut rx = frame.clone();
        rx[0..6].copy_from_slice(&ep().local_mac); // dest = us
                                                   // The frame carries src_port=7, dest_port=9. A receiver treats dest_port
                                                   // as its local port and src_port as the remote's, so this matches
                                                   // local_port=9, remote_port=7.
        assert!(filter_pass(&rx, &ep().local_mac, 9, 7));
        assert!(!filter_pass(&rx, &ep().local_mac, 9, 1234));
    }

    #[rstest]
    #[case(&[192, 168, 2, 101], true)] // unicast
    #[case(&[255, 255, 255, 255], true)] // global broadcast
    #[case(&[192, 168, 2, 255], true)] // subnet broadcast
    #[case(&[10, 0, 0, 1], false)] // someone else
    fn is_for_us_cases(#[case] dst: &[u8], #[case] expected: bool) {
        assert_eq!(is_for_us(dst, &[192, 168, 2, 101]), expected);
    }

    #[rstest]
    #[case(b"ab\r\nc", 4)]
    #[case(b"xyz", 3)] // no terminator -> full length
    #[case(&[CR, LF], 2)]
    fn line_length_cases(#[case] payload: &[u8], #[case] expected: usize) {
        assert_eq!(line_length(payload), expected);
    }

    #[test]
    fn parse_frame_ignores_ethernet_padding() {
        let frame = build_udp_frame(b"SHORT", 1000, 2000, &ep()).unwrap();
        // Frame is HDR_LEN(42) + 5 + 2 = 49 bytes.
        // Ethernet standard pads frames to 60 bytes.
        let mut padded = frame.clone();
        padded.resize(60, 0xAA); // trailing padding bytes

        let v = parse_frame(&padded).expect("must parse padded frame");
        assert_eq!(v.payload, b"SHORT\r\n"); // does NOT include 0xAA padding
    }

    #[test]
    fn is_for_us_with_custom_mask() {
        let local_ip = [10, 0, 1, 5];
        let mask = [255, 255, 0, 0]; // /16 -> broadcast 10.0.255.255
        assert!(is_for_us_with_mask(&[10, 0, 255, 255], &local_ip, &mask));
        assert!(!is_for_us_with_mask(&[10, 0, 1, 255], &local_ip, &mask));
    }

    #[test]
    fn build_udp_frame_into_works() {
        let mut buf = [0u8; 128];
        let len = build_udp_frame_into(b"HELLO", 5000, 6000, &ep(), 0x1234, &mut buf).unwrap();
        assert_eq!(len, HDR_LEN + 5 + 2);
        let v = parse_frame(&buf[..len]).unwrap();
        assert_eq!(v.payload, b"HELLO\r\n");
    }
}
