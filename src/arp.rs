//! ARP request/reply handling, ported from `arp.c` / `arp.h`.

use crate::net::{is_same_ip, is_same_mac, IPV4_ADDRESS_LENGTH, MAC_ADDRESS_LENGTH};

/// EtherType for ARP (`ARP_FRAME_TYPE`).
pub const ARP_FRAME_TYPE: u16 = 0x0806;
/// ARP payload length (`ARP_LEN`).
pub const ARP_LEN: usize = 28;
const ETHER_HDR_LEN: usize = 14;
/// Total length of an Ethernet+ARP frame.
pub const ARP_FRAME_LEN: usize = ETHER_HDR_LEN + ARP_LEN;
/// Resolution timeout in timer ticks (`ARP_TIMEOUT`).
pub const ARP_TIMEOUT: u64 = 30_000;

/// ARP operation codes (`typ_arp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ArpOp {
    Request = 1,
    Reply = 2,
}

impl ArpOp {
    fn from_u16(v: u16) -> Option<Self> {
        match v {
            1 => Some(ArpOp::Request),
            2 => Some(ArpOp::Reply),
            _ => None,
        }
    }
}

/// Decoded ARP packet fields.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArpPacket {
    pub eth_src: [u8; MAC_ADDRESS_LENGTH],
    pub operation: u16,
    pub sender_mac: [u8; MAC_ADDRESS_LENGTH],
    pub sender_ip: [u8; IPV4_ADDRESS_LENGTH],
    pub target_mac: [u8; MAC_ADDRESS_LENGTH],
    pub target_ip: [u8; IPV4_ADDRESS_LENGTH],
}

/// Build an ARP frame (Ethernet header + 28-byte ARP payload).
///
/// Port of `send_arp()`. Returns a fixed-size array on the stack (zero heap allocation).
pub fn build_arp_frame(
    op: ArpOp,
    remote_ip: &[u8; IPV4_ADDRESS_LENGTH],
    remote_mac: &[u8; MAC_ADDRESS_LENGTH],
    local_ip: &[u8; IPV4_ADDRESS_LENGTH],
    local_mac: &[u8; MAC_ADDRESS_LENGTH],
) -> [u8; ARP_FRAME_LEN] {
    let mut f = [0u8; ARP_FRAME_LEN];

    // Ethernet header.
    f[0..6].copy_from_slice(remote_mac);
    f[6..12].copy_from_slice(local_mac);
    f[12..14].copy_from_slice(&ARP_FRAME_TYPE.to_be_bytes());

    // ARP payload.
    let a = ETHER_HDR_LEN;
    f[a..a + 2].copy_from_slice(&1u16.to_be_bytes()); // hdw_type = Ethernet
    f[a + 2..a + 4].copy_from_slice(&ARP_FRAME_TYPE.to_be_bytes()); // protocol_type
    f[a + 4] = MAC_ADDRESS_LENGTH as u8;
    f[a + 5] = IPV4_ADDRESS_LENGTH as u8;
    f[a + 6..a + 8].copy_from_slice(&(op as u16).to_be_bytes());
    f[a + 8..a + 14].copy_from_slice(local_mac); // sender_mac
    f[a + 14..a + 18].copy_from_slice(local_ip); // sender_ip
    f[a + 18..a + 24].copy_from_slice(remote_mac); // target_mac
    f[a + 24..a + 28].copy_from_slice(remote_ip); // target_ip

    f
}

/// Parse an ARP frame. Returns `None` if too short.
pub fn parse_arp(frame: &[u8]) -> Option<ArpPacket> {
    if frame.len() < ETHER_HDR_LEN + ARP_LEN {
        return None;
    }
    let a = ETHER_HDR_LEN;
    let mut p = ArpPacket {
        eth_src: [0; 6],
        operation: u16::from_be_bytes([frame[a + 6], frame[a + 7]]),
        sender_mac: [0; 6],
        sender_ip: [0; 4],
        target_mac: [0; 6],
        target_ip: [0; 4],
    };
    p.eth_src.copy_from_slice(&frame[6..12]);
    p.sender_mac.copy_from_slice(&frame[a + 8..a + 14]);
    p.sender_ip.copy_from_slice(&frame[a + 14..a + 18]);
    p.target_mac.copy_from_slice(&frame[a + 18..a + 24]);
    p.target_ip.copy_from_slice(&frame[a + 24..a + 28]);
    Some(p)
}

/// ARP request progress (`arp_req`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArpState {
    Idle,
    Waiting,
    Resolved,
}

/// Result of feeding a received ARP packet to the resolver.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArpEvent {
    /// Nothing to do.
    None,
    /// We should transmit this reply frame.
    SendReply([u8; ARP_FRAME_LEN]),
    /// The address we were waiting for is now resolved to this MAC.
    Resolved([u8; MAC_ADDRESS_LENGTH]),
}

/// Stateful ARP resolver (the C global `arp_sended` plus `remote_mac`).
#[derive(Debug, Clone)]
pub struct ArpResolver {
    pub state: ArpState,
    pub timeout: u64,
    pub ip_req: [u8; IPV4_ADDRESS_LENGTH],
    pub remote_mac: [u8; MAC_ADDRESS_LENGTH],
}

impl Default for ArpResolver {
    fn default() -> Self {
        Self {
            state: ArpState::Idle,
            timeout: 0,
            ip_req: [0; 4],
            remote_mac: [0xff; 6],
        }
    }
}

impl ArpResolver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Begin resolving `remote_ip`. Returns the request frame to send, or
    /// `None` if the parameters are rejected (target is a broadcast .255).
    ///
    /// Port of `send_arp_req()`.
    pub fn send_request(
        &mut self,
        remote_ip: &[u8; IPV4_ADDRESS_LENGTH],
        local_ip: &[u8; IPV4_ADDRESS_LENGTH],
        local_mac: &[u8; MAC_ADDRESS_LENGTH],
        now: u64,
    ) -> Option<[u8; ARP_FRAME_LEN]> {
        if remote_ip[3] == 255 {
            return None;
        }
        self.state = ArpState::Waiting;
        self.timeout = now + ARP_TIMEOUT;
        self.ip_req = *remote_ip;
        let broadcast = [0xffu8; MAC_ADDRESS_LENGTH];
        Some(build_arp_frame(
            ArpOp::Request,
            remote_ip,
            &broadcast,
            local_ip,
            local_mac,
        ))
    }

    /// Process a received ARP packet.
    ///
    /// Port of `check_arp()` combined with the request-handling in
    /// `send_arp_resp()`: an incoming request for our IP yields a reply, an
    /// incoming reply for our pending request resolves the MAC.
    pub fn handle(
        &mut self,
        pkt: &ArpPacket,
        local_ip: &[u8; IPV4_ADDRESS_LENGTH],
        local_mac: &[u8; MAC_ADDRESS_LENGTH],
    ) -> ArpEvent {
        match ArpOp::from_u16(pkt.operation) {
            Some(ArpOp::Request) => {
                if is_same_ip(&pkt.target_ip, local_ip) {
                    // Reply to the requester.
                    return ArpEvent::SendReply(build_arp_frame(
                        ArpOp::Reply,
                        &pkt.sender_ip,
                        &pkt.sender_mac,
                        local_ip,
                        local_mac,
                    ));
                }
                ArpEvent::None
            }
            Some(ArpOp::Reply) => {
                if self.state == ArpState::Waiting
                    && is_same_ip(&pkt.sender_ip, &self.ip_req)
                    && is_same_mac(&pkt.target_mac, local_mac)
                {
                    self.state = ArpState::Resolved;
                    self.remote_mac = pkt.sender_mac;
                    return ArpEvent::Resolved(pkt.sender_mac);
                }
                ArpEvent::None
            }
            None => ArpEvent::None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOCAL_IP: [u8; 4] = [192, 168, 2, 101];
    const LOCAL_MAC: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
    const PEER_IP: [u8; 4] = [192, 168, 2, 50];
    const PEER_MAC: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

    #[test]
    fn build_and_parse_roundtrip() {
        let frame = build_arp_frame(ArpOp::Request, &PEER_IP, &[0xff; 6], &LOCAL_IP, &LOCAL_MAC);
        assert_eq!(&frame[12..14], &[0x08, 0x06]); // ARP ethertype
        let p = parse_arp(&frame).unwrap();
        assert_eq!(p.operation, ArpOp::Request as u16);
        assert_eq!(p.sender_mac, LOCAL_MAC);
        assert_eq!(p.sender_ip, LOCAL_IP);
        assert_eq!(p.target_ip, PEER_IP);
    }

    #[test]
    fn rejects_broadcast_request_target() {
        let mut r = ArpResolver::new();
        assert!(r
            .send_request(&[192, 168, 2, 255], &LOCAL_IP, &LOCAL_MAC, 0)
            .is_none());
        assert_eq!(r.state, ArpState::Idle);
    }

    #[test]
    fn request_starts_waiting_with_timeout() {
        let mut r = ArpResolver::new();
        let frame = r
            .send_request(&PEER_IP, &LOCAL_IP, &LOCAL_MAC, 1000)
            .unwrap();
        assert_eq!(r.state, ArpState::Waiting);
        assert_eq!(r.timeout, 1000 + ARP_TIMEOUT);
        assert_eq!(r.ip_req, PEER_IP);
        // Sent to broadcast MAC.
        assert_eq!(&frame[0..6], &[0xff; 6]);
    }

    #[test]
    fn replies_to_request_for_our_ip() {
        let mut r = ArpResolver::new();
        let incoming = build_arp_frame(ArpOp::Request, &LOCAL_IP, &LOCAL_MAC, &PEER_IP, &PEER_MAC);
        let pkt = parse_arp(&incoming).unwrap();
        match r.handle(&pkt, &LOCAL_IP, &LOCAL_MAC) {
            ArpEvent::SendReply(reply) => {
                let rp = parse_arp(&reply).unwrap();
                assert_eq!(rp.operation, ArpOp::Reply as u16);
                assert_eq!(rp.target_ip, PEER_IP);
                assert_eq!(rp.sender_ip, LOCAL_IP);
            }
            other => panic!("expected SendReply, got {other:?}"),
        }
    }

    #[test]
    fn ignores_request_for_other_ip() {
        let mut r = ArpResolver::new();
        let incoming = build_arp_frame(
            ArpOp::Request,
            &[192, 168, 2, 200],
            &PEER_MAC,
            &PEER_IP,
            &PEER_MAC,
        );
        let pkt = parse_arp(&incoming).unwrap();
        assert_eq!(r.handle(&pkt, &LOCAL_IP, &LOCAL_MAC), ArpEvent::None);
    }

    #[test]
    fn reply_resolves_pending_request() {
        let mut r = ArpResolver::new();
        r.send_request(&PEER_IP, &LOCAL_IP, &LOCAL_MAC, 0).unwrap();
        // Peer replies: sender=peer, target=us.
        let reply = build_arp_frame(ArpOp::Reply, &LOCAL_IP, &LOCAL_MAC, &PEER_IP, &PEER_MAC);
        let pkt = parse_arp(&reply).unwrap();
        assert_eq!(
            r.handle(&pkt, &LOCAL_IP, &LOCAL_MAC),
            ArpEvent::Resolved(PEER_MAC)
        );
        assert_eq!(r.state, ArpState::Resolved);
        assert_eq!(r.remote_mac, PEER_MAC);
    }

    #[test]
    fn reply_for_other_request_is_ignored() {
        let mut r = ArpResolver::new();
        r.send_request(&PEER_IP, &LOCAL_IP, &LOCAL_MAC, 0).unwrap();
        // Reply from a different IP than requested.
        let reply = build_arp_frame(
            ArpOp::Reply,
            &LOCAL_IP,
            &LOCAL_MAC,
            &[10, 0, 0, 9],
            &PEER_MAC,
        );
        let pkt = parse_arp(&reply).unwrap();
        assert_eq!(r.handle(&pkt, &LOCAL_IP, &LOCAL_MAC), ArpEvent::None);
        assert_eq!(r.state, ArpState::Waiting);
    }
}
