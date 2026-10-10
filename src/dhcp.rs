//! DHCP client (BOOTP), ported from `dhcp.c` / `dhcp.h`.
//!
//! The C client is a small state machine that builds DISCOVER/REQUEST frames,
//! parses OFFER/ACK/NAK replies, and drives retransmissions from the main loop.
//! Here the state machine is decoupled from the wire so the transitions and the
//! option TLV parsing can be tested without any network.

use crate::net::{chksum16, IPV4_ADDRESS_LENGTH, MAC_ADDRESS_LENGTH};

#[cfg(not(test))]
use alloc::{vec, vec::Vec};

pub const DHCP_SERVER_PORT: u16 = 67;
pub const DHCP_CLIENT_PORT: u16 = 68;
pub const IP_PROTO_UDP: u8 = 0x11;

const DHCP_OP_BOOTREQUEST: u8 = 1;
const DHCP_OP_BOOTREPLY: u8 = 2;
const DHCP_HTYPE_ETHERNET: u8 = 1;
const DHCP_HLEN_ETHERNET: u8 = 6;
const DHCP_FLAG_BROADCAST: u16 = 0x8000;
const DHCP_MAGIC_COOKIE: u32 = 0x6382_5363;

// Message types (option 53).
pub const DHCPDISCOVER: u8 = 1;
pub const DHCPOFFER: u8 = 2;
pub const DHCPREQUEST: u8 = 3;
pub const DHCPACK: u8 = 5;
pub const DHCPNAK: u8 = 6;

// Option codes.
const OPT_PAD: u8 = 0;
const OPT_SUBNET_MASK: u8 = 1;
const OPT_ROUTER: u8 = 3;
const OPT_DNS: u8 = 6;
const OPT_REQUESTED_IP: u8 = 50;
const OPT_MSG_TYPE: u8 = 53;
const OPT_SERVER_ID: u8 = 54;
const OPT_PARAM_REQ: u8 = 55;
const OPT_END: u8 = 255;

const ETHER_HDR_LEN: usize = 14;
const IP_HDR_LEN: usize = 20;
const UDP_HDR_LEN: usize = 8;
/// Fixed BOOTP part length (everything before `options`): op..magic_cookie.
const BOOTP_FIXED_LEN: usize = 1 + 1 + 1 + 1 + 4 + 2 + 2 + 4 + 4 + 4 + 4 + 16 + 64 + 128 + 4;

pub const DHCP_RETRY_TIMEOUT_MS: u64 = 2000;
pub const DHCP_TICKS_PER_MS: u64 = 10;
pub const DHCP_MAX_RETRIES: u8 = 4;

/// DHCP client state (`dhcp_state`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DhcpState {
    Init,
    Selecting,
    Requesting,
    Bound,
}

/// Lease info populated from an ACK (the C globals `dhcp_*`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DhcpLease {
    pub assigned_ip: [u8; IPV4_ADDRESS_LENGTH],
    pub subnet_mask: [u8; IPV4_ADDRESS_LENGTH],
    pub gateway: [u8; IPV4_ADDRESS_LENGTH],
    pub dns: [u8; IPV4_ADDRESS_LENGTH],
    pub server_ip: [u8; IPV4_ADDRESS_LENGTH],
}

/// Build the DHCP options block. Returns the bytes written.
///
/// Port of `dhcp_build_options()`.
fn build_options(msg_type: u8, lease: &DhcpLease, out: &mut Vec<u8>) {
    out.push(OPT_MSG_TYPE);
    out.push(1);
    out.push(msg_type);

    if msg_type == DHCPREQUEST {
        out.push(OPT_REQUESTED_IP);
        out.push(IPV4_ADDRESS_LENGTH as u8);
        out.extend_from_slice(&lease.assigned_ip);

        out.push(OPT_SERVER_ID);
        out.push(IPV4_ADDRESS_LENGTH as u8);
        out.extend_from_slice(&lease.server_ip);
    }

    out.push(OPT_PARAM_REQ);
    out.push(3);
    out.push(OPT_SUBNET_MASK);
    out.push(OPT_ROUTER);
    out.push(OPT_DNS);

    out.push(OPT_END);
}

/// Build a complete DHCP frame (Ethernet/IP/UDP/BOOTP) for `msg_type`.
///
/// Port of `dhcp_send()`. Always broadcast, source IP 0.0.0.0.
pub fn build_dhcp_frame(
    msg_type: u8,
    xid: u32,
    local_mac: &[u8; MAC_ADDRESS_LENGTH],
    lease: &DhcpLease,
) -> Vec<u8> {
    let mut options = Vec::new();
    build_options(msg_type, lease, &mut options);

    let dhcp_len = BOOTP_FIXED_LEN + options.len();
    let udp_len = UDP_HDR_LEN + dhcp_len;
    let ip_len = IP_HDR_LEN + udp_len;
    let total = ETHER_HDR_LEN + ip_len;

    let mut f = vec![0u8; total];

    // ---- Ethernet header ----
    f[0..6].copy_from_slice(&[0xff; 6]); // broadcast
    f[6..12].copy_from_slice(local_mac);
    f[12..14].copy_from_slice(&crate::udp::UDP_FRAME_TYPE.to_be_bytes());

    // ---- IP header ----
    let ip = ETHER_HDR_LEN;
    f[ip] = 0x45;
    f[ip + 1] = 0x00;
    f[ip + 2..ip + 4].copy_from_slice(&(ip_len as u16).to_be_bytes());
    f[ip + 4..ip + 6].copy_from_slice(&0u16.to_be_bytes()); // Identification
    f[ip + 6..ip + 8].copy_from_slice(&0u16.to_be_bytes());
    f[ip + 8] = 0x80; // TTL
    f[ip + 9] = IP_PROTO_UDP;
    f[ip + 12..ip + 16].copy_from_slice(&[0, 0, 0, 0]); // src 0.0.0.0
    f[ip + 16..ip + 20].copy_from_slice(&[255, 255, 255, 255]); // dst broadcast
    let cks = chksum16(&f[ip..ip + IP_HDR_LEN]);
    f[ip + 10..ip + 12].copy_from_slice(&cks.to_be_bytes());

    // ---- UDP header ----
    let udp = ip + IP_HDR_LEN;
    f[udp..udp + 2].copy_from_slice(&DHCP_CLIENT_PORT.to_be_bytes());
    f[udp + 2..udp + 4].copy_from_slice(&DHCP_SERVER_PORT.to_be_bytes());
    f[udp + 4..udp + 6].copy_from_slice(&(udp_len as u16).to_be_bytes());

    // ---- BOOTP/DHCP body ----
    let d = udp + UDP_HDR_LEN;
    f[d] = DHCP_OP_BOOTREQUEST;
    f[d + 1] = DHCP_HTYPE_ETHERNET;
    f[d + 2] = DHCP_HLEN_ETHERNET;
    f[d + 4..d + 8].copy_from_slice(&xid.to_le_bytes()); // xid kept opaque (matches C memcpy of native)
    f[d + 10..d + 12].copy_from_slice(&DHCP_FLAG_BROADCAST.to_be_bytes());
    // chaddr at offset 28 within BOOTP.
    let chaddr = d + 28;
    f[chaddr..chaddr + 6].copy_from_slice(local_mac);
    // magic cookie just before options: offset 28+16+64+128 = 236.
    let cookie = d + 236;
    f[cookie..cookie + 4].copy_from_slice(&DHCP_MAGIC_COOKIE.to_be_bytes());
    // options follow.
    let opt = d + 240;
    f[opt..opt + options.len()].copy_from_slice(&options);

    f
}

/// Walk a DHCP option TLV list, returning the message type (option 53) and
/// updating `lease` from any recognised options.
///
/// Port of `dhcp_parse_options()`.
pub fn parse_options(options: &[u8], lease: &mut DhcpLease) -> u8 {
    let mut pos = 0usize;
    let mut msg_type = 0u8;

    while pos < options.len() {
        let code = options[pos];
        pos += 1;

        if code == OPT_END {
            break;
        }
        if code == OPT_PAD {
            continue;
        }
        if pos >= options.len() {
            break;
        }

        let opt_len = options[pos] as usize;
        pos += 1;
        if pos + opt_len > options.len() {
            break;
        }
        let val = &options[pos..pos + opt_len];

        match code {
            OPT_MSG_TYPE if opt_len >= 1 => msg_type = val[0],
            OPT_SUBNET_MASK if opt_len >= 4 => lease.subnet_mask.copy_from_slice(&val[..4]),
            OPT_ROUTER if opt_len >= 4 => lease.gateway.copy_from_slice(&val[..4]),
            OPT_DNS if opt_len >= 4 => lease.dns.copy_from_slice(&val[..4]),
            OPT_SERVER_ID if opt_len >= 4 => lease.server_ip.copy_from_slice(&val[..4]),
            _ => {}
        }

        pos += opt_len;
    }

    msg_type
}

/// What the caller should do after `DhcpClient::input` / `tick`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DhcpAction {
    /// Nothing to transmit.
    None,
    /// Transmit this frame (DISCOVER or REQUEST).
    Send(Vec<u8>),
}

/// DHCP client state machine (C statics `state`, `transaction_id`, ...).
#[derive(Debug, Clone)]
pub struct DhcpClient {
    pub state: DhcpState,
    pub xid: u32,
    pub retries: u8,
    pub next_retry_at: u64,
    pub lease: DhcpLease,
    local_mac: [u8; MAC_ADDRESS_LENGTH],
}

impl DhcpClient {
    pub fn new(local_mac: [u8; MAC_ADDRESS_LENGTH]) -> Self {
        Self {
            state: DhcpState::Init,
            xid: 0,
            retries: 0,
            next_retry_at: 0,
            lease: DhcpLease::default(),
            local_mac,
        }
    }

    fn arm_retry(&mut self, now: u64) {
        self.next_retry_at = now + DHCP_RETRY_TIMEOUT_MS * DHCP_TICKS_PER_MS;
    }

    /// Begin a transaction by sending a DISCOVER. Port of `dhcp_start()`.
    pub fn start(&mut self, now: u64) -> DhcpAction {
        let m = &self.local_mac;
        self.xid =
            ((m[2] as u32) << 24) | ((m[3] as u32) << 16) | ((m[4] as u32) << 8) | m[5] as u32;
        self.xid ^= now as u32;

        self.lease.assigned_ip = [0; 4];
        self.lease.server_ip = [0; 4];
        self.retries = 0;
        self.state = DhcpState::Selecting;
        self.arm_retry(now);

        DhcpAction::Send(build_dhcp_frame(
            DHCPDISCOVER,
            self.xid,
            &self.local_mac,
            &self.lease,
        ))
    }

    /// `true` once a lease has been bound (`dhcp_is_bound()`).
    pub fn is_bound(&self) -> bool {
        self.state == DhcpState::Bound
    }

    /// Feed a received DHCP reply frame. Port of `dhcp_input()`.
    pub fn input(&mut self, frame: &[u8], now: u64) -> DhcpAction {
        let min = ETHER_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN + BOOTP_FIXED_LEN;
        if frame.len() < min {
            return DhcpAction::None;
        }

        let udp = ETHER_HDR_LEN + IP_HDR_LEN;
        let dest_port = u16::from_be_bytes([frame[udp + 2], frame[udp + 3]]);
        if dest_port != DHCP_CLIENT_PORT {
            return DhcpAction::None;
        }

        let d = udp + UDP_HDR_LEN;
        if frame[d] != DHCP_OP_BOOTREPLY {
            return DhcpAction::None;
        }
        let xid = u32::from_le_bytes([frame[d + 4], frame[d + 5], frame[d + 6], frame[d + 7]]);
        if xid != self.xid {
            return DhcpAction::None;
        }
        let cookie = u32::from_be_bytes([
            frame[d + 236],
            frame[d + 237],
            frame[d + 238],
            frame[d + 239],
        ]);
        if cookie != DHCP_MAGIC_COOKIE {
            return DhcpAction::None;
        }

        // yiaddr at BOOTP offset 16.
        let mut yiaddr = [0u8; 4];
        yiaddr.copy_from_slice(&frame[d + 16..d + 20]);

        let opt_start = d + 240;
        let msg_type = parse_options(&frame[opt_start..], &mut self.lease);

        match self.state {
            DhcpState::Selecting if msg_type == DHCPOFFER => {
                self.lease.assigned_ip = yiaddr;
                self.state = DhcpState::Requesting;
                self.retries = 0;
                self.arm_retry(now);
                DhcpAction::Send(build_dhcp_frame(
                    DHCPREQUEST,
                    self.xid,
                    &self.local_mac,
                    &self.lease,
                ))
            }
            DhcpState::Requesting if msg_type == DHCPACK => {
                self.state = DhcpState::Bound;
                DhcpAction::None
            }
            DhcpState::Requesting if msg_type == DHCPNAK => self.start(now),
            _ => DhcpAction::None,
        }
    }

    /// Drive retransmissions / timeouts. Port of `dhcp_tick()`.
    pub fn tick(&mut self, now: u64) -> DhcpAction {
        if self.state == DhcpState::Bound || self.state == DhcpState::Init {
            return DhcpAction::None;
        }
        if now < self.next_retry_at {
            return DhcpAction::None;
        }
        if self.retries >= DHCP_MAX_RETRIES {
            return self.start(now);
        }

        self.retries += 1;
        let msg_type = match self.state {
            DhcpState::Selecting => DHCPDISCOVER,
            DhcpState::Requesting => DHCPREQUEST,
            _ => return DhcpAction::None,
        };
        self.arm_retry(now);
        DhcpAction::Send(build_dhcp_frame(
            msg_type,
            self.xid,
            &self.local_mac,
            &self.lease,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAC: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];

    /// Build a server reply (BOOTREPLY) with the given message type and yiaddr.
    fn server_reply(xid: u32, msg_type: u8, yiaddr: [u8; 4], extra_opts: &[u8]) -> Vec<u8> {
        let total = ETHER_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN + BOOTP_FIXED_LEN + 64;
        let mut f = vec![0u8; total];
        let udp = ETHER_HDR_LEN + IP_HDR_LEN;
        // dest port = client
        f[udp + 2..udp + 4].copy_from_slice(&DHCP_CLIENT_PORT.to_be_bytes());
        let d = udp + UDP_HDR_LEN;
        f[d] = DHCP_OP_BOOTREPLY;
        f[d + 4..d + 8].copy_from_slice(&xid.to_le_bytes());
        f[d + 16..d + 20].copy_from_slice(&yiaddr);
        f[d + 236..d + 240].copy_from_slice(&DHCP_MAGIC_COOKIE.to_be_bytes());
        let opt = d + 240;
        let mut opts = vec![OPT_MSG_TYPE, 1, msg_type];
        opts.extend_from_slice(extra_opts);
        opts.push(OPT_END);
        f[opt..opt + opts.len()].copy_from_slice(&opts);
        f
    }

    #[test]
    fn discover_frame_is_well_formed() {
        let lease = DhcpLease::default();
        let f = build_dhcp_frame(DHCPDISCOVER, 0x1234, &MAC, &lease);
        // Broadcast destination MAC.
        assert_eq!(&f[0..6], &[0xff; 6]);
        // IPv4 ethertype.
        assert_eq!(&f[12..14], &[0x08, 0x00]);
        // UDP src=68 dst=67.
        let udp = ETHER_HDR_LEN + IP_HDR_LEN;
        assert_eq!(u16::from_be_bytes([f[udp], f[udp + 1]]), DHCP_CLIENT_PORT);
        assert_eq!(
            u16::from_be_bytes([f[udp + 2], f[udp + 3]]),
            DHCP_SERVER_PORT
        );
        // Magic cookie present.
        let d = udp + UDP_HDR_LEN;
        assert_eq!(
            u32::from_be_bytes([f[d + 236], f[d + 237], f[d + 238], f[d + 239]]),
            DHCP_MAGIC_COOKIE
        );
        // Broadcast flag set (0x8000 in network order).
        assert_eq!(&f[d + 10..d + 12], &[0x80, 0x00]);
        // IP checksum valid.
        assert_eq!(chksum16(&f[ETHER_HDR_LEN..ETHER_HDR_LEN + IP_HDR_LEN]), 0);
    }

    #[test]
    fn parse_options_extracts_fields() {
        let mut lease = DhcpLease::default();
        let opts = [
            OPT_MSG_TYPE,
            1,
            DHCPOFFER,
            OPT_SUBNET_MASK,
            4,
            255,
            255,
            255,
            0,
            OPT_ROUTER,
            4,
            192,
            168,
            2,
            1,
            OPT_DNS,
            4,
            8,
            8,
            8,
            8,
            OPT_SERVER_ID,
            4,
            192,
            168,
            2,
            1,
            OPT_END,
        ];
        let mt = parse_options(&opts, &mut lease);
        assert_eq!(mt, DHCPOFFER);
        assert_eq!(lease.subnet_mask, [255, 255, 255, 0]);
        assert_eq!(lease.gateway, [192, 168, 2, 1]);
        assert_eq!(lease.dns, [8, 8, 8, 8]);
        assert_eq!(lease.server_ip, [192, 168, 2, 1]);
    }

    #[test]
    fn parse_options_stops_on_malformed_length() {
        let mut lease = DhcpLease::default();
        // opt_len claims 4 bytes but only 1 is present after it.
        let opts = [OPT_SUBNET_MASK, 4, 255];
        let mt = parse_options(&opts, &mut lease);
        assert_eq!(mt, 0);
        assert_eq!(lease.subnet_mask, [0, 0, 0, 0]);
    }

    #[test]
    fn full_handshake_discover_offer_request_ack() {
        let mut c = DhcpClient::new(MAC);
        // DISCOVER
        let action = c.start(0);
        assert!(matches!(action, DhcpAction::Send(_)));
        assert_eq!(c.state, DhcpState::Selecting);

        // OFFER -> client should REQUEST
        let offer = server_reply(
            c.xid,
            DHCPOFFER,
            [192, 168, 2, 123],
            &[OPT_SERVER_ID, 4, 192, 168, 2, 1],
        );
        let action = c.input(&offer, 10);
        assert!(matches!(action, DhcpAction::Send(_)));
        assert_eq!(c.state, DhcpState::Requesting);
        assert_eq!(c.lease.assigned_ip, [192, 168, 2, 123]);

        // ACK -> bound
        let ack = server_reply(c.xid, DHCPACK, [192, 168, 2, 123], &[]);
        let action = c.input(&ack, 20);
        assert_eq!(action, DhcpAction::None);
        assert!(c.is_bound());
    }

    #[test]
    fn nak_restarts_transaction() {
        let mut c = DhcpClient::new(MAC);
        c.start(0);
        let offer = server_reply(c.xid, DHCPOFFER, [192, 168, 2, 5], &[]);
        c.input(&offer, 1);
        assert_eq!(c.state, DhcpState::Requesting);

        let old_xid = c.xid;
        let nak = server_reply(old_xid, DHCPNAK, [0, 0, 0, 0], &[]);
        let action = c.input(&nak, 2);
        // A fresh DISCOVER is sent and we are selecting again.
        assert!(matches!(action, DhcpAction::Send(_)));
        assert_eq!(c.state, DhcpState::Selecting);
    }

    #[test]
    fn reply_with_wrong_xid_is_ignored() {
        let mut c = DhcpClient::new(MAC);
        c.start(0);
        let offer = server_reply(c.xid ^ 0xFFFF, DHCPOFFER, [1, 2, 3, 4], &[]);
        assert_eq!(c.input(&offer, 1), DhcpAction::None);
        assert_eq!(c.state, DhcpState::Selecting);
    }

    #[test]
    fn tick_retransmits_until_max_then_restarts() {
        let mut c = DhcpClient::new(MAC);
        c.start(0);
        for _ in 0..DHCP_MAX_RETRIES {
            let now = c.next_retry_at;
            assert!(matches!(c.tick(now), DhcpAction::Send(_)));
        }
        assert_eq!(c.retries, DHCP_MAX_RETRIES);
        // Next tick exceeds max retries -> restart (new DISCOVER, retries reset).
        let now = c.next_retry_at;
        assert!(matches!(c.tick(now), DhcpAction::Send(_)));
        assert_eq!(c.retries, 0);
    }

    #[test]
    fn tick_does_nothing_before_timeout_or_when_bound() {
        let mut c = DhcpClient::new(MAC);
        c.start(100);
        assert_eq!(c.tick(0), DhcpAction::None); // before next_retry_at
        c.state = DhcpState::Bound;
        assert_eq!(c.tick(u64::MAX), DhcpAction::None);
    }
}
