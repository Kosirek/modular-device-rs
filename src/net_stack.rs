//! Network stack coordinator, combining EMAC, ARP, DHCP, and UDP.
//!
//! Port of the networking logic from `mainPrg.c` (the `ENET_IRQ` packet dispatcher
//! and the main-loop DHCP/ARP ticks). On hardware this wraps an
//! [`crate::hal::EthernetDevice`] and manages IP configuration, address resolution,
//! packet demultiplexing, and UDP transmission.

use crate::arp::{parse_arp, ArpEvent, ArpResolver, ArpState, ARP_FRAME_TYPE};
use crate::dhcp::{DhcpAction, DhcpClient, DhcpState, DHCP_CLIENT_PORT};
use crate::hal::{EthernetConfig, EthernetDevice, HalError, HalResult};
use crate::net::{is_same_ip, is_same_mac, IPV4_ADDRESS_LENGTH, MAC_ADDRESS_LENGTH};
use crate::udp::{build_udp_frame, parse_frame, Endpoint, PACKET_BUF_SIZE, UDP_FRAME_TYPE};

#[cfg(not(test))]
use alloc::vec::Vec;

/// A received UDP datagram.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedUdpPacket {
    /// IPv4 address of the sender.
    pub src_ip: [u8; IPV4_ADDRESS_LENGTH],
    /// Source UDP port.
    pub src_port: u16,
    /// Destination UDP port.
    pub dest_port: u16,
    /// Datagram payload (clipped to UDP length, excluding Ethernet padding).
    pub payload: Vec<u8>,
}

impl ReceivedUdpPacket {
    /// Return the payload with any trailing CR / LF delimiters trimmed.
    pub fn payload_trimmed(&self) -> &[u8] {
        let mut p = self.payload.as_slice();
        while p.ends_with(b"\n") || p.ends_with(b"\r") {
            p = &p[..p.len() - 1];
        }
        p
    }

    /// View trimmed payload as a UTF-8 string slice, if valid.
    pub fn as_str(&self) -> Result<&str, core::str::Utf8Error> {
        core::str::from_utf8(self.payload_trimmed())
    }
}

/// Cohesive network stack wrapping an [`EthernetDevice`].
pub struct NetworkStack<D: EthernetDevice> {
    device: D,
    config: EthernetConfig,
    current_ip: [u8; IPV4_ADDRESS_LENGTH],
    current_subnet: [u8; IPV4_ADDRESS_LENGTH],
    current_gateway: [u8; IPV4_ADDRESS_LENGTH],
    arp: ArpResolver,
    dhcp: DhcpClient,
    remote_ip: [u8; IPV4_ADDRESS_LENGTH],
    remote_mac: [u8; MAC_ADDRESS_LENGTH],
    incoming_udp: Vec<ReceivedUdpPacket>,
}

impl<D: EthernetDevice> NetworkStack<D> {
    /// Create a new network stack for the given Ethernet peripheral and configuration.
    pub fn new(device: D, config: EthernetConfig) -> Self {
        let current_ip = config.ip_address;
        let current_subnet = config.subnet_mask;
        let current_gateway = config.gateway;
        let local_mac = config.mac_address;

        // Default remote target is subnet broadcast (e.g. 192.168.2.255)
        let mut remote_ip = current_ip;
        remote_ip[3] = 255;
        let remote_mac = [0xFF; MAC_ADDRESS_LENGTH];

        Self {
            device,
            current_ip,
            current_subnet,
            current_gateway,
            arp: ArpResolver::new(),
            dhcp: DhcpClient::new(local_mac),
            remote_ip,
            remote_mac,
            incoming_udp: Vec::new(),
            config,
        }
    }

    /// Access the underlying Ethernet device.
    pub fn device(&self) -> &D {
        &self.device
    }

    /// Mutably access the underlying Ethernet device.
    pub fn device_mut(&mut self) -> &mut D {
        &mut self.device
    }

    /// Access the static configuration.
    pub fn config(&self) -> &EthernetConfig {
        &self.config
    }

    /// Current operational IPv4 address (assigned via DHCP or static fallback).
    pub fn current_ip(&self) -> [u8; IPV4_ADDRESS_LENGTH] {
        self.current_ip
    }

    /// Current operational subnet mask.
    pub fn subnet_mask(&self) -> [u8; IPV4_ADDRESS_LENGTH] {
        self.current_subnet
    }

    /// Current operational default gateway.
    pub fn gateway(&self) -> [u8; IPV4_ADDRESS_LENGTH] {
        self.current_gateway
    }

    /// `true` if DHCP has completed and a lease is active.
    pub fn is_dhcp_bound(&self) -> bool {
        self.dhcp.is_bound()
    }

    /// Current DHCP state.
    pub fn dhcp_state(&self) -> DhcpState {
        self.dhcp.state
    }

    /// Current ARP resolver state.
    pub fn arp_state(&self) -> ArpState {
        self.arp.state
    }

    /// Current target remote IP for telemetry / UDP emissions.
    pub fn remote_ip(&self) -> [u8; IPV4_ADDRESS_LENGTH] {
        self.remote_ip
    }

    /// Current target remote MAC for telemetry / UDP emissions.
    pub fn remote_mac(&self) -> [u8; MAC_ADDRESS_LENGTH] {
        self.remote_mac
    }

    /// Set the destination IPv4 address for default outgoing UDP frames.
    pub fn set_remote_ip(&mut self, ip: [u8; IPV4_ADDRESS_LENGTH]) {
        self.remote_ip = ip;
    }

    /// Set the destination MAC address for default outgoing UDP frames.
    pub fn set_remote_mac(&mut self, mac: [u8; MAC_ADDRESS_LENGTH]) {
        self.remote_mac = mac;
    }

    /// Start network services (e.g. initiate DHCP negotiation if enabled).
    pub fn start(&mut self, now_ticks: u64) -> HalResult<()> {
        if self.config.dhcp_enabled {
            if let DhcpAction::Send(packet) = self.dhcp.start(now_ticks) {
                self.device.transmit(&packet)?;
            }
        }
        Ok(())
    }

    /// Process incoming Ethernet packets and advance periodic timers (DHCP / ARP).
    ///
    /// Mirrors the dispatching in `ENET_IRQ` combined with the main-loop ticks.
    pub fn poll(&mut self, now_ticks: u64) -> HalResult<()> {
        let mut rx_buf = [0u8; PACKET_BUF_SIZE];
        while let Some(len) = self.device.receive(&mut rx_buf)? {
            if len < 14 {
                continue;
            }
            let frame = &rx_buf[..len];
            let ether_type = u16::from_be_bytes([frame[12], frame[13]]);

            match ether_type {
                ARP_FRAME_TYPE => {
                    if let Some(arp_pkt) = parse_arp(frame) {
                        match self
                            .arp
                            .handle(&arp_pkt, &self.current_ip, &self.config.mac_address)
                        {
                            ArpEvent::SendReply(reply_frame) => {
                                self.device.transmit(&reply_frame)?;
                            }
                            ArpEvent::Resolved(mac) => {
                                self.remote_mac = mac;
                            }
                            ArpEvent::None => {}
                        }
                    }
                }
                UDP_FRAME_TYPE => {
                    // Check if packet is a UDP frame directed to the DHCP client port (68)
                    let is_udp = frame.len() >= 34 && frame[23] == 0x11;
                    if is_udp {
                        let ihl = (frame[14] & 0x0F) as usize * 4;
                        let udp_start = 14 + ihl;
                        if frame.len() >= udp_start + 4 {
                            let dest_port =
                                u16::from_be_bytes([frame[udp_start + 2], frame[udp_start + 3]]);
                            if dest_port == DHCP_CLIENT_PORT && self.config.dhcp_enabled {
                                let action = self.dhcp.input(frame, now_ticks);
                                if let DhcpAction::Send(send_frame) = action {
                                    self.device.transmit(&send_frame)?;
                                }
                                if self.dhcp.is_bound() {
                                    self.current_ip = self.dhcp.lease.assigned_ip;
                                    self.current_subnet = self.dhcp.lease.subnet_mask;
                                    self.current_gateway = self.dhcp.lease.gateway;
                                }
                                continue;
                            }
                        }
                    }

                    // Otherwise parse as general UDP datagram
                    if let Some(view) = parse_frame(frame) {
                        let for_us_ip = is_same_ip(view.dest_ip, &self.current_ip)
                            || is_same_ip(view.dest_ip, &[255, 255, 255, 255])
                            || (view.dest_ip[3] == 255
                                && view.dest_ip[0..3] == self.current_ip[0..3]);
                        let for_us_mac = is_same_mac(view.dest_mac, &self.config.mac_address)
                            || is_same_mac(view.dest_mac, &[0xFF; 6]);

                        if for_us_ip && for_us_mac {
                            let src_ip = [frame[26], frame[27], frame[28], frame[29]];
                            self.incoming_udp.push(ReceivedUdpPacket {
                                src_ip,
                                src_port: view.src_port,
                                dest_port: view.dest_port,
                                payload: view.payload.to_vec(),
                            });
                        }
                    }
                }
                _ => {}
            }
        }

        // Periodic DHCP maintenance
        if self.config.dhcp_enabled {
            let action = self.dhcp.tick(now_ticks);
            if let DhcpAction::Send(send_frame) = action {
                self.device.transmit(&send_frame)?;
            }
            if self.dhcp.is_bound() {
                self.current_ip = self.dhcp.lease.assigned_ip;
                self.current_subnet = self.dhcp.lease.subnet_mask;
                self.current_gateway = self.dhcp.lease.gateway;
            }
        }

        // Periodic ARP timeout
        if self.arp.state == ArpState::Waiting && now_ticks >= self.arp.timeout {
            self.arp.state = ArpState::Idle;
        }

        Ok(())
    }

    /// Retrieve the next received UDP packet, if any.
    pub fn pop_received_udp(&mut self) -> Option<ReceivedUdpPacket> {
        if self.incoming_udp.is_empty() {
            None
        } else {
            Some(self.incoming_udp.remove(0))
        }
    }

    /// Initiate ARP resolution for `target_ip`.
    ///
    /// Transmits an ARP Request frame onto the wire.
    pub fn resolve_arp(
        &mut self,
        target_ip: &[u8; IPV4_ADDRESS_LENGTH],
        now_ticks: u64,
    ) -> HalResult<()> {
        if let Some(req_frame) = self.arp.send_request(
            target_ip,
            &self.current_ip,
            &self.config.mac_address,
            now_ticks,
        ) {
            self.device.transmit(&req_frame)?;
        }
        Ok(())
    }

    /// Build and transmit a UDP frame to the default remote endpoint (`remote_ip` / `remote_mac`).
    ///
    /// Port of `EmacSendUDP()`.
    pub fn send_udp(&mut self, payload: &[u8], local_port: u16, remote_port: u16) -> HalResult<()> {
        let ep = Endpoint {
            local_mac: self.config.mac_address,
            remote_mac: self.remote_mac,
            local_ip: self.current_ip,
            remote_ip: self.remote_ip,
        };
        let frame = build_udp_frame(payload, local_port, remote_port, &ep).ok_or(HalError::Bus)?;
        self.device.transmit(&frame)
    }

    /// Build and transmit a string message (e.g. telemetry line) over UDP.
    pub fn send_str(&mut self, text: &str, local_port: u16, remote_port: u16) -> HalResult<()> {
        self.send_udp(text.as_bytes(), local_port, remote_port)
    }

    /// Build and transmit a UDP frame to an explicit destination IP address.
    pub fn send_udp_to(
        &mut self,
        payload: &[u8],
        local_port: u16,
        remote_ip: &[u8; IPV4_ADDRESS_LENGTH],
        remote_port: u16,
    ) -> HalResult<()> {
        let ep = Endpoint {
            local_mac: self.config.mac_address,
            remote_mac: self.remote_mac,
            local_ip: self.current_ip,
            remote_ip: *remote_ip,
        };
        let frame = build_udp_frame(payload, local_port, remote_port, &ep).ok_or(HalError::Bus)?;
        self.device.transmit(&frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arp::{build_arp_frame, ArpOp};
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    #[derive(Clone, Default)]
    struct FakeEthernetDevice {
        rx_queue: Rc<RefCell<VecDeque<Vec<u8>>>>,
        tx_log: Rc<RefCell<Vec<Vec<u8>>>>,
    }

    impl FakeEthernetDevice {
        fn push_rx(&self, packet: Vec<u8>) {
            self.rx_queue.borrow_mut().push_back(packet);
        }
    }

    impl EthernetDevice for FakeEthernetDevice {
        fn transmit(&mut self, packet: &[u8]) -> HalResult<()> {
            self.tx_log.borrow_mut().push(packet.to_vec());
            Ok(())
        }

        fn receive(&mut self, buffer: &mut [u8]) -> HalResult<Option<usize>> {
            if let Some(pkt) = self.rx_queue.borrow_mut().pop_front() {
                let len = pkt.len().min(buffer.len());
                buffer[..len].copy_from_slice(&pkt[..len]);
                Ok(Some(len))
            } else {
                Ok(None)
            }
        }
    }

    fn test_config() -> EthernetConfig {
        EthernetConfig {
            mac_address: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            ip_address: [192, 168, 2, 101],
            subnet_mask: [255, 255, 255, 0],
            gateway: [192, 168, 2, 1],
            dhcp_enabled: false,
            phy_address: 1,
            rmii: true,
            speed_mbps: 100,
            full_duplex: true,
        }
    }

    #[test]
    fn send_udp_transmits_valid_ethernet_frame() {
        let fake = FakeEthernetDevice::default();
        let mut stack = NetworkStack::new(fake.clone(), test_config());

        stack.send_str("HELLO", 22223, 22223).unwrap();

        let tx = fake.tx_log.borrow();
        assert_eq!(tx.len(), 1);
        let frame = &tx[0];
        assert_eq!(&frame[12..14], &[0x08, 0x00]); // IPv4
        assert_eq!(frame[23], 0x11); // UDP
        assert_eq!(&frame[26..30], &[192, 168, 2, 101]); // Source IP
        assert_eq!(&frame[30..34], &[192, 168, 2, 255]); // Dest IP (broadcast default)
    }

    #[test]
    fn poll_answers_arp_request_for_our_ip() {
        let fake = FakeEthernetDevice::default();
        let mut stack = NetworkStack::new(fake.clone(), test_config());

        // Peer 192.168.2.50 asks for our IP 192.168.2.101
        let peer_ip = [192, 168, 2, 50];
        let peer_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let arp_req = build_arp_frame(
            ArpOp::Request,
            &stack.current_ip(),
            &[0xFF; 6],
            &peer_ip,
            &peer_mac,
        );

        fake.push_rx(arp_req.to_vec());
        stack.poll(100).unwrap();

        let tx = fake.tx_log.borrow();
        assert_eq!(tx.len(), 1);
        let reply = parse_arp(&tx[0]).unwrap();
        assert_eq!(reply.operation, ArpOp::Reply as u16);
        assert_eq!(reply.sender_ip, stack.current_ip());
        assert_eq!(reply.sender_mac, stack.config().mac_address);
        assert_eq!(reply.target_ip, peer_ip);
        assert_eq!(reply.target_mac, peer_mac);
    }

    #[test]
    fn poll_dispatches_incoming_udp_packet() {
        let fake = FakeEthernetDevice::default();
        let mut stack = NetworkStack::new(fake.clone(), test_config());

        let ep = Endpoint {
            local_mac: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
            remote_mac: stack.config().mac_address,
            local_ip: [192, 168, 2, 50],
            remote_ip: stack.current_ip(),
        };
        let frame = build_udp_frame(b"PING", 5000, 6000, &ep).unwrap();
        fake.push_rx(frame);

        stack.poll(200).unwrap();

        let received = stack.pop_received_udp().expect("expected udp packet");
        assert_eq!(received.src_ip, [192, 168, 2, 50]);
        assert_eq!(received.src_port, 5000);
        assert_eq!(received.dest_port, 6000);
        assert_eq!(received.payload, b"PING\r\n");
        assert_eq!(received.payload_trimmed(), b"PING");
        assert_eq!(received.as_str().unwrap(), "PING");
    }

    #[test]
    fn arp_resolution_updates_remote_mac() {
        let fake = FakeEthernetDevice::default();
        let mut stack = NetworkStack::new(fake.clone(), test_config());

        let peer_ip = [192, 168, 2, 50];
        let peer_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

        // Start resolution
        stack.resolve_arp(&peer_ip, 1000).unwrap();
        assert_eq!(stack.arp_state(), ArpState::Waiting);

        // Receive reply from peer
        let arp_reply = build_arp_frame(
            ArpOp::Reply,
            &stack.current_ip(),
            &stack.config().mac_address,
            &peer_ip,
            &peer_mac,
        );
        fake.push_rx(arp_reply.to_vec());

        stack.poll(1100).unwrap();

        assert_eq!(stack.arp_state(), ArpState::Resolved);
        assert_eq!(stack.remote_mac(), peer_mac);
    }

    #[test]
    fn start_dhcp_sends_discover_when_enabled() {
        let fake = FakeEthernetDevice::default();
        let mut cfg = test_config();
        cfg.dhcp_enabled = true;
        let mut stack = NetworkStack::new(fake.clone(), cfg);

        stack.start(500).unwrap();
        assert_eq!(stack.dhcp_state(), DhcpState::Selecting);

        let tx = fake.tx_log.borrow();
        assert_eq!(tx.len(), 1);
        // Discover frame is broadcast
        assert_eq!(&tx[0][0..6], &[0xFF; 6]);
    }
}
