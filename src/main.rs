//! Demo runner for the `um_rust` firmware port.
//!
//! On the real LPC1768 the application is an endless super-loop. Here we wire
//! the pure-logic modules together with simple in-process fakes so the program
//! runs on a PC and shows the modules cooperating. This is **not** the embedded
//! target image — it exists to exercise the library end-to-end.

use um_rust::accelerometer::AccSample;
use um_rust::app::{App, AppConfig, SensorInputs};
use um_rust::arp::{ArpEvent, ArpResolver};
use um_rust::compass::Orientation;
use um_rust::dhcp::{DhcpAction, DhcpClient};
use um_rust::gps::GpsReceiver;
use um_rust::hal::{EthernetConfig, EthernetDevice, HalResult, LogSink};
use um_rust::leds::{InMemoryLedDriver, Led, LedManager};
use um_rust::net::chksum16;
use um_rust::net_stack::NetworkStack;
use um_rust::power::{InMemoryPowerPort, PowerDomain, PowerManager};
use um_rust::pressure::{convert, PressureCalibration};
use um_rust::rtc::TimeStruct;
use um_rust::telemetry::DataLogger;
use um_rust::temperature::{iir, raw_to_value};
use um_rust::timer::SystemTimer;
use um_rust::udp::{build_udp_frame, Endpoint};

const LOCAL_MAC: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];

struct DemoEthernet;
impl EthernetDevice for DemoEthernet {
    fn transmit(&mut self, _packet: &[u8]) -> HalResult<()> {
        Ok(())
    }
    fn receive(&mut self, _buffer: &mut [u8]) -> HalResult<Option<usize>> {
        Ok(None)
    }
}

struct DemoLogSink(Vec<String>);
impl LogSink for DemoLogSink {
    fn write_line(&mut self, line: &str) -> HalResult<()> {
        self.0.push(line.to_string());
        Ok(())
    }
    fn flush(&mut self) -> HalResult<()> {
        Ok(())
    }
}

fn main() {
    println!("== um_rust demo ==");

    // --- Timer ---
    let mut timer = SystemTimer::new(100);
    timer.simulate_delay_us(1000);
    println!("timer: {} ticks after a 1ms delay", timer.now());

    // --- RTC ---
    let now = TimeStruct::from_ctime(
        (6 << 24) | (23 << 16) | (45 << 8) | 30,
        (2024 << 16) | (11 << 8) | 7,
        312,
    );
    println!("rtc: {}", now.to_iso());

    // --- GPS ---
    let mut gps = GpsReceiver::new();
    for &b in b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,070624,,,*\r\n" {
        gps.feed_byte(b);
    }
    let g = gps.data();
    println!(
        "gps: {}:{}:{} lat {} {} lon {} {} speed {:.1} km/h",
        g.time.hour,
        g.time.minute,
        g.time.second,
        g.coords.latitude,
        g.coords.lat_hemisphere,
        g.coords.longitude,
        g.coords.lon_hemisphere,
        g.speed
    );

    // --- Temperature ---
    let mut t = 0.0f32;
    for _ in 0..50 {
        t = iir(t, raw_to_value(2048));
    }
    println!("temperature: filtered ~{:.1}", t);

    // --- Pressure ---
    let cal = PressureCalibration {
        c1: 24000,
        c2: 4000,
        c3: 350,
        c4: 400,
        c5: 7000,
        c6: 4500,
        c7: 2500,
        a: 100,
        b: 110,
        c: 8,
        d: 8,
    };
    let r = convert(&cal, 9000, 7200);
    println!("pressure: {:.1}, temp {:.1}", r.pressure, r.temperature);

    // --- UDP ---
    let ep = Endpoint {
        local_mac: LOCAL_MAC,
        remote_mac: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
        local_ip: [192, 168, 2, 101],
        remote_ip: [192, 168, 2, 255],
    };
    let frame = build_udp_frame(b"PING", 5000, 6000, &ep).expect("frame");
    println!(
        "udp: built {}-byte frame, IP checksum field valid = {}",
        frame.len(),
        chksum16(&frame[14..34]) == 0
    );

    // --- ARP ---
    let mut arp = ArpResolver::new();
    if let Some(req) = arp.send_request(&[192, 168, 2, 50], &ep.local_ip, &LOCAL_MAC, timer.now()) {
        println!(
            "arp: sent {}-byte request, state {:?}",
            req.len(),
            arp.state
        );
        let _ = ArpEvent::None;
    }

    // --- DHCP ---
    let mut dhcp = DhcpClient::new(LOCAL_MAC);
    if let DhcpAction::Send(disc) = dhcp.start(timer.now()) {
        println!(
            "dhcp: sent {}-byte DISCOVER, state {:?}",
            disc.len(),
            dhcp.state
        );
    }

    // --- App Super-Loop ---
    let eth_cfg = EthernetConfig::default();
    let net_stack = NetworkStack::new(DemoEthernet, eth_cfg);
    let logger = DataLogger::new(DemoLogSink(Vec::new()));
    let mut app = App::new(net_stack, logger, AppConfig::default());
    app.start(timer.now(), &now).expect("app start");

    // --- Power Domains & LEDs ---
    let pwr = PowerManager::with_default_board_power(InMemoryPowerPort::default()).unwrap();
    println!(
        "power: default mask 0x{:02X} (GPS={}, Compass={}, Acc1={}, Acc2={})",
        pwr.current_mask(),
        pwr.is_enabled(PowerDomain::Gps),
        pwr.is_enabled(PowerDomain::Compass),
        pwr.is_enabled(PowerDomain::Acc1),
        pwr.is_enabled(PowerDomain::Acc2),
    );

    let mut leds = LedManager::new(InMemoryLedDriver::default());
    leds.on(Led::Led2).unwrap();
    println!("leds: LED2 (heartbeat) on = {}", leds.is_on(Led::Led2));

    let inputs = SensorInputs {
        keys: Some(0),
        acc: Some(AccSample {
            x: 5,
            y: -10,
            z: 120,
        }),
        compass: Some(Orientation {
            heading: 180.0,
            pitch: 0.0,
            roll: 0.0,
        }),
        gps: Some(gps.data()),
        pressure: Some(r),
        temp: Some(([21.5, 22.0, 23.5], 12.3)),
        rtc: Some(&now),
    };
    let report = app.step(50000, inputs).expect("app step");
    println!(
        "app: step at 50000 ticks: acc={}, compass_gps={}, temp={}, pressure={}, sync={}",
        report.acc_emitted,
        report.compass_gps_emitted,
        report.temp_emitted,
        report.pressure_emitted,
        report.file_synced
    );

    // --- Remote Commands ---
    let cmd = um_rust::commands::Command::parse("STATUS");
    let reply = app.execute_remote_command(&cmd, Some(&now)).unwrap();
    println!("commands: executed 'STATUS' -> {}", reply.trim_end());

    println!("== done ==");
}
