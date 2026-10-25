//! Application super-loop coordinator, port of `main()` from `mainPrg.c`.
//!
//! Integrates all subsystem drivers into a single cohesive runtime:
//! * Periodic scheduling based on [`crate::timer::SystemTimer`] tick counts.
//! * Sensor reading and dual-path telemetry emission (SD card + Ethernet UDP).
//! * Dynamic runtime control via 5-key keypad (SW1..SW5 toggling SD/Ethernet).
//! * Periodic SD card metadata sync (`f_sync`) and RTC timestamping.
//! * Ethernet network polling (ARP, DHCP, and incoming UDP packet queue).
//! * Status LED toggling on compass/GPS emission ticks.

use crate::accelerometer::AccSample;
use crate::compass::Orientation;
use crate::gps::GpsData;
use crate::hal::{EthernetDevice, HalResult, LogSink, WatchdogFeed};
use crate::keys::{KEY_SW1, KEY_SW2, KEY_SW3, KEY_SW4, KEY_SW5};
use crate::net_stack::{NetworkStack, ReceivedUdpPacket};
use crate::rtc::TimeStruct;
use crate::telemetry::{DataLogger, TelemetryRecord};

#[cfg(not(test))]
use alloc::vec::Vec;

/// Accelerometer sample emission interval matching `ACC_TIME` in `mainPgr.h` (5000 ticks = 500 ms).
pub const DEFAULT_ACC_INTERVAL: u64 = 5000;

/// Compass, GPS, and LED2 toggle interval matching `LED_TIME` in `mainPgr.h` (5000 ticks = 500 ms).
pub const DEFAULT_LED_INTERVAL: u64 = 5000;

/// Temperature and power voltage acquisition interval matching `TEMP_TIME` (3333 ticks = ~333 ms).
pub const DEFAULT_TEMP_INTERVAL: u64 = 3333;

/// Keypad scanning interval matching `KEY_TIME` in `mainPgr.h` (300 ticks = 30 ms).
pub const DEFAULT_KEY_INTERVAL: u64 = 300;

/// SD card sync and timestamp emission interval matching `FILE_RELOAD_TIME` (50000 ticks = 5000 ms).
pub const DEFAULT_FILE_SYNC_INTERVAL: u64 = 50000;

/// Network link check interval matching `LINK_TIME` in `mainPgr.h` (5000 ticks = 500 ms).
pub const DEFAULT_LINK_INTERVAL: u64 = 5000;

/// Default UDP local source port matching `mainPrg.c`.
pub const DEFAULT_LOCAL_PORT: u16 = 22223;

/// Default UDP remote destination port matching `mainPrg.c`.
pub const DEFAULT_REMOTE_PORT: u16 = 22223;

/// Configuration parameters for the application super-loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AppConfig {
    /// Interval between accelerometer emissions (ticks).
    pub acc_interval: u64,
    /// Interval between compass/GPS readings and LED toggle (ticks).
    pub led_interval: u64,
    /// Interval between temperature readings (ticks).
    pub temp_interval: u64,
    /// Interval between keypad scans (ticks).
    pub key_interval: u64,
    /// Interval between SD card file syncs and RTC timestamps (ticks).
    pub file_sync_interval: u64,
    /// Interval between network link status checks (ticks).
    pub link_interval: u64,
    /// Local UDP port for telemetry transmissions.
    pub local_port: u16,
    /// Remote destination UDP port for telemetry transmissions.
    pub remote_port: u16,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            acc_interval: DEFAULT_ACC_INTERVAL,
            led_interval: DEFAULT_LED_INTERVAL,
            temp_interval: DEFAULT_TEMP_INTERVAL,
            key_interval: DEFAULT_KEY_INTERVAL,
            file_sync_interval: DEFAULT_FILE_SYNC_INTERVAL,
            link_interval: DEFAULT_LINK_INTERVAL,
            local_port: DEFAULT_LOCAL_PORT,
            remote_port: DEFAULT_REMOTE_PORT,
        }
    }
}

/// Dynamic runtime state of the application super-loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AppState {
    /// Logging to SD card active (`sd_on`).
    pub sd_on: bool,
    /// UDP broadcast active (`eth_on`).
    pub eth_on: bool,
    /// Current logical state of status LED2 (toggled periodically).
    pub led_state: bool,
    /// Previous key bitmask for rising-edge detection.
    pub last_keys: u8,
    /// Next scheduled tick for accelerometer emission.
    pub acc_deadline: u64,
    /// Next scheduled tick for compass/GPS/LED update.
    pub led_deadline: u64,
    /// Next scheduled tick for temperature reading.
    pub temp_deadline: u64,
    /// Next scheduled tick for keypad scan.
    pub key_deadline: u64,
    /// Next scheduled tick for SD card sync and timestamp emission.
    pub file_sync_deadline: u64,
}

impl AppState {
    /// Initialize state with all task deadlines scheduled relative to `start_ticks`.
    pub fn new(start_ticks: u64, config: &AppConfig) -> Self {
        Self {
            sd_on: true,
            eth_on: true,
            led_state: false,
            last_keys: 0,
            acc_deadline: start_ticks + config.acc_interval,
            led_deadline: start_ticks + config.led_interval,
            temp_deadline: start_ticks + config.temp_interval,
            key_deadline: start_ticks + config.key_interval,
            file_sync_deadline: start_ticks + config.file_sync_interval,
        }
    }

    /// Process raw key input using rising-edge detection matching `mainPrg.c`:
    /// * SW1 -> enable SD logging (`sd_on = 1`)
    /// * SW2 -> disable SD logging (`sd_on = 0`)
    /// * SW3 -> enable Ethernet UDP broadcast (`eth_on = 1`)
    /// * SW4 -> disable Ethernet UDP broadcast (`eth_on = 0`)
    /// * SW5 -> disable both SD and Ethernet (`sd_on = 0, eth_on = 0`)
    pub fn process_keys(&mut self, keys: u8) -> KeyAction {
        let newly_pressed = keys & !self.last_keys;
        self.last_keys = keys;

        let action = KeyAction {
            sw1_pressed: newly_pressed & KEY_SW1 != 0,
            sw2_pressed: newly_pressed & KEY_SW2 != 0,
            sw3_pressed: newly_pressed & KEY_SW3 != 0,
            sw4_pressed: newly_pressed & KEY_SW4 != 0,
            sw5_pressed: newly_pressed & KEY_SW5 != 0,
        };

        if action.sw1_pressed {
            self.sd_on = true;
        }
        if action.sw2_pressed {
            self.sd_on = false;
        }
        if action.sw3_pressed {
            self.eth_on = true;
        }
        if action.sw4_pressed {
            self.eth_on = false;
        }
        if action.sw5_pressed {
            self.sd_on = false;
            self.eth_on = false;
        }

        action
    }
}

/// Newly pressed keys detected during a keypad scan.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct KeyAction {
    pub sw1_pressed: bool,
    pub sw2_pressed: bool,
    pub sw3_pressed: bool,
    pub sw4_pressed: bool,
    pub sw5_pressed: bool,
}

/// Summary report of actions performed during an [`App::step`] iteration.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct StepReport {
    /// Ethernet network was polled during this step.
    pub network_polled: bool,
    /// Keypad was scanned and processed during this step.
    pub keys_scanned: bool,
    /// Accelerometer telemetry was formatted and dispatched.
    pub acc_emitted: bool,
    /// Compass and GPS telemetry were formatted and dispatched.
    pub compass_gps_emitted: bool,
    /// Temperature telemetry was formatted and dispatched.
    pub temp_emitted: bool,
    /// SD card sync and RTC timestamp were written.
    pub file_synced: bool,
    /// Watchdog timer was fed during this step.
    pub watchdog_fed: bool,
    /// Number of incoming UDP packets received from the network.
    pub received_packets: usize,
}

/// Sensor inputs and peripheral readings passed into a super-loop step.
#[derive(Debug, Default, Clone)]
pub struct SensorInputs<'a> {
    /// Current keypad bitmask (e.g. from [`crate::keys::Keypad::read`]).
    pub keys: Option<u8>,
    /// Latest accelerometer reading.
    pub acc: Option<AccSample>,
    /// Latest electronic compass orientation.
    pub compass: Option<Orientation>,
    /// Latest decoded GPS data.
    pub gps: Option<&'a GpsData>,
    /// Latest 3-channel temperatures and supply voltage.
    pub temp: Option<([f32; 3], f32)>,
    /// Current RTC timestamp for periodic log sync.
    pub rtc: Option<&'a TimeStruct>,
}

/// Application super-loop combining networking, data logging, and sensor dispatch.
pub struct App<D: EthernetDevice, S: LogSink> {
    net: NetworkStack<D>,
    logger: DataLogger<S>,
    config: AppConfig,
    state: AppState,
}

impl<D: EthernetDevice, S: LogSink> App<D, S> {
    /// Create a new application instance with the given network stack, data logger, and configuration.
    pub fn new(net: NetworkStack<D>, logger: DataLogger<S>, config: AppConfig) -> Self {
        let state = AppState::new(0, &config);
        Self {
            net,
            logger,
            config,
            state,
        }
    }

    /// Access the network stack.
    pub fn net(&self) -> &NetworkStack<D> {
        &self.net
    }

    /// Mutably access the network stack.
    pub fn net_mut(&mut self) -> &mut NetworkStack<D> {
        &mut self.net
    }

    /// Access the data logger.
    pub fn logger(&self) -> &DataLogger<S> {
        &self.logger
    }

    /// Mutably access the data logger.
    pub fn logger_mut(&mut self) -> &mut DataLogger<S> {
        &mut self.logger
    }

    /// Access the configuration.
    pub fn config(&self) -> &AppConfig {
        &self.config
    }

    /// Access the current runtime state.
    pub fn state(&self) -> &AppState {
        &self.state
    }

    /// `true` if SD card logging is currently enabled.
    pub fn is_sd_enabled(&self) -> bool {
        self.state.sd_on
    }

    /// `true` if Ethernet UDP telemetry is currently enabled.
    pub fn is_eth_enabled(&self) -> bool {
        self.state.eth_on
    }

    /// Current logical state of status LED2.
    pub fn led_state(&self) -> bool {
        self.state.led_state
    }

    /// Manually enable or disable SD card logging.
    pub fn set_sd_enabled(&mut self, enabled: bool) {
        self.state.sd_on = enabled;
        self.logger.set_enabled(enabled);
    }

    /// Manually enable or disable Ethernet UDP broadcasting.
    pub fn set_eth_enabled(&mut self, enabled: bool) {
        self.state.eth_on = enabled;
    }

    /// Start application services (initiates DHCP if enabled and opens SD card log with RTC banner).
    pub fn start(&mut self, now_ticks: u64, rtc: &TimeStruct) -> HalResult<()> {
        self.state = AppState::new(now_ticks, &self.config);
        self.net.start(now_ticks)?;
        self.logger.open(rtc)?;
        Ok(())
    }

    /// Feed an active watchdog timer instance.
    pub fn feed_watchdog<F: FnMut(u8)>(
        &mut self,
        watchdog: &mut crate::watchdog::Watchdog<F>,
    ) -> bool {
        watchdog.feed()
    }

    /// Feed hardware watchdog directly via [`WatchdogFeed`].
    pub fn feed_watchdog_feed<W: WatchdogFeed>(&mut self, hw: &mut W) {
        hw.write_byte(crate::watchdog::WDT_FEED_1);
        hw.write_byte(crate::watchdog::WDT_FEED_2);
    }

    /// Poll incoming Ethernet frames and advance internal network state machines (DHCP/ARP).
    ///
    /// Drains and returns all received UDP datagrams.
    pub fn poll_network(&mut self, now_ticks: u64) -> HalResult<Vec<ReceivedUdpPacket>> {
        self.net.poll(now_ticks)?;
        let mut packets = Vec::new();
        while let Some(pkt) = self.net.pop_received_udp() {
            packets.push(pkt);
        }
        Ok(packets)
    }

    /// Scan keypad bitmask if scheduled, applying SW1..SW5 controls.
    ///
    /// Returns `true` if keypad scanning was performed.
    pub fn scan_keys(&mut self, now_ticks: u64, raw_keys: u8) -> bool {
        if now_ticks >= self.state.key_deadline {
            self.state.key_deadline = now_ticks + self.config.key_interval;
            self.state.process_keys(raw_keys);
            self.logger.set_enabled(self.state.sd_on);
            true
        } else {
            false
        }
    }

    /// Emit an accelerometer telemetry reading if scheduled.
    ///
    /// Returns `Ok(true)` if the sample was formatted and emitted.
    pub fn emit_accelerometer(&mut self, now_ticks: u64, sample: &AccSample) -> HalResult<bool> {
        if now_ticks >= self.state.acc_deadline {
            self.state.acc_deadline = now_ticks + self.config.acc_interval;
            let record = TelemetryRecord::Accelerometer(*sample);
            self.dispatch_record(&record)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Emit compass and GPS telemetry readings and toggle status LED if scheduled.
    ///
    /// Returns `Ok(true)` if readings were formatted and emitted.
    pub fn emit_compass_and_gps(
        &mut self,
        now_ticks: u64,
        orientation: &Orientation,
        gps: &GpsData,
    ) -> HalResult<bool> {
        if now_ticks >= self.state.led_deadline {
            self.state.led_deadline = now_ticks + self.config.led_interval;
            self.state.led_state = !self.state.led_state;

            let comp_rec = TelemetryRecord::Compass(*orientation);
            self.dispatch_record(&comp_rec)?;

            let gps_rec = TelemetryRecord::Gps(gps.clone());
            self.dispatch_record(&gps_rec)?;

            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Emit temperature and power supply voltage readings if scheduled.
    ///
    /// Returns `Ok(true)` if readings were formatted and emitted.
    pub fn emit_temperature(
        &mut self,
        now_ticks: u64,
        temp: &[f32; 3],
        power_volt: f32,
    ) -> HalResult<bool> {
        if now_ticks >= self.state.temp_deadline {
            self.state.temp_deadline = now_ticks + self.config.temp_interval;
            let record = TelemetryRecord::Temperature {
                temp: *temp,
                power_volt,
            };
            self.dispatch_record(&record)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Perform periodic SD card sync and timestamp write if scheduled and `sd_on` is active.
    ///
    /// Returns `Ok(true)` if sync was performed.
    pub fn sync_storage(&mut self, now_ticks: u64, rtc: &TimeStruct) -> HalResult<bool> {
        if now_ticks >= self.state.file_sync_deadline && self.state.sd_on {
            self.state.file_sync_deadline = now_ticks + self.config.file_sync_interval;
            self.logger.sync()?;
            self.logger.log_timestamp(rtc)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Execute one comprehensive iteration of the application super-loop.
    pub fn step<'a>(&mut self, now_ticks: u64, inputs: SensorInputs<'a>) -> HalResult<StepReport> {
        let mut report = StepReport::default();

        // 1. Network polling (DHCP/ARP ticks + packet dispatch)
        let rx_pkts = self.poll_network(now_ticks)?;
        report.network_polled = true;
        report.received_packets = rx_pkts.len();

        // 2. Keypad scanning
        if let Some(keys) = inputs.keys {
            report.keys_scanned = self.scan_keys(now_ticks, keys);
        }

        // 3. Accelerometer telemetry
        if let Some(ref sample) = inputs.acc {
            report.acc_emitted = self.emit_accelerometer(now_ticks, sample)?;
        }

        // 4. Compass, GPS, and LED toggling
        if let (Some(ref compass), Some(gps)) = (inputs.compass, inputs.gps) {
            report.compass_gps_emitted = self.emit_compass_and_gps(now_ticks, compass, gps)?;
        }

        // 5. Temperature telemetry
        if let Some((temp, pwr)) = inputs.temp {
            report.temp_emitted = self.emit_temperature(now_ticks, &temp, pwr)?;
        }

        // 6. Periodic SD sync & RTC timestamp
        if let Some(rtc) = inputs.rtc {
            report.file_synced = self.sync_storage(now_ticks, rtc)?;
        }

        Ok(report)
    }

    /// Internal dispatch routing a telemetry record to SD card and/or Ethernet.
    fn dispatch_record(&mut self, record: &TelemetryRecord) -> HalResult<()> {
        if self.state.sd_on {
            self.logger.log_record(record)?;
        }
        if self.state.eth_on {
            let s = record.to_telemetry_string();
            self.net
                .send_str(&s, self.config.local_port, self.config.remote_port)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::EthernetConfig;
    use crate::rtc::TimeStruct;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    #[derive(Clone, Default)]
    struct MockEthernet {
        rx_queue: Rc<RefCell<VecDeque<Vec<u8>>>>,
        tx_frames: Rc<RefCell<Vec<Vec<u8>>>>,
    }

    impl EthernetDevice for MockEthernet {
        fn transmit(&mut self, packet: &[u8]) -> HalResult<()> {
            self.tx_frames.borrow_mut().push(packet.to_vec());
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

    #[derive(Clone, Default)]
    struct MockSink {
        lines: Rc<RefCell<Vec<String>>>,
        sync_count: Rc<RefCell<usize>>,
    }

    impl LogSink for MockSink {
        fn write_line(&mut self, line: &str) -> HalResult<()> {
            self.lines.borrow_mut().push(line.to_string());
            Ok(())
        }

        fn flush(&mut self) -> HalResult<()> {
            *self.sync_count.borrow_mut() += 1;
            Ok(())
        }
    }

    struct MockWatchdogFeed {
        bytes: Vec<u8>,
    }

    impl WatchdogFeed for MockWatchdogFeed {
        fn write_byte(&mut self, byte: u8) {
            self.bytes.push(byte);
        }
    }

    fn test_app() -> (App<MockEthernet, MockSink>, MockEthernet, MockSink) {
        let mock_eth = MockEthernet::default();
        let mock_sink = MockSink::default();

        let eth_cfg = EthernetConfig {
            mac_address: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            ip_address: [192, 168, 2, 101],
            subnet_mask: [255, 255, 255, 0],
            gateway: [192, 168, 2, 1],
            dhcp_enabled: false,
            phy_address: 1,
            rmii: true,
            speed_mbps: 100,
            full_duplex: true,
        };

        let net = NetworkStack::new(mock_eth.clone(), eth_cfg);
        let logger = DataLogger::new(mock_sink.clone());
        let config = AppConfig::default();

        let app = App::new(net, logger, config);
        (app, mock_eth, mock_sink)
    }

    fn sample_rtc() -> TimeStruct {
        TimeStruct {
            year: 2024,
            month: 11,
            day_of_month: 7,
            hour: 14,
            minute: 30,
            second: 15,
            ..Default::default()
        }
    }

    #[test]
    fn app_start_opens_logger_and_initializes_state() {
        let (mut app, _eth, sink) = test_app();
        let rtc = sample_rtc();

        app.start(100, &rtc).unwrap();

        assert!(app.is_sd_enabled());
        assert!(app.is_eth_enabled());
        assert_eq!(app.state().acc_deadline, 100 + DEFAULT_ACC_INTERVAL);

        let lines = sink.lines.borrow();
        assert_eq!(lines.len(), 1);
        assert!(lines[0].contains("Start logging ..."));
    }

    #[test]
    fn keypad_switches_toggle_sd_and_eth_correctly() {
        let (mut app, _eth, _sink) = test_app();
        app.start(0, &sample_rtc()).unwrap();

        // Initially both on
        assert!(app.is_sd_enabled());
        assert!(app.is_eth_enabled());

        // Press SW2 (disable SD)
        assert!(app.scan_keys(300, KEY_SW2));
        assert!(!app.is_sd_enabled());
        assert!(app.is_eth_enabled());

        // Release keys
        assert!(app.scan_keys(600, 0));

        // Press SW4 (disable ETH)
        assert!(app.scan_keys(900, KEY_SW4));
        assert!(!app.is_sd_enabled());
        assert!(!app.is_eth_enabled());

        // Release keys
        assert!(app.scan_keys(1200, 0));

        // Press SW1 (enable SD)
        assert!(app.scan_keys(1500, KEY_SW1));
        assert!(app.is_sd_enabled());
        assert!(!app.is_eth_enabled());

        // Release keys
        assert!(app.scan_keys(1800, 0));

        // Press SW3 (enable ETH)
        assert!(app.scan_keys(2100, KEY_SW3));
        assert!(app.is_sd_enabled());
        assert!(app.is_eth_enabled());

        // Release keys
        assert!(app.scan_keys(2400, 0));

        // Press SW5 (disable both)
        assert!(app.scan_keys(2700, KEY_SW5));
        assert!(!app.is_sd_enabled());
        assert!(!app.is_eth_enabled());
    }

    #[test]
    fn accelerometer_emission_dispatches_to_both_sd_and_udp() {
        let (mut app, eth, sink) = test_app();
        app.start(0, &sample_rtc()).unwrap();

        let acc = AccSample {
            x: 10,
            y: -20,
            z: 30,
        };

        // Before deadline (tick 4000 < 5000) -> should not emit
        let emitted = app.emit_accelerometer(4000, &acc).unwrap();
        assert!(!emitted);
        assert_eq!(sink.lines.borrow().len(), 1); // only header
        assert_eq!(eth.tx_frames.borrow().len(), 0);

        // At deadline (tick 5000) -> should emit
        let emitted = app.emit_accelerometer(5000, &acc).unwrap();
        assert!(emitted);

        // SD verified
        let lines = sink.lines.borrow();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[1], "Accelerometer-X10,Y-20,Z30*");

        // UDP frame verified
        let tx = eth.tx_frames.borrow();
        assert_eq!(tx.len(), 1);
        let frame = &tx[0];
        assert_eq!(frame[23], 0x11); // UDP
    }

    #[test]
    fn compass_and_gps_emission_toggles_led() {
        let (mut app, eth, sink) = test_app();
        app.start(0, &sample_rtc()).unwrap();

        let orientation = Orientation {
            heading: 120.5,
            pitch: 5.0,
            roll: -2.0,
        };
        let gps = GpsData::default();

        assert!(!app.led_state());

        let emitted = app.emit_compass_and_gps(5000, &orientation, &gps).unwrap();
        assert!(emitted);
        assert!(app.led_state()); // LED toggled

        // Both compass and GPS written to SD and transmitted over UDP
        assert_eq!(sink.lines.borrow().len(), 3);
        assert!(sink.lines.borrow()[1].starts_with("Compass-120.50,5.00,-2.00*"));
        assert!(sink.lines.borrow()[2].starts_with("GPS_time-"));
        assert_eq!(eth.tx_frames.borrow().len(), 2);
    }

    #[test]
    fn temperature_emission_dispatches_at_deadline() {
        let (mut app, eth, sink) = test_app();
        app.start(0, &sample_rtc()).unwrap();

        let temp = [22.5, 23.0, 24.5];
        let pwr = 12.0;

        let emitted = app.emit_temperature(3333, &temp, pwr).unwrap();
        assert!(emitted);

        let lines = sink.lines.borrow();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[1], "Temperature-22.5,23.0,24.5,12.0*");
        assert_eq!(eth.tx_frames.borrow().len(), 1);
    }

    #[test]
    fn file_sync_storage_at_file_reload_time() {
        let (mut app, _eth, sink) = test_app();
        let rtc = sample_rtc();
        app.start(0, &rtc).unwrap();

        // At 50000 ticks -> sync + timestamp
        let synced = app.sync_storage(50000, &rtc).unwrap();
        assert!(synced);

        assert_eq!(*sink.sync_count.borrow(), 1);
        let lines = sink.lines.borrow();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[1], "2024-11-07 14:30:15  ");
    }

    #[test]
    fn comprehensive_step_execution() {
        let (mut app, _eth, sink) = test_app();
        let rtc = sample_rtc();
        app.start(0, &rtc).unwrap();

        let inputs = SensorInputs {
            keys: Some(0),
            acc: Some(AccSample { x: 1, y: 2, z: 3 }),
            compass: Some(Orientation::default()),
            gps: Some(&GpsData::default()),
            temp: Some(([20.0, 21.0, 22.0], 12.0)),
            rtc: Some(&rtc),
        };

        // Advance to tick 50000 (triggers all periodic tasks)
        let report = app.step(50000, inputs).unwrap();

        assert!(report.network_polled);
        assert!(report.keys_scanned);
        assert!(report.acc_emitted);
        assert!(report.compass_gps_emitted);
        assert!(report.temp_emitted);
        assert!(report.file_synced);
        assert_eq!(report.received_packets, 0);

        // Multiple lines written to SD
        assert!(sink.lines.borrow().len() >= 5);
    }

    #[test]
    fn watchdog_feeding() {
        let (mut app, _eth, _sink) = test_app();
        let mut log = Vec::new();
        let mut wdt = crate::watchdog::Watchdog::new(|b| log.push(b));
        wdt.set_enabled(true);
        assert!(app.feed_watchdog(&mut wdt));
        assert_eq!(log, vec![0xAA, 0x55]);

        let mut hw = MockWatchdogFeed { bytes: Vec::new() };
        app.feed_watchdog_feed(&mut hw);
        assert_eq!(hw.bytes, vec![0xAA, 0x55]);
    }
}
