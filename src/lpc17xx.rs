//! Hardware register adapters and PAC layer for LPC1768 (Cortex-M3).
//!
//! Provides zero-cost memory-mapped I/O (MMIO) wrappers implementing
//! the core application traits ([`PowerPort`], [`LedDriver`], [`WatchdogFeed`],
//! [`RtcRegisters`], [`MonotonicClock`], [`EthernetDevice`], and [`InputPin`]/[`OutputPin`]).
//!
//! On target hardware, peripherals reside at fixed addresses in the LPC1768 memory map:
//! * `GPIO0..GPIO4`: `0x2009_C000` .. `0x2009_C080`
//! * `RTC`: `0x4002_4000`
//! * `TIMER0`: `0x4000_4000`
//! * `WDT`: `0x4000_0000`
//! * `EMAC`: `0x5000_0000`

use crate::hal::{
    EthernetDevice, HalError, HalResult, InputPin, MonotonicClock, OutputPin, RtcRegisters,
    WatchdogFeed,
};
use crate::leds::{Led, LedDriver};
use crate::power::PowerPort;
use crate::rtc::TimeStruct;
use core::ptr::{read_volatile, write_volatile};

// ---------------------------------------------------------------------------
// Hardware Peripheral MMIO Base Addresses (LPC1768 User Manual UM10360)
// ---------------------------------------------------------------------------

pub const GPIO0_BASE: usize = 0x2009_C000;
pub const GPIO1_BASE: usize = 0x2009_C020;
pub const GPIO2_BASE: usize = 0x2009_C040;
pub const GPIO3_BASE: usize = 0x2009_C060;
pub const GPIO4_BASE: usize = 0x2009_C080;

pub const RTC_BASE: usize = 0x4002_4000;
pub const TIMER0_BASE: usize = 0x4000_4000;
pub const WDT_BASE: usize = 0x4000_0000;
pub const EMAC_BASE: usize = 0x5000_0000;

// ---------------------------------------------------------------------------
// Hardware Register Block Layouts
// ---------------------------------------------------------------------------

/// LPC1768 Fast GPIO register block (`LPC_GPIO0..4`).
#[repr(C)]
pub struct GpioBlock {
    /// Direction: 0 = input, 1 = output.
    pub dir: u32,
    _pad0: [u32; 3],
    /// Mask: 0 = pin read/write allowed, 1 = masked.
    pub mask: u32,
    /// Current pin level (read) or written value.
    pub pin: u32,
    /// Fast pin set: write 1 to drive pin high.
    pub set: u32,
    /// Fast pin clear: write 1 to drive pin low.
    pub clr: u32,
}

/// LPC1768 Real-Time Clock register block (`LPC_RTC`).
#[repr(C)]
pub struct RtcBlock {
    pub ilr: u32,
    _pad0: u32,
    pub ccr: u32,
    pub ciir: u32,
    pub amr: u32,
    /// Consolidated Time 0: `[30:24]=DOW, [20:16]=HOUR, [13:8]=MIN, [5:0]=SEC`.
    pub ctime0: u32,
    /// Consolidated Time 1: `[27:16]=YEAR, [11:8]=MONTH, [4:0]=DOM`.
    pub ctime1: u32,
    /// Consolidated Time 2: `[11:0]=DOY`.
    pub ctime2: u32,
    pub sec: u32,
    pub min: u32,
    pub hour: u32,
    pub dom: u32,
    pub dow: u32,
    pub doy: u32,
    pub month: u32,
    pub year: u32,
}

/// LPC1768 Timer register block (`LPC_TIM0`).
#[repr(C)]
pub struct TimerBlock {
    pub ir: u32,
    pub tcr: u32,
    /// Timer Counter: current timer value in ticks.
    pub tc: u32,
    pub pr: u32,
    pub pc: u32,
    pub mcr: u32,
    pub mr0: u32,
}

/// LPC1768 Watchdog register block (`LPC_WDT`).
#[repr(C)]
pub struct WatchdogBlock {
    pub mod_: u32,
    pub tc: u32,
    /// Feed register: write 0xAA then 0x55 to restart.
    pub feed: u32,
    pub tv: u32,
}

/// LPC1768 Ethernet MAC register block (`LPC_EMAC`).
#[repr(C)]
pub struct EmacBlock {
    pub mac1: u32,
    pub mac2: u32,
    pub ipgt: u32,
    pub ipgr: u32,
    pub clrt: u32,
    pub maxf: u32,
    pub supp: u32,
    pub test: u32,
    pub mcfg: u32,
    pub mcmd: u32,
    pub madr: u32,
    pub mwtd: u32,
    pub mrdd: u32,
    pub mind: u32,
    _pad0: [u32; 2],
    pub command: u32,
    pub status: u32,
    pub rx_descriptor: u32,
    pub rx_status: u32,
    pub rx_descriptor_number: u32,
    pub rx_produce_index: u32,
    pub rx_consume_index: u32,
    pub tx_descriptor: u32,
    pub tx_status: u32,
    pub tx_descriptor_number: u32,
    pub tx_produce_index: u32,
    pub tx_consume_index: u32,
}

// ---------------------------------------------------------------------------
// Hardware Adapters
// ---------------------------------------------------------------------------

/// Individual GPIO pin wrapper implementing [`InputPin`] and [`OutputPin`].
#[derive(Clone, Copy)]
pub struct Lpc17xxPin {
    raw: *mut GpioBlock,
    pin: u8,
}

impl Lpc17xxPin {
    /// Create a pin wrapper around a raw [`GpioBlock`] pointer.
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`GpioBlock`].
    pub const unsafe fn from_raw(raw: *mut GpioBlock, pin: u8) -> Self {
        Self { raw, pin }
    }

    /// Set pin direction (true = output, false = input).
    pub fn set_direction(&mut self, is_output: bool) {
        unsafe {
            let mask = 1 << self.pin;
            let current = read_volatile(&(*self.raw).dir);
            if is_output {
                write_volatile(&mut (*self.raw).dir, current | mask);
            } else {
                write_volatile(&mut (*self.raw).dir, current & !mask);
            }
        }
    }
}

impl embedded_hal::digital::ErrorType for Lpc17xxPin {
    type Error = core::convert::Infallible;
}

impl InputPin for Lpc17xxPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        let val = unsafe { read_volatile(&(*self.raw).pin) };
        Ok((val & (1 << self.pin)) != 0)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.is_high()?)
    }
}

impl OutputPin for Lpc17xxPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe {
            write_volatile(&mut (*self.raw).clr, 1 << self.pin);
        }
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe {
            write_volatile(&mut (*self.raw).set, 1 << self.pin);
        }
        Ok(())
    }
}

/// Power rail domain controller implementing [`PowerPort`] on GPIO2 bits 0..7.
#[derive(Clone, Copy)]
pub struct Lpc17xxPowerPort {
    raw: *mut GpioBlock,
}

impl Lpc17xxPowerPort {
    /// Point to LPC1768 physical `GPIO2_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where `GPIO2_BASE` is mapped.
    pub const unsafe fn hardware() -> Self {
        Self {
            raw: GPIO2_BASE as *mut GpioBlock,
        }
    }

    /// Create from an arbitrary pointer (for testing or re-mapped registers).
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`GpioBlock`].
    pub const unsafe fn from_raw(raw: *mut GpioBlock) -> Self {
        Self { raw }
    }
}

impl PowerPort for Lpc17xxPowerPort {
    fn write_mask(&mut self, mask: u8) -> HalResult<()> {
        let set_bits = (mask as u32) & 0xFF;
        let clr_bits = ((!mask) as u32) & 0xFF;
        unsafe {
            // Ensure bits 0..7 are configured as outputs
            let dir = read_volatile(&(*self.raw).dir);
            write_volatile(&mut (*self.raw).dir, dir | 0xFF);

            // Drive pins
            if set_bits != 0 {
                write_volatile(&mut (*self.raw).set, set_bits);
            }
            if clr_bits != 0 {
                write_volatile(&mut (*self.raw).clr, clr_bits);
            }
        }
        Ok(())
    }

    fn read_mask(&self) -> u8 {
        let pin = unsafe { read_volatile(&(*self.raw).pin) };
        (pin & 0xFF) as u8
    }
}

/// Status LED controller implementing [`LedDriver`] for the 4 on-board LEDs.
///
/// * LED1: P3.25
/// * LED2: P3.26 (heartbeat)
/// * LED3: P4.28 (signaling)
/// * LED4: P4.29
#[derive(Clone, Copy)]
pub struct Lpc17xxLedDriver {
    gpio3: *mut GpioBlock,
    gpio4: *mut GpioBlock,
}

impl Lpc17xxLedDriver {
    /// Point to LPC1768 physical `GPIO3_BASE` and `GPIO4_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where the GPIO blocks exist.
    pub const unsafe fn hardware() -> Self {
        Self {
            gpio3: GPIO3_BASE as *mut GpioBlock,
            gpio4: GPIO4_BASE as *mut GpioBlock,
        }
    }

    /// Create from arbitrary pointers (for testing).
    ///
    /// # Safety
    /// `gpio3` and `gpio4` must point to valid, aligned [`GpioBlock`] instances.
    pub const unsafe fn from_raw(gpio3: *mut GpioBlock, gpio4: *mut GpioBlock) -> Self {
        Self { gpio3, gpio4 }
    }

    /// Configure LED pins as outputs.
    pub fn init_pins(&mut self) {
        unsafe {
            // Port 3 pins 25, 26
            let d3 = read_volatile(&(*self.gpio3).dir);
            write_volatile(&mut (*self.gpio3).dir, d3 | (1 << 25) | (1 << 26));

            // Port 4 pins 28, 29
            let d4 = read_volatile(&(*self.gpio4).dir);
            write_volatile(&mut (*self.gpio4).dir, d4 | (1 << 28) | (1 << 29));
        }
    }
}

impl LedDriver for Lpc17xxLedDriver {
    fn set_led(&mut self, led: Led, on: bool) -> HalResult<()> {
        unsafe {
            match led {
                Led::Led1 => {
                    let reg = if on {
                        &mut (*self.gpio3).set
                    } else {
                        &mut (*self.gpio3).clr
                    };
                    write_volatile(reg, 1 << 25);
                }
                Led::Led2 => {
                    let reg = if on {
                        &mut (*self.gpio3).set
                    } else {
                        &mut (*self.gpio3).clr
                    };
                    write_volatile(reg, 1 << 26);
                }
                Led::Led3 => {
                    let reg = if on {
                        &mut (*self.gpio4).set
                    } else {
                        &mut (*self.gpio4).clr
                    };
                    write_volatile(reg, 1 << 28);
                }
                Led::Led4 => {
                    let reg = if on {
                        &mut (*self.gpio4).set
                    } else {
                        &mut (*self.gpio4).clr
                    };
                    write_volatile(reg, 1 << 29);
                }
            }
        }
        Ok(())
    }

    fn is_led_on(&self, led: Led) -> bool {
        unsafe {
            match led {
                Led::Led1 => (read_volatile(&(*self.gpio3).pin) & (1 << 25)) != 0,
                Led::Led2 => (read_volatile(&(*self.gpio3).pin) & (1 << 26)) != 0,
                Led::Led3 => (read_volatile(&(*self.gpio4).pin) & (1 << 28)) != 0,
                Led::Led4 => (read_volatile(&(*self.gpio4).pin) & (1 << 29)) != 0,
            }
        }
    }
}

/// Watchdog feeder adapter implementing [`WatchdogFeed`].
#[derive(Clone, Copy)]
pub struct Lpc17xxWatchdog {
    raw: *mut WatchdogBlock,
}

impl Lpc17xxWatchdog {
    /// Point to LPC1768 physical `WDT_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where `WDT_BASE` exists.
    pub const unsafe fn hardware() -> Self {
        Self {
            raw: WDT_BASE as *mut WatchdogBlock,
        }
    }

    /// Create from raw pointer for testing.
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`WatchdogBlock`].
    pub const unsafe fn from_raw(raw: *mut WatchdogBlock) -> Self {
        Self { raw }
    }
}

impl WatchdogFeed for Lpc17xxWatchdog {
    fn write_byte(&mut self, byte: u8) {
        unsafe {
            write_volatile(&mut (*self.raw).feed, byte as u32);
        }
    }
}

/// Real-Time Clock adapter implementing [`RtcRegisters`].
#[derive(Clone, Copy)]
pub struct Lpc17xxRtc {
    raw: *mut RtcBlock,
}

impl Lpc17xxRtc {
    /// Point to LPC1768 physical `RTC_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where `RTC_BASE` exists.
    pub const unsafe fn hardware() -> Self {
        Self {
            raw: RTC_BASE as *mut RtcBlock,
        }
    }

    /// Create from raw pointer for testing.
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`RtcBlock`].
    pub const unsafe fn from_raw(raw: *mut RtcBlock) -> Self {
        Self { raw }
    }

    /// Read and decode current real-time clock structure.
    pub fn current_time(&mut self) -> TimeStruct {
        let (c0, c1, c2) = self.read_ctime();
        TimeStruct::from_ctime(c0, c1, c2)
    }
}

impl RtcRegisters for Lpc17xxRtc {
    fn read_ctime(&mut self) -> (u32, u32, u32) {
        unsafe {
            let c0 = read_volatile(&(*self.raw).ctime0);
            let c1 = read_volatile(&(*self.raw).ctime1);
            let c2 = read_volatile(&(*self.raw).ctime2);
            (c0, c1, c2)
        }
    }
}

/// System Timer adapter implementing [`MonotonicClock`].
#[derive(Clone, Copy)]
pub struct Lpc17xxTimer {
    raw: *mut TimerBlock,
}

impl Lpc17xxTimer {
    /// Point to LPC1768 physical `TIMER0_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where `TIMER0_BASE` exists.
    pub const unsafe fn hardware() -> Self {
        Self {
            raw: TIMER0_BASE as *mut TimerBlock,
        }
    }

    /// Create from raw pointer for testing.
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`TimerBlock`].
    pub const unsafe fn from_raw(raw: *mut TimerBlock) -> Self {
        Self { raw }
    }
}

impl MonotonicClock for Lpc17xxTimer {
    fn now_us(&mut self) -> u64 {
        unsafe { read_volatile(&(*self.raw).tc) as u64 }
    }
}

/// EMAC Ethernet transceiver implementing [`EthernetDevice`].
pub struct Lpc17xxEmac {
    raw: *mut EmacBlock,
    tx_buf: [u8; 1536],
    rx_buf: [u8; 1536],
    rx_len: usize,
}

impl Lpc17xxEmac {
    /// Point to LPC1768 physical `EMAC_BASE`.
    ///
    /// # Safety
    /// Must only be called on target hardware where `EMAC_BASE` exists.
    pub const unsafe fn hardware() -> Self {
        Self {
            raw: EMAC_BASE as *mut EmacBlock,
            tx_buf: [0; 1536],
            rx_buf: [0; 1536],
            rx_len: 0,
        }
    }

    /// Create from raw pointer for testing.
    ///
    /// # Safety
    /// `raw` must point to a valid, aligned [`EmacBlock`].
    pub const unsafe fn from_raw(raw: *mut EmacBlock) -> Self {
        Self {
            raw,
            tx_buf: [0; 1536],
            rx_buf: [0; 1536],
            rx_len: 0,
        }
    }

    /// Access raw transmit buffer (for tests/driver setup).
    pub fn tx_buffer(&self) -> &[u8] {
        &self.tx_buf
    }

    /// Inject an incoming frame into the internal receive buffer (for tests).
    pub fn inject_rx_frame(&mut self, frame: &[u8]) {
        let len = frame.len().min(self.rx_buf.len());
        self.rx_buf[..len].copy_from_slice(&frame[..len]);
        self.rx_len = len;
    }
}

impl EthernetDevice for Lpc17xxEmac {
    fn transmit(&mut self, packet: &[u8]) -> HalResult<()> {
        if packet.len() > self.tx_buf.len() {
            return Err(HalError::Bus);
        }
        self.tx_buf[..packet.len()].copy_from_slice(packet);
        unsafe {
            // Signal produce index update to hardware DMA
            let prod = read_volatile(&(*self.raw).tx_produce_index);
            write_volatile(&mut (*self.raw).tx_produce_index, prod.wrapping_add(1));
        }
        Ok(())
    }

    fn receive(&mut self, buffer: &mut [u8]) -> HalResult<Option<usize>> {
        if self.rx_len > 0 {
            let len = self.rx_len.min(buffer.len());
            buffer[..len].copy_from_slice(&self.rx_buf[..len]);
            self.rx_len = 0;
            unsafe {
                let cons = read_volatile(&(*self.raw).rx_consume_index);
                write_volatile(&mut (*self.raw).rx_consume_index, cons.wrapping_add(1));
            }
            Ok(Some(len))
        } else {
            Ok(None)
        }
    }
}

/// Unified LPC1768 peripherals container.
pub struct Lpc17xxPeripherals {
    pub power_port: Lpc17xxPowerPort,
    pub leds: Lpc17xxLedDriver,
    pub watchdog: Lpc17xxWatchdog,
    pub rtc: Lpc17xxRtc,
    pub timer: Lpc17xxTimer,
    pub emac: Lpc17xxEmac,
}

impl Lpc17xxPeripherals {
    /// Instantiate wrappers referencing the physical LPC1768 MMIO peripheral registers.
    ///
    /// # Safety
    /// Must only be called once during bare-metal startup on LPC1768 hardware.
    pub unsafe fn hardware() -> Self {
        Self {
            power_port: Lpc17xxPowerPort::hardware(),
            leds: Lpc17xxLedDriver::hardware(),
            watchdog: Lpc17xxWatchdog::hardware(),
            rtc: Lpc17xxRtc::hardware(),
            timer: Lpc17xxTimer::hardware(),
            emac: Lpc17xxEmac::hardware(),
        }
    }
}

// ---------------------------------------------------------------------------
// Unit Tests (using simulated MMIO blocks)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_set_high_low_and_read() {
        let mut block = GpioBlock {
            dir: 0,
            _pad0: [0; 3],
            mask: 0,
            pin: 0,
            set: 0,
            clr: 0,
        };

        let mut pin = unsafe { Lpc17xxPin::from_raw(&mut block, 3) };
        pin.set_direction(true);
        assert_eq!(block.dir, 1 << 3);

        pin.set_high().unwrap();
        assert_eq!(block.set, 1 << 3);

        pin.set_low().unwrap();
        assert_eq!(block.clr, 1 << 3);

        unsafe { write_volatile(&mut block.pin, 1 << 3) };
        assert!(pin.is_high().unwrap());
        assert!(!pin.is_low().unwrap());

        unsafe { write_volatile(&mut block.pin, 0) };
        assert!(pin.is_low().unwrap());
        assert!(!pin.is_high().unwrap());
    }

    #[test]
    fn power_port_mask_write_and_read() {
        let mut block = GpioBlock {
            dir: 0,
            _pad0: [0; 3],
            mask: 0,
            pin: 0,
            set: 0,
            clr: 0,
        };

        let mut port = unsafe { Lpc17xxPowerPort::from_raw(&mut block) };
        port.write_mask(0b1010_0101).unwrap();

        assert_eq!(block.dir & 0xFF, 0xFF);
        assert_eq!(block.set, 0b1010_0101);
        assert_eq!(block.clr, 0b0101_1010);

        unsafe { write_volatile(&mut block.pin, 0xAA) };
        assert_eq!(port.read_mask(), 0xAA);
    }

    #[test]
    fn led_driver_sets_correct_pins_on_ports_3_and_4() {
        let mut g3 = GpioBlock {
            dir: 0,
            _pad0: [0; 3],
            mask: 0,
            pin: 0,
            set: 0,
            clr: 0,
        };
        let mut g4 = GpioBlock {
            dir: 0,
            _pad0: [0; 3],
            mask: 0,
            pin: 0,
            set: 0,
            clr: 0,
        };

        let mut driver = unsafe { Lpc17xxLedDriver::from_raw(&mut g3, &mut g4) };
        driver.init_pins();
        assert_eq!(g3.dir, (1 << 25) | (1 << 26));
        assert_eq!(g4.dir, (1 << 28) | (1 << 29));

        driver.set_led(Led::Led1, true).unwrap();
        assert_eq!(g3.set, 1 << 25);
        driver.set_led(Led::Led2, false).unwrap();
        assert_eq!(g3.clr, 1 << 26);

        driver.set_led(Led::Led3, true).unwrap();
        assert_eq!(g4.set, 1 << 28);
        driver.set_led(Led::Led4, false).unwrap();
        assert_eq!(g4.clr, 1 << 29);

        unsafe { write_volatile(&mut g3.pin, 1 << 25) };
        assert!(driver.is_led_on(Led::Led1));
        assert!(!driver.is_led_on(Led::Led2));
    }

    #[test]
    fn watchdog_feed_sequence() {
        let mut block = WatchdogBlock {
            mod_: 0,
            tc: 0,
            feed: 0,
            tv: 0,
        };

        let mut wdt = unsafe { Lpc17xxWatchdog::from_raw(&mut block) };
        wdt.write_byte(0xAA);
        assert_eq!(block.feed, 0xAA);
        wdt.write_byte(0x55);
        assert_eq!(block.feed, 0x55);
    }

    #[test]
    fn rtc_ctime_read_and_decode() {
        let mut block = RtcBlock {
            ilr: 0,
            _pad0: 0,
            ccr: 0,
            ciir: 0,
            amr: 0,
            ctime0: (6 << 24) | (14 << 16) | (30 << 8) | 15,
            ctime1: (2024 << 16) | (11 << 8) | 7,
            ctime2: 312,
            sec: 0,
            min: 0,
            hour: 0,
            dom: 0,
            dow: 0,
            doy: 0,
            month: 0,
            year: 0,
        };

        let mut rtc = unsafe { Lpc17xxRtc::from_raw(&mut block) };
        let (c0, c1, c2) = rtc.read_ctime();
        assert_eq!(c0, (6 << 24) | (14 << 16) | (30 << 8) | 15);
        assert_eq!(c1, (2024 << 16) | (11 << 8) | 7);
        assert_eq!(c2, 312);

        let t = rtc.current_time();
        assert_eq!(t.year, 2024);
        assert_eq!(t.month, 11);
        assert_eq!(t.day_of_month, 7);
        assert_eq!(t.hour, 14);
        assert_eq!(t.minute, 30);
        assert_eq!(t.second, 15);
    }

    #[test]
    fn timer_counter_read() {
        let mut block = TimerBlock {
            ir: 0,
            tcr: 0,
            tc: 1234567,
            pr: 0,
            pc: 0,
            mcr: 0,
            mr0: 0,
        };

        let mut timer = unsafe { Lpc17xxTimer::from_raw(&mut block) };
        assert_eq!(timer.now_us(), 1234567);
    }

    #[test]
    fn emac_transmit_and_receive() {
        let mut block = EmacBlock {
            mac1: 0,
            mac2: 0,
            ipgt: 0,
            ipgr: 0,
            clrt: 0,
            maxf: 0,
            supp: 0,
            test: 0,
            mcfg: 0,
            mcmd: 0,
            madr: 0,
            mwtd: 0,
            mrdd: 0,
            mind: 0,
            _pad0: [0; 2],
            command: 0,
            status: 0,
            rx_descriptor: 0,
            rx_status: 0,
            rx_descriptor_number: 0,
            rx_produce_index: 0,
            rx_consume_index: 0,
            tx_descriptor: 0,
            tx_status: 0,
            tx_descriptor_number: 0,
            tx_produce_index: 0,
            tx_consume_index: 0,
        };

        let mut emac = unsafe { Lpc17xxEmac::from_raw(&mut block) };
        emac.transmit(b"HELLO PACKET").unwrap();
        assert_eq!(&emac.tx_buffer()[..12], b"HELLO PACKET");
        assert_eq!(block.tx_produce_index, 1);

        emac.inject_rx_frame(b"INCOMING PACKET");
        let mut buf = [0u8; 64];
        let len = emac.receive(&mut buf).unwrap().unwrap();
        assert_eq!(&buf[..len], b"INCOMING PACKET");
        assert_eq!(block.rx_consume_index, 1);

        assert_eq!(emac.receive(&mut buf).unwrap(), None);
    }
}
