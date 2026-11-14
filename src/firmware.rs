//! Bare-metal firmware entry point for the LPC1768 (Cortex-M3).
//!
//! This is the real `no_std` binary that gets flashed to hardware.
//! Build with:  cargo make release  (or cargo make debug)

#![no_std]
#![no_main]

extern crate alloc;

use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use panic_halt as _;

use um_rust::app::{App, AppConfig, SensorInputs};
use um_rust::hal::{EthernetConfig, HalResult, LogSink, MonotonicClock};
use um_rust::leds::{Led, LedManager};
use um_rust::lpc17xx::Lpc17xxPeripherals;
use um_rust::net_stack::NetworkStack;
use um_rust::power::PowerManager;
use um_rust::telemetry::DataLogger;

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// Size of the heap in bytes. Adjust based on available RAM.
const HEAP_SIZE: usize = 4096;

struct FirmwareLogSink;
impl LogSink for FirmwareLogSink {
    fn write_line(&mut self, _line: &str) -> HalResult<()> {
        Ok(())
    }
    fn flush(&mut self) -> HalResult<()> {
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    // Initialise the heap allocator with a static buffer.
    {
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        #[allow(static_mut_refs)]
        unsafe {
            HEAP.init(HEAP_MEM.as_mut_ptr() as usize, HEAP_SIZE);
        }
    }

    // 1. Initialise LPC1768 hardware peripheral wrappers
    let mut periph = unsafe { Lpc17xxPeripherals::hardware() };
    periph.leds.init_pins();

    // 2. Power up default sensor rails (GPS, Compass, Acc2, SD 3V3, Temp, Pressure)
    let _power = PowerManager::with_default_board_power(periph.power_port).ok();

    // 3. Status signaling (turn on heartbeat LED)
    let mut leds = LedManager::new(periph.leds);
    leds.on(Led::Led2).ok();

    // 4. Initialise real-time clock and read boot time
    let rtc_time = periph.rtc.current_time();

    // 5. Wire into application coordinator
    let eth_config = EthernetConfig::default();
    let net = NetworkStack::new(periph.emac, eth_config);
    let logger = DataLogger::new(FirmwareLogSink);
    let mut app = App::new(net, logger, AppConfig::default());
    app.start(0, &rtc_time).ok();

    // 6. Super-loop execution
    loop {
        let now_ticks = periph.timer.now_us();
        app.step(now_ticks, SensorInputs::default()).ok();
        cortex_m::asm::wfi();
    }
}
