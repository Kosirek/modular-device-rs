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

// Re-use all driver/protocol logic from the library crate.
use um_rust as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// Size of the heap in bytes. Adjust based on available RAM.
const HEAP_SIZE: usize = 4096;

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

    // Peripheral initialization and App super-loop:
    //
    // 1. Initialise LPC1768 peripheral wrappers implementing hal traits:
    //    * EMAC -> hal::EthernetDevice
    //    * SD/SPI -> hal::LogSink (via embedded-sdmmc and storage::SdLogSink)
    //    * Sensors -> Accelerometer, Compass, Temperature, Gps, Keypad
    //
    // 2. Wire into application coordinator:
    //    let net_stack = um_rust::net_stack::NetworkStack::new(emac, eth_config);
    //    let logger = um_rust::telemetry::DataLogger::new(sd_log_sink);
    //    let mut app = um_rust::app::App::new(net_stack, logger, app_config);
    //    app.start(0, &rtc_time).unwrap();
    //
    // 3. Super-loop execution:
    //    loop {
    //        let now_ticks = timer.now();
    //        app.step(now_ticks, inputs).ok();
    //        cortex_m::asm::wfi();
    //    }

    loop {
        cortex_m::asm::wfi();
    }
}
