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

    // TODO: initialise LPC1768 peripherals, construct HAL trait objects,
    // and wire them into the um_rust driver modules.
    //
    // Example (pseudo-code):
    //   let spi = LpcSpi::new(..);
    //   let eeprom = um_rust::eeprom::Eeprom::new(spi);
    //   ...

    loop {
        cortex_m::asm::wfi();
    }
}
