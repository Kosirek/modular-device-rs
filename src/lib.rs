//! Rust port of the LPC1768 **UM** sensor-logging firmware.
//!
//! The original C application talks directly to LPC17xx peripheral registers.
//! This port keeps the *exact same feature set* but applies a few engineering
//! principles so the logic can be developed and tested on a host machine:
//!
//! * **KISS** – each module mirrors one C translation unit and exposes a small,
//!   obvious API.
//! * **SOLID / Dependency Inversion** – hardware access goes through the traits
//!   in [`hal`]. Drivers depend on those traits, never on concrete registers,
//!   so they can be exercised with [`mockall`] mocks or hand-written fakes.
//! * **Globals when feasible** – peripheral-style singletons (e.g. the GPS
//!   parser state, the system clock) are modelled as owned structs that the
//!   demo `main` wires together once, matching the "single global instance"
//!   nature of the firmware without resorting to `static mut`.
//!
//! Every module ships with unit tests (`rstest` for table-driven cases,
//! `mockall`/`automock` for the bus traits, plus plain fakes where that is
//! simpler and clearer).

// The library is `no_std` on the target (LPC1768 / Cortex-M3). `std` is only
// pulled in for the test harness and the host demo binary. Dynamic containers
// (`Vec`, `String`) come from `alloc`, which the firmware image backs with a
// global allocator.
#![cfg_attr(not(test), no_std)]

extern crate alloc;

pub mod hal;

pub mod accelerometer;
pub mod app;
pub mod arp;
pub mod commands;
pub mod compass;
pub mod dhcp;
pub mod eeprom;
pub mod gps;
pub mod keys;
pub mod leds;
pub mod logs;
pub mod lpc17xx;
pub mod net;
pub mod net_stack;
pub mod nmea;
pub mod nvram;
pub mod phy;
pub mod power;
pub mod pressure;
pub mod rtc;
pub mod storage;
pub mod telemetry;
pub mod temperature;
pub mod timer;
pub mod udp;
pub mod watchdog;
