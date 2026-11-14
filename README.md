# um_rust

A Rust port of the LPC1768 **UM** sensor-logging firmware (originally in
`../um`, written in C for CrossWorks / GNU Arm). It keeps the same feature set
but restructures the code so the logic runs and is **fully unit-tested on a host
PC**.

## Why this exists

The C firmware talks straight to LPC17xx registers, so almost none of it can be
tested off-target. This port applies a few principles:

- **KISS** — one Rust module per C translation unit, with a small obvious API.
- **SOLID / Dependency Inversion** — hardware access goes through the standard
  [`embedded-hal`](https://docs.rs/embedded-hal) 1.0 traits (re-exported from
  [`src/hal.rs`](src/hal.rs)), so the drivers run against any real LPC1768 HAL
  and are driven by [`embedded-hal-mock`](https://docs.rs/embedded-hal-mock) in
  tests. Two capabilities with no `embedded-hal` equivalent (a one-shot ADC read
  and a log sink) stay as local `#[automock]` traits.
- **Reuse over reinvention** — storage uses the maintained
  [`embedded-sdmmc`](https://docs.rs/embedded-sdmmc) crate (SD-over-SPI block
  driver + FAT filesystem) instead of a hand-written SD/FAT stack.
- **Globals when feasible** — peripheral singletons (GPS parser, system clock,
  DHCP/ARP state) are owned structs wired up once in `main`, mirroring the
  firmware's single-instance globals without `static mut`.

## Feature mapping

| C source            | Rust module            | What is tested                                   |
| ------------------- | ---------------------- | ------------------------------------------------ |
| `emac.c` helpers    | `net`                  | byte swap, IP checksum, MAC/IP compare           |
| `GPS.c`             | `gps` / `nmea`         | NMEA GPGGA/GPZDA/GPRMC/GPVTG/GPGLL + framer      |
| `rtc.c`             | `rtc`                  | CTIME decode, FatFs time, ISO formatting         |
| `timer0.c`          | `timer`                | delay tick rounding                              |
| `udp.c`             | `udp`                  | frame build/parse, checksum, filtering           |
| `arp.c`             | `arp`                  | request/reply state machine                      |
| `dhcp.c`            | `dhcp`                 | DISCOVER→OFFER→REQUEST→ACK, option TLV parsin    |
| `mainPrg.c ENET_IRQ`| `net_stack`            | packet dispatching (ARP auto-reply, DHCP client, UDP routing) |
| `mainPrg.c logging` | `telemetry`            | sensor formatting (`DATA.LOG`), pressure record, batched SD logging, sync |
| `mainPrg.c main()`  | `app`                  | super-loop scheduling, SW1..SW5 keys, telemetry dispatch |
| `25lc1024.c`        | `eeprom`               | SPI command/address sequencing (`embedded-hal` SpiDevice) |
| `akcelerometr.c`    | `accelerometer`        | signed-axis conversion (`embedded-hal` SpiDevice)|
| `kompas.c`          | `compass`              | heading/pitch/roll decode (`embedded-hal` I2c)   |
| `termometr.c`       | `temperature`          | 12-bit scale + IIR filter (mocked ADC)           |
| `higro.c`           | `pressure`             | MS5534 calibration polynomial + step sequencer + I2C EEPROM/ADC driver |
| `dev_conf.h` rails  | `power`                | Peripheral power domain rails (GPIO2 pins 0..7) matching `mainPrg.c` |
| `dev_conf.h` + LEDs | `leds`                 | 4 status LEDs & `led_sig` burst patterns (`embedded-hal` delay) |
| `mainPrg.c get_key` | `keys`                 | switch bitmask combining (`embedded-hal` InputPin) |
| `wtd.c`             | `watchdog`             | WDTC reload maths + feed sequence                |
| `logs.c`            | `logs`                 | timestamped line formatting (in-memory sink)     |
| `mmcbb.c` + `ff.c`  | `storage`              | SD+FAT via `embedded-sdmmc`; append/read `DEBUG.LOG` (RAM disk) |
| `mainPrg.c UDP rx`  | `commands`             | Remote control UDP command parser (PING, STATUS, LOG, ETH, SYNC, HELP) |
| `LPC17xx registers` | `lpc17xx`              | PAC MMIO register adapters (`Lpc17xxPin`, `PowerPort`, `LedDriver`, `WatchdogFeed`, `RtcRegisters`, `MonotonicClock`, `Emac`) |

## Try it

```powershell
# from c:\projekty\um_rust
cargo test          # run every module's unit tests
cargo run --bin um_demo   # run the host demo that wires modules together
```

## Building for the LPC1768

The library is `no_std` + `alloc` whenever it is not compiled for tests, so it
cross-compiles to the Cortex-M3 core of the LPC1768
(`thumbv7m-none-eabi`). Only the library is built for the target; the `um_demo`
binary is host-only (it needs `std`/`println!`) and is gated behind the default
`std` feature.

```powershell
rustup target add thumbv7m-none-eabi
# size-optimised release library (opt-level "z" + LTO + panic=abort):
cargo build --release --target thumbv7m-none-eabi --lib --no-default-features
```

This produces `target/thumbv7m-none-eabi/release/libum_rust.rlib`. To link it
into a flashable image, an application crate provides the Cortex-M runtime
(`cortex-m-rt`), a `#[global_allocator]` (e.g. `embedded-alloc`) and a
`#[panic_handler]`, then supplies `embedded-hal` 1.0 implementations for the
LPC17xx peripherals (any LPC HAL crate, or thin register wrappers). All
protocol/driver logic in this crate stays unchanged.

The `[profile.release]` in `Cargo.toml` is tuned for the MCU: `opt-level = "z"`
(minimal size), `lto = true` and `codegen-units = 1` (cross-module inlining),
`panic = "abort"` (no unwinder), and `strip = true`.

Testing uses [`rstest`](https://crates.io/crates/rstest) for table-driven cases,
[`embedded-hal-mock`](https://crates.io/crates/embedded-hal-mock) for the
SPI/I2C/digital buses, and [`mockall`](https://crates.io/crates/mockall)
(`#[automock]`) for the two local traits (ADC, log sink), plus small hand-written
fakes where that is clearer.

## SD card + FAT storage

The C firmware logs to an SD card through ChaN's `mmcbb.c` (SD-over-SPI block
driver) and `ff.c` (FatFs). The Rust port delegates both jobs to the
[`embedded-sdmmc`](https://docs.rs/embedded-sdmmc) crate, which provides the
same stack — an `SdCard` SPI block driver plus a FAT16/FAT32 `VolumeManager` —
so none of that protocol/filesystem code is hand-maintained here.

The `storage` module adds the one thing the firmware actually needs on top:
`storage::SdLogSink` wraps a `VolumeManager` and implements `hal::LogSink` by
appending each line to `DEBUG.LOG` (open in append mode → write → close),
`logs::Logger` can write straight to the card.

On hardware you build the manager from
`embedded_sdmmc::SdCard::new(spi_device, delay)` (an `embedded-hal` `SpiDevice`
+ `DelayNs`) wrapped in a `VolumeManager`. As with the original, the card must
already be FAT-formatted with a partition table — `embedded-sdmmc` mounts, it
does not format. Tests round-trip against an in-RAM `BlockDevice` whose image is
pre-formatted (MBR + FAT16) with the `fatfs` crate, then write log lines and
read them back, including across a fresh `SdLogSink` on the same disk.
