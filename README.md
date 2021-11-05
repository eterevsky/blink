# Blinking some LEDs on Raspberry Pi Pico

A minimal Rust firmware for Raspberry Pi Pico that blinks the onboard LED and writes messages to USB serial. Based on [rp2040-project-template](https://github.com/rp-rs/rp2040-project-template).

## Requirements

```
rustup target install thumbv6m-none-eabi
cargo install --locked elf2uf2-rs
```

## Running

* Connect Raspberry Pi Pico by USB while holding BOOTSEL.
* Optionally connect LEDs to GPIO14 and GPIO15 via 50 Ohm resistors.

```
cargo run --release
```

(For some reason USB works only in release build.)