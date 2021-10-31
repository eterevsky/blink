# Blinking some LEDs on Raspberry Pi Pico

A minimal Rust firmware for Raspberry Pi Pico. Based on [rp2040-project-template](https://github.com/rp-rs/rp2040-project-template), but without the debugging bits.

## Installing requirements

```
rustup target install thumbv6m-none-eabi
cargo install --locked elf2uf2-rs
```

## Running

* Connect Raspberry Pi Pico by USB while holding BOOTSEL.
* Optionally connect LEDs to GPIO14 and GPIO15 via 50 Ohm resistors.

```
cargo run [--release]
```