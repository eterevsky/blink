# Blinking some LEDs on Raspberry Pi Pico

A minimal Rust firmware for Raspberry Pi Pico that blinks the onboard LED and writes messages to USB serial. Based on [rp2040-project-template](https://github.com/rp-rs/rp2040-project-template).

## Compatibility

Tested with:

* Raspberry Pi Pico
* Pimoroni Pico LiPo
* Arduino Nano RP2040 Connect (doesn't have a LED on the same port)

and

* Windows 10/11
* macOS

## Requirements

```
rustup target install thumbv6m-none-eabi
cargo install --locked elf2uf2-rs
```

## Running

Connect Raspberry Pi Pico by USB while holding BOOTSEL (for Arduino Nano Connect ground 13th pin).

```
cargo run --release
```

(For some reason USB works only in release build.)