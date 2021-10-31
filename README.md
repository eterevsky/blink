# Blinking some LEDs on Raspberry Pi Pico

## Installing requirements

```
rustup target install thumbv6m-none-eabi
cargo install --git https://github.com/rp-rs/probe-run --branch rp2040-support
cargo install flip-link
cargo install elf2uf2-rs
```

## Running

* Connect Raspberry Pi Pico by USB while holding BOOTSEL.
* Optionally connect LEDs to GPIO14 and GPIO15 via 50 Ohm resistors.

```
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/blink \
           target/thumbv6m-none-eabi/release/blink.uf2
cp target/thumbv6m-none-eabi/release/blink.uf2 <RPico mount point>
```
