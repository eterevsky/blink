//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
//! It will also alternate the LEDs connected to GP14 and GP15.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin as _;
use embedded_time::fixed_point::FixedPoint as _;
use rp2040_hal as hal;
use rp2040_hal::{clocks::Clock as _, pac, sio::Sio, watchdog::Watchdog};
use usb_device;
// use usb_device::bus::UsbBus as _;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// External high-speed crystal on the pico board is 12Mhz
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut pin14 = pins.gpio14.into_push_pull_output();
    let mut pin15 = pins.gpio15.into_push_pull_output();

    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

     // Set up the USB driver
     let usb_bus = usb_device::bus::UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    for _ in 0..5 {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }

    match serial.write(b"Hello, World!\r\n") {
        Ok(_) => { pin14.set_high().unwrap(); },
        Err(_) => { pin15.set_high().unwrap(); }
    };

    led_pin.set_high().unwrap();


    loop {
        serial.write(b"on\n").unwrap();
        led_pin.set_high().unwrap();
        pin14.set_high().unwrap();
        pin15.set_low().unwrap();
        delay.delay_ms(500);

        serial.write(b"off\n").unwrap();
        led_pin.set_low().unwrap();
        pin14.set_low().unwrap();
        pin15.set_high().unwrap();
        delay.delay_ms(500);
    }
}
