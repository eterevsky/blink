//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
//! It will also alternate the LEDs connected to GP14 and GP15.
#![no_std]
#![no_main]

use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint as _;
use rp2040_hal as hal;
use rp2040_hal::{clocks::Clock as _, pac, sio::Sio, watchdog::Watchdog};
use usb_device;
use usb_device::{
    bus::UsbBus,
    device::{UsbDeviceBuilder, UsbVidPid},
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// External high-speed crystal on the pico board is 12Mhz
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

fn write_usb<B: UsbBus>(
    message: &[u8],
    serial: &mut usbd_serial::SerialPort<B>,
    success_pin: &mut impl OutputPin,
    error_pin: &mut impl OutputPin,
) {
    match serial.write(message) {
        Ok(_) => {
            success_pin.set_high().ok();
            error_pin.set_low().ok();
        }
        Err(_) => {
            success_pin.set_low().ok();
            error_pin.set_high().ok();
        }
    };
}

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
    let mut green_pin = pins.gpio16.into_push_pull_output();
    let mut red_pin = pins.gpio17.into_push_pull_output();

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
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Set up the USB driver
    let usb_bus = usb_device::bus::UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000a))
        .manufacturer("Raspberry Pi")
        .product("Pico")
        .serial_number("TEST")
        .device_class(2)
        .device_protocol(1)
        .build();

    led_pin.set_high().unwrap();

    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut next_message = 2_000_000;
    let mut led_state = false;
    let mut said_hello = false;
    loop {
        if timer.get_counter() >= next_message {
            next_message += 2_000_000;
            if !said_hello {
                write_usb(
                    b"Hello, world!\n",
                    &mut serial,
                    &mut green_pin,
                    &mut red_pin,
                );
                said_hello = true;
            };

            if led_state {
                write_usb(b"On\n", &mut serial, &mut green_pin, &mut red_pin);
                led_pin.set_high().unwrap();
            } else {
                write_usb(b"Off\n", &mut serial, &mut green_pin, &mut red_pin);
                led_pin.set_low().unwrap();
            }
            led_state = !led_state;
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        let _ = serial.write(wr_ptr).map(|len| {
                            wr_ptr = &wr_ptr[len..];
                        });
                    }
                }
            }
        }
    }
}
