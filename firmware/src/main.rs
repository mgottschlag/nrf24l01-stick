//! Firmware for the NRF24L01 stick.
#![no_main]
#![no_std]

use core::mem::MaybeUninit;

use embedded_hal::digital::v2::OutputPin;
use panic_semihosting as _;
use stm32f4xx_hal::gpio::{gpiog, Output, PushPull};
use stm32f4xx_hal::otg_hs::{UsbBus, USB};
use stm32f4xx_hal::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBus<USB>>> = MaybeUninit::uninit();

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        serial: usbd_serial::SerialPort<'static, UsbBus<USB>>,
        green: gpiog::PG13<Output<PushPull>>,
        red: gpiog::PG14<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let rcc = ctx.device.RCC.constrain();

        let _clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        let gpiog = ctx.device.GPIOG.split();
        let mut green = gpiog.pg13.into_push_pull_output();
        green.set_high().ok(); // Turn on
        let mut red = gpiog.pg14.into_push_pull_output();
        red.set_high().ok(); // Turn on

        let gpiob = ctx.device.GPIOB.split();

        let usb = USB {
            usb_global: ctx.device.OTG_HS_GLOBAL,
            usb_device: ctx.device.OTG_HS_DEVICE,
            usb_pwrclk: ctx.device.OTG_HS_PWRCLK,
            pin_dm: gpiob.pb14.into_alternate_af12(),
            pin_dp: gpiob.pb15.into_alternate_af12(),
            //hclk: clocks.hclk(),
        };

        unsafe { USB_BUS.as_mut_ptr().write(UsbBus::new(usb, &mut EP_MEMORY)) };

        let usb_bus = unsafe { USB_BUS.as_ptr().as_ref().unwrap() };

        let serial = usbd_serial::SerialPort::new(usb_bus);

        // This PID/VID combination is selected from the pid.codes PID space and only intended for
        // software development. It is not universally unique and should not be used outside of
        // test environments!
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x000d))
            .manufacturer("Mathias Gottschlag")
            .product("nrf24l01-stick")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        init::LateResources {
            usb_dev,
            serial,
            green,
            red,
        }
    }

    #[idle(resources = [usb_dev, serial])]
    fn idle(ctx: idle::Context) -> ! {
        let serial = ctx.resources.serial;
        let usb_dev = ctx.resources.usb_dev;
        loop {
            if !usb_dev.poll(&mut [serial]) {
                continue;
            }

            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
};
