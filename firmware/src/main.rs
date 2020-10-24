//! Firmware for the NRF24L01 stick.
#![no_main]
#![no_std]

use cortex_m::asm::delay;
use embedded_hal::digital::v2::OutputPin;
use panic_semihosting as _;
use stm32f1xx_hal::gpio::{gpioc, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;

use adapter::Adapter;

mod adapter;

#[rtic::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        led: gpioc::PC13<Output<PushPull>>,
        adapter: Adapter,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.constrain();
        let mut afio = ctx.device.AFIO.constrain(&mut rcc.apb2);
        let mut exti = ctx.device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut gpioa = ctx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc.apb2);

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().ok(); // Turn on

        // Force reset using the pull-up resistor on the D+ line (only for development).
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().unwrap();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));

        let serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());

        // This PID/VID combination is selected from the pid.codes PID space and only intended for
        // software development. It is not universally unique and should not be used outside of
        // test environments!
        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x000d))
            .manufacturer("Mathias Gottschlag")
            .product("nrf24l01-stick")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let radio_irq = gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        let radio_cs = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        let radio_ce = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
        let radio_sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let radio_miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        let radio_mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
        let adapter = Adapter::new(
            ctx.device.SPI1,
            radio_irq,
            radio_cs,
            radio_ce,
            radio_sck,
            radio_miso,
            radio_mosi,
            clocks,
            &mut rcc.apb2,
            &mut afio,
            &mut exti,
        );

        init::LateResources {
            usb_dev,
            serial,
            led,
            adapter,
        }
    }

    #[task(binds = EXTI9_5, resources = [usb_dev, serial, adapter])]
    fn radio_irq(mut ctx: radio_irq::Context) {
        ctx.resources.adapter.poll_radio();
        // We might have to send packets to the host.
        usb_poll(
            &mut ctx.resources.usb_dev,
            &mut ctx.resources.serial,
            ctx.resources.adapter,
        );
    }

    #[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial, adapter])]
    fn usb_tx(mut ctx: usb_tx::Context) {
        usb_poll(
            &mut ctx.resources.usb_dev,
            &mut ctx.resources.serial,
            ctx.resources.adapter,
        );
    }

    #[task(binds = USB_LP_CAN_RX0, resources = [usb_dev, serial, adapter])]
    fn usb_rx0(mut ctx: usb_rx0::Context) {
        usb_poll(
            &mut ctx.resources.usb_dev,
            &mut ctx.resources.serial,
            ctx.resources.adapter,
        );
    }
};

fn usb_poll(
    usb_dev: &mut UsbDevice<'static, UsbBusType>,
    serial: &mut usbd_serial::SerialPort<'static, UsbBusType>,
    adapter: &mut Adapter,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 64];

    while let Ok(count) = serial.read(&mut buf) {
        adapter.data_from_serial(&buf[0..count]);
    }
    adapter.send_usb_serial(serial);
    // TODO: Do we have to poll again after a serial write?
}
