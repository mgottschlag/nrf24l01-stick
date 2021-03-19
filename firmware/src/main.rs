//! Firmware for the NRF24L01 stick.
#![no_main]
#![no_std]

use panic_semihosting as _;
use stm32l0xx_hal::exti::Exti;
use stm32l0xx_hal::prelude::*;
use stm32l0xx_hal::rcc;
use stm32l0xx_hal::syscfg::SYSCFG;
use stm32l0xx_hal::usb::{UsbBus, UsbBusType, USB};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;

use adapter::Adapter;

mod adapter;
mod buffer;

#[rtic::app(device = stm32l0xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        adapter: Adapter,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let mut rcc = ctx.device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = SYSCFG::new(ctx.device.SYSCFG, &mut rcc);
        let hsi48 = rcc.enable_hsi48(&mut syscfg, ctx.device.CRS);
        let mut exti = Exti::new(ctx.device.EXTI);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);

        let tx_led = gpiob.pb4.into_push_pull_output();
        let rx_led = gpiob.pb5.into_push_pull_output();

        let usb = USB::new(ctx.device.USB, gpioa.pa11, gpioa.pa12, hsi48);

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

        let radio_irq = gpioa.pa3.into_pull_up_input();
        let radio_cs = gpioa.pa4.into_push_pull_output();
        let radio_ce = gpioa.pa9.into_push_pull_output();
        let radio_sck = gpioa.pa5;
        let radio_miso = gpioa.pa6;
        let radio_mosi = gpioa.pa7;
        let adapter = Adapter::new(
            ctx.device.SPI1,
            radio_irq,
            radio_cs,
            radio_ce,
            radio_sck,
            radio_miso,
            radio_mosi,
            tx_led.downgrade(),
            rx_led.downgrade(),
            &mut rcc,
            &mut exti,
            &mut syscfg,
        );

        init::LateResources {
            usb_dev,
            serial,
            adapter,
        }
    }

    /*#[task(binds = EXTI0, resources = [usb_dev, serial, adapter])]
    fn radio_irq(mut ctx: radio_irq::Context) {
        ctx.resources.adapter.poll_radio();
        // We might have to send packets to the host.
        usb_poll(
            &mut ctx.resources.usb_dev,
            &mut ctx.resources.serial,
            ctx.resources.adapter,
        );
    }*/

    /*#[task(binds = USB_HP_CAN_TX, resources = [usb_dev, serial, adapter])]
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
    }*/

    #[idle(resources = [usb_dev, serial, adapter])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            ctx.resources.adapter.poll_radio();
            // TODO: On STM32F4xx, interrupt-based USB is broken. Use interrupts on other MCUs!
            usb_poll(
                &mut ctx.resources.usb_dev,
                &mut ctx.resources.serial,
                ctx.resources.adapter,
            );
            ctx.resources
                .adapter
                .send_usb_serial(&mut ctx.resources.serial);
        }
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
