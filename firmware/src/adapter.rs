use core::convert::Infallible;

use cortex_m::asm::delay;
use embedded_nrf24l01::{self, StandbyMode, NRF24L01};
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::gpioa::{PA5, PA6, PA7};
use stm32f1xx_hal::gpio::gpiob::{PB0, PB1, PB10};
use stm32f1xx_hal::gpio::{Alternate, Floating, Input, Output, PullUp, PushPull};
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::rcc::{Clocks, APB2};
use stm32f1xx_hal::spi::{self, Phase, Polarity, Spi, Spi1NoRemap};
use stm32f1xx_hal::usb::UsbBusType;

use nrf24l01_stick_protocol::Configuration;

pub type RadioIrq = PB0<Input<PullUp>>;
pub type RadioCs = PB1<Output<PushPull>>;
pub type RadioCe = PB10<Output<PushPull>>;

pub type RadioSck = PA5<Alternate<PushPull>>;
pub type RadioMiso = PA6<Input<Floating>>;
pub type RadioMosi = PA7<Alternate<PushPull>>;
pub type RadioSpi = Spi<SPI1, Spi1NoRemap, (RadioSck, RadioMiso, RadioMosi), u8>;

pub struct Adapter {
    nrf: StandbyMode<NRF24L01<Infallible, RadioCe, RadioCs, RadioSpi>>,
}

impl Adapter {
    pub fn new(
        spi: SPI1,
        irq: RadioIrq,
        cs: RadioCs,
        ce: RadioCe,
        sck: RadioSck,
        miso: RadioMiso,
        mosi: RadioMosi,
        clocks: Clocks,
        apb2: &mut APB2,
        mapr: &mut MAPR,
    ) -> Self {
        // Initialize SPI.
        let spi_mode = spi::Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::spi1(
            spi,
            (sck, miso, mosi),
            mapr,
            spi_mode,
            500.khz(),
            clocks,
            apb2,
        );

        // Power up the NRF module and load a default configuration.
        let nrf = NRF24L01::new(ce, cs, spi).unwrap();
        delay(clocks.sysclk().0 / 100);

        // TODO: IRQ?

        let mut adapter = Adapter { nrf };
        adapter.configure_radio(Configuration::default());
        adapter
    }

    pub fn poll_radio(&mut self) {
        // TODO
    }

    pub fn data_from_serial(&mut self) {
        // TODO
    }

    pub fn send_usb_serial(&mut self, _serial: &mut usbd_serial::SerialPort<'static, UsbBusType>) {
        // TODO
    }

    pub fn configure_radio(&mut self, config: Configuration) {
        // TODO: Error?
        // TODO
    }
}
