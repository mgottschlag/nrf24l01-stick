use core::convert::Infallible;

use cortex_m::asm::delay;
use embedded_hal::digital::v2::InputPin;
use embedded_nrf24l01::{self, StandbyMode, NRF24L01};
use stm32f1xx_hal::afio::Parts;
use stm32f1xx_hal::gpio::gpioa::{PA5, PA6, PA7};
use stm32f1xx_hal::gpio::gpiob::{PB0, PB1, PB10};
use stm32f1xx_hal::gpio::{Alternate, Edge, ExtiPin, Floating, Input, Output, PullUp, PushPull};
use stm32f1xx_hal::pac::{EXTI, SPI1};
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
    irq: RadioIrq,
    next_command_len: Option<usize>,
    next_command_received: usize,
    command_buffer: [u8; 256],
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
        afio: &mut Parts,
        exti: &mut EXTI,
    ) -> Self {
        // Initialize SPI.
        let spi_mode = spi::Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi = Spi::spi1(
            spi,
            (sck, miso, mosi),
            &mut afio.mapr,
            spi_mode,
            500.khz(),
            clocks,
            apb2,
        );

        // Power up the NRF module and load a default configuration.
        let nrf = NRF24L01::new(ce, cs, spi).unwrap();
        delay(clocks.sysclk().0 / 100);

        let mut adapter = Adapter {
            nrf,
            irq,
            next_command_len: None,
            next_command_received: 0,
            command_buffer: [0; 256],
        };
        adapter.configure_radio(Configuration::default());

        // We want to be interrupted whenever the IRQ line is pulled low.
        adapter.irq.make_interrupt_source(afio);
        adapter.irq.trigger_on_edge(exti, Edge::FALLING);
        adapter.irq.clear_interrupt_pending_bit();
        adapter.irq.enable_interrupt(exti);

        adapter
    }

    pub fn poll_radio(&mut self) {
        // TODO

        // Reset interrupt pending bit whenever line is high to simulate level-triggered
        // interrupts. Also reset the pending interrupt bit when the radio is in standby mode.
        if self.irq.is_high().unwrap() {
            self.irq.clear_interrupt_pending_bit();
        }
        // Also reset the interrupt bit
    }

    pub fn data_from_serial(&mut self, mut data: &[u8]) {
        while data.len() > 0 {
            if let Some(len) = self.next_command_len {
                let received = self.next_command_received;
                let remaining = len - received;
                if remaining >= data.len() {
                    self.command_buffer[received..len].copy_from_slice(&data[..remaining]);
                    self.deserialize_command();
                    data = &data[remaining..];
                    self.next_command_len = None;
                } else {
                    self.command_buffer[received..received + data.len()].copy_from_slice(data);
                    self.next_command_received += data.len();
                    break;
                }
            } else {
                // The first byte of a packet encodes the length-1.
                self.next_command_len = Some(data[0] as usize + 1);
                self.next_command_received = 0;
                data = &data[1..];
            }
        }
    }

    fn deserialize_command(&mut self) {
        let _command = &self.command_buffer[0..self.next_command_len.unwrap()];
        // TODO
    }

    pub fn send_usb_serial(&mut self, _serial: &mut usbd_serial::SerialPort<'static, UsbBusType>) {
        // TODO
        /*match serial.write(&buf[write_offset..count]) {
            Ok(len) if len > 0 => {
                write_offset += len;
            }
            _ => {}
        }*/
    }

    pub fn configure_radio(&mut self, _config: Configuration) {
        // TODO: Error?
        // TODO
    }
}
