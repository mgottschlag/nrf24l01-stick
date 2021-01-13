use core::convert::Infallible;

use cortex_m::asm::delay;
use embedded_hal::digital::v2::InputPin;
use embedded_nrf24l01::{self, StandbyMode, NRF24L01};
use stm32f4xx_hal::gpio::gpioa::{PA5, PA6, PA7};
use stm32f4xx_hal::gpio::gpioc::{PC0, PC1, PC2};
use stm32f4xx_hal::gpio::{Alternate, Edge, ExtiPin, Input, Output, PullUp, PushPull, Speed, AF5};
use stm32f4xx_hal::otg_hs::UsbBusType;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::spi::{self, Phase, Polarity, Spi};
use stm32f4xx_hal::stm32::{EXTI, SPI1, SYSCFG};

use crate::buffer::RingBuffer;
use nrf24l01_stick_protocol::{Configuration, ErrorCode, Packet, PacketType, RadioPacket};

pub type RadioIrq = PC2<Input<PullUp>>;
pub type RadioCs = PC1<Output<PushPull>>;
pub type RadioCe = PC0<Output<PushPull>>;

pub type RadioSck = PA5<Alternate<AF5>>;
pub type RadioMiso = PA6<Alternate<AF5>>;
pub type RadioMosi = PA7<Alternate<AF5>>;
pub type RadioSpi = Spi<SPI1, (RadioSck, RadioMiso, RadioMosi)>;

pub struct Adapter {
    nrf: StandbyMode<NRF24L01<Infallible, RadioCe, RadioCs, RadioSpi>>,
    irq: RadioIrq,
    next_command_len: Option<usize>,
    next_command_received: usize,
    command_buffer: [u8; 256],
    serial_send_buffer: RingBuffer<[u8; 1024]>,
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
        exti: &mut EXTI,
        syscfg: &mut SYSCFG,
    ) -> Self {
        // Initialize SPI.
        let spi_mode = spi::Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let sck = sck.set_speed(Speed::High);
        let miso = miso.set_speed(Speed::High);
        let mosi = mosi.set_speed(Speed::High);
        let spi = Spi::spi1(spi, (sck, miso, mosi), spi_mode, 500.khz().into(), clocks);

        // Power up the NRF module and load a default configuration.
        let nrf = NRF24L01::new(ce, cs, spi).unwrap();
        delay(clocks.sysclk().0 / 100);

        let mut adapter = Adapter {
            nrf,
            irq,
            next_command_len: None,
            next_command_received: 0,
            command_buffer: [0; 256],
            serial_send_buffer: RingBuffer::new([0; 1024]),
        };
        adapter.configure_radio(&Configuration::default()).ok();

        // We want to be interrupted whenever the IRQ line is pulled low.
        adapter.irq.make_interrupt_source(syscfg);
        adapter.irq.enable_interrupt(exti);
        adapter.irq.trigger_on_edge(exti, Edge::FALLING);
        adapter.irq.clear_interrupt_pending_bit();

        adapter
    }

    pub fn poll_radio(&mut self) {
        // Reset interrupt pending bit whenever line is high to simulate level-triggered
        // interrupts. Also reset the pending interrupt bit when the radio is in standby mode.
        if self.irq.is_high().unwrap() {
            self.irq.clear_interrupt_pending_bit();
            return;
        }
        // Also reset the interrupt bit.
        // TODO

        // TODO
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
        let command = &self.command_buffer[0..self.next_command_len.unwrap()];
        match Packet::deserialize(command) {
            Some(packet) => self.process_command(&packet),
            None => self.send_error(ErrorCode::ProtocolError),
        }
    }

    fn process_command(&mut self, packet: &Packet) {
        // We only accept commands (i.e., calls which expect a response).
        if packet.call == 0 {
            self.send_error(ErrorCode::InvalidOperation);
            return;
        }

        let status = match &packet.content {
            PacketType::Reset => self.reset(),
            PacketType::Config(_config) => self.configure_radio(_config),
            PacketType::SetAddress(_addresses) => self.set_address(_addresses),
            PacketType::Standby => self.standby(),
            PacketType::StartReceive => self.start_receive(),
            PacketType::Send(radio_packet) => self.send(radio_packet),
            _ => Err(ErrorCode::InvalidOperation),
        };
        self.send_ack_or_error(packet.call, status);
    }

    fn send_ack_or_error(&mut self, call: u8, status: Result<(), ErrorCode>) {
        match status {
            Ok(_) => self.send_packet_to_usb(Packet {
                call,
                content: PacketType::Ack,
            }),
            Err(e) => self.send_error(e),
        }
    }

    fn send_error(&mut self, error: ErrorCode) {
        self.send_packet_to_usb(Packet {
            call: 0,
            content: PacketType::Error(error),
        });
    }

    fn send_packet_to_usb(&mut self, packet: Packet) {
        // Serialize the packet.
        let mut buffer = [0u8; 257];
        let length = packet.serialize(&mut buffer[1..]).unwrap();
        buffer[0] = (length - 1) as u8;

        // Append the data to the ring buffer.
        // TODO: Can we perform any error handling here?
        self.serial_send_buffer.append(&buffer[..length + 1]).ok();
    }

    pub fn send_usb_serial(&mut self, serial: &mut usbd_serial::SerialPort<'static, UsbBusType>) {
        while self.serial_send_buffer.len() != 0 {
            let data = self.serial_send_buffer.next_slice();
            match serial.write(data) {
                Ok(len) if len > 0 => {
                    self.serial_send_buffer.consume(len);
                }
                _ => break,
            }
        }
    }

    fn reset(&mut self) -> Result<(), ErrorCode> {
        // TODO
        Ok(())
    }

    fn configure_radio(&mut self, _config: &Configuration) -> Result<(), ErrorCode> {
        // Check whether we are in standby mode.
        // TODO

        // Apply the configuration.
        // TODO
        Ok(())
    }

    fn set_address(&mut self, _addresses: &[Option<[u8; 5]>; 5]) -> Result<(), ErrorCode> {
        // Check whether we are in standby mode.
        // TODO

        // Set the addresses.
        // TODO
        Ok(())
    }

    fn standby(&mut self) -> Result<(), ErrorCode> {
        // TODO
        Ok(())
    }

    fn start_receive(&mut self) -> Result<(), ErrorCode> {
        // TODO
        Ok(())
    }

    fn send(&mut self, _packet: &RadioPacket) -> Result<(), ErrorCode> {
        // TODO
        Ok(())
    }
}
