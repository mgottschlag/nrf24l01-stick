use embedded_hal::digital::v2::InputPin;
use embedded_nrf24l01::{self, Configuration, RxMode, StandbyMode, TxMode, NRF24L01};
use stm32l0xx_hal::exti::{Exti, ExtiLine, GpioLine, TriggerEdge};
use stm32l0xx_hal::gpio::gpioa::{PA3, PA4, PA5, PA6, PA7, PA9};
use stm32l0xx_hal::gpio::gpiob::PB;
use stm32l0xx_hal::gpio::{Analog, Input, Output, PullUp, PushPull, Speed};
use stm32l0xx_hal::pac::SPI1;
use stm32l0xx_hal::prelude::*;
use stm32l0xx_hal::rcc::Rcc;
use stm32l0xx_hal::spi::{self, Phase, Polarity, Spi};
use stm32l0xx_hal::syscfg::SYSCFG;
use stm32l0xx_hal::usb::UsbBusType;
use void::Void;

use crate::buffer::RingBuffer;
use nrf24l01_stick_protocol as protocol;
use nrf24l01_stick_protocol::{
    CrcMode, DataRate, ErrorCode, Packet, PacketType, RxPacket, TxPacket, Version, CURRENT_VERSION,
    DEVICE_ID,
};

pub type RadioIrq = PA3<Input<PullUp>>;
pub type RadioCs = PA4<Output<PushPull>>;
pub type RadioCe = PA9<Output<PushPull>>;

pub type RadioSck = PA5<Analog>;
pub type RadioMiso = PA6<Analog>;
pub type RadioMosi = PA7<Analog>;
pub type RadioSpi = Spi<SPI1, (RadioSck, RadioMiso, RadioMosi)>;

pub type TxLed = PB<Output<PushPull>>;
pub type RxLed = PB<Output<PushPull>>;

pub struct Adapter {
    standby_nrf: Option<StandbyMode<NRF24L01<Void, RadioCe, RadioCs, RadioSpi>>>,
    rx_nrf: Option<RxMode<NRF24L01<Void, RadioCe, RadioCs, RadioSpi>>>,
    tx_nrf: Option<TxMode<NRF24L01<Void, RadioCe, RadioCs, RadioSpi>>>,
    irq: RadioIrq,
    next_command_len: Option<usize>,
    next_command_received: usize,
    command_buffer: [u8; 256],
    serial_send_buffer: RingBuffer<[u8; 1024]>,
    addr_len: usize,
    pipes_rx_enable: [bool; 6],
    tx_addr: [u8; 5],
    tx_call: u8,
    tx_was_standby: bool,
    tx_led: TxLed,
    rx_led: RxLed,
    led_on: bool,
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
        mut tx_led: TxLed,
        mut rx_led: RxLed,
        rcc: &mut Rcc,
        exti: &mut Exti,
        syscfg: &mut SYSCFG,
    ) -> Self {
        // Initialize SPI.
        let spi_mode = spi::Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let cs = cs.set_speed(Speed::VeryHigh);
        let ce = ce.set_speed(Speed::VeryHigh);
        let sck = sck.set_speed(Speed::VeryHigh);
        let miso = miso.set_speed(Speed::VeryHigh);
        let mosi = mosi.set_speed(Speed::VeryHigh);
        let spi = Spi::spi1(spi, (sck, miso, mosi), spi_mode, 500.khz(), rcc);

        // Power up the NRF module and load a default configuration.
        // TODO: Error handling.
        let mut nrf = NRF24L01::new(ce, cs, spi).unwrap();

        // Do not receive anything until we have valid addresses.
        // TODO: Error handling.
        nrf.set_pipes_rx_enable(&[false, false, false, false, false, false])
            .ok();
        let addr_len = nrf.get_address_width().unwrap() as usize;

        // Reset the TX address to match the address on the chip.
        let addr = [0; 5];
        nrf.set_tx_addr(&addr[..addr_len]).ok();
        nrf.set_rx_addr(0, &addr[0..addr_len]).ok();

        // No interrupts unless we are receiving or sending.
        // TODO: Error handling.
        nrf.set_interrupt_mask(true, true, true).ok();

        rx_led.set_high().ok();
        tx_led.set_high().ok();
        let led_on = true;

        let mut adapter = Adapter {
            standby_nrf: Some(nrf),
            rx_nrf: None,
            tx_nrf: None,
            irq,
            next_command_len: None,
            next_command_received: 0,
            command_buffer: [0; 256],
            serial_send_buffer: RingBuffer::new([0; 1024]),
            addr_len,
            pipes_rx_enable: [false; 6],
            tx_addr: [0; 5],
            tx_call: 0,
            tx_was_standby: false,
            tx_led,
            rx_led,
            led_on,
        };
        // TODO: Error handling.
        adapter
            .configure_radio(&protocol::Configuration::default())
            .ok();

        // We want to be interrupted whenever the IRQ line is pulled low.
        let line = GpioLine::from_raw_line(adapter.irq.pin_number()).unwrap();
        exti.listen_gpio(syscfg, adapter.irq.port(), line, TriggerEdge::Falling);
        Exti::unpend(GpioLine::from_raw_line(adapter.irq.pin_number()).unwrap());

        adapter
    }

    pub fn poll_radio(&mut self) {
        // Reset interrupt pending bit whenever line is high to simulate level-triggered
        // interrupts. Also reset the pending interrupt bit when the radio is in standby mode.
        if self.irq.is_high().unwrap() {
            Exti::unpend(GpioLine::from_raw_line(self.irq.pin_number()).unwrap());
            return;
        }

        if let Err(e) = self.handle_interrupts() {
            self.send_error(e);
        }
    }

    fn handle_interrupts(&mut self) -> Result<(), ErrorCode> {
        if let Some(nrf) = self.standby_nrf.as_mut() {
            // We should not be getting any interrupts, just clear them.
            nrf.clear_interrupts()
                .map_err(|_| ErrorCode::InternalError)?;
        } else if let Some(nrf) = self.rx_nrf.as_mut() {
            // We only get RX interrupts, so we can clear all of them.
            nrf.clear_interrupts()
                .map_err(|_| ErrorCode::InternalError)?;

            // We cleared the interrupt, so we need to completely empty the RX FIFOs.
            while let Some(pipe) = nrf.can_read().map_err(|_| ErrorCode::InternalError)? {
                let packet = nrf.read().map_err(|_| ErrorCode::InternalError)?;
                let mut rx_packet = RxPacket {
                    pipe,
                    length: packet.len() as u8,
                    payload: [0; 32],
                };
                rx_packet.payload[..rx_packet.length as usize]
                    .copy_from_slice(&packet[..rx_packet.length as usize]);
                Self::copy_packet_to_buffer(
                    &mut self.serial_send_buffer,
                    Packet {
                        call: 0,
                        content: PacketType::Receive(rx_packet),
                    },
                );
            }
        } else {
            let mut ack_payload = heapless::Vec::<u8, 32>::new();
            match self.tx_nrf.as_mut().unwrap().poll_send(&mut ack_payload) {
                Err(nb::Error::WouldBlock) => {
                    // Nothing to do, try again later.
                }
                Err(nb::Error::Other(_)) => {
                    self.send_error(ErrorCode::InternalError);
                }
                Ok(success) => {
                    if success {
                        if ack_payload.len() != 0 {
                            let mut rx_packet = RxPacket {
                                pipe: 0,
                                length: ack_payload.len() as u8,
                                payload: [0; 32],
                            };
                            rx_packet.payload[..rx_packet.length as usize]
                                .copy_from_slice(&ack_payload[..rx_packet.length as usize]);
                            self.send_packet_to_usb(Packet {
                                call: self.tx_call,
                                content: PacketType::ReceiveAck(Some(rx_packet)),
                            });
                        } else {
                            self.send_packet_to_usb(Packet {
                                call: self.tx_call,
                                content: PacketType::ReceiveAck(None),
                            });
                        }
                    } else {
                        self.send_packet_to_usb(Packet {
                            call: self.tx_call,
                            content: PacketType::PacketLost,
                        });
                    }

                    // We are done, so we return to the previous state.
                    let nrf = self.tx_nrf.take().unwrap();
                    let mut standby_nrf = nrf.standby().unwrap();

                    self.pipes_rx_enable[0] = false;
                    if standby_nrf
                        .set_pipes_rx_enable(&self.pipes_rx_enable)
                        .is_err()
                    {
                        self.send_error(ErrorCode::InternalError);
                    }

                    if self.tx_was_standby {
                        standby_nrf
                            .set_interrupt_mask(true, true, true)
                            .map_err(|_| ErrorCode::InternalError)?;
                        self.standby_nrf = Some(standby_nrf);
                    } else {
                        standby_nrf
                            .set_interrupt_mask(false, true, true)
                            .map_err(|_| ErrorCode::InternalError)?;

                        self.rx_nrf = Some(standby_nrf.rx().unwrap());

                        // We might have received packets, so we try to poll again.
                        self.poll_radio();
                    }

                    // We stopped executing commands during transmission, so we need to drain the
                    // command queue.
                    // TODO
                }
            }
        }
        Ok(())
    }

    pub fn data_from_serial(&mut self, mut data: &[u8]) {
        // TODO: We have to buffer the data while we are transmitting.
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
            PacketType::Send(radio_packet) => {
                self.tx_call = packet.call; // TODO: Ugly.
                let status = self.send(radio_packet);
                if let Err(e) = status {
                    Err(e)
                } else {
                    // Sending does not cause an immediate response.
                    return;
                }
            }
            _ => Err(ErrorCode::InvalidOperation),
        };
        self.send_or_error(packet.call, status);
    }

    fn send_or_error(&mut self, call: u8, status: Result<PacketType, ErrorCode>) {
        match status {
            Ok(content) => self.send_packet_to_usb(Packet { call, content }),
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
        Self::copy_packet_to_buffer(&mut self.serial_send_buffer, packet);
    }

    fn copy_packet_to_buffer(target: &mut RingBuffer<[u8; 1024]>, packet: Packet) {
        // Serialize the packet.
        let mut buffer = [0u8; 257];
        let length = packet.serialize(&mut buffer[1..]).unwrap();
        buffer[0] = (length - 1) as u8;

        // Append the data to the ring buffer.
        // TODO: Can we perform any error handling here?
        target.append(&buffer[..length + 1]).ok();
    }

    pub fn send_usb_serial(&mut self, serial: &mut usbd_serial::SerialPort<'static, UsbBusType>) {
        while self.serial_send_buffer.len() != 0 {
            if self.led_on {
                self.tx_led.set_low().ok();
                self.rx_led.set_low().ok();
                self.led_on = false;
            } else {
                self.tx_led.set_high().ok();
                self.rx_led.set_high().ok();
                self.led_on = true;
            }
            let data = self.serial_send_buffer.next_slice();
            match serial.write(data) {
                Ok(len) if len > 0 => {
                    self.serial_send_buffer.consume(len);
                }
                _ => break,
            }
        }
    }

    fn reset(&mut self) -> Result<PacketType, ErrorCode> {
        self.standby()?;
        self.configure_radio(&protocol::Configuration::default())?;
        self.set_address(&[None; 5])?;
        Ok(PacketType::ResetDone(Version {
            device: DEVICE_ID,
            version: CURRENT_VERSION,
        }))
    }

    fn configure_radio(
        &mut self,
        config: &protocol::Configuration,
    ) -> Result<PacketType, ErrorCode> {
        // Check whether we are in standby mode.
        let nrf = match self.standby_nrf.as_mut() {
            Some(nrf) => nrf,
            None => return Err(ErrorCode::InvalidState),
        };

        // Apply the configuration.
        if config.channel > 0x3f {
            return Err(ErrorCode::InvalidInput);
        }
        if config.addr_len < 3 || config.addr_len > 5 {
            return Err(ErrorCode::InvalidInput);
        }
        if config.power > 3 {
            return Err(ErrorCode::InvalidInput);
        }
        // TODO: Configurable address size?
        if config.addr_len != self.addr_len as u8 {
            return Err(ErrorCode::InvalidInput);
        }
        nrf.set_frequency(config.channel)
            .map_err(|_| ErrorCode::InternalError)?;
        nrf.set_rf(
            match config.rate {
                DataRate::R250Kbps => &embedded_nrf24l01::DataRate::R250Kbps,
                DataRate::R1Mbps => &embedded_nrf24l01::DataRate::R1Mbps,
                DataRate::R2Mbps => &embedded_nrf24l01::DataRate::R2Mbps,
            },
            config.power,
        )
        .map_err(|_| ErrorCode::InternalError)?;
        nrf.set_crc(match config.crc {
            None => embedded_nrf24l01::CrcMode::Disabled,
            Some(CrcMode::OneByte) => embedded_nrf24l01::CrcMode::OneByte,
            Some(CrcMode::TwoBytes) => embedded_nrf24l01::CrcMode::TwoBytes,
        })
        .map_err(|_| ErrorCode::InternalError)?;
        if let Some((delay, count)) = config.auto_retransmit_delay_count {
            // TODO: Check values.
            nrf.set_auto_retransmit(delay, count)
                .map_err(|_| ErrorCode::InternalError)?;
        } else {
            nrf.set_auto_retransmit(250, 3)
                .map_err(|_| ErrorCode::InternalError)?;
        }
        /*nrf.set_rx_addr(0, &[0xB3, 0xB3, 0xB3, 0xB3, 0x00])
            .map_err(|_| ErrorCode::InternalError)?;
        nrf.set_rx_addr(1, &[0xB3, 0xB3, 0xB3, 0xB3, 0x00])
            .map_err(|_| ErrorCode::InternalError)?;*/
        nrf.set_auto_ack(&[true; 6])
            .map_err(|_| ErrorCode::InternalError)?;
        nrf.set_pipes_rx_lengths(&[None; 6], true)
            .map_err(|_| ErrorCode::InternalError)?;
        nrf.flush_rx().map_err(|_| ErrorCode::InternalError)?;
        nrf.flush_tx().map_err(|_| ErrorCode::InternalError)?;

        Ok(PacketType::Ack)
    }

    fn set_address(&mut self, addresses: &[Option<[u8; 5]>; 5]) -> Result<PacketType, ErrorCode> {
        // Check whether we are in standby mode.
        let nrf = match self.standby_nrf.as_mut() {
            Some(nrf) => nrf,
            None => return Err(ErrorCode::InvalidState),
        };

        self.pipes_rx_enable = [
            false,
            addresses[0].is_some(),
            addresses[1].is_some(),
            addresses[2].is_some(),
            addresses[3].is_some(),
            addresses[4].is_some(),
        ];
        nrf.set_pipes_rx_enable(&self.pipes_rx_enable)
            .map_err(|_| ErrorCode::InternalError)?;

        // Set the addresses.
        for i in 0..5 {
            if let Some(address) = addresses[i] {
                nrf.set_rx_addr(i + 1, &address[0..self.addr_len])
                    .map_err(|_| ErrorCode::InternalError)?;
            }
        }

        Ok(PacketType::Ack)
    }

    fn standby(&mut self) -> Result<PacketType, ErrorCode> {
        if self.standby_nrf.is_some() {
            // Nothing to do, CE is already low.
            return Ok(PacketType::Ack);
        }

        let mut nrf = self.rx_nrf.take().unwrap();

        // Disable interrupts.
        nrf.set_interrupt_mask(true, true, true)
            .map_err(|_| ErrorCode::InternalError)?;

        self.standby_nrf = Some(nrf.standby());

        Ok(PacketType::Ack)
    }

    fn start_receive(&mut self) -> Result<PacketType, ErrorCode> {
        if self.rx_nrf.is_some() {
            return Ok(PacketType::Ack);
        }

        let mut nrf = self.standby_nrf.take().unwrap();

        // Enable RX interrupts.
        nrf.set_interrupt_mask(false, true, true)
            .map_err(|_| ErrorCode::InternalError)?;

        self.rx_nrf = Some(nrf.rx().unwrap());

        Ok(PacketType::Ack)
    }

    fn send(&mut self, packet: &TxPacket) -> Result<(), ErrorCode> {
        // Switch to standby, then to TX.
        let (nrf_standby, was_standby) = if let Some(nrf) = self.standby_nrf.take() {
            (nrf, true)
        } else {
            (self.rx_nrf.take().unwrap().standby(), false)
        };

        let mut nrf = nrf_standby.tx().unwrap();

        // Prevent interrupt flood if there are received packets in the FIFO.
        nrf.set_interrupt_mask(true, false, false)
            .map_err(|_| ErrorCode::InternalError)?;

        // Only update the address if we need to.
        if packet.addr != self.tx_addr {
            nrf.set_tx_addr(&packet.addr[0..self.addr_len as usize])
                .map_err(|_| ErrorCode::InternalError)?;
            nrf.set_rx_addr(0, &packet.addr[0..self.addr_len as usize])
                .map_err(|_| ErrorCode::InternalError)?;
            self.tx_addr = packet.addr;
        }
        // We want the first RX pipe to be enabled only while sending, otherwise we would receive
        // all packets targeted at the other device.
        self.pipes_rx_enable[0] = true;
        nrf.set_pipes_rx_enable(&self.pipes_rx_enable)
            .map_err(|_| ErrorCode::InternalError)?;
        while !nrf.can_send().map_err(|_| ErrorCode::InternalError)? {}
        nrf.send(&packet.payload[0..packet.length as usize], Some(0))
            .map_err(|_| ErrorCode::InternalError)?;

        // Wait until sending has failed or succeeded.
        self.tx_nrf = Some(nrf);
        self.tx_was_standby = was_standby;

        Ok(())
    }

    /*fn toggle_led(&mut self) {
        if self.led_on {
            self.rx_led.set_low().ok();
            self.led_on = false;
        } else {
            self.rx_led.set_high().ok();
            self.led_on = true;
        }
    }*/
}
