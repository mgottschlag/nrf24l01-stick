use std::io;
use std::path::Path;
use std::thread;
use std::time::Duration;

use thiserror::Error;
use tokio_serial::{Serial, SerialPortSettings};

pub use nrf24l01_stick_protocol::{Configuration, CrcMode, DataRate};
use nrf24l01_stick_protocol::{Packet, RadioPacket, CURRENT_VERSION, DEVICE_ID};

pub const MAX_PAYLOAD_LEN: usize = 32;
const DEFAULT_TTY: &str = "/dev/ttyUSB_nrf24l01";

pub struct NRF24L01 {
    port: Serial,
    addr_len: usize,
}

impl NRF24L01 {
    pub async fn open<P: AsRef<Path>>(device: P, config: Configuration) -> Result<Standby, Error> {
        // Open the serial port.
        let settings = SerialPortSettings::default();
        let mut port = Serial::from_path(device, &settings).unwrap();
        port.set_exclusive(true)?;
        let mut nrf = NRF24L01 { port, addr_len: 5 };

        // Wait a second to let the device synchronize to the packet protocol.
        thread::sleep(Duration::from_secs(1));

        // Identify the device and try to reset the radio module.
        if let Packet::ResetDone(version) = nrf.command(Packet::Reset, true).await? {
            if version.device != DEVICE_ID {
                return Err(Error::Device(format!(
                    "the device does not seem to be an NRF24L01 stick, device id {} should be {}",
                    version.device, DEVICE_ID
                )));
            }
            if version.version != CURRENT_VERSION {
                return Err(Error::Device(format!(
                    "wrong device version, version {} should be {}",
                    version.version, CURRENT_VERSION
                )));
            }
        } else {
            return Err(Error::Device(
                "reset failed, unexpected response".to_owned(),
            ));
        }

        // Apply the initial configuration.
        nrf.configure(config).await?;

        Ok(Standby { nrf })
    }

    pub async fn open_default(config: Configuration) -> Result<Standby, Error> {
        Self::open(DEFAULT_TTY, config).await
    }

    async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.addr_len = config.addr_len as usize;
        self.command_ack(Packet::Config(config), true).await
    }

    async fn command(&mut self, _packet: Packet, _discard_rx: bool) -> Result<Packet, Error> {
        // TODO: Discard RX queue?
        // TODO
        panic!("Not yet implemented.");
    }

    async fn command_ack(&mut self, packet: Packet, discard_rx: bool) -> Result<(), Error> {
        let response = self.command(packet, discard_rx).await?;
        match response {
            Packet::Ack => Ok(()),
            _ => Err(Error::Device(
                "unexpected response type instead of ACK".to_owned(),
            )),
        }
    }

    async fn read_packet(&mut self, timeout: Option<Duration>) -> Result<Packet, Error> {
        // TODO
        panic!("Not yet implemented.");
    }
}

impl Drop for NRF24L01 {
    fn drop(&mut self) {
        // Try to place the radio chip in standby.
        // TODO
    }
}

pub struct Standby {
    nrf: NRF24L01,
}

impl Standby {
    pub async fn set_receive_addr(
        &mut self,
        a1: Option<Address>,
        a2: Option<Address>,
        a3: Option<Address>,
        a4: Option<Address>,
        a5: Option<Address>,
    ) -> Result<(), Error> {
        let addresses = [a1, a2, a3, a4, a5];
        let mut addr_bytes = [None, None, None, None, None];
        for (i, addr) in addresses.iter().enumerate() {
            if let Some(addr) = addr {
                if addr.length as usize != self.nrf.addr_len {
                    return Err(Error::InvalidParam("wrong address length".to_owned()));
                }
                addr_bytes[i] = Some(addr.addr);
            }
        }
        self.nrf
            .command_ack(Packet::SetAddress(addr_bytes), true)
            .await
    }

    pub async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.nrf.configure(config).await
    }

    pub async fn receive(mut self) -> Result<Receiver, Error> {
        self.nrf.command_ack(Packet::RX, true).await?;
        Ok(Receiver { nrf: self.nrf })
    }

    pub async fn send(mut self) -> Result<Sender, Error> {
        self.nrf.command_ack(Packet::TX, true).await?;
        Ok(Sender { nrf: self.nrf })
    }
}

pub struct Sender {
    nrf: NRF24L01,
}

impl Sender {
    pub async fn send(&mut self, address: Address, payload: &[u8]) -> Result<(), Error> {
        let mut payload_array = [0u8; 32];
        payload_array[0..payload.len()].copy_from_slice(payload);
        let response = self
            .nrf
            .command(
                Packet::Send(RadioPacket {
                    addr: address.addr,
                    length: payload.len() as u8,
                    payload: payload_array,
                }),
                false,
            )
            .await?;
        match response {
            Packet::Ack => Ok(()),
            Packet::PacketLost => Err(Error::PacketLost),
            _ => Err(Error::Device(
                "unexpected packet from device during send operation".to_owned(),
            )),
        }
    }

    pub async fn receive(mut self) -> Result<Receiver, Error> {
        self.nrf.command_ack(Packet::RX, true).await?;
        Ok(Receiver { nrf: self.nrf })
    }

    pub async fn standby(mut self) -> Result<Standby, Error> {
        self.nrf.command_ack(Packet::Standby, true).await?;
        Ok(Standby { nrf: self.nrf })
    }
}

pub struct Receiver {
    nrf: NRF24L01,
}

impl Receiver {
    pub async fn receive(&mut self) -> Result<ReceivedPacket, Error> {
        // TODO: Packets received during the last commands (e.g., during the last TX operation?)
        loop {
            let packet = self.nrf.read_packet(None).await?;
            match packet {
                Packet::Receive(packet) => {
                    return Ok(ReceivedPacket {
                        addr: (&packet.addr[0..self.nrf.addr_len]).into(),
                        payload: packet.payload.to_vec(),
                    });
                }
                _ => {
                    // We should never receive any other type of packet, so we should treat this as
                    // an error.
                    return Err(Error::Device(
                        "received other packet from device during receive operation".to_owned(),
                    ));
                }
            }
        }
    }

    pub async fn standby(mut self) -> Result<Standby, Error> {
        self.nrf.command_ack(Packet::Standby, true).await?;
        Ok(Standby { nrf: self.nrf })
    }

    pub async fn send(mut self) -> Result<Sender, Error> {
        self.nrf.command_ack(Packet::TX, true).await?;
        Ok(Sender { nrf: self.nrf })
    }
}

pub struct ReceivedPacket {
    pub addr: Address,
    pub payload: Vec<u8>,
}

pub struct Address {
    pub length: u8,
    pub addr: [u8; 5],
}

impl From<&[u8]> for Address {
    fn from(slice: &[u8]) -> Address {
        if slice.len() < 3 || slice.len() > 5 {
            panic!("Invalid address length.");
        }
        let mut addr = Address {
            length: 0,
            addr: [0; 5],
        };
        for i in 0..slice.len() {
            addr.addr[i] = slice[i];
        }
        addr.length = slice.len() as u8;
        addr
    }
}

#[derive(Error, Debug)]
pub enum Error {
    #[error("I/O error")]
    Io(#[from] io::Error),
    #[error("serial port error")]
    Serial(#[from] tokio_serial::Error),
    #[error("invalid parameter: {0}")]
    InvalidParam(String),
    #[error("device error: {0}")]
    Device(String),
    #[error("no ack received for packet")]
    PacketLost,
}
