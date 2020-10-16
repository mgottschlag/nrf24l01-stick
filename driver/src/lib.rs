use std::io;
use std::path::Path;
use std::thread;
use std::time::Duration;

use thiserror::Error;
use tokio_serial::{Serial, SerialPortSettings};

use nrf24l01_stick_protocol::Packet;
pub use nrf24l01_stick_protocol::{Configuration, CrcMode, DataRate};

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
        if let Packet::ResetDone = nrf.command(Packet::Reset, true).await? {
            // TODO: Check version information in response packet.
        } else {
            // TODO: Error
        }

        // TODO: Also set the address length

        // Apply the initial configuration.
        nrf.configure(config).await?;

        Ok(Standby { nrf })
    }

    pub async fn open_default(config: Configuration) -> Result<Standby, Error> {
        Self::open(DEFAULT_TTY, config).await
    }

    async fn command(&mut self, _packet: Packet, _discard_rx: bool) -> Result<Packet, Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    async fn command_ack(&mut self, packet: Packet, _discard_rx: bool) -> Result<(), Error> {
        let response = self.command(packet, _discard_rx).await?;
        match response {
            Packet::Ack => Ok(()),
            _ => {
                // TODO: Error
                Ok(())
            }
        }
    }

    async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.command_ack(Packet::Config(config), true).await
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
        _a1: Option<Address>,
        _a2: Option<Address>,
        _a3: Option<Address>,
        _a4: Option<Address>,
        _a5: Option<Address>,
    ) -> Result<(), Error> {
        // TODO: self.nrf.address_len = config.address_len as usize;
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn receive(self) -> Result<Receiver, Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.nrf.configure(config).await
    }
}

pub struct Sender {
    nrf: NRF24L01,
}

impl Sender {
    pub async fn send(&mut self, _address: Address, _payload: &[u8]) -> Result<(), Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn receive(self) -> Result<Receiver, Error> {
        // TODO
        panic!("Not yet implemented.");
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
                    // TODO
                }
            }
        }
    }

    pub async fn send(self) -> Result<Sender, Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn standby(mut self) -> Result<Standby, Error> {
        self.nrf.command_ack(Packet::Standby, true).await?;
        Ok(Standby { nrf: self.nrf })
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
}
