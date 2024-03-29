use std::io;
use std::path::Path;
use std::thread;
use std::time::Duration;

use futures::SinkExt;
use futures::StreamExt;
use thiserror::Error;
use tokio_serial::SerialStream;
use tokio_util::codec::Decoder;
use tokio_util::codec::Framed;

use codec::Codec;
pub use nrf24l01_stick_protocol::{Configuration, CrcMode, DataRate};
use nrf24l01_stick_protocol::{
    ErrorCode, Packet, PacketType, TxPacket, CURRENT_VERSION, DEVICE_ID,
};

mod codec;

pub const MAX_PAYLOAD_LEN: usize = 32;
pub const DEFAULT_TTY: &str = "/dev/ttyUSB_nrf24l01";

pub struct NRF24L01 {
    port: Framed<SerialStream, Codec>,
    addr_len: usize,
}

impl NRF24L01 {
    pub async fn open<P: AsRef<Path>>(device: P, config: Configuration) -> Result<Standby, Error> {
        // Open the serial port.
        let tty = device.as_ref().as_os_str().to_str().unwrap();
        let mut stream = SerialStream::open(&tokio_serial::new(tty, 9600)).unwrap();
        #[cfg(not(target_os = "macos"))]
        stream.set_exclusive(true)?;
        let mut nrf = NRF24L01 {
            port: Codec.framed(stream),
            addr_len: 5,
        };

        // Wait a second to let the device synchronize to the packet protocol.
        thread::sleep(Duration::from_secs(1));

        // Identify the device and try to reset the radio module.
        if let PacketType::ResetDone(version) = nrf.command(PacketType::Reset, true).await? {
            if version.device != DEVICE_ID {
                return Err(Error::Protocol(format!(
                    "the device does not seem to be an NRF24L01 stick, device id {} should be {}",
                    version.device, DEVICE_ID
                )));
            }
            if version.version != CURRENT_VERSION {
                return Err(Error::Protocol(format!(
                    "wrong device version, version {} should be {}",
                    version.version, CURRENT_VERSION
                )));
            }
        } else {
            return Err(Error::Protocol(
                "reset failed, unexpected response".to_owned(),
            ));
        }

        // Apply the initial configuration.
        nrf.configure(config).await?;

        Ok(Standby { nrf })
    }

    pub async fn open_default(config: Configuration, tty: &str) -> Result<Standby, Error> {
        Self::open(tty, config).await
    }

    async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.addr_len = config.addr_len as usize;
        self.command_ack(PacketType::Config(config), true).await
    }

    async fn send(
        &mut self,
        address: Address,
        payload: &[u8],
    ) -> Result<Option<ReceivedPacket>, Error> {
        let mut payload_array = [0u8; 32];
        payload_array[0..payload.len()].copy_from_slice(payload);
        let response = self
            .command(
                PacketType::Send(TxPacket {
                    addr: address.addr,
                    length: payload.len() as u8,
                    payload: payload_array,
                }),
                false,
            )
            .await?;
        match response {
            PacketType::ReceiveAck(packet) => Ok(packet.map(|packet| ReceivedPacket {
                pipe: packet.pipe,
                payload: packet.payload[..packet.length as usize].to_vec(),
            })),
            PacketType::PacketLost => Err(Error::PacketLost),
            _ => Err(Error::Protocol(
                "unexpected packet from device during send operation".to_owned(),
            )),
        }
    }

    async fn command(
        &mut self,
        packet: PacketType,
        _discard_rx: bool,
    ) -> Result<PacketType, Error> {
        // TODO: Discard RX queue?
        let packet = Packet {
            call: 1,
            content: packet,
        };
        self.port.send(packet).await?;
        loop {
            let packet = self.read_packet(Some(Duration::from_millis(100))).await?;
            if packet.call == 1 {
                return Ok(packet.content);
            } else {
                match packet.content {
                    PacketType::Error(code) => {
                        return Err(code.into());
                    }
                    _content => {
                        // TODO: Queue for received packets?
                    }
                }
            }
        }
    }

    async fn command_ack(&mut self, packet: PacketType, discard_rx: bool) -> Result<(), Error> {
        let response = self.command(packet, discard_rx).await?;
        match response {
            PacketType::Ack => Ok(()),
            _ => Err(Error::Protocol(
                "unexpected response type instead of ACK".to_owned(),
            )),
        }
    }

    async fn read_packet(&mut self, timeout: Option<Duration>) -> Result<Packet, Error> {
        let item = if let Some(timeout) = timeout {
            match tokio::time::timeout(timeout, self.port.next()).await {
                Err(_) => return Err(Error::Protocol("timeout".to_owned())),
                Ok(item) => item,
            }
        } else {
            self.port.next().await
        };
        match item {
            Some(packet) => packet,
            None => return Err(Error::Protocol("connection closed".to_owned())),
        }
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
            .command_ack(PacketType::SetAddress(addr_bytes), true)
            .await
    }

    pub async fn configure(&mut self, config: Configuration) -> Result<(), Error> {
        self.nrf.configure(config).await
    }

    pub async fn receive(mut self) -> Result<Receiver, Error> {
        self.nrf.command_ack(PacketType::StartReceive, true).await?;
        Ok(Receiver { nrf: self.nrf })
    }

    pub async fn send(
        &mut self,
        address: Address,
        payload: &[u8],
    ) -> Result<Option<ReceivedPacket>, Error> {
        self.nrf.send(address, payload).await
    }
}

pub struct Receiver {
    nrf: NRF24L01,
}

impl Receiver {
    pub async fn receive(&mut self) -> Result<ReceivedPacket, Error> {
        // TODO: PacketTypes received during the last commands (e.g., during the last TX operation?)
        loop {
            let packet = self.nrf.read_packet(None).await?;
            if packet.call != 0 {
                // The packet belonged to a call from the host.
                return Err(Error::Protocol(
                    "received unexpected response from previous call to device".to_owned(),
                ));
            }
            match packet.content {
                PacketType::Receive(packet) => {
                    return Ok(ReceivedPacket {
                        pipe: packet.pipe,
                        payload: packet.payload[..packet.length as usize].to_vec(),
                    });
                }
                _ => {
                    // We should never receive any other type of packet, so we should treat this as
                    // an error.
                    return Err(Error::Protocol(
                        "received other packet from device during receive operation".to_owned(),
                    ));
                }
            }
        }
    }

    pub async fn send(
        &mut self,
        address: Address,
        payload: &[u8],
    ) -> Result<Option<ReceivedPacket>, Error> {
        self.nrf.send(address, payload).await
    }

    pub async fn standby(mut self) -> Result<Standby, Error> {
        self.nrf.command_ack(PacketType::Standby, true).await?;
        Ok(Standby { nrf: self.nrf })
    }
}

pub struct ReceivedPacket {
    pub pipe: u8,
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
    #[error("protocol error: {0}")]
    Protocol(String),
    #[error("device error: {0}")]
    Device(&'static str),
    #[error("no ack received for packet")]
    PacketLost,
}

impl From<ErrorCode> for Error {
    fn from(e: ErrorCode) -> Error {
        Error::Device(match e {
            ErrorCode::InvalidState => "invalid state",
            ErrorCode::InvalidInput => "invalid input",
            ErrorCode::InvalidOperation => "invalid operation",
            ErrorCode::InternalError => "internal error",
            ErrorCode::ProtocolError => "protocol error",
        })
    }
}
