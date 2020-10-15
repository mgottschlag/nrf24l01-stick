use std::io;
use std::path::Path;

use thiserror::Error;

pub const MAX_PAYLOAD_LEN: usize = 32;

pub struct NRF24L01 {
    // TODO
}

impl NRF24L01 {
    pub async fn open<P: AsRef<Path>>(
        _device: P,
        _config: Configuration,
    ) -> Result<Standby, Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn open_default(
        _config: Configuration,
    ) -> Result<Standby, Error> {
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
    pub async fn set_receive_addr(&mut self, _a1: Option<Address>, _a2: Option<Address>, _a3: Option<Address>, _a4: Option<Address>, _a5: Option<Address>) -> Result<(), Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn receive(self) -> Result<Receiver, Error> {
        // TODO
        panic!("Not yet implemented.");
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
}

pub struct Receiver {
    nrf: NRF24L01,
}

impl Receiver {
    pub async fn receive(&mut self) -> Result<ReceivedPacket, Error> {
        // TODO
        panic!("Not yet implemented.");
    }

    pub async fn send(self) -> Result<Sender, Error> {
        // TODO
        panic!("Not yet implemented.");
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

pub struct Configuration {
    pub frequency: u8,
    pub rate: DataRate,
    pub power: u8,
    pub crc: Option<CrcMode>,
    pub auto_retransmit_delay_count: Option<(u8, u8)>,
    // TODO
}

impl Default for Configuration {
    fn default() -> Configuration {
        Configuration {
            frequency: 0,
            rate: DataRate::R2Mbps,
            power: 3,
            crc: Some(CrcMode::OneByte),
            auto_retransmit_delay_count: Some((250, 3)),
        }
    }
}

pub enum DataRate {
    R250Kbps,
    R1Mbps,
    R2Mbps,
}

pub enum CrcMode {
    OneByte,
    TwoByte,
}

#[derive(Error, Debug)]
pub enum Error {
    #[error("I/O error")]
    Io(#[from] io::Error),
    // TODO
}
