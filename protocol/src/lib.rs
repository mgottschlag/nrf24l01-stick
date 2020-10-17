#![no_std]

use serde::{Deserialize, Serialize};

/// ID to test whether the connected device is really an NRF24L01 stick.
pub const DEVICE_ID: u32 = 0x3246524e;
/// Device packet interface version.
pub const CURRENT_VERSION: u32 = 1;

#[derive(Serialize, Deserialize)]
pub struct Packet {
    call: u8,
    content: PacketType,
}

impl Packet {
    pub fn serialize(&self, buffer: &mut [u8]) -> Option<usize> {
        Some(postcard::to_slice(self, buffer).ok()?.len())
    }

    pub fn deserialize(buffer: &[u8]) -> Option<Packet> {
        postcard::from_bytes(buffer).ok()
    }
}

#[derive(Serialize, Deserialize)]
pub enum PacketType {
    Reset,
    ResetDone(Version),
    Config(Configuration),
    SetAddress([Option<[u8; 5]>; 5]),
    Standby,
    RX,
    TX,
    Send(RadioPacket),
    Receive(RadioPacket),
    Ack,
    PacketLost,
    Error,
}

#[derive(Serialize, Deserialize)]
pub struct Version {
    pub device: u32,
    pub version: u32,
}

#[derive(Serialize, Deserialize)]
pub struct RadioPacket {
    pub addr: [u8; 5],
    pub length: u8,
    pub payload: [u8; 32],
}

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub addr_len: u8,
    pub channel: u8,
    pub rate: DataRate,
    pub power: u8,
    pub crc: Option<CrcMode>,
    pub auto_retransmit_delay_count: Option<(u8, u8)>,
}

impl Default for Configuration {
    fn default() -> Configuration {
        Configuration {
            addr_len: 5,
            channel: 0,
            rate: DataRate::R2Mbps,
            power: 3,
            crc: Some(CrcMode::OneByte),
            auto_retransmit_delay_count: Some((250, 3)),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub enum DataRate {
    R250Kbps,
    R1Mbps,
    R2Mbps,
}

#[derive(Serialize, Deserialize)]
pub enum CrcMode {
    OneByte,
    TwoByte,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
