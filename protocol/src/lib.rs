#![no_std]

use serde::{Deserialize, Serialize};

/// ID to test whether the connected device is really an NRF24L01 stick.
pub const DEVICE_ID: u32 = 0x3246524e;
/// Device packet interface version.
pub const CURRENT_VERSION: u32 = 1;

#[derive(Serialize, Deserialize)]
pub struct Packet {
    pub call: u8,
    pub content: PacketType,
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
    /// Resets the NRF24L01 module and places it in standby mode.
    ///
    /// The host should not make any assumptions about the configuration of the radio module after
    /// the reset call. The device responds with a `ResetDone` packet.
    ///
    /// Before sending a `Reset` packet, the host should wait for 200 milliseconds to let the
    /// device resynchronize to the packet protocol.
    Reset,
    /// Response to the Reset packet, containing information identifying the device and its
    /// version.
    ResetDone(Version),
    /// Configures the radio interface.
    ///
    /// This packet has to be sent before receiving/sending data and may only be sent while the
    /// device is in standby mode. The device responds with an `Ack` packet.
    Config(Configuration),
    /// Sends the addresses of the device which are used to receive packets.
    ///
    /// This packet may only be sent while the device is in standby mode. The device responds with
    /// an `Ack` packet.
    SetAddress([Option<[u8; 5]>; 5]),
    /// Places the device in standby mode where it does not listen for incoming packets.
    ///
    /// In standby mode, the module can still be used to send packets. The device responds with an
    /// `Ack` packet.
    Standby,
    /// Places the device in RX mode in which it listens for incoming packets and forwards them to
    /// the host.
    ///
    /// The device responds with an `Ack` packet.
    StartReceive,
    /// Sent by the host to send a radio packet.
    ///
    /// The device responds either with `Ack` or `PacketLost`.
    Send(RadioPacket),
    /// Sent by the device to signal reception of a radio packet.
    ///
    /// The device only sends this packet while in receive mode.
    Receive(RadioPacket),
    /// Acknowledgement of a command from the host, sent by the device.
    Ack,
    /// The device did not receive any acknowledgement packet from the destination.
    PacketLost,
    /// An error occurred.
    ///
    /// When this packet is received, the host should not make any assumptions about the state of
    /// the device and should issue a `Reset` packet.
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
