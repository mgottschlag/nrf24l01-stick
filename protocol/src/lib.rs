#![no_std]

pub enum Packet {
    Reset,
    ResetDone, // TODO: Version information.
    Config(Configuration),
    Standby,
    Send,
    Receive(ReceivedPacket),
    Ack,
    Error,
}

pub struct ReceivedPacket {
    pub addr: [u8; 5],
    pub payload: [u8; 32],
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

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
