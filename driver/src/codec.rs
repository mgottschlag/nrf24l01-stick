use bytes::BytesMut;
use tokio_util::codec::{Decoder, Encoder};

use crate::Error;
use nrf24l01_stick_protocol::Packet;

pub struct Codec;

impl Decoder for Codec {
    type Item = Packet;
    type Error = Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if src.len() == 0 {
            Ok(None)
        } else {
            // We encode (length-1), not length, as that eliminates the special case where the
            // lenght byte is 0.
            let length = src.as_ref()[0] as usize + 1;
            if src.len() < length + 1 {
                Ok(None)
            } else {
                let data = src.split_to(length + 1);
                match Packet::deserialize(&data.as_ref()[1..]) {
                    Some(packet) => Ok(Some(packet)),
                    None => Err(Error::Device("malformed packet from device".to_owned())),
                }
            }
        }
    }
}

impl Encoder<Packet> for Codec {
    type Error = Error;

    fn encode(&mut self, item: Packet, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 257];
        let length = item.serialize(&mut buffer[1..]).unwrap();
        buffer[0] = (length - 1) as u8;
        dst.extend_from_slice(&buffer[0..length + 1]);
        Ok(())
    }
}
