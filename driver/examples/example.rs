use tokio::io::AsyncBufReadExt;
use tokio::io::BufReader;

use nrf24l01_stick_driver::{Configuration, CrcMode, DataRate, MAX_PAYLOAD_LEN, NRF24L01};

#[tokio::main]
async fn main() {
    let mut config = Configuration::default();
    config.channel = 0x32;
    config.rate = DataRate::R2Mbps;
    config.power = 3;
    config.crc = Some(CrcMode::OneByte);
    config.auto_retransmit_delay_count = Some((250, 3));

    let mut nrf24l01 = NRF24L01::open_default(config)
        .await
        .expect("could not open device");
    nrf24l01
        .set_receive_addr(
            Some((&[0xb3u8, 0xb3u8, 0xb3u8, 0xb3u8, 0x00u8] as &[u8]).into()),
            None,
            None,
            None,
            None,
        )
        .await
        .expect("could not set receive address");
    let mut receive = nrf24l01.receive().await.expect("could not start receiving");

    let stdin = BufReader::new(tokio::io::stdin());
    let mut lines = stdin.lines();

    println!("Listening for packets:");
    loop {
        tokio::select! {
            packet = receive.receive() => {
                let packet = packet.expect("could not receive packet");
                println!("Received {:?} from {}", packet.payload, packet.pipe);
            },
            line = lines.next_line() => {
                let line = line.unwrap();
                if line.is_none() {
                    break;
                }
                let line = line.unwrap();
                let payload = if line.as_bytes().len() > MAX_PAYLOAD_LEN {
                    &line.as_bytes()[0..MAX_PAYLOAD_LEN]
                } else {
                    line.as_bytes()
                };

                match receive.send((&[0xe7u8, 0xe7u8, 0xe7u8, 0xe7u8, 0xe7u8][..]).into(), payload).await {
                    Ok(Some(ack_payload)) => {
                        println!("Received ACK payload: {:?}", ack_payload.payload);
                    },
                    Ok(None) => {
                        println!("Received no ACK payload.");
                    },
                    Err(e) => println!("could not send: {:?}", e),
                }
            },

        }
    }
}
