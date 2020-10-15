use tokio::io::BufReader;
use tokio::io::AsyncBufReadExt;


use nrf24l01_stick_driver::{Configuration, CrcMode, DataRate, NRF24L01, MAX_PAYLOAD_LEN};

#[tokio::main]
async fn main() {
    let mut config = Configuration::default();
    config.frequency = 0x32;
    config.rate = DataRate::R2Mbps;
    config.power = 3;
    config.crc = Some(CrcMode::OneByte);
    config.auto_retransmit_delay_count = Some((250, 3));

    let mut nrf24l01 = NRF24L01::open_default(config)
        .await
        .expect("could not open device");
    nrf24l01.set_receive_addr(
        Some((&[0xb3u8, 0xb3u8, 0xb3u8, 0xb3u8, 0x00u8] as &[u8]).into()),
        None,
        None,
        None,
        None,
    ).await.expect("could not set receive address");
    let mut receive = Some(nrf24l01.receive().await.expect("could not start receiving"));

    let stdin = BufReader::new(tokio::io::stdin());
    let mut lines = stdin.lines();

    loop {
        tokio::select! {
            packet = receive.as_mut().unwrap().receive() => {
                let packet = packet.expect("could not receive packet");
                println!("Received {:?} from {:?}", packet.payload, &packet.addr.addr[0..packet.addr.length as usize]);
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

                let mut send = receive.take().unwrap().send().await.expect("could not start sending");

                send.send((&[0xb3u8, 0xb3u8, 0xb3u8, 0xb3u8, 0x01u8][..]).into(), payload).await.expect("could not send packet");

                receive = Some(send.receive().await.expect("could not start receiving"));
            },

        }
    }
}
