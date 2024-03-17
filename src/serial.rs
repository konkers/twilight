use de1::{
    serial::{Frame, LineReader},
    Command, Error, Result,
};
use embassy_futures::select::select;
use embedded_io_async::{Read, Write};
use esp_println::println;

use crate::{FrameReceiver, FrameSender};

pub struct SerialInterface<'rx_ch, 'tx_ch, R: Read, W: Write> {
    link_rx: R,
    link_tx: W,
    frame_rx: FrameReceiver<'rx_ch>,
    update_tx: FrameSender<'tx_ch>,
    line_reader: LineReader<64>,
}

impl<'rx_ch, 'tx_ch, R: Read, W: Write> SerialInterface<'rx_ch, 'tx_ch, R, W> {
    pub fn new(
        link_rx: R,
        link_tx: W,
        frame_rx: FrameReceiver<'rx_ch>,
        update_tx: FrameSender<'tx_ch>,
    ) -> Self {
        Self {
            link_rx,
            link_tx,
            frame_rx,
            update_tx,
            line_reader: LineReader::new(),
        }
    }

    pub async fn run(&mut self) {
        // Send two new lines to ensure that the remote side's buffers are cleared.
        let _ = self.link_tx.write_all(b"\n\n").await;
        loop {
            let mut buf = [0u8; Command::MAX_DATA_LENGTH];
            match select(self.frame_rx.receive(), self.link_rx.read(&mut buf)).await {
                embassy_futures::select::Either::First(frame) => {
                    if let Err(e) = self.handle_frame_to_send(frame).await {
                        println!("Error sending frame: {e:?}");
                    }
                }
                embassy_futures::select::Either::Second(read_len) => {
                    if let Ok(read_len) = read_len {
                        self.handle_serial_input(&buf[..read_len]).await;
                    }
                }
            }
        }
    }

    async fn handle_char(&mut self, c: char) -> Result<()> {
        if let Some(frame) = self.line_reader.handle_char(c)? {
            //   println!("update_frame: {frame:?}");
            self.update_tx.send(frame).await;
        }
        Ok(())
    }

    async fn handle_serial_input(&mut self, data: &[u8]) {
        println!("rx SER: {data:?}");
        println!("SER: input {}", unsafe {
            core::str::from_utf8_unchecked(data)
        });
        for c in data.iter().map(|b| *b as char) {
            if let Err(e) = self.handle_char(c).await {
                println!("SER: error handling char '{c}': {e:?}");
            }
        }
    }

    async fn handle_frame_to_send(&mut self, frame: Frame) -> Result<()> {
        let mut buf = [0u8; de1::serial::MAX_ENCODED_LENGTH];

        let len = frame.write(&mut buf[..]).await?;
        let encoded_str = core::str::from_utf8(&buf[..len]).unwrap();
        println!("Sending frame {encoded_str}");
        self.link_tx
            .write_all(&buf[..len])
            .await
            .map_err(|_| Error::IoError)?;

        Ok(())
    }
}
