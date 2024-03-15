use bleps::{
    att::{Att, AttErrorCode},
    attribute::AttData,
};
use core::cmp::min;
use de1::{Command, CommandFrame, Frame};
use esp_println::println;
use heapless::Vec;

use crate::{FrameSender, Mutex};

pub struct Charactaristic<'ch, const DATA_LEN: usize> {
    command: Command,
    data: Mutex<[u8; DATA_LEN]>,
    write_sender: FrameSender<'ch>,
}

impl<'ch, const DATA_LEN: usize> Charactaristic<'ch, DATA_LEN> {
    pub fn new(command: Command, write_sender: FrameSender<'ch>) -> Self {
        assert_eq!(command.data_len(), DATA_LEN);
        Self {
            command,
            data: Mutex::new([0u8; DATA_LEN]),
            write_sender,
        }
    }
}

impl<'ch, const DATA_LEN: usize> AttData for &Charactaristic<'ch, DATA_LEN> {
    fn readable(&self) -> bool {
        println!("{:?} redable", self.command);
        true
    }

    fn writable(&self) -> bool {
        println!("{:?} writeable", self.command);
        true
    }

    fn enable_notification(&mut self, enabled: bool) -> Result<(), AttErrorCode> {
        println!("{:?} notification enable {}", self.command, enabled);
        let frame = match enabled {
            true => Frame::Subscribe(self.command.serial_command()),
            false => Frame::Unsubscribe(self.command.serial_command()),
        };

        self.write_sender
            .try_send(frame)
            .map_err(|_| AttErrorCode::PrepareQueueFull)
    }

    fn read(&mut self, offset: usize, read_data: &mut [u8]) -> Result<usize, AttErrorCode> {
        let Ok(data) = self.data.try_lock() else {
            // Since we're single threaded we should never contend on this lock.
            // TODO: log error.
            return Err(AttErrorCode::UnlikelyError);
        };

        if offset >= data.len() {
            return Err(AttErrorCode::InvalidOffset);
        }

        let read_len = min(data.len() - offset, read_data.len());

        read_data[..read_len].copy_from_slice(&data[offset..offset + read_len]);
        println!(
            "read {:?}: {} {:?}",
            self.command,
            offset,
            &read_data[..read_len]
        );

        Ok(read_len)
    }

    fn write(&mut self, offset: usize, write_data: &[u8]) -> Result<(), bleps::att::AttErrorCode> {
        println!("write {:?}: {} {:?}", self.command, offset, write_data);
        let Ok(mut data) = self.data.try_lock() else {
            // Since we're single threaded we should never contend on this lock.
            // TODO: log error.
            println!("lock failed");
            return Err(AttErrorCode::UnlikelyError);
        };

        if offset >= data.len() {
            println!("bad offset");
            return Err(AttErrorCode::InvalidOffset);
        }

        let write_len = min(data.len() - offset, write_data.len());

        data[offset..offset + write_len].copy_from_slice(&write_data[..write_len]);

        let Ok(frame_data) = Vec::from_slice(&*data) else {
            println!("bad offset");
            // TOOD: log error
            return Err(AttErrorCode::InvalidOffset);
        };

        self.write_sender
            .try_send(Frame::ToDe1(CommandFrame {
                command: self.command.serial_command(),
                data: frame_data,
            }))
            .map_err(|_| AttErrorCode::PrepareQueueFull)
    }
}
