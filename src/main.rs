#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use de1::{Command, Frame};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embedded_hal_async::digital::Wait;
use esp32c3_hal as hal;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, initialize, EspWifiInitFor};
use hal::{clock::ClockControl, embassy, peripherals::*, prelude::*, timer::TimerGroup, Rng, IO};

mod charactaristic;

use charactaristic::Charactaristic;

type FrameChannel = Channel<NoopRawMutex, Frame, 4>;
type FrameSender<'ch> = Sender<'ch, NoopRawMutex, Frame, 4>;
type FrameReceiver<'ch> = Receiver<'ch, NoopRawMutex, Frame, 4>;

enum Subscribtion {
    Subscribe(Command),
    Unsubscribe(Command),
}

type SubscribtionChannel = Channel<NoopRawMutex, Subscribtion, 4>;
type SubscribtionSender<'ch> = Sender<'ch, NoopRawMutex, Subscribtion, 4>;
type SubscribtionReceiver<'ch> = Receiver<'ch, NoopRawMutex, Subscribtion, 4>;

type Mutex<T> = embassy_sync::mutex::Mutex<NoopRawMutex, T>;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

macro_rules! charactaristic {
    ($command:expr, $frame_channel:expr) => {{
        const data_len: usize = $command.data_len();
        Charactaristic::<'_, data_len>::new($command, $frame_channel.sender())
    }};
}

#[main]
async fn main(_spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(target_arch = "riscv32")]
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let button = io.pins.gpio9.into_pull_down_input();

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let mut bluetooth = peripherals.BT;

    let connector = BleConnector::new(&init, &mut bluetooth);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    println!("Connector created");

    let pin_ref = RefCell::new(button);
    let pin_ref = &pin_ref;

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName("konkers"),
                ])
                .unwrap()
            )
            .await
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

        println!("started advertising");
        let mut frame_channel = FrameChannel::new();

        let versions_charactaristic = charactaristic!(Command::Versions, frame_channel);
        let requested_state_charactaristic =
            charactaristic!(Command::RequestedState, frame_channel);
        let read_from_mmr_charactaristic = charactaristic!(Command::ReadFromMmr, frame_channel);
        let write_to_mmr_charactaristic = charactaristic!(Command::WriteToMmr, frame_channel);
        let fw_map_request_charactaristic = charactaristic!(Command::FwMapRequest, frame_channel);
        let shot_settings_charactaristic = charactaristic!(Command::ShotSettings, frame_channel);
        let shot_sample_charactaristic = charactaristic!(Command::ShotSample, frame_channel);
        let state_info_charactaristic = charactaristic!(Command::StateInfo, frame_channel);
        let header_write_charactaristic = charactaristic!(Command::HeaderWrite, frame_channel);
        let frame_write_charactaristic = charactaristic!(Command::FrameWrite, frame_channel);
        let water_levels_charactaristic = charactaristic!(Command::WaterLevels, frame_channel);
        let calibration_charactaristic = charactaristic!(Command::Calibration, frame_channel);

        gatt!([service {
            uuid: "0000a000-0000-1000-8000-00805f9b34fb",
            characteristics: [
                characteristic {
                    name: "versions",
                    uuid: "0000A001-0000-1000-8000-00805F9B34FB",
                    data: versions_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "requested_state",
                    uuid: "0000A002-0000-1000-8000-00805F9B34FB",
                    data: requested_state_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "read_from_mmr",
                    uuid: "0000A005-0000-1000-8000-00805F9B34FB",
                    data: read_from_mmr_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "write_to_mmr",
                    uuid: "0000A006-0000-1000-8000-00805F9B34FB",
                    data: write_to_mmr_charactaristic,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "fw_map_request",
                    uuid: "0000A009-0000-1000-8000-00805F9B34FB",
                    data: fw_map_request_charactaristic,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "shot_setings",
                    uuid: "0000A00B-0000-1000-8000-00805F9B34FB",
                    data: shot_settings_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "shot_sample",
                    uuid: "0000A00D-0000-1000-8000-00805F9B34FB",
                    data: shot_sample_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "state_info",
                    uuid: "0000A00E-0000-1000-8000-00805F9B34FB",
                    data: state_info_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "header_write",
                    uuid: "0000A00F-0000-1000-8000-00805F9B34FB",
                    data: header_write_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "frame_write",
                    uuid: "0000A010-0000-1000-8000-00805F9B34FB",
                    data: frame_write_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "water_levels",
                    uuid: "0000A011-0000-1000-8000-00805F9B34FB",
                    data: water_levels_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "calibration",
                    uuid: "0000A012-0000-1000-8000-00805F9B34FB",
                    data: calibration_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let counter = RefCell::new(0u8);
        let counter = &counter;

        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characterisic value
            // probably passing in the attribute server won't work?
            async {
                #[allow(clippy::await_holding_refcell_ref)]
                pin_ref.borrow_mut().wait_for_rising_edge().await.unwrap();
                let mut data = [0u8; 13];
                data.copy_from_slice(b"Notification0");
                {
                    let mut counter = counter.borrow_mut();
                    data[data.len() - 1] += *counter;
                    *counter = (*counter + 1) % 10;
                }
                NotificationData::new(requested_state_handle, &data)
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}
