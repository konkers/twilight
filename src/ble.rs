use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use de1::Command;
use embassy_time::Timer;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitialization};

use crate::hal::peripherals::BT;

use crate::charactaristic::Charactaristic;
use crate::FrameSender;

macro_rules! charactaristic {
    ($command:expr, $frame_tx:expr) => {{
        const DATA_LEN: usize = $command.data_len();
        Charactaristic::<'_, DATA_LEN>::new($command, $frame_tx)
    }};
}

pub async fn ble_task(mut bluetooth: BT, init: &EspWifiInitialization, frame_tx: FrameSender<'_>) {
    let connector = BleConnector::new(init, &mut bluetooth);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    println!("Connector created");

    let versions_charactaristic = charactaristic!(Command::Versions, frame_tx);
    let requested_state_charactaristic = charactaristic!(Command::RequestedState, frame_tx);
    let read_from_mmr_charactaristic = charactaristic!(Command::ReadFromMmr, frame_tx);
    let write_to_mmr_charactaristic = charactaristic!(Command::WriteToMmr, frame_tx);
    let fw_map_request_charactaristic = charactaristic!(Command::FwMapRequest, frame_tx);
    let shot_settings_charactaristic = charactaristic!(Command::ShotSettings, frame_tx);
    let shot_sample_charactaristic = charactaristic!(Command::ShotSample, frame_tx);
    let state_info_charactaristic = charactaristic!(Command::StateInfo, frame_tx);
    let header_write_charactaristic = charactaristic!(Command::HeaderWrite, frame_tx);
    let frame_write_charactaristic = charactaristic!(Command::FrameWrite, frame_tx);
    let water_levels_charactaristic = charactaristic!(Command::WaterLevels, frame_tx);
    let calibration_charactaristic = charactaristic!(Command::Calibration, frame_tx);

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

        gatt!([service {
            uuid: "0000a000-0000-1000-8000-00805f9b34fb",
            characteristics: [
                characteristic {
                    name: "versions",
                    uuid: "0000A001-0000-1000-8000-00805F9B34FB",
                    data: &versions_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "requested_state",
                    uuid: "0000A002-0000-1000-8000-00805F9B34FB",
                    data: &requested_state_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "read_from_mmr",
                    uuid: "0000A005-0000-1000-8000-00805F9B34FB",
                    data: &read_from_mmr_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "write_to_mmr",
                    uuid: "0000A006-0000-1000-8000-00805F9B34FB",
                    data: &write_to_mmr_charactaristic,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "fw_map_request",
                    uuid: "0000A009-0000-1000-8000-00805F9B34FB",
                    data: &fw_map_request_charactaristic,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "shot_setings",
                    uuid: "0000A00B-0000-1000-8000-00805F9B34FB",
                    data: &shot_settings_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "shot_sample",
                    uuid: "0000A00D-0000-1000-8000-00805F9B34FB",
                    data: &shot_sample_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "state_info",
                    uuid: "0000A00E-0000-1000-8000-00805F9B34FB",
                    data: &state_info_charactaristic,
                    readable: true,
                    notify: true,
                },
                characteristic {
                    name: "header_write",
                    uuid: "0000A00F-0000-1000-8000-00805F9B34FB",
                    data: &header_write_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "frame_write",
                    uuid: "0000A010-0000-1000-8000-00805F9B34FB",
                    data: &frame_write_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "water_levels",
                    uuid: "0000A011-0000-1000-8000-00805F9B34FB",
                    data: &water_levels_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
                characteristic {
                    name: "calibration",
                    uuid: "0000A012-0000-1000-8000-00805F9B34FB",
                    data: &calibration_charactaristic,
                    readable: true,
                    writable: true,
                    notify: true,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characterisic value
            // probably passing in the attribute server won't work?
            async {
                // #[allow(clippy::await_holding_refcell_ref)]
                // pin_ref.borrow_mut().wait_for_rising_edge().await.unwrap();
                Timer::after_secs(10).await;
                let mut data = [0u8; 13];
                // data.copy_from_slice(b"Notification0");
                // {
                //     let mut counter = counter.borrow_mut();
                //     data[data.len() - 1] += *counter;
                //     *counter = (*counter + 1) % 10;
                // }
                NotificationData::new(requested_state_handle, &data)
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}
