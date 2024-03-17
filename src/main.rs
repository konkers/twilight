#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use de1::Frame;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use esp32c3_hal as hal;
use esp_backtrace as _;
use esp_wifi::{initialize, EspWifiInitFor};
use hal::{clock::ClockControl, embassy, peripherals::*, prelude::*, timer::TimerGroup, Rng};
use log::info;

mod ble;
mod charactaristic;
mod serial;

use ble::ble_task;

type FrameChannel = Channel<NoopRawMutex, Frame, 4>;
type FrameSender<'ch> = Sender<'ch, NoopRawMutex, Frame, 4>;
type FrameReceiver<'ch> = Receiver<'ch, NoopRawMutex, Frame, 4>;

type Mutex<T> = embassy_sync::mutex::Mutex<NoopRawMutex, T>;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

#[main]
async fn main(_spawner: Spawner) -> ! {
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

    #[cfg(not(feature = "fake"))]
    let io = hal::IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let bluetooth = peripherals.BT;

    let frame_channel = FrameChannel::new();
    let update_channel = FrameChannel::new();

    #[cfg(not(feature = "fake"))]
    let (uart_tx, uart_rx) = {
        let config = hal::uart::config::Config {
            baudrate: 115200,
            ..Default::default()
        };
        let pins = hal::uart::AllPins::new(
            io.pins.gpio21.into_push_pull_output(),
            io.pins.gpio20.into_pull_up_input(),
            io.pins.gpio7.into_floating_input(),
            io.pins.gpio8.into_push_pull_output(),
        );
        let uart0 = hal::Uart::new_with_config(peripherals.UART0, config, Some(pins), &clocks);
        uart0.split()
    };

    #[cfg(feature = "fake")]
    let mut pipe1 = embassy_sync::pipe::Pipe::new();
    #[cfg(feature = "fake")]
    let mut pipe2 = embassy_sync::pipe::Pipe::new();
    #[cfg(feature = "fake")]
    let (mut de1, uart_tx, uart_rx) = {
        let (fake_rx, uart_tx) = pipe1.split();
        let (uart_rx, fake_tx) = pipe2.split();
        (de1::fake::De1::new(fake_rx, fake_tx), uart_tx, uart_rx)
    };

    let mut serial = serial::SerialInterface::new(
        uart_rx,
        uart_tx,
        frame_channel.receiver(),
        update_channel.sender(),
    );

    info!("Initialized: running tasks");

    #[cfg(feature = "fake")]
    embassy_futures::join::join3(
        ble_task(
            bluetooth,
            &init,
            frame_channel.sender(),
            update_channel.receiver(),
        ),
        serial.run(),
        de1.run(),
    )
    .await;

    #[cfg(not(feature = "fake"))]
    embassy_futures::join::join(
        ble_task(
            bluetooth,
            &init,
            frame_channel.sender(),
            update_channel.receiver(),
        ),
        serial.run(),
    )
    .await;

    panic!()
}
