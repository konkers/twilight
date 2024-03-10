#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use de1::{Command, Frame};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use esp32c3_hal as hal;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{initialize, EspWifiInitFor};
use hal::{clock::ClockControl, embassy, peripherals::*, prelude::*, timer::TimerGroup, Rng, IO};

mod ble;
mod charactaristic;

use ble::ble_task;

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

    ble_task(bluetooth, &init, frame_channel.sender()).await;

    panic!()
}
