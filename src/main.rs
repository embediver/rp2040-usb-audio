#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{self, Driver},
};
// use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_usb::{Builder, Config, class::uac1::speaker::Stream};
use embassy_usb::{
    UsbDevice,
    class::uac1::{
        Channel, FeedbackRefresh, SampleWidth,
        speaker::{Speaker, State, Volume},
    },
};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    bind_interrupts!(struct UsbIrqs {
        USBCTRL_IRQ => usb::InterruptHandler<embassy_rp::peripherals::USB>;
    });

    static UAC_STATE: StaticCell<State> = StaticCell::new();

    let usb_driver = Driver::new(p.USB, UsbIrqs);
    let mut usb_builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        Builder::new(
            usb_driver,
            create_usb_conf(),
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [],
            CONTROL_BUF.init([0; 64]),
        )
    };

    let (mut audio_stream, mut feedback, audio_ctrl) = Speaker::new(
        &mut usb_builder,
        UAC_STATE.init(State::new()),
        576,
        SampleWidth::Width3Byte,
        &[44100, 48000, 96000],
        &[Channel::LeftFront, Channel::RightFront],
        FeedbackRefresh::Period16Frames,
    );

    let usb = usb_builder.build();

    spawner.must_spawn(usb_task(usb));

    feedback.wait_connection().await;
    audio_stream.wait_connection().await;

    spawner.must_spawn(audio_processor(audio_stream));
    loop {
        audio_ctrl.changed().await;
        let volume = audio_ctrl.volume(Channel::LeftFront);
        let rate = audio_ctrl.sample_rate_hz();
        if let Some(Volume::DeciBel(db)) = volume {
            defmt::info!("Volume is {}dB", db);
        }
        match volume {
            Some(Volume::Muted) => defmt::info!("Speaker is muted"),
            Some(Volume::DeciBel(db)) => defmt::info!("Volume is {}dB", db),
            None => (),
        }
        defmt::info!("Sample rate is {}", rate);
        // defmt::info!("Blink");
        // Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn audio_processor(mut stream: Stream<'static, Driver<'static, USB>>) -> ! {
    stream.wait_connection().await;
    let mut buf = [0; 576];
    loop {
        match stream.read_packet(&mut buf).await {
            Ok(_x) => {
                // defmt::info!("Read {} bytes from endpoint", x);
            }
            Err(e) => defmt::info!("Stream read error: {}", e),
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

fn create_usb_conf() -> Config<'static> {
    let mut conf = Config::new(0x4242, 0x1234);
    conf.product = Some("RP2040 Soundcard");
    conf.manufacturer = Some("RT-CUSTOMZ");
    conf
}
