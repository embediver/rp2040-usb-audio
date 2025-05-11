#![no_std]
#![no_main]

use core::sync::atomic::AtomicU8;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{self, Driver},
};
use embassy_time::Instant;
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

use byteorder::{BigEndian, ByteOrder};
use proc_lut::lut_100;

/// Look-up-table for factors from 0-100dB
const LUT_100DB: [u32; 101] = lut_100!();

/// Maximum USB PCM Packet size
///
/// 24bit * 2samples * 96kHz * 1ms
const MAX_PACKET_SIZE: usize = (3 * 2) * (96000 / 1000);

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

    static VOLUME: AtomicU8 = AtomicU8::new(0);

    spawner.must_spawn(audio_processor(audio_stream, &VOLUME));
    loop {
        audio_ctrl.changed().await;
        let volume = audio_ctrl.volume(Channel::LeftFront);
        let rate = audio_ctrl.sample_rate_hz();
        match volume {
            Some(Volume::Muted) => defmt::info!("Speaker is muted"),
            Some(Volume::DeciBel(db)) => {
                defmt::info!("Volume is {}dB", db);
                VOLUME.store(db.abs() as u8, core::sync::atomic::Ordering::Relaxed);
            }
            None => (),
        }
        defmt::info!("Sample rate is {}", rate);
        // defmt::info!("Blink");
        // Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn audio_processor(
    mut stream: Stream<'static, Driver<'static, USB>>,
    volume: &'static AtomicU8,
) -> ! {
    stream.wait_connection().await;
    let mut buf = [0; MAX_PACKET_SIZE];
    loop {
        let n = match stream.read_packet(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                defmt::info!("Stream read error: {}", e);
                continue;
            }
        };
        let now = Instant::now();
        let vol = volume.load(core::sync::atomic::Ordering::Relaxed);
        for i in (0..n).step_by(3) {
            // let mut sample = read_be_i24(&buf[i..i + 3]);
            let mut sample = BigEndian::read_i24(&buf[i..i + 3]);
            sample /= LUT_100DB[vol as usize] as i32;
            // write_be_i24(&mut buf[i..i + 3], sample);
            BigEndian::write_i24(&mut buf[i..i + 3], sample);
        }
        core::hint::black_box(&buf);
        defmt::info!(
            "Applied volume (-{}dB) to {} samples in {}us",
            vol,
            n / 3,
            now.elapsed().as_micros()
        );
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

// #[inline]
// fn read_be_i24(buf: &[u8]) -> i32 {
//     let mut bytes = [0; 4];
//     assert!(buf.len() == 3);
//     bytes[1..].copy_from_slice(buf);
//     extend_i24_sign(u32::from_be_bytes(bytes))
// }
// #[inline]
// fn extend_i24_sign(n: u32) -> i32 {
//     #[allow(clippy::identity_op)]
//     let shift = (4 - 3) * 4;
//     (n << shift) as i32 >> shift
// }

// #[inline]
// fn write_be_i24(buf: &mut [u8], n: i32) {
//     assert!(buf.len() == 3);
//     let out = unextend_i24_sign(n);
//     unsafe {
//         let bytes = *(&out.to_be() as *const u32 as *const [u8; 4]);
//         core::ptr::copy_nonoverlapping(
//             bytes.as_ptr().offset((4 - 3) as isize),
//             buf.as_mut_ptr(),
//             3,
//         );
//     }
// }
// #[inline]
// fn unextend_i24_sign(n: i32) -> u32 {
//     let shift = (4 - 3) * 4;
//     (n << shift) as u32 >> shift
// }
