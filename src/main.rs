#![no_std]
#![no_main]

use core::{mem, sync::atomic::AtomicU8};

use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::{PIO0, USB},
    pio::Pio,
    pio_programs::i2s::{PioI2sOut, PioI2sOutProgram},
    usb::{self, Driver},
};
use embassy_time::Instant;
// use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
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

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

struct I2SPeripherals {
    bit_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_19>,
    left_right_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_20>,
    data_pin: Peri<'static, embassy_rp::peripherals::PIN_21>,
    dma_channel: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    pio0: Peri<'static, embassy_rp::peripherals::PIO0>,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let i2s_peri = I2SPeripherals {
        bit_clock_pin: p.PIN_19,
        left_right_clock_pin: p.PIN_20,
        data_pin: p.PIN_21,
        dma_channel: p.DMA_CH0,
        pio0: p.PIO0,
    };

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
    static SAMPLE_RATE: Signal<ThreadModeRawMutex, u32> = Signal::new();

    spawner.must_spawn(audio_processor(
        audio_stream,
        &VOLUME,
        &SAMPLE_RATE,
        i2s_peri,
    ));

    let mut old_rate = 0;
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
        if rate != 0 && rate != old_rate {
            old_rate = rate;
            SAMPLE_RATE.signal(rate);
        }
    }
}

#[embassy_executor::task]
async fn audio_processor(
    mut stream: Stream<'static, Driver<'static, USB>>,
    volume: &'static AtomicU8,
    sample_rate: &'static Signal<ThreadModeRawMutex, u32>,
    mut i2s_peri: I2SPeripherals,
) -> ! {
    stream.wait_connection().await;
    let mut current_rate = sample_rate.wait().await;

    // let mut in_buf = [0; MAX_PACKET_SIZE];

    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    static DMA_BUFFER: StaticCell<[u32; MAX_PACKET_SIZE / 4 * 2]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; MAX_PACKET_SIZE / 4 * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(MAX_PACKET_SIZE / 4);

    loop {
        // Setup pio state machine for i2s output
        let Pio {
            mut common, sm0, ..
        } = Pio::new(i2s_peri.pio0.reborrow(), PioIrqs);

        let program = PioI2sOutProgram::new(&mut common);
        let mut i2s = PioI2sOut::new(
            &mut common,
            sm0,
            i2s_peri.dma_channel.reborrow(),
            i2s_peri.data_pin.reborrow(),
            i2s_peri.bit_clock_pin.reborrow(),
            i2s_peri.left_right_clock_pin.reborrow(),
            current_rate,
            24,
            &program,
        );

        // process audio data until sample-rate changes
        let mut n_bytes;
        'inner: loop {
            if let Some(rate) = sample_rate.try_take() {
                current_rate = rate;
                break 'inner;
            }

            let (prefix, in_buf, suffix) = unsafe { back_buffer.align_to_mut::<u8>() };
            assert!(prefix.is_empty());
            assert!(suffix.is_empty());

            n_bytes = match stream.read_packet(in_buf).await {
                Ok(n) => n,
                Err(e) => {
                    defmt::info!("Stream read error: {}", e);
                    continue 'inner;
                }
            };

            let dma_future = i2s.write(&front_buffer[..n_bytes / 4]);

            let now = Instant::now();
            let vol = volume.load(core::sync::atomic::Ordering::Relaxed);
            apply_volume(&mut in_buf[..n_bytes], vol);
            let vol_calc_time = now.elapsed();

            let now = Instant::now();
            dma_future.await;
            let dma_wait_time = now.elapsed();
            mem::swap(&mut back_buffer, &mut front_buffer);

            defmt::info!(
                "Waited {}us for DMA to finish, volume calc (-{}dB) took {}us",
                dma_wait_time.as_micros(),
                vol,
                vol_calc_time.as_micros()
            );
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

/// Apply `vol` dB attenuation to 24bit PCM data
#[inline]
fn apply_volume(buf: &mut [u8], vol: u8) {
    if LUT_100DB[vol as usize] == 1 {
        return;
    }
    for i in (0..buf.len()).step_by(3) {
        let mut sample = BigEndian::read_i24(&buf[i..i + 3]);
        sample /= LUT_100DB[vol as usize] as i32;
        BigEndian::write_i24(&mut buf[i..i + 3], sample);
    }
}

// 24bit big endian read and write implementation
// No performance increase compared to byteorder's implementation (which uses u64/i64 instead of u32/i32)

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
