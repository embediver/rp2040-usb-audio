#![no_std]
#![no_main]

use core::{mem, sync::atomic::AtomicU8};

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::{PIO0, USB},
    pio::Pio,
    pio_programs::i2s::{PioI2sOut, PioI2sOutProgram},
    usb::{self, Driver},
};
use embassy_time::{Duration, Instant};
// use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_usb::{
    Builder, Config,
    class::uac1::speaker::{Feedback, Stream},
};
use embassy_usb::{
    UsbDevice,
    class::uac1::{
        Channel, FeedbackRefresh, SampleWidth,
        speaker::{Speaker, State, Volume},
    },
};

use byteorder::{BigEndian, ByteOrder, LittleEndian};
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

// Pinout: GP0, GP1, GND, GP2, .., GP5, GND, GP6, GP7, GP8
//         1,   2, ...                  8,   9,   10,  11
struct I2SPeripherals {
    bit_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_6>,
    left_right_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_7>,
    data_pin: Peri<'static, embassy_rp::peripherals::PIN_8>,
    dma_channel: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    pio0: Peri<'static, embassy_rp::peripherals::PIO0>,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let i2s_peri = I2SPeripherals {
        bit_clock_pin: p.PIN_6,
        left_right_clock_pin: p.PIN_7,
        data_pin: p.PIN_8,
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
        &[48000],
        &[Channel::LeftFront, Channel::RightFront],
        FeedbackRefresh::Period64Frames,
    );

    let usb = usb_builder.build();

    spawner.must_spawn(usb_task(usb));

    feedback.wait_connection().await;
    audio_stream.wait_connection().await;

    static VOLUME: AtomicU8 = AtomicU8::new(0);
    static SAMPLE_RATE: Signal<ThreadModeRawMutex, u32> = Signal::new();
    static SYNCH_FEEDBACK: Signal<ThreadModeRawMutex, Duration> = Signal::new();

    spawner.must_spawn(feedback_task(feedback, &SYNCH_FEEDBACK));

    spawner.must_spawn(audio_processor(
        audio_stream,
        &VOLUME,
        &SAMPLE_RATE,
        &SYNCH_FEEDBACK,
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
    synch_feedback: &'static Signal<ThreadModeRawMutex, Duration>,
    mut i2s_peri: I2SPeripherals,
) -> ! {
    stream.wait_connection().await;
    let mut current_rate = sample_rate.wait().await;

    let mut in_buf = [0; MAX_PACKET_SIZE];

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
        defmt::info!("I2S set-up to {}Hz", current_rate);

        let mut leftover_bytes = 0;

        // process audio data until sample-rate changes
        let mut n_bytes = 0;
        'inner: loop {
            if let Some(rate) = sample_rate.try_take() {
                current_rate = rate;
                defmt::info!("Sample rate changed to {}Hz, restarting I2S...", rate);
                break 'inner;
            }

            let dma_future = i2s.write(&front_buffer[..n_bytes / 4]);
            let i2s_start_time = Instant::now();

            n_bytes = match stream.read_packet(&mut in_buf[leftover_bytes..]).await {
                Ok(n) => n,
                Err(e) => {
                    defmt::info!("Stream read error: {}", e);
                    continue 'inner;
                }
            };
            let usb_packet_delay = i2s_start_time.elapsed();
            synch_feedback.signal(usb_packet_delay);
            defmt::info!(
                "Got {} samples ({} bytes) and {} leftover samples",
                n_bytes / 3,
                n_bytes,
                leftover_bytes / 3
            );

            // 1. Calculate the number of bytes in our in_buf we need to send out.
            //    This is the received bytes + leftover bytes.
            // 2. Calculate how many bytes will be left over again.
            // 3. Create a slice which holds all the bytes we wan't to process and send out in this pass
            n_bytes += leftover_bytes;
            leftover_bytes = n_bytes % 12; // 12 = least common multiple of 3 and 4
            let work_slice_size = n_bytes - leftover_bytes;
            assert_eq!(work_slice_size % 4, 0);
            let work_slice = &mut in_buf[..work_slice_size];

            let vol = volume.load(core::sync::atomic::Ordering::Relaxed);
            convert_buf(work_slice, vol);

            // Incoming PCM data is little endian
            // b0:lo_L, b1:mid_L, b2:hi_L, b3:lo_R, b4:mid_R, b5:hi_R
            // I2S demands big endian data
            // The individual samples were already stored in the right format in the volume application step,
            // so we just copy from u8 slice to u32 slice in big endian format
            BigEndian::read_u32_into(work_slice, &mut back_buffer[..work_slice_size / 4]);

            // Bytes have now been written to the outgoing buffer,
            // Copy any leftover bytes to the start of the incoming buffer.
            if leftover_bytes != 0 {
                let (start, leftover) = in_buf.split_at_mut(work_slice_size);
                start[..leftover_bytes].copy_from_slice(&leftover[..leftover_bytes]);
            }

            let i2s_passed_time = i2s_start_time.elapsed();

            if i2s_passed_time > Duration::from_millis(1) {
                defmt::warn!("I2S already ran for {}us", i2s_passed_time.as_micros());
            } else if i2s_passed_time > Duration::from_micros(800) {
                defmt::warn!(
                    "Estimated I2S remaining time is under {}us",
                    (Duration::from_millis(1) - i2s_passed_time).as_micros()
                );
                // This happens when the I2S Clock is running marginally faster than the PC delivers the Samples.
                // We compensate by repeating the last 4 samples (2 left, 2 right) for the current buffer.
                // let end_pos = n_bytes / 4;
                // let (front, back) = back_buffer.split_at_mut(end_pos);
                // back[..3].copy_from_slice(&front[end_pos - 3..]);
                // n_bytes += 12;
            }

            dma_future.await;
            mem::swap(&mut back_buffer, &mut front_buffer);
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn feedback_task(
    mut feedback: Feedback<'static, Driver<'static, USB>>,
    synch_feedback: &'static Signal<ThreadModeRawMutex, Duration>,
) -> ! {
    use fixed::types::U10F22;
    const FEEDBACK_RATE: usize = 64;

    let mut desired_spp: f64 = 48.;
    loop {
        let prev_delay = synch_feedback.wait().await.as_micros() as i64;
        for _ in 0..FEEDBACK_RATE - 2 {
            info!("delay: {}", synch_feedback.wait().await);
        }
        let delay = synch_feedback.wait().await.as_micros() as i64;
        let delta = prev_delay - delay;
        info!(
            "Delay changed from {}us to {}us ({}us delta) over the last 64 packets",
            prev_delay, delay, delta
        );

        if delta > 500 {
            defmt::warn!("Found delta greater than 20us, skipping feedback!");
            continue;
        }

        // delta us per packet
        let d_us_pp = delta as f64 / FEEDBACK_RATE as f64;
        // desired samples per packet
        desired_spp = (-1. * d_us_pp * samples_p_us(48000.)) + 48.;
        info!("Desired samples per packet should thus be {}", desired_spp);
        let desired_spp_10_22: U10F22 = U10F22::from_num(desired_spp);

        let data = desired_spp_10_22.to_le_bytes();

        info!("In 10.14 format: {:x}", data[1..]);

        match feedback.write_packet(&data[1..]).await {
            Ok(_) => {}
            Err(e) => {
                defmt::error!("Error writing feedback packet: {}", e);
            }
        }
    }
}

const fn samples_p_us(sample_rate_hz: f64) -> f64 {
    sample_rate_hz / 1_000_000.
}

fn create_usb_conf() -> Config<'static> {
    let mut conf = Config::new(0x4242, 0x1234);
    conf.product = Some("RP2040 Soundcard");
    conf.manufacturer = Some("RT-CUSTOMZ");
    conf
}

/// Convert the buffer from 24bit PCM big- to little-endian and apply a attenuation
///
/// The volume (attenuation) has to be in dB from 0dB (no attenuation) to 100dB (maximum attenuation).
#[inline]
fn convert_buf(buf: &mut [u8], vol: u8) {
    assert!(vol <= 100);
    for i in (0..buf.len()).step_by(3) {
        let mut sample = LittleEndian::read_i24(&buf[i..i + 3]);
        if LUT_100DB[vol as usize] != 1 {
            sample /= LUT_100DB[vol as usize] as i32;
        }
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
