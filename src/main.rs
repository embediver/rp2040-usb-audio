#![no_std]
#![no_main]

use core::{mem, sync::atomic::AtomicU8};

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::{PIO0, USB},
    pio::Pio,
    pio_programs::i2s::{PioI2sOut, PioI2sOutProgram},
    usb::{self, Driver},
};
use embassy_time::{Duration, Instant};
#[cfg(feature = "panic_probe")]
use panic_probe as _;
use static_cell::StaticCell;

use embassy_sync::signal::Signal;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, watch};
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
#[cfg(feature = "lut_based_volume")]
use proc_lut::lut_100;

#[cfg(all(feature = "hid_buttons", feature = "hid_rotary"))]
compile_error!("Only one of the features 'hid_buttons', 'hid_rotary' may be activated.");

#[cfg(feature = "hid_buttons")]
mod hid_button;
#[cfg(feature = "hid_buttons")]
use hid_button as hid;
#[cfg(feature = "hid_rotary")]
mod hid_rotary;
#[cfg(feature = "hid_rotary")]
use hid_rotary as hid;

/// Look-up-table for factors from 0-100dB
#[cfg(feature = "lut_based_volume")]
const LUT_100DB: [u32; 101] = lut_100!();

/// Maximum USB PCM Packet/Frame size (in bytes)
///
/// 24bit (3 Byte) * 2samples (left/right) * 96kHz * 1ms (packet/frame interval) + 24bit * 6samples
///
/// 6sample (3left, 3right) extra to account for sync mismatch compensation.
const MAX_PACKET_SIZE: usize = (3 * 2) * (96000 / 1000) + (3 * 6);

/// u32 DMA buffer size
///
/// Two times the size to hold a single frame.
const DMA_BUFFER_SIZE: usize = MAX_PACKET_SIZE / 4 * 2;

/// Synch feedback every x packets
const FEEDBACK_RATE: FeedbackRefresh = FeedbackRefresh::Period32Frames;

/// Supported sample rates
// TODO: While 96kHz works, the performance budget is a litle tight.
const SUPPORTED_SAMPLE_RATES: [u32; 3] = [44100, 48000, 96000];

/// Timeout after which the DAC gets muted when no data arrived
const PACKET_TIMEOUT_US: u64 = 900;

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// Pinout: GP0, GP1, GND, GP2, .., GP5, GND, GP6, GP7, GP8
//         1,   2, ...                  8,   9,   10,  11
struct I2SPeripherals {
    bit_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_2>,
    left_right_clock_pin: Peri<'static, embassy_rp::peripherals::PIN_3>,
    data_pin: Peri<'static, embassy_rp::peripherals::PIN_4>,
    dma_channel: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    pio0: Peri<'static, embassy_rp::peripherals::PIO0>,
    mute: Peri<'static, embassy_rp::peripherals::PIN_1>,
}

#[cfg(not(feature = "panic_probe"))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let i2s_peri = I2SPeripherals {
        bit_clock_pin: p.PIN_2,
        left_right_clock_pin: p.PIN_3,
        data_pin: p.PIN_4,
        dma_channel: p.DMA_CH0,
        pio0: p.PIO0,
        mute: p.PIN_1,
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
        MAX_PACKET_SIZE as u16,
        SampleWidth::Width3Byte,
        &SUPPORTED_SAMPLE_RATES,
        &[Channel::LeftFront, Channel::RightFront],
        FEEDBACK_RATE,
    );

    #[cfg(feature = "hid")]
    {
        use embassy_usb::class::hid::{HidWriter, State};

        static HID_STATE: StaticCell<State> = StaticCell::new();

        let hid: hid::RpHidWriter = HidWriter::new(
            &mut usb_builder,
            HID_STATE.init(State::new()),
            crate::hid::hid_config(),
        );

        let inputs = hid::Inputs::new(p.PIN_8, p.PIN_9, p.PIN_10, Some(p.PIN_11.into()), p.PIO1);

        spawner.must_spawn(hid::hid_task(inputs, hid));
    }

    let usb = usb_builder.build();

    spawner.must_spawn(usb_task(usb));

    feedback.wait_connection().await;
    audio_stream.wait_connection().await;

    static VOLUME: AtomicU8 = AtomicU8::new(0);
    static SAMPLE_RATE: watch::Watch<ThreadModeRawMutex, u32, 2> = watch::Watch::new();
    static SYNCH_FEEDBACK: Signal<ThreadModeRawMutex, Duration> = Signal::new();

    spawner.must_spawn(feedback_task(
        feedback,
        &SYNCH_FEEDBACK,
        SAMPLE_RATE.dyn_receiver().unwrap(),
    ));

    spawner.must_spawn(audio_processor(
        audio_stream,
        &VOLUME,
        SAMPLE_RATE.dyn_receiver().unwrap(),
        &SYNCH_FEEDBACK,
        i2s_peri,
    ));

    let mut old_rate = 0;
    loop {
        audio_ctrl.changed().await;
        let volume = audio_ctrl.volume(Channel::LeftFront);
        let rate = audio_ctrl.sample_rate_hz();
        match volume {
            Some(Volume::Muted) => info!("Speaker is muted"),
            Some(Volume::DeciBel(db)) => {
                info!("Volume is {}dB", db);
                VOLUME.store(db.abs() as u8, core::sync::atomic::Ordering::Relaxed);
            }
            None => (),
        }
        info!("Sample rate is {}", rate);
        if rate != 0 && rate != old_rate {
            old_rate = rate;
            SAMPLE_RATE.sender().send(rate);
        }
    }
}

#[embassy_executor::task]
async fn audio_processor(
    mut stream: Stream<'static, Driver<'static, USB>>,
    volume: &'static AtomicU8,
    mut sample_rate: watch::DynReceiver<'static, u32>,
    synch_feedback: &'static Signal<ThreadModeRawMutex, Duration>,
    mut i2s_peri: I2SPeripherals,
) -> ! {
    // Activate DAC mute
    let mut not_mute = embassy_rp::gpio::Output::new(i2s_peri.mute, embassy_rp::gpio::Level::Low);

    stream.wait_connection().await;
    let mut current_rate = sample_rate.get().await;

    let mut in_buf = [0; MAX_PACKET_SIZE];

    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    static DMA_BUFFER: StaticCell<[u32; DMA_BUFFER_SIZE]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; DMA_BUFFER_SIZE]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(DMA_BUFFER_SIZE / 2);

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
        info!("I2S set-up to {}Hz", current_rate);

        let mut leftover_bytes = 0;

        // process audio data until sample-rate changes
        let mut next_buf_size = 0;
        let mut data_good = false;
        'inner: loop {
            if let Some(rate) = sample_rate.try_changed() {
                current_rate = rate;
                info!("Sample rate changed to {}Hz, restarting I2S...", rate);
                not_mute.set_low();
                break 'inner;
            }

            let dma_future = i2s.write(&front_buffer[..next_buf_size]);
            let i2s_start_time = Instant::now();
            if data_good {
                // In case of good data, unmute after transfer is started
                not_mute.set_high();
                info!("Unmute");
            }

            let watch_dog_timer =
                embassy_time::Timer::after(Duration::from_micros(PACKET_TIMEOUT_US));
            let packet_result = match embassy_futures::select::select(
                stream.read_packet(&mut in_buf[leftover_bytes..]),
                watch_dog_timer,
            )
            .await
            {
                embassy_futures::select::Either::First(p) => p,
                embassy_futures::select::Either::Second(_) => {
                    info!("Mute (packet timeout)");
                    not_mute.set_low();
                    dma_future.await;
                    continue 'inner;
                }
            };
            let mut n_bytes = match packet_result {
                Ok(n) => n,
                Err(e) => {
                    info!("Stream read error: {}", e);
                    not_mute.set_low();
                    dma_future.await;
                    continue 'inner;
                }
            };
            // sanity check if we got an equal amount of left and right samples with 24bit each
            // TODO: this apparently triggers sometimes, find out why.
            //       Can a packet end with a uneven number of samples
            //       and thus miss a sample for one of the channels?
            //       Will the next packet start with the missing sample?
            //
            // Use assert to panic (and reset the chip if the panic-reset panic-handler is used,
            //    `panic_probe` feature disabled).
            // assert_eq!(n_bytes % 6, 0);

            // Or try to mute and wait for a good packet
            if n_bytes % 6 == 0 {
                // Data good
                data_good = true;
            } else {
                // Funny amount of bytes received
                not_mute.set_low(); // instantly mute
                data_good = false;
                info!("mute (malformed packet)")
            }

            let usb_packet_delay = i2s_start_time.elapsed();
            synch_feedback.signal(usb_packet_delay);
            // info!(
            //     "Got {} samples ({} bytes) and {} leftover samples",
            //     n_bytes / 3,
            //     n_bytes,
            //     leftover_bytes / 3
            // );

            // 1. Calculate the number of bytes in our in_buf we need to send out.
            //    This is the received bytes + leftover bytes.
            // 2. Calculate how many bytes will be left over again.
            // 3. Create a slice which holds all the bytes we wan't to process and send out in this pass
            n_bytes += leftover_bytes;
            leftover_bytes = n_bytes % 12; // 12 = least common multiple of 3 (one sample) and 4 (one dma buffer value)
            let work_slice_size = n_bytes - leftover_bytes;
            assert_eq!(work_slice_size % 4, 0);
            let work_slice = &mut in_buf[..work_slice_size];

            // info!(
            //     "Working with {} bytes ({} samples)",
            //     work_slice_size,
            //     work_slice_size / 3
            // );

            let vol = volume.load(core::sync::atomic::Ordering::Relaxed);
            convert_and_attenuate_buf(work_slice, vol);

            // Incoming PCM data is little endian
            // b0:lo_L, b1:mid_L, b2:hi_L, b3:lo_R, b4:mid_R, b5:hi_R
            // I2S demands big endian data
            // The individual samples were already stored in the right format in the volume application step,
            // so we just copy from u8 slice to u32 slice in big endian format
            next_buf_size = work_slice_size / 4;
            BigEndian::read_u32_into(work_slice, &mut back_buffer[..next_buf_size]);

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
    mut sample_rate: watch::DynReceiver<'static, u32>,
) -> ! {
    let mut desired_spp = 48.;
    let mut prev_avg = 0;
    loop {
        let sample_rate = sample_rate.get().await;
        // Nominal samples per packet
        let nominal_spp = sample_rate as f64 / 1000.0;

        let mut values = [0; FEEDBACK_RATE.frame_count()];
        for val in &mut values {
            *val = synch_feedback.wait().await.as_micros() as u32;
        }
        let sum: u32 = values.iter().sum();
        let avg_delay = sum / FEEDBACK_RATE.frame_count() as u32;

        let delta = prev_avg as i16 - avg_delay as i16;
        prev_avg = avg_delay;

        // info!(
        //     "Avg delay for last {} packets is {}us, {}us delta to previous",
        //     FEEDBACK_RATE.frame_count(),
        //     avg_delay,
        //     delta
        // );

        if avg_delay > 100 {
            // Packets arrive to late
            // Set to a faster rate...
            desired_spp = nominal_spp + 0.03;
            defmt::warn!(
                "Packets arrive to late, setting spp hard to {}spp",
                desired_spp
            );
            write_feedback(&mut feedback, desired_spp).await;
            continue;
        }

        if avg_delay < 20 {
            // Packets arrive to early to determine a correct delta
            // Set to a slower rate...
            desired_spp = nominal_spp - 0.03;
            defmt::warn!(
                "Packets arive to early, setting spp hard to {}spp",
                desired_spp
            );
            write_feedback(&mut feedback, desired_spp).await;
            continue;
        }

        if delta > 200 {
            defmt::warn!("Found delta greater than 100us, skipping feedback!");
            continue;
        }

        // Average delay delta in us per packet.
        // Estimate of the difference between the usb and i2s packet/transfer rate.
        // 1us means the usb hosts sample rate is 1us/packet slower than the i2s sample rate.
        let d_us_pp = delta as f64 / FEEDBACK_RATE.frame_count() as f64;
        // desired samples per packet
        desired_spp += -0.3 * d_us_pp * samples_p_us(sample_rate as f64);
        // info!("Desired samples per packet should thus be {}", desired_spp);

        write_feedback(&mut feedback, desired_spp).await;
    }
}

/// Formats a value as 10.14 fixed point value as per USB 2.0 spec Section 5.12.4.2
fn format_10_14(num: f64) -> [u8; 3] {
    use fixed::types::U10F22;
    let desired_spp_10_22: U10F22 = U10F22::from_num(num);

    let mut out = [0; 3];
    out.copy_from_slice(&desired_spp_10_22.to_le_bytes()[1..]);
    out
}

/// Write a feedback value to the feedback endpoint
///
/// `desired_spp` is the desired samples per packet.
async fn write_feedback(feedback: &mut Feedback<'static, Driver<'static, USB>>, desired_spp: f64) {
    let data = format_10_14(desired_spp);
    match feedback.write_packet(&data).await {
        Ok(_) => {}
        Err(e) => {
            defmt::error!("Error writing feedback packet: {}", e);
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
    conf.max_power = 400;
    conf
}

/// Convert the buffer from 24bit PCM big- to little-endian and apply a attenuation
///
/// The volume (attenuation) has to be in dB from 0dB (no attenuation) to 100dB (maximum attenuation).
#[inline]
#[cfg(feature = "lut_based_volume")]
fn convert_and_attenuate_buf(buf: &mut [u8], vol: u8) {
    assert!(vol <= 100);
    for i in (0..buf.len()).step_by(3) {
        let mut sample = LittleEndian::read_i24(&buf[i..i + 3]);
        if LUT_100DB[vol as usize] != 1 {
            sample /= LUT_100DB[vol as usize] as i32;
        }
        BigEndian::write_i24(&mut buf[i..i + 3], sample);
    }
}

/// Convert the buffer from 24bit PCM big- to little-endian and apply a attenuation
///
/// The volume (attenuation) has to be in dB from 0dB (no attenuation) to 192dB (maximum attenuation).
/// Attenuation is applied in 3dB steps with shift operations.
#[inline]
#[cfg(not(feature = "lut_based_volume"))]
fn convert_and_attenuate_buf(buf: &mut [u8], vol: u8) {
    assert!(vol <= 192);

    // 6dB attenuation equals a right shift of 1 (division by two)
    // By shifting a second time and substracting the result from the first operation,
    // a further 3dB attenuation can be archieved.

    let steps_6db = vol / 6; // how many 6dB shifts to apply
    let step_3db = vol % 6 >= 3; // how many 3dB shifts to apply

    for i in (0..buf.len()).step_by(3) {
        let mut sample = LittleEndian::read_i24(&buf[i..i + 3]);
        if vol >= 3 {
            let mut tmp = sample >> steps_6db;
            if step_3db {
                tmp -= sample >> 2;
            }
            sample = tmp;
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
