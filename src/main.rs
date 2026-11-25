#![no_std]
#![no_main]

use core::{mem, sync::atomic::AtomicU8};

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
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

/// Size of the incoming byte buffer
///
/// A full packet plus some leftover bytes from the previous packet. If the read buffer
/// is smaller than a packet, the rp2040 usb driver returns `BufferOverflow` without re-arming
/// the endpoint, after which every following read fails and audio stays dead.
const IN_BUF_SIZE: usize = MAX_PACKET_SIZE + 12;

/// I2S slot width in bits
///
/// The PCM5102A (clocked from BCK by its internal PLL, no MCLK) only supports 32fs or 64fs
/// bit clocks. 2x24bit slots (48fs) happen to work at 48kHz but not at 96kHz, so 24bit
/// samples are sent MSB aligned in 32bit slots (64fs).
const I2S_SLOT_BITS: u32 = 32;

/// u32 DMA buffer size
///
/// Two times the size to hold the processed data of one packet (one u32 per 3 byte sample).
const DMA_BUFFER_SIZE: usize = IN_BUF_SIZE / 3 * 2;

/// Synch feedback every x packets
const FEEDBACK_RATE: FeedbackRefresh = FeedbackRefresh::Period32Frames;

/// Supported sample rates
// TODO: While 96kHz works, the performance budget is a litle tight.
const SUPPORTED_SAMPLE_RATES: [u32; 3] = [44100, 48000, 96000];

/// Timeout after which the DAC gets muted when no data arrived
const PACKET_TIMEOUT_US: u64 = 900;

/// Reset the chip on a sample rate change instead of only restarting I2S
///
/// The PCM5102A stays silent after a sample rate change at runtime, while every sample rate
/// works when it is set on startup.
const RESET_ON_RATE_CHANGE: bool = true;

/// Time the I2S clocks are stopped on a sample rate change, so the DAC re-detects the rate
/// (only used when `RESET_ON_RATE_CHANGE` is disabled)
const DAC_CLOCK_HALT: Duration = Duration::from_millis(500);

/// Time without any audio packets after which the amplifier power gets switched off
const AMP_POWER_OFF_DELAY: Duration = Duration::from_secs(5);

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
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    #[cfg(feature = "power_control")]
    let amp_power = Output::new(p.PIN_19, embassy_rp::gpio::Level::Low);

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
            create_usb_conf(p.FLASH),
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

    #[cfg(feature = "power_control")]
    let amp_power = Some(amp_power);
    #[cfg(not(feature = "power_control"))]
    let amp_power = None;
    spawner.must_spawn(audio_processor(
        audio_stream,
        &VOLUME,
        SAMPLE_RATE.dyn_receiver().unwrap(),
        &SYNCH_FEEDBACK,
        i2s_peri,
        amp_power,
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
    mut amp_power: Option<Output<'static>>,
) -> ! {
    // Activate DAC mute
    let mut not_mute = embassy_rp::gpio::Output::new(i2s_peri.mute, embassy_rp::gpio::Level::Low);

    stream.wait_connection().await;
    let mut current_rate = sample_rate.get().await;

    let mut in_buf = [0; IN_BUF_SIZE];

    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    static DMA_BUFFER: StaticCell<[u32; DMA_BUFFER_SIZE]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; DMA_BUFFER_SIZE]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(DMA_BUFFER_SIZE / 2);

    let mut restart = false;
    loop {
        if restart {
            // Stop BCK/LRCK for a while before restarting with a new sample rate, so the
            // restart looks like a fresh power-up to the PCM5102A. Without a clean clock halt
            // it doesn't pick up the new rate and stays silent.
            //
            // The lines have to be actively driven low: `make_pio_pin` disables the pad pulls,
            // so after the PIO got dropped they would float and the DAC would see noise
            // instead of stopped clocks.
            //
            // The amp power is switched off as well and switched on again with the first
            // good packet, when the I2S clocks are already running (same as after boot).
            if let Some(ref mut p) = amp_power {
                p.set_low();
            }
            let _bck = Output::new(i2s_peri.bit_clock_pin.reborrow(), Level::Low);
            let _lrck = Output::new(i2s_peri.left_right_clock_pin.reborrow(), Level::Low);
            let _data = Output::new(i2s_peri.data_pin.reborrow(), Level::Low);
            embassy_time::Timer::after(DAC_CLOCK_HALT).await;
        }
        restart = true;

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
            I2S_SLOT_BITS,
            &program,
        );
        info!("I2S set-up to {}Hz", current_rate);

        let mut leftover_bytes = 0;
        let mut malformed_packets = 0u32;
        let mut last_packet_time = Instant::now();
        #[cfg(feature = "panic_probe")]
        let mut recent_packet_lengths = [0u16; 16];
        #[cfg(feature = "panic_probe")]
        let mut recent_packet_count = 0usize;

        // process audio data until sample-rate changes
        let mut next_buf_size = 0;
        let mut data_good = false;
        'inner: loop {
            if let Some(rate) = sample_rate.try_changed() {
                current_rate = rate;
                not_mute.set_low();
                if RESET_ON_RATE_CHANGE {
                    // The PCM5102A reliably picks up a sample rate only on a fresh start;
                    // restarting the I2S clocks (even with a clock halt) leaves it silent.
                    // Reset the whole chip instead: the host re-enumerates the device and
                    // sets the selected sample rate again right away.
                    info!("Sample rate changed to {}Hz, resetting...", rate);
                    if let Some(ref mut p) = amp_power {
                        p.set_low();
                    }
                    // let the soft mute ramp (104 samples) finish before the reset
                    embassy_time::Timer::after(Duration::from_millis(10)).await;
                    cortex_m::peripheral::SCB::sys_reset();
                }
                info!("Sample rate changed to {}Hz, restarting I2S...", rate);
                break 'inner;
            }

            let dma_future = i2s.write(&front_buffer[..next_buf_size]);
            let i2s_start_time = Instant::now();
            if data_good {
                // In case of good data, unmute after transfer is started
                if not_mute.is_set_low() {
                    info!("Unmute");
                }
                if let Some(ref mut p) = amp_power {
                    p.set_high();
                }
                not_mute.set_high();
            }

            let watch_dog_timer =
                embassy_time::Timer::after(Duration::from_micros(PACKET_TIMEOUT_US));
            #[cfg(feature = "panic_probe")]
            let packet_read = async {
                use core::future::{Future, poll_fn};

                let mut read = core::pin::pin!(stream.read_packet(&mut in_buf[leftover_bytes..]));
                poll_fn(|context| {
                    let endpoint = embassy_rp::pac::USB_DPRAM.ep_out_control(0).read().0;
                    let buffer = embassy_rp::pac::USB_DPRAM.ep_out_buffer_control(1).read().0;
                    read.as_mut()
                        .poll(context)
                        .map(|result| (result, endpoint, buffer))
                })
                .await
            };
            #[cfg(not(feature = "panic_probe"))]
            let packet_read = async {
                (stream.read_packet(&mut in_buf[leftover_bytes..]).await, 0u32, 0u32)
            };
            let packet_result = match embassy_futures::select::select(
                packet_read,
                watch_dog_timer,
            )
            .await
            {
                embassy_futures::select::Either::First(p) => p,
                embassy_futures::select::Either::Second(_) => {
                    data_good = false;
                    if not_mute.is_set_high() {
                        info!("Mute (packet timeout)");
                    }
                    not_mute.set_low();
                    dma_future.await;

                    // Only cut amp power after a longer pause. Single missed packets happen
                    // under host load (e.g. gaming) and power-cycling the amp makes a short
                    // hiccup a long dropout.
                    if last_packet_time.elapsed() > AMP_POWER_OFF_DELAY {
                        if let Some(ref mut p) = amp_power {
                            if p.is_set_high() {
                                info!("Amp power off (idle)");
                            }
                            p.set_low();
                        }
                    }

                    continue 'inner;
                }
            };
            let (packet_result, _endpoint_status, _buffer_status) = packet_result;
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
                #[cfg(feature = "panic_probe")]
                {
                    recent_packet_lengths[recent_packet_count % recent_packet_lengths.len()] =
                        n_bytes as u16;
                    recent_packet_count = recent_packet_count.wrapping_add(1);
                }
            } else {
                // Funny amount of bytes received
                #[cfg(feature = "panic_probe")]
                {
                    let count = recent_packet_count.min(recent_packet_lengths.len());
                    let mut ordered = recent_packet_lengths;
                    if recent_packet_count >= recent_packet_lengths.len() {
                        ordered.rotate_left(recent_packet_count % recent_packet_lengths.len());
                    }
                    defmt::error!(
                        "Last packet lengths (oldest first): {:?}",
                        &ordered[..count]
                    );
                    defmt::error!(
                        "Audio OUT EP1 before read poll: enabled={}, full={}, available={}, register length={}, received={}, leftover={}, sample rate={}",
                        _endpoint_status & 0x8000_0000 != 0,
                        _buffer_status & 0x8000 != 0,
                        _buffer_status & 0x0400 != 0,
                        _buffer_status & 0x03ff,
                        n_bytes,
                        leftover_bytes,
                        current_rate,
                    );
                }
                // Isochronous transfers have no retries, so a corrupted/truncated packet
                // can always happen (busy bus, bit errors, hubs). Don't panic/reset for that:
                // replace the packet with silence of (roughly) the same length so the I2S
                // stream and the sync feedback keep running. A 1ms gap is barely audible,
                // a reset and re-enumeration is not.
                malformed_packets = malformed_packets.wrapping_add(1);
                defmt::warn!(
                    "Malformed packet ({} bytes), replacing with silence ({} so far)",
                    n_bytes,
                    malformed_packets
                );
                n_bytes -= n_bytes % 6;
                in_buf[leftover_bytes..leftover_bytes + n_bytes].fill(0);
            }
            last_packet_time = Instant::now();

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
            leftover_bytes = n_bytes % 6; // 6 = one stereo frame (2 * 24bit)
            let work_slice_size = n_bytes - leftover_bytes;
            let work_slice = &in_buf[..work_slice_size];

            // info!(
            //     "Working with {} bytes ({} samples)",
            //     work_slice_size,
            //     work_slice_size / 3
            // );

            // Incoming PCM data is little endian
            // b0:lo_L, b1:mid_L, b2:hi_L, b3:lo_R, b4:mid_R, b5:hi_R
            // I2S gets one MSB aligned u32 word per sample (32bit slots).
            let vol = volume.load(core::sync::atomic::Ordering::Relaxed);
            next_buf_size = work_slice_size / 3;
            convert_and_attenuate_buf(work_slice, &mut back_buffer[..next_buf_size], vol);

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
    let mut prev_rate = 0;
    loop {
        let sample_rate = sample_rate.get().await;
        // Nominal samples per packet
        let nominal_spp = sample_rate as f64 / 1000.0;

        if sample_rate != prev_rate {
            // Restart the control loop from the nominal value, otherwise e.g. 48 spp
            // would still be reported after switching to 96kHz.
            prev_rate = sample_rate;
            desired_spp = nominal_spp;
            prev_avg = 0;
        }

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
            defmt::debug!(
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
            defmt::debug!(
                "Packets arive to early, setting spp hard to {}spp",
                desired_spp
            );
            write_feedback(&mut feedback, desired_spp).await;
            continue;
        }

        if delta > 200 {
            defmt::debug!("Found delta greater than 100us, skipping feedback!");
            continue;
        }

        // Average delay delta in us per packet.
        // Estimate of the difference between the usb and i2s packet/transfer rate.
        // 1us means the usb hosts sample rate is 1us/packet slower than the i2s sample rate.
        let d_us_pp = delta as f64 / FEEDBACK_RATE.frame_count() as f64;
        // desired samples per packet
        desired_spp += -0.3 * d_us_pp * samples_p_us(sample_rate as f64);
        // Keep the feedback within one sample of nominal, so the host never sends
        // more than fits into a packet and the control loop can't run away.
        desired_spp = desired_spp.clamp(nominal_spp - 1.0, nominal_spp + 1.0);
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

fn create_usb_conf(flash: Peri<'static, embassy_rp::peripherals::FLASH>) -> Config<'static> {
    use embassy_rp::flash::Flash;
    use embassy_sync::once_lock::OnceLock;
    let mut flash: Flash<'_, _, _, 0> = Flash::new_blocking(flash);
    let mut serial = [0; 16];
    let serial_ok = flash.blocking_unique_id(&mut serial).is_ok();
    static SERIAL_STR_BUF: OnceLock<[u8; 8]> = OnceLock::new();
    let serial_str = SERIAL_STR_BUF.get_or_init(|| {
        let mut tmp = [0; 8];
        const_hex::encode_to_str_upper(&serial[12..], &mut tmp).unwrap();
        tmp
    });

    let mut conf = Config::new(0x4242, 0x1234);
    conf.product = Some("RP2040 Soundcard");
    conf.manufacturer = Some("RT-CUSTOMZ");
    if serial_ok {
        conf.serial_number = Some(str::from_utf8(serial_str).unwrap())
    }
    conf.max_power = 400;
    conf
}

/// Attenuate interleaved 24bit little endian PCM and convert it for the I2S DMA
///
/// `input.len()` has to be a multiple of 3 and `output.len()` has to be `input.len() / 3`.
/// Every sample becomes one MSB aligned u32 word (24bit sample in a 32bit I2S slot).
///
/// This runs once per USB packet in the audio path: at 96kHz there are 192 samples per
/// millisecond on a Cortex-M0+.
#[inline(never)]
fn convert_and_attenuate_buf(input: &[u8], output: &mut [u32], vol: u8) {
    debug_assert_eq!(input.len() % 3, 0);
    debug_assert_eq!(input.len() / 3, output.len());
    let attenuate = attenuation(vol);

    for (inp, out) in input.chunks_exact(3).zip(output.iter_mut()) {
        // little endian 24bit -> sign extended i32
        let sample = i32::from_le_bytes([0, inp[0], inp[1], inp[2]]) >> 8;
        *out = (attenuate(sample) as u32) << 8;
    }
}

/// Attenuation function for a volume (attenuation) in dB from 0dB (no attenuation) to 100dB
/// (maximum attenuation).
///
/// The division uses the rp2040 hardware divider (embassy-rp `intrinsics` feature).
#[inline(always)]
#[cfg(feature = "lut_based_volume")]
fn attenuation(vol: u8) -> impl Fn(i32) -> i32 {
    let divisor = LUT_100DB[vol.min(100) as usize] as i32;
    move |sample| {
        if divisor == 1 {
            sample
        } else {
            sample / divisor
        }
    }
}

/// Attenuation function for a volume (attenuation) in dB from 0dB (no attenuation) to 192dB
/// (maximum attenuation).
///
/// Attenuation is applied in 3dB steps with shift operations.
#[inline(always)]
#[cfg(not(feature = "lut_based_volume"))]
fn attenuation(vol: u8) -> impl Fn(i32) -> i32 {
    assert!(vol <= 192);

    // 6dB attenuation equals a right shift of 1 (division by two)
    // By shifting a second time and substracting the result from the first operation,
    // a further 3dB attenuation can be archieved.

    let steps_6db = (vol / 6) as u32; // how many 6dB shifts to apply
    let step_3db = vol % 6 >= 3; // how many 3dB shifts to apply

    move |sample| {
        if vol < 3 {
            return sample;
        }
        let mut tmp = sample >> steps_6db.min(31);
        if step_3db {
            tmp -= sample >> 2;
        }
        tmp
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
