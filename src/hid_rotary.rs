use embassy_futures::select::{Either, Either3, select, select3};
use embassy_rp::{Peri, bind_interrupts, gpio::Input, mode::Async, peripherals::PIO1, pio::Pio};
use embassy_time::Delay;
use embassy_usb::class::hid::device::{Config, HidWriter};
use embedded_hal_async::delay::DelayNs;
use usbd_hid::descriptor::{AsInputReport, MediaKey, MediaKeyboardReport, SerializedDescriptor};

pub struct Inputs {
    rotary_a: Peri<'static, embassy_rp::peripherals::PIN_8>,
    rotary_b: Peri<'static, embassy_rp::peripherals::PIN_9>,
    rotary_sw: Input<'static, Async>,
    button: Option<Input<'static, Async>>,
    pio: Peri<'static, embassy_rp::peripherals::PIO1>,
}

use defmt::info;

bind_interrupts!(struct PioIrqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

impl Inputs {
    /// Create a new input configuration from pins
    pub fn new(
        rotary_a: Peri<'static, embassy_rp::peripherals::PIN_8>,
        rotary_b: Peri<'static, embassy_rp::peripherals::PIN_9>,
        rotary_sw: Input<'static, Async>,
        button: Option<Input<'static, Async>>,
        pio1: Peri<'static, PIO1>,
    ) -> Inputs {
        let inp = Inputs {
            rotary_a,
            rotary_b,
            rotary_sw,
            button,
            pio: pio1,
        };
        inp
    }
}

pub type RpHidWriter =
    HidWriter<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>, 8>;

const VOL_UP_REPORT: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::VolumeIncrement as u16,
};
const VOL_DOWN_REPORT: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::VolumeDecrement as u16,
};
const PLAY_PAUSE_REPORT: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::PlayPause as u16,
};
const NEXT_TRACK_REPORT: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::NextTrack as u16,
};
const NO_MEDIA_KEY_PRESSED: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::Zero as u16,
};

#[embassy_executor::task]
pub async fn hid_task(inputs: Inputs, mut hid_writer: RpHidWriter) -> ! {
    use embassy_rp::pio_programs::rotary_encoder::*;

    hid_writer.ready().await;

    let mut report = [0; 4];

    let mut rotary_sw = inputs.rotary_sw;
    let mut button = inputs.button;
    rotary_sw.set_schmitt(true);
    if let Some(ref mut b) = button {
        b.set_schmitt(true);
    }

    let Pio {
        mut common, sm0, ..
    } = Pio::new(inputs.pio, PioIrqs);
    let rot_prog = PioEncoderProgram::new(&mut common, true);
    let mut rot_pio = PioEncoder::new(
        &mut common,
        sm0,
        inputs.rotary_a,
        inputs.rotary_b,
        &rot_prog,
    );

    loop {
        let (rotary, sw, btn_event) = if let Some(ref mut b) = button {
            match select3(rot_pio.read(), rotary_sw.wait_for_low(), b.wait_for_low()).await {
                Either3::First(d) => (Some(d), false, false),
                Either3::Second(_) => (None, true, false),
                Either3::Third(_) => (None, false, true),
            }
        } else {
            match select(rot_pio.read(), rotary_sw.wait_for_low()).await {
                Either::First(d) => (Some(d), false, false),
                Either::Second(_) => (None, true, false),
            }
        };

        if let Some(Direction::Clockwise) = rotary {
            info!("Rotary CW");
            let len = VOL_UP_REPORT.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            Delay.delay_ms(10).await;
            let len = NO_MEDIA_KEY_PRESSED.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
        }
        if let Some(Direction::CounterClockwise) = rotary {
            info!("Rotary CCW");
            let len = VOL_DOWN_REPORT.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            Delay.delay_ms(10).await;
            let len = NO_MEDIA_KEY_PRESSED.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
        }

        if sw {
            info!("Rotary SW pressed");
            let len = PLAY_PAUSE_REPORT.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            rotary_sw.wait_for_high().await;
            info!("Rotary SW released");
            let len = NO_MEDIA_KEY_PRESSED.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            Delay.delay_ms(100).await;
        }

        if btn_event {
            info!("Button pressed");
            let len = NEXT_TRACK_REPORT.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            rotary_sw.wait_for_high().await;
            info!("Button released");
            let len = NO_MEDIA_KEY_PRESSED.serialize(&mut report).unwrap();
            hid_writer.write(&report[..len]).await.unwrap();
            Delay.delay_ms(100).await;
        }
    }
}

pub fn hid_config() -> Config<'static> {
    Config {
        report_descriptor: MediaKeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
        hid_subclass: embassy_usb::class::hid::HidSubclass::No,
        hid_boot_protocol: embassy_usb::class::hid::HidBootProtocol::None,
    }
}
