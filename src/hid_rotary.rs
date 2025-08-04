use embassy_futures::select::{Either, select};
use embassy_rp::{Peri, bind_interrupts, gpio::Input, peripherals::PIO1, pio::Pio};
use embassy_time::Delay;
use embassy_usb::class::hid::{Config, HidWriter};
use embedded_hal_async::delay::DelayNs;
use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport, SerializedDescriptor};

pub struct Inputs {
    rotary_a: Peri<'static, embassy_rp::peripherals::PIN_8>,
    rotary_b: Peri<'static, embassy_rp::peripherals::PIN_9>,
    rotary_sw: Input<'static>,
    pio: Peri<'static, embassy_rp::peripherals::PIO1>,
}

use defmt::info;

bind_interrupts!(struct PioIrqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

impl Inputs {
    /// Create a new input configuration from pins
    ///
    /// Configures the switch pin with schmitt-trigger, no pull.
    pub fn new(
        rotary_a: Peri<'static, embassy_rp::peripherals::PIN_8>,
        rotary_b: Peri<'static, embassy_rp::peripherals::PIN_9>,
        rotary_sw: Peri<'static, embassy_rp::peripherals::PIN_10>,
        pio1: Peri<'static, PIO1>,
    ) -> Inputs {
        let mut inp = Inputs {
            rotary_a,
            rotary_b,
            rotary_sw: Input::new(rotary_sw, embassy_rp::gpio::Pull::None),
            pio: pio1,
        };
        inp.rotary_sw.set_schmitt(true);
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
const NO_MEDIA_KEY_PRESSED: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::Zero as u16,
};

#[embassy_executor::task]
pub async fn hid_task(inputs: Inputs, mut hid_writer: RpHidWriter) -> ! {
    use embassy_rp::pio_programs::rotary_encoder::*;

    hid_writer.ready().await;

    let mut rotary_sw = inputs.rotary_sw;

    let Pio {
        mut common, sm0, ..
    } = Pio::new(inputs.pio, PioIrqs);
    let rot_prog = PioEncoderProgram::new(&mut common);
    let mut rot_pio = PioEncoder::new(
        &mut common,
        sm0,
        inputs.rotary_a,
        inputs.rotary_b,
        &rot_prog,
    );

    loop {
        let (rotary, sw) = match select(rot_pio.read(), rotary_sw.wait_for_low()).await {
            Either::First(d) => (Some(d), false),
            Either::Second(_) => (None, true),
        };

        if let Some(Direction::Clockwise) = rotary {
            info!("Rotary CW");
            hid_writer.write_serialize(&VOL_UP_REPORT).await.unwrap();
            Delay.delay_ms(10).await;
            hid_writer
                .write_serialize(&NO_MEDIA_KEY_PRESSED)
                .await
                .unwrap();
        }
        if let Some(Direction::CounterClockwise) = rotary {
            info!("Rotary CCW");
            hid_writer.write_serialize(&VOL_DOWN_REPORT).await.unwrap();
            Delay.delay_ms(10).await;
            hid_writer
                .write_serialize(&NO_MEDIA_KEY_PRESSED)
                .await
                .unwrap();
        }

        if sw {
            info!("Rotary SW pressed");
            hid_writer
                .write_serialize(&PLAY_PAUSE_REPORT)
                .await
                .unwrap();
            rotary_sw.wait_for_high().await;
            info!("Rotary SW released");
            hid_writer
                .write_serialize(&NO_MEDIA_KEY_PRESSED)
                .await
                .unwrap();
        }
    }
}

pub fn hid_config() -> Config<'static> {
    Config {
        report_descriptor: MediaKeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    }
}
