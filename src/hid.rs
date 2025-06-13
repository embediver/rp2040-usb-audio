use embassy_futures::select::{Either, select};
use embassy_rp::{
    Peri,
    gpio::{Input, Pin},
};
use embassy_usb::class::hid::{Config, HidWriter};
use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport, SerializedDescriptor};

pub struct Inputs {
    vol_up: Input<'static>,
    vol_down: Input<'static>,
}

use defmt::info;

impl Inputs {
    pub fn new(vol_up: Peri<'static, impl Pin>, vol_down: Peri<'static, impl Pin>) -> Inputs {
        let mut inp = Inputs {
            vol_up: Input::new(vol_up, embassy_rp::gpio::Pull::Up),
            vol_down: Input::new(vol_down, embassy_rp::gpio::Pull::Up),
        };
        inp.vol_up.set_schmitt(true);
        inp.vol_down.set_schmitt(true);
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
const NO_MEDIA_KEY_PRESSED: MediaKeyboardReport = MediaKeyboardReport {
    usage_id: MediaKey::Zero as u16,
};

#[embassy_executor::task]
pub async fn hid_task(inputs: Inputs, mut hid_writer: RpHidWriter) -> ! {
    hid_writer.ready().await;

    let (mut vol_up, mut vol_down) = (inputs.vol_up, inputs.vol_down);
    loop {
        let up = match select(vol_up.wait_for_low(), vol_down.wait_for_low()).await {
            Either::First(_) => true,
            Either::Second(_) => false,
        };

        if up {
            info!("Volume up key pressed!");
            hid_writer.write_serialize(&VOL_UP_REPORT).await.unwrap();
            vol_up.wait_for_high().await;
            info!("Volume up key released!");
            hid_writer
                .write_serialize(&NO_MEDIA_KEY_PRESSED)
                .await
                .unwrap();
        } else {
            info!("Volume down key pressed!");
            hid_writer.write_serialize(&VOL_DOWN_REPORT).await.unwrap();
            vol_down.wait_for_high().await;
            info!("Volume down key released!");
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
