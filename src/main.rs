#![no_std]
#![no_main]

mod ffb;
mod hid;
mod io;
mod vesc;

use defmt::{debug, info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embassy_usb::class::hid::{HidReader, HidReaderWriter, HidWriter, ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type Pid = ffb::pid::PidEngine<4>;
type PidMutex = Mutex<NoopRawMutex, Pid>;

// USB descriptor/control buffers must be 'static because the UsbDevice is spawned into tasks.
static CONFIG_DESCRIPTOR: StaticCell<[u8; 512]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 1024]> = StaticCell::new();

#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn hid_out_task(
    mut reader: HidReader<'static, Driver<'static, USB>, 64>,
    pid: &'static PidMutex,
) -> ! {
    struct OutHandler {
        pid: &'static PidMutex,
    }

    impl RequestHandler for OutHandler {
        fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
            match id {
                ReportId::Out(rid) | ReportId::Feature(rid) => {
                    debug!("HID SET_REPORT rid=0x{:02x} len={}", rid, data.len());
                    unsafe {
                        self.pid.lock_mut(|p| p.handle_report_out(rid, data));
                    }
                    OutResponse::Accepted
                }
                _ => OutResponse::Rejected,
            }
        }

        fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
            match id {
                ReportId::Feature(rid) => self.pid.lock(|p| p.get_feature_report(rid, buf)),
                ReportId::In(rid) => self.pid.lock(|p| p.get_input_report(rid, buf)),
                _ => None,
            }
        }
    }

    reader.ready().await;
    let mut handler = OutHandler { pid };
    // Use report IDs: first byte of the OUT report is Report ID.
    reader.run(true, &mut handler).await
}

#[embassy_executor::task]
async fn hid_in_task(
    mut writer: HidWriter<'static, Driver<'static, USB>, 64>,
    pid: &'static PidMutex,
) -> ! {
    writer.ready().await;
    let mut wheel = io::wheel::StubWheel::new();
    let mut pedals = io::axes::StubPedals::new();

    loop {
        let wheel_x = wheel.sample_wheel();
        let (accel, brake) = pedals.sample();

        // Apply current force from PID engine to the wheel actuator (stubbed here).
        let force = pid.lock(|p| p.current_force());
        wheel.set_force(force).await;

        // [ReportId][Buttons(1 byte)][X i16][Y u16][Rz u16]
        let mut report = [0u8; 8];
        report[0] = hid::descriptor::RID_AXES_IN;
        report[1] = 0x00; // buttons bitmask (Button1..8), stubbed to 0
        report[2..4].copy_from_slice(&wheel_x.to_le_bytes());
        report[4..6].copy_from_slice(&accel.to_le_bytes());
        report[6..8].copy_from_slice(&brake.to_le_bytes());

        if let Err(e) = writer.write(&report).await {
            warn!("HID IN write failed: {:?}", e);
        }

        Timer::after_millis(5).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("ffwheel HID starting");

    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    let mut usb_config = Config::new(0xCAFE, 0x4001);
    usb_config.manufacturer = Some("ffwheel");
    usb_config.product = Some("HID Wheel (PID FFB)");
    usb_config.serial_number = None;
    usb_config.max_power = 250;
    usb_config.max_packet_size_0 = 64;

    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 512]);
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
    let msos_descriptor = MSOS_DESCRIPTOR.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 1024]);

    let mut builder = Builder::new(
        driver,
        usb_config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );

    static HID_STATE: StaticCell<embassy_usb::class::hid::State<'static>> = StaticCell::new();
    let hid_state = HID_STATE.init(embassy_usb::class::hid::State::new());

    static PID: StaticCell<PidMutex> = StaticCell::new();
    let pid = PID.init(PidMutex::new(Pid::new()));

    // The HID class will also handle control GET_REPORT/SET_REPORT via this request handler.
    struct ControlHandler {
        pid: &'static PidMutex,
    }
    impl RequestHandler for ControlHandler {
        fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
            match id {
                ReportId::Feature(rid) => self.pid.lock(|p| p.get_feature_report(rid, buf)),
                ReportId::In(rid) => self.pid.lock(|p| p.get_input_report(rid, buf)),
                _ => None,
            }
        }
        fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
            match id {
                ReportId::Out(rid) | ReportId::Feature(rid) => {
                    unsafe { self.pid.lock_mut(|p| p.handle_report_out(rid, data)) };
                    OutResponse::Accepted
                }
                _ => OutResponse::Rejected,
            }
        }
    }

    static CONTROL_HANDLER: StaticCell<ControlHandler> = StaticCell::new();
    let control_handler = CONTROL_HANDLER.init(ControlHandler { pid });

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: hid::descriptor::REPORT_DESCRIPTOR,
        request_handler: Some(control_handler),
        poll_ms: 4,
        max_packet_size: 64,
    };

    let hid = HidReaderWriter::<_, 64, 64>::new(&mut builder, hid_state, hid_config);
    let (reader, writer) = hid.split();

    let usb = builder.build();

    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(hid_out_task(reader, pid)).unwrap();
    spawner.spawn(hid_in_task(writer, pid)).unwrap();

    loop {
        Timer::after_secs(5).await;
        //let (gain, enabled, paused) = pid.lock(|p| (p.device_gain, p.actuators_enabled, p.paused));
        //debug!("alive gain={} enabled={} paused={}", gain, enabled, paused);
    }
}
