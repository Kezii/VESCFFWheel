#![no_std]
#![no_main]

mod descriptor;
mod ffb;
mod io;
mod perfcounter;
mod vesc;

use crate::io::wheel::{VescWheelSampler, VescWheelSetter};
use crate::perfcounter::PerfCounter;
use defmt::{debug, info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART1, USB};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig,
};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embassy_usb::class::hid::{HidReader, HidReaderWriter, HidWriter, ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type Pid = ffb::PidEngine<4>;
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
    perf_counter: &'static PerfCounter,
) -> ! {
    struct OutHandler {
        pid: &'static PidMutex,
        perf_counter: &'static PerfCounter,
    }

    impl RequestHandler for OutHandler {
        fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
            match id {
                ReportId::Out(rid) | ReportId::Feature(rid) => {
                    debug!("HID SET_REPORT rid=0x{:02x} len={}", rid, data.len());
                    unsafe {
                        self.pid
                            .lock_mut(|p| p.handle_report_out(rid, data, self.perf_counter));
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
    let mut handler = OutHandler { pid, perf_counter };
    // Use report IDs: first byte of the OUT report is Report ID.
    reader.run(true, &mut handler).await
}

#[embassy_executor::task]
async fn hid_in_task(
    mut writer: HidWriter<'static, Driver<'static, USB>, 64>,
    perf_counter: &'static PerfCounter,
) -> ! {
    writer.ready().await;
    let mut pedals = io::axes::StubPedals::new();

    loop {
        let wheel_x = WHEEL_POS_SIGNAL.wait().await;

        perf_counter.increment_hid_write();

        let (accel, brake) = pedals.sample();

        // [ReportId][Buttons(1 byte)][X i16][Y u16][Rz u16]
        let mut report = [0u8; 8];
        report[0] = descriptor::RID_AXES_IN;
        report[1] = 0x00; // buttons bitmask (Button1..8), stubbed to 0
        report[2..4].copy_from_slice(&wheel_x.to_le_bytes());
        report[4..6].copy_from_slice(&accel.to_le_bytes());
        report[6..8].copy_from_slice(&brake.to_le_bytes());

        if let Err(e) = writer.write(&report).await {
            warn!("HID IN write failed: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn wheel_sample_task(
    mut sampler: VescWheelSampler<BufferedUartRx>,
    perf_counter: &'static PerfCounter,
) -> ! {
    loop {
        let wheel_x = sampler.sample_wheel().await;
        if let Some(wheel_x) = wheel_x {
            WHEEL_POS_SIGNAL.signal(wheel_x);
            perf_counter.increment_vesc_read();
        }
    }
}

#[embassy_executor::task]
async fn wheel_apply_force_task(
    mut wheel: VescWheelSetter<BufferedUartTx>,
    perf_counter: &'static PerfCounter,
) -> ! {
    loop {
        let force = WHEEL_FORCE_SIGNAL.wait().await;
        wheel.set_force(force).await;
        perf_counter.increment_vesc_write();
    }
}

pub static WHEEL_FORCE_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();
static WHEEL_POS_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("ffwheel HID starting");

    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    // UART0 for VESC
    static UART_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let uart_tx_buf = &mut UART_TX_BUF.init([0; 256])[..];
    static UART_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let uart_rx_buf = &mut UART_RX_BUF.init([0; 256])[..];
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart = BufferedUart::new(
        p.UART1,
        p.PIN_4,
        p.PIN_5,
        Irqs,
        uart_tx_buf,
        uart_rx_buf,
        uart_cfg,
    );
    let (uart_tx, uart_rx) = uart.split();

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
        perf_counter: &'static PerfCounter,
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
                    unsafe {
                        self.pid
                            .lock_mut(|p| p.handle_report_out(rid, data, self.perf_counter))
                    };
                    OutResponse::Accepted
                }
                _ => OutResponse::Rejected,
            }
        }
    }

    static PERF_COUNTER: StaticCell<PerfCounter> = StaticCell::new();
    let perf_counter = PERF_COUNTER.init(PerfCounter::default());
    let perf_counter = &*perf_counter;

    static CONTROL_HANDLER: StaticCell<ControlHandler> = StaticCell::new();
    let control_handler = CONTROL_HANDLER.init(ControlHandler { pid, perf_counter });

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: descriptor::REPORT_DESCRIPTOR,
        request_handler: Some(control_handler),
        poll_ms: 4,
        max_packet_size: 64,
    };

    let hid = HidReaderWriter::<_, 64, 64>::new(&mut builder, hid_state, hid_config);
    let (reader, writer) = hid.split();

    let usb = builder.build();

    spawner.spawn(usb_task(usb)).unwrap();
    spawner
        .spawn(hid_out_task(reader, pid, perf_counter))
        .unwrap();

    let (sampler, setter) = io::wheel::prepare_vesc(uart_tx, uart_rx).await;

    spawner.spawn(hid_in_task(writer, perf_counter)).unwrap();
    spawner
        .spawn(wheel_apply_force_task(setter, perf_counter))
        .unwrap();

    spawner
        .spawn(wheel_sample_task(sampler, perf_counter))
        .unwrap();

    loop {
        Timer::after_secs(1).await;
        perf_counter.log_performance();
    }
}
