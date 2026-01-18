use defmt::{debug, info, warn};
use embassy_futures::select::{Either, select};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};

use crate::vesc::{DispPosMode, VescCommand, VescReply};

pub const CONFIG_DEFAULT_GAIN: f32 = 0.30;
pub const CONFIG_DEFAULT_CENTERPOINT: f32 = 145.0 / 360.0;
pub const CONFIG_DEFAULT_ANGLE_DEG: f32 = 540.0;

/// VESC-backed wheel implementation over UART.
///
/// - Sends `SetDisp(ENCODER)` once on init; after that the VESC streams rotor position packets.
/// - Reads `COMM_ROTOR_POSITION` packets and unwraps the angle to support multiple turns.
/// - Converts unwrapped position to HID X (`-32767..32767`) using `centerpoint` + `angle_deg`.
/// - Converts force command to `SetDuty()` proportionally using `gain`.
pub struct VescWheel<Rx, Tx> {
    rx: Rx,
    tx: Tx,

    // state
    last_force: i16,
    last_x: i16,
    disable_ffb: bool,
    has_pos: bool,
    old_pos: f32,  // 0..1
    real_pos: f32, // unwrapped, turns (can exceed 0..1)
}

impl<Rx, Tx> VescWheel<Rx, Tx>
where
    Rx: Read,
    Tx: Write,
{
    pub async fn new(mut tx: Tx, rx: Rx) -> Self {
        // Enable continuous encoder-based rotor position streaming.
        let cmd = VescCommand::SetDisp(DispPosMode::DISP_POS_MODE_ENCODER);
        let frame = cmd.serialize();
        if tx.write_all(frame.as_slice()).await.is_err() {
            warn!("VESC SetDisp write failed");
        } else {
            info!("VESC SetDisp(ENCODER) sent");
        }

        Self {
            rx,
            tx,
            last_force: 0,
            last_x: 0,
            disable_ffb: false,
            has_pos: false,
            old_pos: 0.0,
            real_pos: 0.0,
        }
    }

    /// Try reading a single 10-byte rotor-position packet without stalling the HID loop.
    async fn try_read_rotor_position_deg(&mut self) -> Option<f32> {
        let mut buf = [0u8; 10];

        // If the VESC isn't streaming yet, don't block forever: we just keep the last value.
        let read_fut = self.rx.read_exact(&mut buf);
        let timeout_fut = Timer::after_millis(1);

        match select(read_fut, timeout_fut).await {
            Either::First(r) => {
                r.ok()?;
            }
            Either::Second(_) => return None,
        }

        match VescReply::parse_from_buffer(&buf) {
            Some(VescReply::MotorPosition(pos_deg)) => Some(pos_deg),
            None => {
                debug!("VESC packet parse failed: {:?}", buf);
                None
            }
        }
    }

    fn unwrap_multi_turn(&mut self, pos_turns_0_1: f32) -> f32 {
        // Port of the old project logic (tracks wraparound at 0/1 boundary).
        if !self.has_pos {
            self.has_pos = true;
            self.old_pos = pos_turns_0_1;
            self.real_pos = pos_turns_0_1;
            return self.real_pos;
        }

        let pos = pos_turns_0_1;
        let old = self.old_pos;

        if (pos - old).abs() > 0.5 {
            if pos > 0.5 && old < 0.5 {
                // crossed wrap boundary
                self.real_pos -= old + (1.0 - pos);
            }
            if pos < 0.5 && old > 0.5 {
                self.real_pos += pos + (1.0 - old);
            }
        } else {
            self.real_pos += pos - old;
        }

        // If we are in the central range, re-lock to the absolute position to avoid drift.
        if self.real_pos > 0.1 && self.real_pos < 0.9 {
            if (self.real_pos - pos).abs() > 0.1 {
                // In the old project this was a panic; here we just log once in a while.
                warn!(
                    "VESC unwrap drift: abs(real-pos)={} (real={}, pos={})",
                    (self.real_pos - pos).abs(),
                    self.real_pos,
                    pos
                );
            }
            self.real_pos = pos;
        }

        self.old_pos = pos;
        self.real_pos
    }

    fn turns_to_hid_x(real_pos_turns: f32) -> i16 {
        let angle_turns = CONFIG_DEFAULT_ANGLE_DEG / 360.0;
        if angle_turns <= 0.0 {
            return 0;
        }

        // Map unwrapped position to wheel range around centerpoint.
        let valuef = (real_pos_turns - CONFIG_DEFAULT_CENTERPOINT) / angle_turns;
        let x = (valuef * 32767.0) as i32;
        x.clamp(-32767, 32767) as i16
    }

    /// Sample wheel position as HID X (signed 16-bit).
    pub async fn sample_wheel(&mut self) -> i16 {
        // Drain a few packets if available, keeping the newest.
        for _ in 0..4 {
            let Some(pos_deg) = self.try_read_rotor_position_deg().await else {
                break;
            };

            // VESC returns degrees in [0..360) (typically). Convert to 0..1 turns.
            let pos_turns = pos_deg / 360.0;
            let real = self.unwrap_multi_turn(pos_turns);
            self.last_x = Self::turns_to_hid_x(real);
        }

        self.last_x
    }

    /// Apply current force/torque command (maps to VESC duty).
    pub async fn set_force(&mut self, force: i16) {
        self.last_force = force;

        if self.disable_ffb {
            return;
        }

        let force_norm = (force as f32) / 32767.0;
        let duty = (CONFIG_DEFAULT_GAIN * force_norm).clamp(-1.0, 1.0);
        let cmd = VescCommand::SetDuty(duty);
        let frame = cmd.serialize();

        if self.tx.write_all(frame.as_slice()).await.is_err() {
            warn!("VESC SetDuty write failed");
        }
    }
}
