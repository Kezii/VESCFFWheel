use defmt::{debug, info, warn};
use embedded_io_async::{Read, Write};

use crate::vesc::{DispPosMode, VescCommand, VescReply};

pub const CONFIG_DEFAULT_GAIN: f32 = 0.30;
pub const CONFIG_DEFAULT_CENTERPOINT: f32 = 145.0 / 360.0;
pub const CONFIG_DEFAULT_ANGLE_DEG: f32 = 540.0;

pub async fn prepare_vesc<Tx, Rx>(mut tx: Tx, rx: Rx) -> (VescWheelSampler<Rx>, VescWheelSetter<Tx>)
where
    Tx: Write,
    Rx: Read,
{
    let cmd = VescCommand::SetDisp(DispPosMode::DISP_POS_MODE_ENCODER).serialize();
    if tx.write_all(cmd.as_slice()).await.is_err() {
        warn!("VESC SetDisp write failed");
    } else {
        info!("VESC SetDisp(ENCODER) sent");
    }

    let sampler = VescWheelSampler::new(rx);
    let setter = VescWheelSetter::new(tx);
    (sampler, setter)
}

pub struct VescWheelSampler<Rx> {
    rx: Rx,

    // state
    last_x: i16,
    has_pos: bool,
    old_pos: f32,  // 0..1
    real_pos: f32, // unwrapped, turns (can exceed 0..1)
}

impl<Rx> VescWheelSampler<Rx>
where
    Rx: Read,
{
    pub fn new(rx: Rx) -> Self {
        Self {
            rx,
            last_x: 0,
            has_pos: false,
            old_pos: 0.0,
            real_pos: 0.0,
        }
    }

    async fn read_rotor_position_deg(&mut self) -> Option<f32> {
        match VescReply::eat_vesc_packet(&mut self.rx).await {
            Some(VescReply::MotorPosition(pos)) => Some(pos),
            Some(VescReply::FirmwareVersion(version)) => {
                warn!("VESC firmware version: {}", version);
                None
            }
            Some(VescReply::Unknown(payload)) => {
                warn!("VESC unknown packet: {:?}", payload);
                None
            }
            None => {
                warn!("VESC packet parse failed");
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
        let delta = pos - old;

        if delta.abs() > 0.5 {
            // Crossing the wrap boundary (0/1). The sign of delta tells us which way.
            if delta > 0.0 {
                // old ~ 0.0, pos ~ 1.0 (wrapped backwards)
                self.real_pos -= old + (1.0 - pos);
            } else {
                // old ~ 1.0, pos ~ 0.0 (wrapped forwards)
                self.real_pos += pos + (1.0 - old);
            }
        } else {
            self.real_pos += delta;
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
    pub async fn sample_wheel(&mut self) -> Option<i16> {
        let pos_deg = self.read_rotor_position_deg().await?;

        debug!("VESC rotor position: {}", pos_deg);

        // VESC returns degrees in [0..360) (typically). Convert to 0..1 turns.
        let pos_turns = pos_deg / 360.0;
        let real = self.unwrap_multi_turn(pos_turns);
        self.last_x = Self::turns_to_hid_x(real);

        Some(self.last_x)
    }
}

pub struct VescWheelSetter<Tx> {
    tx: Tx,
}

impl<Tx> VescWheelSetter<Tx>
where
    Tx: Write,
{
    pub fn new(tx: Tx) -> Self {
        Self { tx }
    }

    /// Apply current force/torque command (maps to VESC duty).
    pub async fn set_force(&mut self, force: i16) {
        let force_norm = (force as f32) / 32767.0;
        let duty = (CONFIG_DEFAULT_GAIN * force_norm).clamp(-1.0, 1.0);
        let cmd = VescCommand::SetDuty(duty);
        let frame = cmd.serialize();

        if self.tx.write_all(frame.as_slice()).await.is_err() {
            warn!("VESC SetDuty write failed");
        }

        debug!("VESC SetDuty duty={}", duty);
    }
}
