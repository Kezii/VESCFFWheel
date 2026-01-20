use defmt::{debug, info, warn};

use crate::{
    PerfCounter, WHEEL_FORCE_SIGNAL,
    descriptor::{
        RID_PID_BLOCK_FREE_OUT, RID_PID_BLOCK_LOAD_FEATURE, RID_PID_CREATE_NEW_EFFECT_FEATURE,
        RID_PID_DEVICE_CONTROL_OUT, RID_PID_DEVICE_GAIN_OUT, RID_PID_EFFECT_OPERATION_OUT,
        RID_PID_POOL_FEATURE, RID_PID_SET_CONDITION_OUT, RID_PID_SET_CONSTANT_FORCE_OUT,
        RID_PID_SET_CUSTOM_FORCE_DATA_OUT, RID_PID_SET_EFFECT_OUT, RID_PID_SET_ENVELOPE_OUT,
        RID_PID_SET_PERIODIC_OUT, RID_PID_STATE_IN,
    },
};

#[derive(Clone, Copy)]
struct ConstantEffect {
    magnitude: i16,
    playing: bool,
}

impl ConstantEffect {
    const fn new() -> Self {
        Self {
            magnitude: 0,
            playing: false,
        }
    }
}

pub struct PidEngine<const SLOTS: usize> {
    pub device_gain: u8,
    pub actuators_enabled: bool,
    pub paused: bool,

    effects: [Option<ConstantEffect>; SLOTS],
    last_allocated_block: u8,

    // Cached feature report data for Block Load
    last_block_load_status_sel: u8, // 1=success, 2=full, 3=error
    last_ram_pool_available: u16,
}

impl<const SLOTS: usize> PidEngine<SLOTS> {
    pub const fn new() -> Self {
        Self {
            device_gain: 0xFF,
            actuators_enabled: false,
            paused: false,
            effects: [None; SLOTS],
            last_allocated_block: 0,
            last_block_load_status_sel: 1,
            last_ram_pool_available: 0x4000,
        }
    }

    fn block_to_slot(block: u8) -> Option<usize> {
        if block == 0 {
            return None;
        }
        let idx = (block - 1) as usize;
        (idx < SLOTS).then_some(idx)
    }

    fn alloc_slot(&mut self) -> Result<u8, ()> {
        for (i, e) in self.effects.iter_mut().enumerate() {
            if e.is_none() {
                *e = Some(ConstantEffect::new());
                return Ok((i as u8) + 1);
            }
        }
        Err(())
    }

    fn free_slot(&mut self, block: u8) {
        if let Some(i) = Self::block_to_slot(block) {
            self.effects[i] = None;
        }
    }

    fn any_playing(&self) -> bool {
        self.effects
            .iter()
            .any(|e| matches!(e, Some(x) if x.playing))
    }

    fn scaled_force(&self, magnitude: i16) -> i16 {
        let scaled = (magnitude as i32) * (self.device_gain as i32) / 255;
        scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }

    /// Current force command to apply to the actuator.
    ///
    /// For now this is the sum of all playing Constant Force magnitudes,
    /// scaled by device gain, and gated by device control flags.
    #[allow(dead_code)]
    pub fn current_force(&self) -> i16 {
        if !self.actuators_enabled || self.paused {
            return 0;
        }

        let mut sum: i32 = 0;
        for e in self.effects.iter().flatten() {
            if e.playing {
                sum += e.magnitude as i32;
            }
        }

        let sum_i16 = sum.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        self.scaled_force(sum_i16)
    }

    fn parse_with_optional_id_prefix(report_id: u8, data: &[u8]) -> &[u8] {
        match data.split_first() {
            Some((&first, rest)) if first == report_id => rest,
            _ => data,
        }
    }

    pub fn handle_report_out(
        &mut self,
        report_id: u8,
        data: &[u8],
        perf_counter: &'static PerfCounter,
    ) {
        match report_id {
            RID_PID_CREATE_NEW_EFFECT_FEATURE => {
                // Data is Effect Type selection (we only support Constant Force)
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let _effect_type_sel = data.first().copied().unwrap_or(1);

                match self.alloc_slot() {
                    Ok(block) => {
                        self.last_allocated_block = block;
                        self.last_block_load_status_sel = 1; // success
                        self.last_ram_pool_available = 0x4000;
                        info!("PID CreateNewEffect -> allocated block {}", block);
                    }
                    Err(()) => {
                        self.last_allocated_block = 0;
                        self.last_block_load_status_sel = 2; // full
                        self.last_ram_pool_available = 0;
                        warn!("PID CreateNewEffect -> pool full");
                    }
                }
            }

            RID_PID_BLOCK_FREE_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                self.free_slot(block);
                info!("PID BlockFree block={}", block);
            }

            RID_PID_SET_EFFECT_OUT => {
                // We only need the Effect Block Index for routing; other fields ignored for now.
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                debug!("PID SetEffect block={}", block);
            }

            RID_PID_SET_ENVELOPE_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                debug!(
                    "PID SetEnvelope (ignored) block={} len={}",
                    block,
                    data.len()
                );
            }

            RID_PID_SET_CONDITION_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                debug!(
                    "PID SetCondition (ignored) block={} len={}",
                    block,
                    data.len()
                );
            }

            RID_PID_SET_PERIODIC_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                debug!(
                    "PID SetPeriodic (ignored) block={} len={}",
                    block,
                    data.len()
                );
            }

            RID_PID_SET_CONSTANT_FORCE_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                if data.len() < 3 {
                    warn!("PID SetConstantForce short report: {} bytes", data.len());
                    return;
                }

                let block = data[0];
                let mag = i16::from_le_bytes([data[1], data[2]]);

                debug!(
                    "FFB constant magnitude={}, gain={}, block={}",
                    mag, self.device_gain, block
                );

                if let Some(i) = Self::block_to_slot(block)
                    && let Some(e) = self.effects[i].as_mut()
                {
                    e.magnitude = mag;
                    WHEEL_FORCE_SIGNAL.signal(self.current_force());
                    perf_counter.increment_hid_read();
                }
            }

            RID_PID_EFFECT_OPERATION_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                if data.len() < 3 {
                    warn!("PID EffectOperation short report: {} bytes", data.len());
                    return;
                }

                let block = data[0];
                // In hid-pidff this is an ARRAY field (0x78): 1=Start, 2=StartSolo, 3=Stop.
                let op_sel = data[1];
                let _loop_count = data[2];
                let start = op_sel == 1;
                let start_solo = op_sel == 2;
                let stop = op_sel == 3;

                if stop {
                    if let Some(i) = Self::block_to_slot(block) {
                        if let Some(e) = self.effects[i].as_mut() {
                            e.playing = false;
                        }
                    }
                    info!("PID EffectOperation STOP block={}", block);
                } else if start || start_solo {
                    if start_solo {
                        for e in self.effects.iter_mut().flatten() {
                            e.playing = false;
                        }
                    }
                    if let Some(i) = Self::block_to_slot(block) {
                        if let Some(e) = self.effects[i].as_mut() {
                            e.playing = true;
                            info!(
                                "PID EffectOperation START block={} solo={}",
                                block, start_solo
                            );
                        }
                    }
                }
            }

            RID_PID_DEVICE_GAIN_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let gain = data.first().copied().unwrap_or(0xFF);
                self.device_gain = gain;
                info!("PID DeviceGain gain={}", gain);
            }

            RID_PID_DEVICE_CONTROL_OUT => {
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                // In hid-pidff this is an ARRAY field (0x96): 1..6 selects the command.
                let cmd = data.first().copied().unwrap_or(0);
                let enable = cmd == 1;
                let disable = cmd == 2;
                let stop_all = cmd == 3;
                let reset = cmd == 4;
                let pause = cmd == 5;
                let cont = cmd == 6;

                if reset {
                    *self = Self::new();
                    info!("PID DeviceControl RESET");
                    return;
                }
                if enable {
                    self.actuators_enabled = true;
                    info!("PID DeviceControl ENABLE_ACTUATORS");
                }
                if disable {
                    self.actuators_enabled = false;
                    info!("PID DeviceControl DISABLE_ACTUATORS");
                }
                if stop_all {
                    for e in self.effects.iter_mut().flatten() {
                        e.playing = false;
                    }
                    info!("PID DeviceControl STOP_ALL_EFFECTS");
                }
                if pause {
                    self.paused = true;
                    info!("PID DeviceControl PAUSE");
                }
                if cont {
                    self.paused = false;
                    info!("PID DeviceControl CONTINUE");
                }
            }

            RID_PID_SET_CUSTOM_FORCE_DATA_OUT => {
                // Optional report; ignore payload (custom forces not implemented).
                let data = Self::parse_with_optional_id_prefix(report_id, data);
                let block = data.first().copied().unwrap_or(0);
                debug!(
                    "PID SetCustomForceData (ignored) block={}, len={}",
                    block,
                    data.len()
                );
            }

            _ => {
                debug!(
                    "Unhandled OUT report id=0x{:02x}, len={}",
                    report_id,
                    data.len()
                );
            }
        }
    }

    pub fn get_feature_report(&self, report_id: u8, buf: &mut [u8]) -> Option<usize> {
        match report_id {
            RID_PID_POOL_FEATURE => {
                // [id][ram_pool_size: u16 le][simultaneous_max: u8][managed_pool_bits: u8]
                if buf.len() < 5 {
                    return None;
                }
                buf[0] = report_id;
                let ram_pool_size: u16 = 0x4000;
                let sim_max: u8 = SLOTS as u8;
                buf[1..3].copy_from_slice(&ram_pool_size.to_le_bytes());
                buf[3] = sim_max;
                buf[4] = 0x01; // device managed pool = 1
                Some(5)
            }
            RID_PID_BLOCK_LOAD_FEATURE => {
                // [id][block_index: u8][status_sel: u8][ram_pool_available: u16 le]
                if buf.len() < 5 {
                    return None;
                }
                buf[0] = report_id;
                buf[1] = self.last_allocated_block;
                buf[2] = self.last_block_load_status_sel;
                buf[3..5].copy_from_slice(&self.last_ram_pool_available.to_le_bytes());
                Some(5)
            }
            _ => None,
        }
    }

    pub fn get_input_report(&self, report_id: u8, buf: &mut [u8]) -> Option<usize> {
        match report_id {
            RID_PID_STATE_IN => {
                if buf.len() < 2 {
                    return None;
                }
                buf[0] = report_id;
                let mut bits = 0u8;
                // Bit order must match the HID descriptor's PID State fields.
                // We model the common OpenFFBoard order:
                // 0: Device Paused, 1: Actuators Enabled, 2: Safety Switch, 3: Actuator Power, 4: Effect Playing
                if self.paused {
                    bits |= 1 << 0;
                }
                if self.actuators_enabled {
                    bits |= 1 << 1;
                }
                // We don't implement a real safety switch / power sense yet.
                // Advertise them as "OK" so the host can enable FFB.
                bits |= 1 << 2; // safety switch closed
                bits |= 1 << 3; // actuator power on
                if self.any_playing() {
                    bits |= 1 << 4;
                }
                buf[1] = bits;
                Some(2)
            }
            _ => None,
        }
    }
}
