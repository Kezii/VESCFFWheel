use embassy_time::Instant;
use defmt::info;

/// Wheel (X axis) I/O.
///
/// This intentionally combines:
/// - **Axis sampling** (wheel position)
/// - **Actuation** (force/torque command)
///
/// so it can later be backed by a VESC over UART (single physical subsystem).
pub struct StubWheel {
    last_force: i16,
}

impl StubWheel {
    pub const fn new() -> Self {
        Self { last_force: 0 }
    }

    /// Sample wheel position as HID X (signed 16-bit).
    pub fn sample_wheel(&mut self) -> i16 {
        let ms = Instant::now().as_millis() as f32;
        info!("wheel sampled");

        // Steering: 0.5 Hz
        let wheel = (libm::sinf(ms * 0.001 * core::f32::consts::TAU * 0.5) * 32767.0) as i32;
        wheel.clamp(-32767, 32767) as i16
    }

    /// Apply current force/torque command.
    ///
    /// Async so a future VESC UART backend can `await` writes.
    pub async fn set_force(&mut self, force: i16) {
        self.last_force = force;

        let force = force as f32;
        info!("force set to {}", force / 32767.0);
    }

    pub fn last_force(&self) -> i16 {
        self.last_force
    }
}
