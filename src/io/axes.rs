use embassy_time::Instant;

/// Stub pedal axes source (Y and Rz).
///
/// - Accelerator/brake are slow phase-shifted sines mapped to 0..32767.
pub struct StubPedals;

impl StubPedals {
    pub const fn new() -> Self {
        Self
    }

    pub fn sample(&mut self) -> (u16, u16) {
        let ms = Instant::now().as_millis() as f32;

        // Pedals: ~0.1 Hz, 0..32767
        let accel_f = (libm::sinf(ms * 0.001 * core::f32::consts::TAU * 0.1) * 0.5 + 0.5) * 32767.0;
        let brake_f =
            (libm::sinf(ms * 0.001 * core::f32::consts::TAU * 0.1 + 1.0) * 0.5 + 0.5) * 32767.0;

        let accel = (accel_f as i32).clamp(0, 32767) as u16;
        let brake = (brake_f as i32).clamp(0, 32767) as u16;

        (accel, brake)
    }
}
