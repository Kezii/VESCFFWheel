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
        (0, 0)
    }
}
