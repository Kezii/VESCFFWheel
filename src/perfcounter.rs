use core::sync::atomic::Ordering;

use defmt::info;
use portable_atomic::AtomicU16;

#[derive(Default)]
pub struct PerfCounter {
    n_hid_writes: AtomicU16,
    n_hid_reads: AtomicU16,
    n_vesc_reads: AtomicU16,
    n_vesc_writes: AtomicU16,
}

impl PerfCounter {
    pub fn increment_hid_write(&self) {
        self.n_hid_writes.fetch_add(1, Ordering::Relaxed);
    }

    pub fn increment_hid_read(&self) {
        self.n_hid_reads.fetch_add(1, Ordering::Relaxed);
    }

    pub fn increment_vesc_read(&self) {
        self.n_vesc_reads.fetch_add(1, Ordering::Relaxed);
    }

    pub fn increment_vesc_write(&self) {
        self.n_vesc_writes.fetch_add(1, Ordering::Relaxed);
    }

    pub fn log_performance(&self) {
        let vesc_reads = self.n_vesc_reads.swap(0, Ordering::Relaxed);
        let vesc_writes = self.n_vesc_writes.swap(0, Ordering::Relaxed);
        let hid_reads = self.n_hid_reads.swap(0, Ordering::Relaxed);
        let hid_writes = self.n_hid_writes.swap(0, Ordering::Relaxed);
        info!(
            "PERF vesc_reads={:03} -> hid_writes={:03} ;  hid_reads={:03} -> vesc_writes={:03}",
            vesc_reads, hid_writes, hid_reads, vesc_writes
        );
    }
}
