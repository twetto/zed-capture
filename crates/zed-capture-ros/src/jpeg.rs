//! An image stream's JPEGs from the Jetson's hardware encoder (zed-capture's `nvjpg`), published
//! as sensor_msgs/CompressedImage on `<image topic>/nvjpg`, as mr-neuro-nav's nvjpg_encoder.py
//! does (its format, "bgr8; jpeg compressed bgr8": three channels, the chroma flat; reliable,
//! keep-last 10). Each stream encodes in a thread of its own, after its image went out
//! uncompressed: one image waits while another is encoded; one more is skipped and counted.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
#[cfg(feature = "nvjpg")]
use std::time::Instant;

use r2r::std_msgs::msg::Header;
#[cfg(feature = "nvjpg")]
use {
    r2r::sensor_msgs::msg::CompressedImage,
    zed_capture::nvjpg::{FORMAT, Nvjpg},
};

/// All streams' counts since they were last taken.
#[derive(Default)]
pub struct JpegStats {
    jpegs: AtomicU64,
    bytes: AtomicU64,
    encode_ns: AtomicU64,
    encode_max_ns: AtomicU64,
    skipped: AtomicU64,
    errors: AtomicU64,
}

/// [`JpegStats`] taken: over a log period, or added up.
#[derive(Default, Clone, Copy)]
pub struct JpegCounts {
    pub jpegs: u64,
    pub bytes: u64,
    pub encode_ns: u64,
    pub encode_max_ns: u64,
    pub skipped: u64,
    pub errors: u64,
}

impl JpegStats {
    /// The counts since the last take, and zero again.
    pub fn take(&self) -> JpegCounts {
        let t = |a: &AtomicU64| a.swap(0, Ordering::Relaxed);
        JpegCounts {
            jpegs: t(&self.jpegs),
            bytes: t(&self.bytes),
            encode_ns: t(&self.encode_ns),
            encode_max_ns: t(&self.encode_max_ns),
            skipped: t(&self.skipped),
            errors: t(&self.errors),
        }
    }
}

impl JpegCounts {
    pub fn add(&mut self, o: &JpegCounts) {
        self.jpegs += o.jpegs;
        self.bytes += o.bytes;
        self.encode_ns += o.encode_ns;
        self.encode_max_ns = self.encode_max_ns.max(o.encode_max_ns);
        self.skipped += o.skipped;
        self.errors += o.errors;
    }

    /// "N JPEGs (encoded in X ms, at most Y; Z KB each), S skipped, E errors"
    pub fn summary(&self) -> String {
        let n = self.jpegs.max(1) as f64;
        format!(
            "{} JPEGs (encoded in {:.2} ms, at most {:.2}; {:.0} KB each), {} skipped, {} errors",
            self.jpegs,
            self.encode_ns as f64 / n / 1e6,
            self.encode_max_ns as f64 / 1e6,
            self.bytes as f64 / n / 1e3,
            self.skipped,
            self.errors
        )
    }
}

/// One image stream's encoder thread.
pub struct JpegStream {
    tx: Option<SyncSender<(Header, Vec<u8>)>>,
    thread: Option<JoinHandle<()>>,
    stats: Arc<JpegStats>,
}

#[cfg(feature = "nvjpg")]
impl JpegStream {
    /// Encodes with `encoder` and publishes with `publisher`, counting into `stats`.
    pub fn start(
        encoder: Nvjpg,
        publisher: r2r::Publisher<CompressedImage>,
        stats: Arc<JpegStats>,
    ) -> JpegStream {
        let (tx, rx) = sync_channel::<(Header, Vec<u8>)>(1);
        let s = stats.clone();
        let thread = std::thread::spawn(move || {
            let mut reported = 0;
            for (header, image) in rx {
                let t0 = Instant::now();
                match encoder.encode(image) {
                    Ok(data) => {
                        let ns = t0.elapsed().as_nanos() as u64;
                        s.encode_ns.fetch_add(ns, Ordering::Relaxed);
                        s.encode_max_ns.fetch_max(ns, Ordering::Relaxed);
                        s.bytes.fetch_add(data.len() as u64, Ordering::Relaxed);
                        let msg = CompressedImage {
                            header,
                            format: FORMAT.into(),
                            data,
                        };
                        let counter = match publisher.publish(&msg) {
                            Ok(()) => &s.jpegs,
                            Err(_) => &s.errors,
                        };
                        counter.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(e) => {
                        s.errors.fetch_add(1, Ordering::Relaxed);
                        if reported < 3 {
                            reported += 1;
                            eprintln!("zed_ros: NVJPG: {e}");
                        }
                    }
                }
            }
        });
        JpegStream {
            tx: Some(tx),
            thread: Some(thread),
            stats,
        }
    }
}

impl JpegStream {
    /// An image to encode, unless one already waits: then it is skipped.
    pub fn send(&self, header: Header, image: Vec<u8>) {
        let Some(tx) = &self.tx else { return };
        let counter = match tx.try_send((header, image)) {
            Ok(()) => return,
            Err(TrySendError::Full(_)) => &self.stats.skipped,
            Err(TrySendError::Disconnected(_)) => &self.stats.errors,
        };
        counter.fetch_add(1, Ordering::Relaxed);
    }
}

impl Drop for JpegStream {
    /// Encodes what waits, then ends the thread.
    fn drop(&mut self) {
        drop(self.tx.take());
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
