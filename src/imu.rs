//! The camera's IMU through hidraw: one 64-byte report per sample, every sample kept.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{self, Read};
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::monotonic_ns;
use crate::sys::{self, READ, WRITE, ioc};

// Report ids and requests (zed-open-capture's sensorcapture_def.hpp).
const REP_ID_SENSOR_DATA: u8 = 0x01;
const REP_ID_REQUEST_SET: u8 = 0x21;
const REP_ID_SENSOR_STREAM_STATUS: u8 = 0x32;
const RQ_CMD_PING: u8 = 0xf2;

/// The packed sensor report's size (`usb::RawData`).
const REPORT_LEN: usize = 62;
const GRAVITY: f32 = 9.8189;
const ACC_SCALE: f32 = GRAVITY * 8.0 / 32768.0; // m/s² per count (±8 g)
const GYRO_SCALE: f32 = 1000.0 / 32768.0 * std::f32::consts::PI / 180.0; // rad/s per count
/// The MCU's clock ticks 39.0625 µs (zed-open-capture's TS_SCALE, in integers).
const fn mcu_ticks_to_ns(ticks: u64) -> u64 {
    ticks * 78_125 / 2
}
/// The IMU's rate.
const NOMINAL_INTERVAL_NS: u64 = 2_500_000;
/// Samples kept for the reader: 10 s at 400 Hz.
const QUEUE_LEN: usize = 4000;
const PING_EVERY: Duration = Duration::from_secs(1);

fn hidiocsfeature(len: usize) -> libc::c_ulong {
    ioc(READ | WRITE, b'H', 0x06, len)
}

/// One IMU sample, in the IMU's own axes.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    /// The camera MCU's clock, in nanoseconds since it started.
    pub mcu_time_ns: u64,
    /// `CLOCK_MONOTONIC` when the report was read.
    pub host_time_ns: u64,
    /// m/s²
    pub accel: [f32; 3],
    /// rad/s
    pub gyro: [f32; 3],
    /// °C
    pub temperature: f32,
    /// The sample the MCU marked as a camera frame's.
    pub frame_sync: bool,
    /// Camera frames the MCU has counted.
    pub frame_sync_count: u32,
}

/// Counts since [`Imu::open`] or the last [`Imu::reset_stats`].
#[derive(Clone, Debug, Default)]
pub struct ImuStats {
    pub reports: u64,
    pub samples: u64,
    /// Reports whose IMU part the MCU marked not valid.
    pub not_valid: u64,
    /// Reports too short or of another id.
    pub bad_reports: u64,
    /// Samples missing by the MCU's clock (gaps of 1.5 intervals or more at 400 Hz).
    pub missing: u64,
    /// Gaps over 5 ms between samples, by the MCU's clock.
    pub gaps_over_5ms: u64,
    pub longest_gap_ns: u64,
    /// The longest time between two reads on the host.
    pub longest_read_gap_ns: u64,
    /// Samples dropped from a full queue: the reader was late by over 10 s.
    pub queue_drops: u64,
    /// Frame marks seen, and the MCU's frame counter's first and last values.
    pub frame_syncs: u64,
    pub first_frame_count: Option<u32>,
    pub last_frame_count: u32,
}

struct Shared {
    queue: Mutex<VecDeque<ImuSample>>,
    stats: Mutex<ImuStats>,
    stop: AtomicBool,
    failed: Mutex<Option<String>>,
}

/// The IMU stream, read by a thread of its own from [`Imu::open`] until dropped.
pub struct Imu {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
    file: File,
}

impl Imu {
    /// Opens the camera's hidraw device (e.g. `/dev/hidraw0`) and turns its stream on. Do it
    /// after the [`Video`](crate::Video)'s first frame (see [`Video::open`](crate::Video::open)),
    /// or use [`Camera`](crate::Camera).
    pub fn open(path: &Path) -> io::Result<Imu> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)
            .map_err(|e| io::Error::new(e.kind(), format!("open {}: {e}", path.display())))?;
        set_feature(file.as_raw_fd(), &[REP_ID_SENSOR_STREAM_STATUS, 1])
            .map_err(|e| io::Error::new(e.kind(), format!("turning the IMU stream on: {e}")))?;

        let shared = Arc::new(Shared {
            queue: Mutex::new(VecDeque::with_capacity(QUEUE_LEN)),
            stats: Mutex::new(ImuStats::default()),
            stop: AtomicBool::new(false),
            failed: Mutex::new(None),
        });
        let reader = Reader {
            file: file.try_clone()?,
            shared: shared.clone(),
        };
        let thread = std::thread::Builder::new()
            .name("zed-imu".into())
            .spawn(move || reader.run())?;
        Ok(Imu {
            shared,
            thread: Some(thread),
            file,
        })
    }

    /// Every sample since the last call, oldest first. Err once the reading thread has stopped
    /// on an error.
    pub fn drain(&self) -> io::Result<Vec<ImuSample>> {
        self.drain_until(u64::MAX)
    }

    /// The samples read up to `host_time_ns` (`CLOCK_MONOTONIC`, as a frame's timestamp), oldest
    /// first; later ones stay queued.
    pub fn drain_until(&self, host_time_ns: u64) -> io::Result<Vec<ImuSample>> {
        let samples: Vec<ImuSample> = {
            let mut queue = self.shared.queue.lock().unwrap();
            let n = queue.partition_point(|s| s.host_time_ns <= host_time_ns);
            queue.drain(..n).collect()
        };
        if samples.is_empty()
            && let Some(e) = self.shared.failed.lock().unwrap().as_ref()
        {
            return Err(io::Error::other(e.clone()));
        }
        Ok(samples)
    }

    pub fn stats(&self) -> ImuStats {
        self.shared.stats.lock().unwrap().clone()
    }

    pub fn reset_stats(&self) {
        *self.shared.stats.lock().unwrap() = ImuStats::default();
    }
}

impl Drop for Imu {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        let _ = set_feature(self.file.as_raw_fd(), &[REP_ID_SENSOR_STREAM_STATUS, 0]);
    }
}

fn set_feature(fd: RawFd, report: &[u8]) -> io::Result<()> {
    let mut buf = report.to_vec();
    sys::ioctl(fd, hidiocsfeature(buf.len()), buf.as_mut_ptr())
}

struct Reader {
    file: File,
    shared: Arc<Shared>,
}

impl Reader {
    fn run(mut self) {
        if let Err(e) = self.read() {
            *self.shared.failed.lock().unwrap() = Some(format!("IMU reading stopped: {e}"));
        }
    }

    fn read(&mut self) -> io::Result<()> {
        let fd = self.file.as_raw_fd();
        let mut buf = [0u8; 64];
        let mut last_ping = Instant::now();
        let mut last_mcu: Option<u64> = None;
        let mut last_read: Option<u64> = None;
        while !self.shared.stop.load(Ordering::Relaxed) {
            if last_ping.elapsed() >= PING_EVERY {
                set_feature(fd, &[REP_ID_REQUEST_SET, RQ_CMD_PING])?;
                last_ping = Instant::now();
            }
            if !sys::wait_readable(fd, 100)? {
                continue;
            }
            let n = match self.file.read(&mut buf) {
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(e),
            };
            let host_time_ns = monotonic_ns();

            let mut s = self.shared.stats.lock().unwrap();
            s.reports += 1;
            if let Some(t) = last_read {
                s.longest_read_gap_ns = s.longest_read_gap_ns.max(host_time_ns - t);
            }
            last_read = Some(host_time_ns);
            let Some(report) = parse(&buf[..n], host_time_ns) else {
                s.bad_reports += 1;
                continue;
            };
            let Some(sample) = report else {
                s.not_valid += 1;
                continue;
            };
            s.samples += 1;
            if let Some(t) = last_mcu
                && sample.mcu_time_ns > t
            {
                let dt = sample.mcu_time_ns - t;
                s.longest_gap_ns = s.longest_gap_ns.max(dt);
                if dt > 5_000_000 {
                    s.gaps_over_5ms += 1;
                }
                let intervals = (dt + NOMINAL_INTERVAL_NS / 2) / NOMINAL_INTERVAL_NS;
                s.missing += intervals.saturating_sub(1);
            }
            last_mcu = Some(sample.mcu_time_ns);
            if sample.frame_sync {
                s.frame_syncs += 1;
            }
            s.first_frame_count.get_or_insert(sample.frame_sync_count);
            s.last_frame_count = sample.frame_sync_count;
            drop(s);

            let dropped = {
                let mut queue = self.shared.queue.lock().unwrap();
                queue.push_back(sample);
                if queue.len() > QUEUE_LEN {
                    queue.pop_front()
                } else {
                    None
                }
            };
            if dropped.is_some() {
                self.shared.stats.lock().unwrap().queue_drops += 1;
            }
        }
        Ok(())
    }
}

/// None: not a sensor report. Some(None): a report whose IMU part is not valid.
fn parse(report: &[u8], host_time_ns: u64) -> Option<Option<ImuSample>> {
    if report.len() < REPORT_LEN || report[0] != REP_ID_SENSOR_DATA {
        return None;
    }
    if report[1] == 1 {
        return Some(None);
    }
    let u32_at = |i: usize| u32::from_le_bytes(report[i..i + 4].try_into().unwrap());
    let i16_at = |i: usize| i16::from_le_bytes(report[i..i + 2].try_into().unwrap()) as f32;
    let ticks = u64::from_le_bytes(report[2..10].try_into().unwrap());
    Some(Some(ImuSample {
        mcu_time_ns: mcu_ticks_to_ns(ticks),
        host_time_ns,
        gyro: [
            i16_at(10) * GYRO_SCALE,
            i16_at(12) * GYRO_SCALE,
            i16_at(14) * GYRO_SCALE,
        ],
        accel: [
            i16_at(16) * ACC_SCALE,
            i16_at(18) * ACC_SCALE,
            i16_at(20) * ACC_SCALE,
        ],
        frame_sync: report[22] != 0,
        frame_sync_count: u32_at(24),
        temperature: i16_at(28) * 0.01,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_packed_report() {
        let mut r = [0u8; 64];
        r[0] = REP_ID_SENSOR_DATA;
        r[2..10].copy_from_slice(&1000u64.to_le_bytes());
        for (k, v) in [100i16, -200, 300, 4096, 0, -4096].into_iter().enumerate() {
            r[10 + 2 * k..12 + 2 * k].copy_from_slice(&v.to_le_bytes());
        }
        r[22] = 1;
        r[24..28].copy_from_slice(&77u32.to_le_bytes());
        r[28..30].copy_from_slice(&2512i16.to_le_bytes());
        let s = parse(&r, 5).unwrap().unwrap();
        assert_eq!(s.mcu_time_ns, 39_062_500);
        assert!(
            (s.gyro[0] - 100.0 * GYRO_SCALE).abs() < 1e-9
                && (s.gyro[1] + 200.0 * GYRO_SCALE).abs() < 1e-9
        );
        assert!(
            (s.accel[0] - GRAVITY).abs() < 1e-4
                && s.accel[1] == 0.0
                && (s.accel[2] + GRAVITY).abs() < 1e-4
        );
        assert!(s.frame_sync && s.frame_sync_count == 77 && (s.temperature - 25.12).abs() < 1e-4);
    }

    #[test]
    fn rejects_short_and_other_reports_and_flags_not_valid() {
        let mut r = [0u8; 64];
        r[0] = REP_ID_SENSOR_DATA;
        assert!(parse(&r[..REPORT_LEN - 1], 0).is_none());
        r[1] = 1;
        assert!(matches!(parse(&r, 0), Some(None)));
        r[0] = 0x32;
        assert!(parse(&r, 0).is_none());
    }

    #[test]
    fn mcu_ticks() {
        // 25,600 ticks a second
        assert_eq!(mcu_ticks_to_ns(25_600), 1_000_000_000);
    }
}
