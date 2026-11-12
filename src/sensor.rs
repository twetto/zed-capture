//! The image sensors' registers, through the camera's extension unit: their timing and the
//! exposure the camera's auto exposure gave them, read while the video streams. The registers are
//! OmniVision's (the ZED 2i's sensors are OV4689s: chip id 0x4688).
//!
//! A sensor that does not answer reads 0xff, or 0x00, in every register, and the camera's
//! answer says nothing else: when the video is not streaming, and, on LabOrin's ZED 2i
//! (2026-11-12), the left one while it streams, for spans of 0.6 s (0xff) to 10 s (0x00) and
//! single reads, a quarter of the time or more (always 0xff with the IMU off). The right one
//! always answered. So each value is read twice and checked: the chip id first, for the reads
//! made once; within its range, for the exposure.
//!
//! One process at a time: a read is a request (SET_CUR) and its answer (GET_CUR), and another
//! process's request between them takes the answer (2026-11-12: a script reading 0x3002 beside
//! the exposure's thread got the exposure's 0x30, and wrote it back with bit 7 as 0xb0).
//! [`Sensors`] keeps a process's requests in turn.

use std::collections::VecDeque;
use std::fs::File;
use std::io;
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::{monotonic_ns, xu};

const REG_CHIP_ID: u16 = 0x300a;
/// Exposure in 1/16 lines, over three registers: bits 19:16 in the first's low nibble, then
/// 15:8, 7:0. (In whole lines, the 11952 read on LabOrin would be 14 frames long, at 30 Hz.)
/// Under 4096 lines a frame, as every ZED mode, the first is 0 and not read.
const REG_EXPOSURE: u16 = 0x3500;
const REG_X_OUTPUT_SIZE: u16 = 0x3808;
const REG_Y_OUTPUT_SIZE: u16 = 0x380a;
/// Pixel clocks per line.
const REG_HTS: u16 = 0x380c;
/// Lines per frame.
const REG_VTS: u16 = 0x380e;

pub const EYES: [&str; 2] = ["left", "right"];

/// What a sensor that does not answer reads.
const NO_ANSWER: [u8; 2] = [0x00, 0xff];
/// How often a sensor that has not answered is asked again.
const RETRY: Duration = Duration::from_millis(20);

struct Device {
    file: File,
    /// The extension unit's request length, once asked.
    len: Option<usize>,
}

impl Device {
    fn len(&mut self) -> io::Result<usize> {
        if let Some(len) = self.len {
            return Ok(len);
        }
        let len = xu::request_len(self.file.as_raw_fd())?;
        self.len = Some(len);
        Ok(len)
    }

    fn read(&mut self, eye: usize, register: u16) -> io::Result<u8> {
        let len = self.len()?;
        xu::read_sensor_register(self.file.as_raw_fd(), len, eye, register)
    }

    /// Whether the sensor answers: its chip id has neither byte of a sensor that does not.
    fn answering(&mut self, eye: usize) -> io::Result<bool> {
        let id = [self.read(eye, REG_CHIP_ID)?, self.read(eye, REG_CHIP_ID + 1)?];
        Ok(!id.iter().any(|b| NO_ANSWER.contains(b)))
    }

    /// `f`'s value once the sensor answers before and after it and `f` gives the same twice,
    /// asking again for up to `wait`; None if it never did.
    fn checked<T: PartialEq>(
        &mut self,
        eye: usize,
        wait: Duration,
        mut f: impl FnMut(&mut Device) -> io::Result<T>,
    ) -> io::Result<Option<T>> {
        let deadline = Instant::now() + wait;
        loop {
            if self.answering(eye)? {
                let v = f(self)?;
                if f(self)? == v && self.answering(eye)? {
                    return Ok(Some(v));
                }
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            std::thread::sleep(RETRY);
        }
    }
}

/// Both image sensors' registers. Clones share the device; one request at a time.
#[derive(Clone)]
pub struct Sensors {
    device: Arc<Mutex<Device>>,
}

/// A sensor's frame timing, from its registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTiming {
    pub chip_id: u16,
    /// Pixel clocks per line (HTS).
    pub line_length: u16,
    /// Lines per frame, the blanking lines included (VTS): the line time is the frame period
    /// over this.
    pub frame_length: u16,
    /// The sensor's output, before the camera crops it to the image.
    pub output_width: u16,
    pub output_height: u16,
}

impl Sensors {
    /// From a file of the camera's video device (a duplicate of [`crate::Video`]'s).
    pub(crate) fn new(file: File) -> Sensors {
        Sensors {
            device: Arc::new(Mutex::new(Device { file, len: None })),
        }
    }

    /// One byte of a sensor's register (`eye` 0 left, 1 right), unchecked: 0xff or 0x00 if
    /// it did not answer (see the module's note).
    pub fn read(&self, eye: usize, register: u16) -> io::Result<u8> {
        self.device.lock().unwrap().read(eye, register)
    }

    /// One byte of a sensor's register, checked (see the module's note), asking again for up to
    /// `wait`. Err if it never answered.
    pub fn read_checked(&self, eye: usize, register: u16, wait: Duration) -> io::Result<u8> {
        let v = self
            .device
            .lock()
            .unwrap()
            .checked(eye, wait, |d| d.read(eye, register))?;
        v.ok_or_else(|| not_answering(eye, wait))
    }

    pub(crate) fn write(&self, eye: usize, register: u16, value: u8) -> io::Result<()> {
        let mut device = self.device.lock().unwrap();
        let len = device.len()?;
        xu::write_sensor_register(device.file.as_raw_fd(), len, eye, register, value)
    }

    /// A sensor's timing, checked, asking again for up to `wait`.
    pub fn timing(&self, eye: usize, wait: Duration) -> io::Result<SensorTiming> {
        let t = self.device.lock().unwrap().checked(eye, wait, |d| {
            let mut r16 = |register| -> io::Result<u16> {
                Ok(u16::from_be_bytes([d.read(eye, register)?, d.read(eye, register + 1)?]))
            };
            Ok(SensorTiming {
                chip_id: r16(REG_CHIP_ID)?,
                line_length: r16(REG_HTS)?,
                frame_length: r16(REG_VTS)?,
                output_width: r16(REG_X_OUTPUT_SIZE)?,
                output_height: r16(REG_Y_OUTPUT_SIZE)?,
            })
        })?;
        t.ok_or_else(|| not_answering(eye, wait))
    }

    /// A sensor's exposure, in 1/16 lines, with frames of `max_lines` (under 4096: the high
    /// bits are 0); None unless the two bytes read the same twice and make 1 to `max_lines`
    /// lines.
    pub fn exposure_16ths(&self, eye: usize, max_lines: u32) -> io::Result<Option<u32>> {
        let mut device = self.device.lock().unwrap();
        let mut r = |register| device.read(eye, register).map(u32::from);
        let first = [r(REG_EXPOSURE + 1)?, r(REG_EXPOSURE + 2)?];
        let second = [r(REG_EXPOSURE + 1)?, r(REG_EXPOSURE + 2)?];
        Ok((first == second)
            .then(|| exposure_from(first[0], first[1], max_lines))
            .flatten())
    }
}

fn not_answering(eye: usize, wait: Duration) -> io::Error {
    io::Error::new(
        io::ErrorKind::TimedOut,
        format!(
            "the {} sensor did not answer in {:.1} s (its registers read 0xff or 0x00)",
            EYES[eye],
            wait.as_secs_f64()
        ),
    )
}

/// The exposure (1/16 lines) from its registers' middle and low bytes, if they hold one: 1
/// line to the frame's.
fn exposure_from(mid: u32, low: u32, max_lines: u32) -> Option<u32> {
    let v = mid << 8 | low;
    (16..=max_lines * 16).contains(&v).then_some(v)
}

/// A reading of a sensor's exposure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExposureReading {
    /// `CLOCK_MONOTONIC` ns: when the reading began.
    pub time_ns: u64,
    /// How long it took (ns).
    pub read_ns: u32,
    /// In lines; None when the sensor did not answer.
    pub lines: Option<f32>,
}

/// Counts since the monitor started.
#[derive(Clone, Debug, Default)]
pub struct ExposureStats {
    pub readings: u64,
    /// Readings the sensor did not answer (see the module's note).
    pub unanswered: u64,
    /// Readings that failed with an error.
    pub failures: u64,
    pub last_error: Option<String>,
    /// The slowest reading (ns).
    pub longest_read_ns: u64,
    pub total_read_ns: u64,
}

/// Readings kept for frames to look up: 2 s at 30 Hz.
const HISTORY: usize = 64;
/// Readings kept for [`ExposureMonitor::drain`] when nobody drains them.
const LOG_LIMIT: usize = 4096;
/// The oldest reading a frame takes (ns): longer than the left sensor's spans without answers.
const STALE_NS: u64 = 1_000_000_000;

#[derive(Default)]
struct MonitorShared {
    history: VecDeque<ExposureReading>,
    log: VecDeque<ExposureReading>,
    stats: ExposureStats,
}

/// A thread reading a sensor's exposure once per `period`, until dropped.
pub(crate) struct ExposureMonitor {
    shared: Arc<Mutex<MonitorShared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ExposureMonitor {
    /// `max_lines`: the sensor's lines per frame; a longer exposure is a misread.
    pub(crate) fn start(
        sensors: Sensors,
        eye: usize,
        period: Duration,
        max_lines: u32,
    ) -> io::Result<ExposureMonitor> {
        let shared = Arc::new(Mutex::new(MonitorShared::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (s, st) = (shared.clone(), stop.clone());
        let thread = std::thread::Builder::new()
            .name("zed-exposure".into())
            .spawn(move || {
                let mut next = Instant::now();
                while !st.load(Ordering::Relaxed) {
                    let t0 = monotonic_ns();
                    let read = sensors.exposure_16ths(eye, max_lines);
                    let read_ns = monotonic_ns() - t0;
                    let mut m = s.lock().unwrap();
                    match read {
                        Ok(sixteenths) => {
                            let reading = ExposureReading {
                                time_ns: t0,
                                read_ns: read_ns.min(u32::MAX as u64) as u32,
                                lines: sixteenths.map(|v| v as f32 / 16.0),
                            };
                            m.stats.unanswered += sixteenths.is_none() as u64;
                            if m.history.len() == HISTORY {
                                m.history.pop_front();
                            }
                            m.history.push_back(reading);
                            if m.log.len() == LOG_LIMIT {
                                m.log.pop_front();
                            }
                            m.log.push_back(reading);
                            m.stats.readings += 1;
                            m.stats.total_read_ns += read_ns;
                            m.stats.longest_read_ns = m.stats.longest_read_ns.max(read_ns);
                        }
                        Err(e) => {
                            m.stats.failures += 1;
                            m.stats.last_error = Some(e.to_string());
                        }
                    }
                    drop(m);
                    next += period;
                    let now = Instant::now();
                    if next < now {
                        next = now;
                    } else {
                        std::thread::sleep(next - now);
                    }
                }
            })?;
        Ok(ExposureMonitor {
            shared,
            stop,
            thread: Some(thread),
        })
    }

    /// The exposure (lines) from the newest answer begun at or before `time_ns`, and at most a
    /// second before.
    pub(crate) fn at(&self, time_ns: u64) -> Option<f32> {
        newest_answer(&self.shared.lock().unwrap().history, time_ns)
    }

    pub(crate) fn drain(&self) -> Vec<ExposureReading> {
        self.shared.lock().unwrap().log.drain(..).collect()
    }

    pub(crate) fn stats(&self) -> ExposureStats {
        self.shared.lock().unwrap().stats.clone()
    }
}

/// The newest answer in `history` (oldest first) begun at or before `time_ns`, and at most
/// `STALE_NS` before.
fn newest_answer(history: &VecDeque<ExposureReading>, time_ns: u64) -> Option<f32> {
    history
        .iter()
        .rev()
        .skip_while(|r| r.time_ns > time_ns)
        .take_while(|r| r.time_ns + STALE_NS >= time_ns)
        .find_map(|r| r.lines)
}

impl Drop for ExposureMonitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exposure_registers_that_hold_one() {
        // LabOrin's: 747 lines in a frame of 832
        assert_eq!(exposure_from(0x2e, 0xb0, 832), Some(11952));
        // A sensor that did not answer, and a read that caught it stopping or starting
        assert_eq!(exposure_from(0xff, 0xff, 832), None);
        assert_eq!(exposure_from(0x00, 0x00, 832), None);
        assert_eq!(exposure_from(0xff, 0xb0, 832), None);
        // The whole frame, and over
        assert_eq!(exposure_from(0x34, 0x00, 832), Some(832 * 16));
        assert_eq!(exposure_from(0x34, 0x01, 832), None);
    }

    #[test]
    fn the_newest_answer() {
        let r = |ms: u64, lines: Option<f32>| ExposureReading {
            time_ns: ms * 1_000_000,
            read_ns: 0,
            lines,
        };
        let h: VecDeque<_> = [
            r(0, Some(779.0)),
            r(33, Some(747.0)),
            r(67, None),
            r(100, Some(740.0)),
        ]
        .into();
        assert_eq!(newest_answer(&h, 80_000_000), Some(747.0));
        assert_eq!(newest_answer(&h, 100_000_000), Some(740.0));
        assert_eq!(newest_answer(&h, 0), Some(779.0));
        // Nothing before the first reading, nothing over a second old
        assert_eq!(newest_answer(&[r(10, Some(1.0))].into(), 9_000_000), None);
        let late: VecDeque<_> = [r(1, Some(1.0)), r(5, None)].into();
        assert_eq!(newest_answer(&late, 1_002_000_000), None);
        assert_eq!(newest_answer(&late, 1_000_500_000), Some(1.0));
    }
}
