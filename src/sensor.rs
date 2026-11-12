//! The image sensors' registers, through the camera's extension unit: their timing and the
//! exposure the camera's auto exposure gave them, read while the video streams. The registers are
//! OmniVision's (the ZED 2i's sensors are OV4689s: chip id 0x4688).
//!
//! A sensor that does not answer reads 0xff, or 0x00, in every register, and the camera's
//! answer says nothing else: when the video is not streaming, and often the left one while it
//! streams, for single reads or seconds on end (the right one always answered; see
//! `docs/measurements.md`). So each value is read twice and checked: the chip id first, for the
//! reads made once; within its range, for the exposure.
//!
//! One process at a time: a read is a request (SET_CUR) and its answer (GET_CUR), and another
//! process's request between them takes the answer. [`Sensors`] keeps a process's requests in
//! turn.

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
/// 15:8, 7:0. Under 4096 lines a frame, as every ZED mode, the first is 0 and not read.
const REG_EXPOSURE: u16 = 0x3500;
const REG_X_OUTPUT_SIZE: u16 = 0x3808;
const REG_Y_OUTPUT_SIZE: u16 = 0x380a;
/// Gain over three registers (the first unused: 0); the sensor takes [`GAIN_RANGES`] only.
const REG_GAIN: u16 = 0x3507;
/// Pixel clocks per line.
const REG_HTS: u16 = 0x380c;
/// Lines per frame.
const REG_VTS: u16 = 0x380e;

pub const EYES: [&str; 2] = ["left", "right"];

/// The sensor's raw gain takes these ranges only, lowest to highest gain (zed-open-capture's
/// gain zones).
pub const GAIN_RANGES: [std::ops::RangeInclusive<u16>; 4] =
    [0..=255, 378..=511, 890..=1023, 1914..=2047];

/// The multiplier a raw gain gives, about: 2^band × (its low byte) / 128, the bands ×1, ×2,
/// ×4 and ×8 for 0x0xx, 0x1xx, 0x3xx and 0x7xx (measured against exposure on a ZED 2i: within
/// ~5 % inside each band; see `docs/measurements.md`). None outside [`GAIN_RANGES`].
pub fn gain_multiplier(raw: u16) -> Option<f32> {
    let band = match raw >> 8 {
        0 => 0,
        1 => 1,
        3 => 2,
        7 => 3,
        _ => return None,
    };
    GAIN_RANGES
        .iter()
        .any(|r| r.contains(&raw))
        .then(|| (1u32 << band) as f32 * (raw & 0xff) as f32 / 128.0)
}

/// Each eye's ISP control register (zed-open-capture's ISP_CTRL_LEFT/RIGHT): bit 1 is the
/// camera's auto exposure and gain for that eye (0x0b on, 0x09 off on a ZED 2i).
const ISP_CONTROL: [u32; 2] = [0x8018_1033, 0x8018_1833];
const ISP_AUTO_EXPOSURE: u8 = 0x02;

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

    /// Consecutive registers from `register` on, in one request (up to [`xu::MAX_BLOCK`]).
    fn read_block(&mut self, eye: usize, register: u16, out: &mut [u8]) -> io::Result<()> {
        let len = self.len()?;
        xu::read_sensor_block(self.file.as_raw_fd(), len, eye, register, out)
    }

    fn write(&mut self, eye: usize, register: u16, value: u8) -> io::Result<()> {
        let len = self.len()?;
        xu::write_sensor_register(self.file.as_raw_fd(), len, eye, register, value)
    }

    fn read_system(&mut self, address: u32) -> io::Result<u8> {
        let len = self.len()?;
        xu::read_system_register(self.file.as_raw_fd(), len, address)
    }

    fn write_system(&mut self, address: u32, value: u8) -> io::Result<()> {
        let len = self.len()?;
        xu::write_system_register(self.file.as_raw_fd(), len, address, value)
    }

    /// `bytes` registers from `register` on, big-endian, read twice; None unless the same.
    fn read_be_twice(&mut self, eye: usize, register: u16, bytes: u16) -> io::Result<Option<u32>> {
        let mut once = || -> io::Result<u32> {
            let mut b = [0u8; 4];
            let b = &mut b[..bytes as usize];
            self.read_block(eye, register, b)?;
            Ok(b.iter().fold(0, |v, &x| v << 8 | x as u32))
        };
        let a = once()?;
        Ok((once()? == a).then_some(a))
    }

    /// Whether the sensor answers: its chip id has neither byte of a sensor that does not.
    fn answering(&mut self, eye: usize) -> io::Result<bool> {
        let id = [
            self.read(eye, REG_CHIP_ID)?,
            self.read(eye, REG_CHIP_ID + 1)?,
        ];
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
        self.device.lock().unwrap().write(eye, register, value)
    }

    /// Whether the camera's auto exposure and gain is on for `eye` (its ISP's control register,
    /// which answers reliably, read twice).
    pub fn auto_exposure(&self, eye: usize) -> io::Result<bool> {
        let mut device = self.device.lock().unwrap();
        loop {
            let v = device.read_system(ISP_CONTROL[eye])?;
            if device.read_system(ISP_CONTROL[eye])? == v {
                return Ok(v & ISP_AUTO_EXPOSURE != 0);
            }
        }
    }

    /// Turns the camera's auto exposure and gain on or off for `eye`, and checks it took. Off,
    /// the ISP can still apply an update it had pending some 3 frames later (zed-open-capture):
    /// wait 5 frames before setting the exposure ([`crate::Video::set_manual_exposure`] does).
    pub fn set_auto_exposure(&self, eye: usize, on: bool) -> io::Result<()> {
        let mut device = self.device.lock().unwrap();
        for _ in 0..5 {
            let v = device.read_system(ISP_CONTROL[eye])?;
            let want = if on {
                v | ISP_AUTO_EXPOSURE
            } else {
                v & !ISP_AUTO_EXPOSURE
            };
            if v == want {
                return Ok(());
            }
            device.write_system(ISP_CONTROL[eye], want)?;
        }
        Err(io::Error::other(format!(
            "the {} eye's auto exposure stays {}",
            EYES[eye],
            if on { "off" } else { "on" }
        )))
    }

    /// Writes a sensor's exposure, `lines` whole lines (1 to `max_lines`), and checks it took,
    /// asking again for up to `wait` while the sensor does not answer.
    pub fn set_exposure_lines(
        &self,
        eye: usize,
        lines: u32,
        max_lines: u32,
        wait: Duration,
    ) -> io::Result<()> {
        if !(1..=max_lines).contains(&lines) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("an exposure of {lines} lines: 1 to {max_lines}"),
            ));
        }
        let v = lines * 16;
        let bytes = [(v >> 16) as u8 & 0x0f, (v >> 8) as u8, v as u8];
        self.write_checked(eye, REG_EXPOSURE, &bytes, wait, |d| {
            d.read_be_twice(eye, REG_EXPOSURE, 3)
                .map(|r| r.map(|r| r & 0xf_ffff == v))
        })
    }

    /// A sensor's raw gain, read twice; None unless the same, and one the sensor takes.
    pub fn gain(&self, eye: usize) -> io::Result<Option<u16>> {
        let g = self
            .device
            .lock()
            .unwrap()
            .read_be_twice(eye, REG_GAIN, 3)?;
        Ok(g.map(|g| (g & 0xffff) as u16)
            .filter(|g| GAIN_RANGES.iter().any(|r| r.contains(g))))
    }

    /// Writes a sensor's raw gain (in one of [`GAIN_RANGES`]) and checks it took.
    pub fn set_gain(&self, eye: usize, gain: u16, wait: Duration) -> io::Result<()> {
        if !GAIN_RANGES.iter().any(|r| r.contains(&gain)) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("a raw gain of {gain}: the sensor takes {GAIN_RANGES:?}"),
            ));
        }
        let bytes = [0, (gain >> 8) as u8, gain as u8];
        self.write_checked(eye, REG_GAIN, &bytes, wait, |d| {
            d.read_be_twice(eye, REG_GAIN, 3)
                .map(|r| r.map(|r| r == gain as u32))
        })
    }

    /// Writes `bytes` from `register` on, until `took` says they did (Some(true)), for up to
    /// `wait`.
    fn write_checked(
        &self,
        eye: usize,
        register: u16,
        bytes: &[u8],
        wait: Duration,
        mut took: impl FnMut(&mut Device) -> io::Result<Option<bool>>,
    ) -> io::Result<()> {
        let deadline = Instant::now() + wait;
        let mut device = self.device.lock().unwrap();
        loop {
            if device.answering(eye)? {
                for (i, b) in bytes.iter().enumerate() {
                    device.write(eye, register + i as u16, *b)?;
                }
                if took(&mut device)? == Some(true) && device.answering(eye)? {
                    return Ok(());
                }
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "the {} sensor's register {register:#06x} did not take {bytes:02x?} in {:.1} s",
                        EYES[eye],
                        wait.as_secs_f64()
                    ),
                ));
            }
            drop(device);
            std::thread::sleep(RETRY);
            device = self.device.lock().unwrap();
        }
    }

    /// A sensor's timing, checked, asking again for up to `wait`.
    pub fn timing(&self, eye: usize, wait: Duration) -> io::Result<SensorTiming> {
        let t = self.device.lock().unwrap().checked(eye, wait, |d| {
            let mut r16 = |register| -> io::Result<u16> {
                Ok(u16::from_be_bytes([
                    d.read(eye, register)?,
                    d.read(eye, register + 1)?,
                ]))
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

    /// A sensor's exposure, in 1/16 lines (see [`Sensors::exposure_and_gain`]).
    pub fn exposure_16ths(&self, eye: usize, max_lines: u32) -> io::Result<Option<u32>> {
        Ok(self.exposure_and_gain(eye, max_lines)?.map(|(e, _)| e))
    }

    /// A sensor's exposure (1/16 lines) and raw gain, with frames of `max_lines` (under 4096):
    /// its registers 0x3500-0x3509 read in one request, twice. None unless both reads are the
    /// same and hold an exposure (1 to `max_lines` lines; its high byte 0) and a gain the sensor
    /// takes (its first byte 0).
    pub fn exposure_and_gain(&self, eye: usize, max_lines: u32) -> io::Result<Option<(u32, u16)>> {
        let mut device = self.device.lock().unwrap();
        let mut a = [0u8; EXPOSURE_GAIN_BLOCK];
        let mut b = [0u8; EXPOSURE_GAIN_BLOCK];
        device.read_block(eye, REG_EXPOSURE, &mut a)?;
        device.read_block(eye, REG_EXPOSURE, &mut b)?;
        Ok((a == b)
            .then(|| exposure_gain_from(&a, max_lines))
            .flatten())
    }
}

/// Registers 0x3500 (exposure) to 0x3509 (gain's low byte).
const EXPOSURE_GAIN_BLOCK: usize = (REG_GAIN - REG_EXPOSURE) as usize + 3;

/// The exposure (1/16 lines) and raw gain from registers 0x3500-0x3509, if they hold them.
fn exposure_gain_from(b: &[u8; EXPOSURE_GAIN_BLOCK], max_lines: u32) -> Option<(u32, u16)> {
    let g = (REG_GAIN - REG_EXPOSURE) as usize;
    let gain = u16::from_be_bytes([b[g + 1], b[g + 2]]);
    let ok = b[0] == 0 && b[g] == 0 && GAIN_RANGES.iter().any(|r| r.contains(&gain));
    ok.then(|| exposure_from(b[1] as u32, b[2] as u32, max_lines))
        .flatten()
        .map(|e| (e, gain))
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

/// A reading of both sensors' exposure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExposureReading {
    /// `CLOCK_MONOTONIC` ns: when the reading began (the left's).
    pub time_ns: u64,
    /// How long both took (ns).
    pub read_ns: u32,
    /// Per eye, in lines; None when the sensor did not answer.
    pub lines: [Option<f32>; 2],
    /// Per eye, the raw gain read with the exposure (see [`GAIN_RANGES`]).
    pub gain: [Option<u16>; 2],
}

/// Counts since the monitor started.
#[derive(Clone, Debug, Default)]
pub struct ExposureStats {
    pub readings: u64,
    /// Per eye, readings the sensor did not answer (see the module's note).
    pub unanswered: [u64; 2],
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
/// The oldest answer a frame takes (ns).
const STALE_NS: u64 = 1_000_000_000;

#[derive(Default)]
struct MonitorShared {
    history: VecDeque<ExposureReading>,
    log: VecDeque<ExposureReading>,
    stats: ExposureStats,
}

/// A thread reading both sensors' exposure once per `period`, until dropped.
pub(crate) struct ExposureMonitor {
    shared: Arc<Mutex<MonitorShared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ExposureMonitor {
    /// `max_lines`: the sensor's lines per frame; a longer exposure is a misread.
    pub(crate) fn start(
        sensors: Sensors,
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
                    let read = sensors
                        .exposure_and_gain(0, max_lines)
                        .and_then(|l| Ok([l, sensors.exposure_and_gain(1, max_lines)?]));
                    let read_ns = monotonic_ns() - t0;
                    let mut m = s.lock().unwrap();
                    match read {
                        Ok(sixteenths) => {
                            let reading = ExposureReading {
                                time_ns: t0,
                                read_ns: read_ns.min(u32::MAX as u64) as u32,
                                lines: sixteenths.map(|v| v.map(|(e, _)| e as f32 / 16.0)),
                                gain: sixteenths.map(|v| v.map(|(_, g)| g)),
                            };
                            for (eye, v) in sixteenths.iter().enumerate() {
                                m.stats.unanswered[eye] += v.is_none() as u64;
                            }
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

    /// Each sensor's exposure (lines) from its newest answer begun at or before `time_ns`, and
    /// at most a second before.
    pub(crate) fn at(&self, time_ns: u64) -> [Option<(f32, u16)>; 2] {
        let m = self.shared.lock().unwrap();
        [0, 1].map(|eye| newest_answer(&m.history, eye, time_ns))
    }

    pub(crate) fn drain(&self) -> Vec<ExposureReading> {
        self.shared.lock().unwrap().log.drain(..).collect()
    }

    pub(crate) fn stats(&self) -> ExposureStats {
        self.shared.lock().unwrap().stats.clone()
    }
}

/// `eye`'s newest answer in `history` (oldest first) begun at or before `time_ns`, and at most
/// `STALE_NS` before.
fn newest_answer(
    history: &VecDeque<ExposureReading>,
    eye: usize,
    time_ns: u64,
) -> Option<(f32, u16)> {
    history
        .iter()
        .rev()
        .skip_while(|r| r.time_ns > time_ns)
        .take_while(|r| r.time_ns + STALE_NS >= time_ns)
        .find_map(|r| Some((r.lines[eye]?, r.gain[eye]?)))
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

    fn lines_at(h: &VecDeque<ExposureReading>, eye: usize, t: u64) -> Option<f32> {
        newest_answer(h, eye, t).map(|(l, _)| l)
    }

    #[test]
    fn gain_multipliers() {
        assert_eq!(gain_multiplier(128), Some(1.0));
        assert_eq!(gain_multiplier(440), Some(2.875));
        assert_eq!(gain_multiplier(2047), Some(8.0 * 255.0 / 128.0));
        assert_eq!(gain_multiplier(300), None);
        assert_eq!(gain_multiplier(4096), None);
    }

    #[test]
    fn exposure_and_gain_registers() {
        // 0x3500-0x3509 on LabOrin's right sensor at 60 Hz: 621 lines, gain 192
        let mut b = [0u8; EXPOSURE_GAIN_BLOCK];
        b[..10].copy_from_slice(&[0x00, 0x26, 0xd0, 0x04, 0, 0, 0, 0x00, 0x00, 0xc0]);
        assert_eq!(exposure_gain_from(&b, 832), Some((621 * 16, 192)));
        // A gain the sensor does not take, a sensor that does not answer
        b[8..10].copy_from_slice(&[0x01, 0x10]);
        assert_eq!(exposure_gain_from(&b, 832), None);
        assert_eq!(exposure_gain_from(&[0xff; EXPOSURE_GAIN_BLOCK], 832), None);
        assert_eq!(exposure_gain_from(&[0x00; EXPOSURE_GAIN_BLOCK], 832), None);
    }

    #[test]
    fn each_eye_from_its_newest_answer() {
        let r = |ms: u64, l: Option<f32>, r: Option<f32>| ExposureReading {
            time_ns: ms * 1_000_000,
            read_ns: 0,
            lines: [l, r],
            gain: [l.map(|_| 440), r.map(|_| 200)],
        };
        let h: VecDeque<_> = [
            r(0, Some(498.0), Some(621.0)),
            r(33, None, Some(620.0)),
            r(67, None, None),
            r(100, Some(500.0), Some(619.0)),
        ]
        .into();
        let at = |t: u64| [0, 1].map(|eye| lines_at(&h, eye, t * 1_000_000));
        assert_eq!(newest_answer(&h, 1, 80_000_000), Some((620.0, 200)));
        assert_eq!(at(80), [Some(498.0), Some(620.0)]);
        assert_eq!(at(100), [Some(500.0), Some(619.0)]);
        assert_eq!(at(0), [Some(498.0), Some(621.0)]);
        // Nothing before the first reading, nothing over a second old
        assert_eq!(lines_at(&h, 0, 0), Some(498.0));
        let late: VecDeque<_> = [r(1, Some(1.0), Some(2.0)), r(5, None, Some(3.0))].into();
        assert_eq!(lines_at(&late, 0, 1_002_000_000), None);
        assert_eq!(lines_at(&late, 1, 1_002_000_000), Some(3.0));
        assert_eq!(lines_at(&late, 0, 1_000_500_000), Some(1.0));
        let one: VecDeque<_> = [r(10, Some(1.0), Some(1.0))].into();
        assert_eq!(lines_at(&one, 0, 9_000_000), None);
    }
}
