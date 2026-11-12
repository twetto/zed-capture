//! Video and IMU together, started in the order that works, each frame with its IMU samples.

use std::io;
use std::path::Path;
use std::sync::Mutex;
use std::time::Duration;

use crate::sensor::{ExposureMonitor, ExposureReading, ExposureStats, SensorTiming};
use crate::{FrameClock, Imu, ImuSample, Video, VideoConfig, find};

/// Marks in the fit before frames are given their pulse (1 s at 30 Hz).
const MIN_MARKS: usize = 30;
/// The longest after the newest mark that a frame is given a pulse (ns): further, the marks
/// have stopped (the left sensor's sync output turned off, say).
const MAX_SINCE_MARK_NS: f64 = 1e9;
/// The sensor whose pulses the MCU marks: the left (see [`Video::enable_frame_sync`]).
const MARKED_EYE: usize = 0;
/// The sensor whose timing registers are read first: the right, which always answered (the
/// left's often do not: [`crate::sensor`]).
const TIMING_EYE: usize = 1;
/// Frames whose arrival after the pulse is averaged before it is used.
const ARRIVAL_FRAMES: usize = 15;
/// When a frame's rows are read, from its sync pulse: a rolling shutter, row after row.
#[derive(Clone, Copy, Debug)]
pub struct Readout {
    /// One line of the sensor (ns): the frame period over the sensor's lines per frame.
    pub line_ns: f64,
    /// The sensor's output rows before the image's first (the camera crops the rows it does not
    /// send evenly, assumed: HD720, 724 rows to 720).
    pub rows_before_image: f64,
    /// From the pulse to the sensor's first output row being read (ns), on the IMU's clock:
    /// [`pulse_to_readout_ns`].
    pub pulse_to_readout_ns: f64,
}

/// How much sooner than the frame's blanking the first row is read after the pulse, by the
/// IMU's clock (ns): measured, HD720 at 30 Hz only (see [`pulse_to_readout_ns`]).
const READOUT_BEFORE_BLANKING_ENDS_NS: f64 = 310_000.0;

/// From the pulse to the sensor's first output row being read (ns), by the IMU's clock: the
/// frame's blanking lines (its lines but the rows output; HD720 at 30 Hz: 108, 4.33 ms) come
/// first, less 0.31 ms. Turning the camera by hand against the gyro (LabOrin's ZED 2i,
/// 2026-11-12, HD720 at 30 Hz: 8 runs at exposures of 1.4, 16, 30 and 31 ms) put the image's
/// middle row at the pulse + 4.02 ms + its readout − 0.502 (±0.005) × the exposure, within
/// ±0.24 ms; the top row read first, 39.1-40.6 µs a row. Whether the 0.31 ms is the pulse coming
/// a few lines into the blanking or a delay in the IMU's path, one mode cannot tell: other modes
/// are not measured. The kernel stamp comes before the first row (1.1-1.7 ms after the pulse):
/// the camera starts each frame's transfer during the blanking.
pub fn pulse_to_readout_ns(frame_length: u16, output_height: u16, line_ns: f64) -> f64 {
    (frame_length as f64 - output_height as f64) * line_ns - READOUT_BEFORE_BLANKING_ENDS_NS
}

impl Readout {
    /// From the pulse to the image's middle row being read (ns), which ends its exposure. The
    /// same whichever way up the sensor reads.
    pub fn middle_row_ns(&self, image_rows: usize) -> f64 {
        self.pulse_to_readout_ns
            + (self.rows_before_image + (image_rows as f64 - 1.0) / 2.0) * self.line_ns
    }
}

/// The camera's video and, optionally, its IMU.
pub struct Camera {
    // The IMU's stream is turned off before the video's.
    imu: Option<Imu>,
    // The exposure's thread is stopped before the video too.
    exposure: Option<ExposureMonitor>,
    video: Video,
    frame_sync: Result<bool, String>,
    arrival: Mutex<Arrival>,
    /// The sensors' timing, read once (the right's, or the left's if the right did not answer).
    timing: Result<SensorTiming, String>,
}

/// How much each frame moves the arrival once it is known.
const ARRIVAL_FOLLOW: f64 = 0.02;

/// How long after its pulse a frame's kernel stamp comes, in frame periods (0 to 1): a circular
/// mean, of the first `ARRIVAL_FRAMES` frames, then following slowly.
#[derive(Default)]
struct Arrival {
    frames: usize,
    sin: f64,
    cos: f64,
}

impl Arrival {
    /// Adds a frame's phase (frames from a pulse to its kernel stamp; the fraction counts) and
    /// returns the arrival, once `ARRIVAL_FRAMES` phases are in.
    fn learn(&mut self, phase: f64) -> Option<f64> {
        let a = std::f64::consts::TAU * phase.rem_euclid(1.0);
        self.frames += 1;
        let w = if self.frames <= ARRIVAL_FRAMES {
            1.0 / self.frames as f64
        } else {
            ARRIVAL_FOLLOW
        };
        self.sin += w * (a.sin() - self.sin);
        self.cos += w * (a.cos() - self.cos);
        self.periods()
    }

    fn periods(&self) -> Option<f64> {
        (self.frames >= ARRIVAL_FRAMES)
            .then(|| (self.sin.atan2(self.cos) / std::f64::consts::TAU).rem_euclid(1.0))
    }
}

impl Camera {
    /// The one ZED attached (see [`find`]), with its IMU.
    pub fn find(config: VideoConfig) -> io::Result<Camera> {
        let d = find()?;
        Camera::open(&d.video, Some(&d.hid), config)
    }

    /// Opens the video and, once its first frame has come, the IMU if `hid` is given (sooner,
    /// the video can stop; see [`Video::open`]), then turns the frame marks on
    /// ([`Video::enable_frame_sync`]; see [`Camera::frame_sync`]): the left sensor's pulse,
    /// waiting up to 5 s for it to answer, and the right's too, as the ZED SDK does.
    pub fn open(video: &Path, hid: Option<&Path>, config: VideoConfig) -> io::Result<Camera> {
        let video = Video::open(video, config)?;
        if hid.is_some() {
            video.wait_for_first_frame(Duration::from_secs(3))?;
        }
        let imu = hid.map(Imu::open).transpose()?;
        let frame_sync = match imu {
            Some(_) => {
                let left = video.enable_frame_sync(MARKED_EYE, Duration::from_secs(5));
                // Not marked: its failure is not the marks'.
                let _ = video.enable_frame_sync(1 - MARKED_EYE, Duration::from_secs(1));
                left.map_err(|e| e.to_string())
            }
            None => Err("no IMU".into()),
        };
        let sensors = video.sensors();
        let timing = sensors
            .timing(TIMING_EYE, Duration::from_secs(1))
            .or_else(|_| sensors.timing(1 - TIMING_EYE, Duration::from_secs(1)))
            .map_err(|e| format!("the sensors' timing: {e}"))
            .and_then(|t| {
                if !(1..4096).contains(&t.frame_length)
                    || t.output_height < video.config().height as u16
                {
                    Err(format!("the sensors' timing does not fit the stream: {t:?}"))
                } else {
                    Ok(t)
                }
            });
        let exposure = match &timing {
            Ok(t) => Some(ExposureMonitor::start(
                sensors.clone(),
                Duration::from_secs_f64(1.0 / video.fps()),
                t.frame_length as u32,
            )?),
            Err(_) => None,
        };
        Ok(Camera {
            imu,
            video,
            frame_sync,
            arrival: Mutex::new(Arrival::default()),
            timing,
            exposure,
        })
    }

    /// Whether the MCU marks each frame's IMU sample: Ok with whether the left sensor's sync
    /// output was already on, or why it could not be turned on (the marks still come if it was
    /// on: [`Camera::next`] goes by the marks that come).
    pub fn frame_sync(&self) -> Result<bool, &str> {
        self.frame_sync.as_ref().copied().map_err(|e| e.as_str())
    }

    /// The oldest frame not yet taken, with its IMU samples, not yet taken. A late caller gets
    /// the same pairs as a prompt one.
    ///
    /// With the frame marks fitted ([`Imu::frame_clock`]; about 1.5 s after opening), the frame
    /// gets its pulse on the IMU's clock ([`crate::Frame::sync_time_ns`]) and the samples up to
    /// it by that clock: camera and IMU on one hardware clock. Each frame is matched to the
    /// pulse that came a steady time before its kernel stamp (LabOrin's ZED 2i: 1.2-1.5 ms),
    /// learned over the first frames, followed after, and taken to be under one frame period.
    /// Before that (the first ~3 s: the MCU clock's rate takes two seconds), without marks, and
    /// over a second after the newest mark, the samples read up to the frame's kernel stamp.
    ///
    /// With its pulse, the frame gets each eye's exposure ([`crate::Frame::exposure_ns`]) and the
    /// middle of the exposure of each eye's middle row ([`crate::Frame::mid_exposure_ns`]): the
    /// pulse, plus the readout to that row ([`Readout`]), less half the eye's exposure. The
    /// pulse is the left sensor's; the right's rows are read with the left's (one shake run on
    /// both eyes: each middle row within 0.06 ms of its stamp). An eye's exposure is the manual one
    /// ([`Video::set_manual_exposure`]) if it was set before the previous frame's pulse, else
    /// its sensor's newest answer to the readings of its registers (both read once a frame, in a
    /// thread of its own) from before the previous frame's pulse, and at most 1 s before:
    /// OmniVision's sensors take a new exposure at a frame's start, for the frame after. While
    /// the auto exposure changes, it may be a frame off; [`Camera::drain_exposure`] has every
    /// reading. The left sensor often does not answer for a second or more: its exposure is then
    /// None (the right's is close, but can be up to ~5 ms apart).
    pub fn next(&self, timeout: Duration) -> io::Result<Option<(crate::Frame, Vec<ImuSample>)>> {
        let Some(mut frame) = self.video.next(timeout)? else {
            return Ok(None);
        };
        let Some(imu) = &self.imu else {
            return Ok(Some((frame, Vec::new())));
        };
        if let Some(clock) = imu.frame_clock().filter(|c| {
            c.marks >= MIN_MARKS && frame.timestamp_ns as f64 - c.host_ns <= MAX_SINCE_MARK_NS
        }) && let Some((time, count)) = self.pulse_of(&clock, frame.timestamp_ns)
        {
            frame.sync_time_ns = Some(time);
            frame.mcu_frame_count = Some(count);
            if let Some(readout) = self.readout_with(&clock) {
                // The exposure in force: set before the previous frame's pulse
                let set_by = time as f64 - clock.period_ns;
                let exposure = match self
                    .video
                    .manual_exposure()
                    .filter(|m| (m.since_ns as f64) < set_by)
                {
                    Some(m) => [Some(m.lines as f64 * readout.line_ns); 2],
                    None => self
                        .exposure_at(set_by)
                        .map(|l| l.map(|l| l as f64 * readout.line_ns)),
                };
                let middle = time as f64 + readout.middle_row_ns(frame.height);
                frame.exposure_ns = exposure.map(|e| e.map(|e| e.round() as u32));
                frame.mid_exposure_ns = exposure.map(|e| e.map(|e| (middle - e / 2.0).round() as u64));
            }
            let samples = imu.drain_until_time(time)?;
            return Ok(Some((frame, samples)));
        }
        let samples = imu.drain_until(frame.timestamp_ns)?;
        Ok(Some((frame, samples)))
    }

    /// The pulse (host time, MCU count) of the frame stamped `kernel_ns`.
    fn pulse_of(&self, clock: &FrameClock, kernel_ns: u64) -> Option<(u64, u32)> {
        let x = clock.frames_to(kernel_ns as f64);
        let arrival = self.arrival.lock().unwrap().learn(x)?;
        let frames = (x - arrival).round() as i64;
        let time = clock.host_ns_after(frames);
        (time > 0.0).then(|| (time as u64, clock.count.wrapping_add(frames as u32)))
    }

    fn exposure_at(&self, time_ns: f64) -> [Option<f32>; 2] {
        match &self.exposure {
            Some(m) => m.at(time_ns.max(0.0) as u64),
            None => [None; 2],
        }
    }

    fn readout_with(&self, clock: &FrameClock) -> Option<Readout> {
        let t = self.timing.as_ref().ok()?;
        let line_ns = clock.period_ns / t.frame_length as f64;
        Some(Readout {
            line_ns,
            rows_before_image: (t.output_height as f64 - self.video.config().height as f64) / 2.0,
            pulse_to_readout_ns: pulse_to_readout_ns(t.frame_length, t.output_height, line_ns),
        })
    }

    /// When the rows are read, once the frame marks are fitted (the line time is the frame
    /// period by the IMU's clock over the sensor's lines per frame).
    pub fn readout(&self) -> Option<Readout> {
        let clock = self.imu.as_ref()?.frame_clock()?;
        self.readout_with(&clock)
    }

    /// The sensors' timing registers, read when the camera opened, or why they could not be.
    pub fn sensor_timing(&self) -> Result<SensorTiming, &str> {
        self.timing.as_ref().copied().map_err(|e| e.as_str())
    }

    /// Both sensors' exposure readings since the last call (up to 4096 kept), oldest first.
    pub fn drain_exposure(&self) -> Vec<ExposureReading> {
        self.exposure.as_ref().map(|m| m.drain()).unwrap_or_default()
    }

    pub fn exposure_stats(&self) -> Option<ExposureStats> {
        self.exposure.as_ref().map(|m| m.stats())
    }

    /// How long after its pulse a frame's kernel stamp comes, in frame periods, once learned.
    pub fn arrival_periods(&self) -> Option<f64> {
        self.arrival.lock().unwrap().periods()
    }

    pub fn video(&self) -> &Video {
        &self.video
    }

    pub fn imu(&self) -> Option<&Imu> {
        self.imu.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn middle_row_from_the_line_time() {
        // LabOrin's ZED 2i at HD720: 832 lines in 33.335 ms, 724 rows output for 720, after
        // 108 lines of blanking
        let line_ns = 33.335e6 / 832.0;
        let r = Readout {
            line_ns,
            rows_before_image: 2.0,
            pulse_to_readout_ns: pulse_to_readout_ns(832, 724, line_ns),
        };
        assert!((r.pulse_to_readout_ns / 1e6 - 4.0171).abs() < 0.001);
        // The shake runs' fit: the middle row 18.52 ms after the pulse, less half the exposure
        let ms = r.middle_row_ns(720) / 1e6;
        assert!((ms - 18.501).abs() < 0.001, "{ms}");
    }

    #[test]
    fn arrival_averages_around_the_wrap() {
        let mut a = Arrival::default();
        for i in 0..ARRIVAL_FRAMES - 1 {
            assert!(a.learn(if i % 2 == 0 { 0.98 } else { 1.02 }).is_none());
        }
        let p = a.learn(5.0).unwrap();
        assert!(!(0.01..=0.99).contains(&p), "{p}");
        // Then it follows, slowly: one frame barely moves it, many do
        let q = a.learn(0.1).unwrap();
        assert!(q > 0.99 || q < 0.01, "{q}");
        for _ in 0..300 {
            a.learn(0.1);
        }
        assert!((a.periods().unwrap() - 0.1).abs() < 0.01);
    }
}
