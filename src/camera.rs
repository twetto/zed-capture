//! Video and IMU together, started in the order that works, each frame with its IMU samples.

use std::io;
use std::path::Path;
use std::sync::Mutex;
use std::time::Duration;

use crate::{FrameClock, Imu, ImuSample, Video, VideoConfig, find};

/// Marks in the fit before frames are given their pulse (1 s at 30 Hz).
const MIN_MARKS: usize = 30;
/// Frames whose arrival after the pulse is averaged before it is used.
const ARRIVAL_FRAMES: usize = 15;

/// The camera's video and, optionally, its IMU.
pub struct Camera {
    // The IMU's stream is turned off before the video's.
    imu: Option<Imu>,
    video: Video,
    frame_sync: Result<[bool; 2], String>,
    arrival: Mutex<Arrival>,
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
    /// ([`Video::enable_frame_sync`]; see [`Camera::frame_sync`]).
    pub fn open(video: &Path, hid: Option<&Path>, config: VideoConfig) -> io::Result<Camera> {
        let video = Video::open(video, config)?;
        if hid.is_some() {
            video.wait_for_first_frame(Duration::from_secs(3))?;
        }
        let imu = hid.map(Imu::open).transpose()?;
        let frame_sync = match imu {
            Some(_) => video.enable_frame_sync().map_err(|e| e.to_string()),
            None => Err("no IMU".into()),
        };
        Ok(Camera {
            imu,
            video,
            frame_sync,
            arrival: Mutex::new(Arrival::default()),
        })
    }

    /// Whether the MCU marks each frame's IMU sample: Ok with whether each sensor's sync output
    /// was already on, or why it could not be turned on.
    pub fn frame_sync(&self) -> Result<[bool; 2], &str> {
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
    /// Before that (the first ~3 s: the MCU clock's rate takes two seconds), and without marks,
    /// the samples read up to the frame's kernel stamp.
    pub fn next(&self, timeout: Duration) -> io::Result<Option<(crate::Frame, Vec<ImuSample>)>> {
        let Some(mut frame) = self.video.next(timeout)? else {
            return Ok(None);
        };
        let Some(imu) = &self.imu else {
            return Ok(Some((frame, Vec::new())));
        };
        if let Some(clock) = imu.frame_clock().filter(|c| c.marks >= MIN_MARKS)
            && let Some((time, count)) = self.pulse_of(&clock, frame.timestamp_ns)
        {
            frame.sync_time_ns = Some(time);
            frame.mcu_frame_count = Some(count);
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
