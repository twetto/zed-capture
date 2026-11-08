//! Video and IMU together, started in the order that works, each frame with its IMU samples.

use std::io;
use std::path::Path;
use std::time::Duration;

use crate::{Imu, ImuSample, Video, VideoConfig, find};

/// The camera's video and, optionally, its IMU.
pub struct Camera {
    // The IMU's stream is turned off before the video's.
    imu: Option<Imu>,
    video: Video,
}

impl Camera {
    /// The one ZED attached (see [`find`]), with its IMU.
    pub fn find(config: VideoConfig) -> io::Result<Camera> {
        let d = find()?;
        Camera::open(&d.video, Some(&d.hid), config)
    }

    /// Opens the video and, once its first frame has come, the IMU if `hid` is given (sooner,
    /// the video can stop; see [`Video::open`]).
    pub fn open(video: &Path, hid: Option<&Path>, config: VideoConfig) -> io::Result<Camera> {
        let video = Video::open(video, config)?;
        if hid.is_some() {
            video.wait_for_first_frame(Duration::from_secs(3))?;
        }
        let imu = hid.map(Imu::open).transpose()?;
        Ok(Camera { imu, video })
    }

    /// The oldest frame not yet taken, with the IMU samples read up to its timestamp and not yet
    /// taken (none without an IMU). A late caller gets the same pairs as a prompt one.
    pub fn next(&self, timeout: Duration) -> io::Result<Option<(crate::Frame, Vec<ImuSample>)>> {
        let Some(frame) = self.video.next(timeout)? else {
            return Ok(None);
        };
        let samples = match &self.imu {
            Some(imu) => imu.drain_until(frame.timestamp_ns)?,
            None => Vec::new(),
        };
        Ok(Some((frame, samples)))
    }

    pub fn video(&self) -> &Video {
        &self.video
    }

    pub fn imu(&self) -> Option<&Imu> {
        self.imu.as_ref()
    }
}
