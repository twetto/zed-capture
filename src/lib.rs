//! The Stereolabs ZED 2 / 2i / Mini read without the ZED SDK, on Linux.
//!
//! - [`Video`]: the side-by-side YUYV stream through V4L2, split into left and right grey (the Y
//!   bytes) in a capture thread and kept in a ring of frames, so a late reader finds its frames
//!   waiting instead of losing them. The kernel's frame timestamps count the frames lost before
//!   they reached it.
//! - [`Imu`]: every sample of the camera's IMU through hidraw, in a thread of its own, with the
//!   camera MCU's clock to count samples lost on the way.
//! - [`Camera`]: both, started in the order that works (the IMU after the first frame), each
//!   frame with the IMU samples read up to its timestamp.
//! - [`find`]: the camera's video and hidraw devices, from sysfs.
//!
//! The USB protocol (report layout, scales, the stream and ping requests) follows Stereolabs'
//! zed-open-capture (MIT). Unlike it and the SDK, nothing here keeps only the newest frame or
//! sample.

pub mod camera;
pub mod device;
pub mod imu;
mod sys;
pub mod video;

pub use camera::Camera;
pub use device::{Devices, find};
pub use imu::{Imu, ImuSample, ImuStats};
pub use video::{Frame, Video, VideoConfig, VideoStats};

/// `CLOCK_MONOTONIC` in nanoseconds: the clock of the kernel's frame timestamps.
pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: a valid clock id and a valid pointer.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}
