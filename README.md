# zed-capture

The Stereolabs ZED 2 / 2i / Mini read without the ZED SDK, on Linux, in Rust.

- **Video** through V4L2: the side-by-side YUYV stream, split into left and right grey (the Y
  bytes) by a capture thread and kept in a ring of frames. A late reader finds its frames waiting
  instead of losing them, and the kernel's frame timestamps count the frames that were lost.
- **IMU** through hidraw: every sample at 400 Hz, read by a thread of its own. The camera MCU's
  clock counts the samples lost on the way.
- **`Camera`**: both together, each frame with the IMU samples read up to its timestamp.

Why: the ZED SDK keeps only the newest frame and the newest IMU sample. Under load on a Jetson
Orin Nano it lost 2-4 % of frames and up to 6 % of IMU samples, while V4L2 alone lost none. See
mr-neuro-nav's `docs/idog/2026-11-03-30hz-recording-plan.md`.

What it doesn't do: the SDK's depth, tracking or rectification. Exposure and gain stay on the
camera's auto settings.

```rust
use std::time::Duration;
use zed_capture::{Camera, VideoConfig};

let camera = Camera::find(VideoConfig::default())?;          // HD720 at 30 Hz
while let Some((frame, imu)) = camera.next(Duration::from_secs(1))? {
    // frame.left, frame.right: 1280 × 720 grey; frame.timestamp_ns: CLOCK_MONOTONIC
    // imu: the samples up to the frame's timestamp (m/s², rad/s, the IMU's axes)
}
```

**Turn the IMU on only after the video's first frame.** `Camera` does this. On a ZED 2i, turning
the IMU's stream on before the video's, or just after it, makes the video return empty buffers
with the error flag, and then stop.

**After the ZED SDK, the IMU comes back by itself.** The SDK detaches the kernel's driver from the
IMU's USB interface and leaves it detached when it closes the camera, so its hidraw node is gone
until a replug. `find` attaches the driver again (as libusb's `libusb_attach_kernel_driver`),
through the camera's `/dev/bus/usb` node, which Stereolabs' udev rule makes writable.

The first buffers after the video starts (one per buffer on a ZED 2i) come back empty, stamped
0: they count as bad buffers, not as lost frames.

## The loss counter

```
zed_count [--seconds 60] [--work-ms 30] [--hogs 6] [--no-imu]
```

It reports frames and IMU samples missing, ring drops (a late reader), the longest gaps, and
how long splitting a frame takes.

```
zed_grab OUT [--frames 5] [--every 15] [--skip 3]
```

It saves raw left and right grey as PGM files, named by timestamp, and the camera's serial
number, which names its factory calibration (`/usr/local/zed/settings/SN<serial>.conf`).

## Building for a Jetson (JetPack 6)

```
cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.35
```

The user needs read and write access to `/dev/videoN` (the `video` group) and
`/dev/hidrawN` (Stereolabs' udev rule `99-slabs.rules`).

## Credit

The USB protocol follows Stereolabs' [zed-open-capture](https://github.com/stereolabs/zed-open-capture)
(MIT): the report layout, the scales, and the stream and ping requests.

Not affiliated with Stereolabs. ZED is a trademark of Stereolabs.

## License

MIT
