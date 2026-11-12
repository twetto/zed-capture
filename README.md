# zed-capture

The Stereolabs ZED 2 / 2i / Mini, read without the ZED SDK, on Linux, in Rust.

- **Video** through V4L2: the side-by-side YUYV stream, split into left and right grey (the Y
  bytes) by a capture thread and kept in a ring of frames. A late reader finds its frames waiting
  instead of losing them, and the kernel's frame timestamps count the frames that were lost.
- **IMU** through hidraw: every sample at 400 Hz, read by a thread of its own. The camera MCU's
  clock counts the samples lost on the way.
- **`Camera`**: both on one hardware clock. Each frame comes with its sync pulse on the IMU's
  clock, the IMU samples up to it, and each eye's exposure and mid-exposure time.
- **Exposure**: the camera's auto exposure, or a manual exposure and gain; the mains frequency for
  its anti-flicker.

Why: the ZED SDK keeps only the newest frame and the newest IMU sample. Under load on a Jetson Orin
Nano it lost 2-4 % of frames and up to 6 % of IMU samples, while V4L2 alone lost none.

Not included: the SDK's depth, tracking or rectification.

## Use

```rust
use std::time::Duration;
use zed_capture::{Camera, VideoConfig};

// HD720 at 30 Hz; the mains frequency of the lights for the camera's anti-flicker
let config = VideoConfig { mains_hz: Some(60), ..Default::default() };
let camera = Camera::find(config)?;
while let Some((frame, imu)) = camera.next(Duration::from_secs(1))? {
    // frame.left, frame.right: 1280 × 720 grey
    // frame.timestamp_ns: the kernel's stamp (CLOCK_MONOTONIC)
    // frame.sync_time_ns: the frame's sync pulse on the IMU's clock (from ~3 s after opening)
    // frame.mid_exposure_ns[eye]: the middle of the exposure of that eye's middle row
    // imu: the samples up to the frame's pulse (ImuSample::time_ns on the same clock;
    //      m/s², rad/s, the IMU's axes)
}
```

For a manual exposure: `camera.video().set_manual_exposure(Duration::from_millis(5), Some(gain))`.

## What to know

**Frame times.** The sensors have a rolling shutter: at HD720 the rows are read top first over
about 29 ms, each after its exposure. `Frame::mid_exposure_ns` is the middle of the exposure of the
image's middle row; another row's is earlier or later by its distance from the middle row times the
line time (`Camera::readout`). At HD720 at 30 Hz these times match the image's motion against the
gyro within about 0.25 ms; other modes are not measured. See [docs/measurements.md](docs/measurements.md).

**The left eye's exposure can be missing.** The camera's auto exposure runs for each eye apart, so
each eye's exposure is read from its own sensor once a frame. The left sensor's registers often
don't answer while the camera streams, so a frame's left exposure (and mid-exposure time) is None
when the left sensor hasn't answered for a second. With a manual exposure, both are known exactly.

**Manual exposure.** `Video::set_manual_exposure` turns the camera's auto exposure off and sets an
exposure and a raw gain on both sensors, as zed-open-capture does. The auto exposure comes back on
when the video is dropped; if the process dies first, it stays off until the camera is unplugged.

**Mains.** The camera powers up with its anti-flicker at 50 Hz. Where the mains are 60 Hz, set
`VideoConfig::mains_hz` to 60: the auto exposure then keeps to whole periods of the lights'
flicker.

**One process at a time.** The camera's settings and registers go through its UVC extension unit,
and a request and its answer are two calls: another process's request between them takes the
answer. Don't run two programs that change camera settings (zed-capture, the ZED SDK, scripts)
on one camera at once.

**The IMU starts after the video's first frame.** `Camera` does this: on a ZED 2i, turning the
IMU's stream on before the video's, or just after it, makes the video stop.

**After the ZED SDK, the IMU comes back by itself.** The SDK leaves the kernel's driver detached
from the IMU, so its hidraw node is gone; `find` attaches it again.

## Tools

```
zed_count [--seconds 60] [--work-ms 30] [--hogs 6] [--no-imu] [--mains 50|60]
```

Counts what was lost (frames, IMU samples, ring drops, the longest gaps) under an optional load,
and checks the frame sync and the exposure readings.

```
zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--mains 50|60]
         [--exposure-ms MS [--gain RAW]]
```

Saves left and right grey as PGM files named by their kernel stamp, and the camera's serial number
(which names its factory calibration, `/usr/local/zed/settings/SN<serial>.conf`). With `--imu`:
`frames.csv` (each frame's stamp, sync pulse, MCU count, each eye's exposure and mid-exposure),
`imu.csv` (every sample), `exposure.csv` (every exposure reading) and `readout.csv` (the line
time).

## Building for a Jetson (JetPack 6)

```
cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.35
```

The user needs read and write access to `/dev/videoN` (the `video` group) and
`/dev/hidrawN` (Stereolabs' udev rule `99-slabs.rules`, which also opens the camera's
`/dev/bus/usb` node used to give the IMU back after the SDK).

## Credit

The USB protocol follows Stereolabs' [zed-open-capture](https://github.com/stereolabs/zed-open-capture)
(MIT): the report layout, the scales, the stream and ping requests, the sensor and ISP requests.

Not affiliated with Stereolabs. ZED is a trademark of Stereolabs.

## License

MIT
