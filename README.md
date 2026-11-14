# zed-capture

The Stereolabs ZED 2 / 2i / Mini, read without the ZED SDK, on Linux, in Rust.

- **Video** through V4L2: the side-by-side YUYV stream, split into left and right grey (the Y
  bytes) by a capture thread and kept in a ring of frames. A late reader finds its frames waiting
  instead of losing them, and the kernel's frame timestamps count the frames that were lost.
- **IMU** through hidraw: every sample at 400 Hz, read by a thread of its own. The camera MCU's
  clock counts the samples lost on the way.
- **`Camera`**: both on one hardware clock. Each frame comes with its sync pulse on the IMU's
  clock, the IMU samples up to it, and each eye's exposure and mid-exposure time.
- **Exposure**: the camera's auto exposure, zed-capture's own (both eyes one exposure, every
  frame's setting known), or a manual exposure and gain; the mains frequency for the anti-flicker.
- **Python**: `zed_capture.Camera` (`crates/zed-capture-python`), frames as numpy arrays.
- **ROS 2**: `zed_ros` (`crates/zed-capture-ros`), both eyes rectified with their camera_info, their
  JPEGs from the Jetson's hardware encoder, and every IMU sample, as topics.
- **uvcvideo with a longer queue** (`dkms/`): with the kernel's, the camera drops frames while
  jtop or htop runs.

Why: the ZED SDK keeps only the newest frame and the newest IMU sample. Under load on a Jetson Orin
Nano it lost 2-4 % of frames and up to 6 % of IMU samples, while V4L2 alone lost none (unless jtop
or htop runs, with the kernel's uvcvideo: below; the SDK reads through it too).

Not included: the SDK's depth or tracking. Rectification is in `zed_ros` only (Rudolf-V's, from the
factory calibration).

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

### From Python

```python
import zed_capture

# exposure: "camera" (the ZED's own), "active" (zed-capture's), "gradient", or "manual"
with zed_capture.Camera(exposure="active", mains_hz=60) as cam:  # waits for the frame sync (~3 s)
    while True:
        f = cam.next(timeout=1.0)  # without the GIL; None if no frame came
        if f is None:
            continue
        # f.left, f.right: uint8 (720, 1280), unrectified
        # f.stamp_ns: the left image's mid-exposure (CLOCK_MONOTONIC); + realtime_offset_ns()
        #             for the system clock
        # f.mid_exposure_ns, f.exposure_ns, f.gain, f.exposure_source: per eye
        # f.imu_time_ns, f.gyro, f.accel: the IMU samples since the previous frame
cam.stats()  # frames, lost, stalls, the auto exposure's counts
```

After 3 s without a frame `next()` opens the camera again by itself. `cam.calibration_file` is the
factory calibration the ZED SDK downloaded (`/usr/local/zed/settings/SN<serial>.conf`), for
rectifying (Rudolf-V's `StereoRectification.from_zed_conf`). Build and install with maturin in
`crates/zed-capture-python` (`maturin develop`, or `maturin build --release --zig --target
aarch64-unknown-linux-gnu` for a Jetson); `pytest tests` there (with a ZED attached and
`ZED_CAPTURE_CAMERA=1`, the tests that take the camera too).

## What to know

**Frame times.** The sensors have a rolling shutter: at HD720 the rows are read top first over
about 29 ms, each after its exposure. `Frame::mid_exposure_ns` is the middle of the exposure of the
image's middle row; another row's is earlier or later by its distance from the middle row times the
line time (`Camera::readout`). At HD720 at 30 Hz these times match the image's motion against the
gyro within about 0.25 ms; other modes are not measured. See [docs/measurements.md](docs/measurements.md).

**Only the right eye's exposure is read.** The camera's auto exposure runs for each eye apart, but
reading the left sensor's registers while the camera streams now and then corrupts the camera's
own exposure writes: the left image goes near black for a second or two. So `Camera` reads the
right sensor's alone (`VideoConfig::read_left_exposure` to read the left's anyway), and a frame's
left exposure is None: the right's is the same in dim light and can be several ms apart in
bright light. `zed_record` stamps the left image with the right's exposure and says so. With a
manual exposure, both eyes are known exactly.

**Manual exposure.** `Video::set_manual_exposure` turns the camera's auto exposure off and sets an
exposure and a raw gain on both sensors, as zed-open-capture does. The auto exposure comes back on
when the video is dropped; if the process dies first, it stays off until the camera is unplugged.

**Active exposure.** `VideoConfig::active_exposure` runs zed-capture's own auto exposure instead
of the camera's: one exposure for both eyes (so both images share their timing), each eye's own
gain, every frame's setting known exactly. Past one flicker period of the lights (`flicker_hz`)
exposures keep to whole periods. Two modes, both `v4l2_trigger_camera`'s: the mean mode holds a
target brightness; the gradient mode (`AutoExposureConfig::gradient`) goes for the most image
gradient, predicting the image after each step through the ZED's measured tone curve, and raises
gain only for a dark frame. In the lab both tracked as well as each other and the camera's own
(docs/measurements.md); the mean mode is the default, at a tenth of the gradient mode's cost. The
camera's auto exposure comes back on when the video is dropped.

**Mains.** The camera powers up with its anti-flicker at 50 Hz. Where the mains are 60 Hz, set
`VideoConfig::mains_hz` to 60: the auto exposure then keeps to whole periods of the lights'
flicker.

**One process at a time.** The camera's settings and registers go through its UVC extension unit,
and a request and its answer are two calls: another process's request between them takes the
answer. Don't run two programs that change camera settings (zed-capture, the ZED SDK, scripts)
on one camera at once.

**Frames lost while jtop or htop runs.** The camera streams ~110 MB/s over USB bulk, and the
kernel's uvcvideo keeps only ~1.5 ms of it queued. When its copy work is held off longer (jtop and
htop do it now and then) the camera's buffer overflows and it aborts the frame: on a Jetson Orin
0.5 % of frames were lost with them running. The ZED SDK reads through the same driver. Either
close them while recording, or install uvcvideo with a longer queue: below.

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

```
zed_record OUT.mcap [--eyes both|left|right] [--encoding jpeg|nvjpg|raw] [--quality 90]
                    [--every 1] [--mains 50|60] [--exposure-ms MS [--gain RAW]] [--seconds S]
                    [--active-exposure [mean|gradient]]
```

Records the images, IMU and frame timing into one MCAP file of ROS 2 messages, with their
definitions in the file: `ros2 bag info`, Foxglove and Python's `mcap-ros2-support` read it.
Recording starts once the frame sync is ready (~3 s) and stops on Ctrl-C, SIGTERM or `--seconds`.
If the video stops (no frame for 3 s: the camera went silent, or was unplugged), the recorder opens
the camera again and goes on in the same file, about 3 s later; each stall is in the file's stats.

| Topic | Type | Stamp |
|---|---|---|
| `/zed/<eye>/image_raw/compressed` | `sensor_msgs/CompressedImage` (grey JPEG) | the eye's mid-exposure |
| `/zed/<eye>/exposure` | `v4l2_trigger_camera/CameraExposure` | its image's |
| `/zed/frame_timing` | `zed_capture/FrameTiming` | the frame's pulse |
| `/zed/imu` | `sensor_msgs/Imu` (the IMU's axes; no orientation) | the sample |
| `/zed/clock` | `zed_capture/ClockOffset` | once a second |

`image_raw` is unrectified; the camera's factory calibration is attached to the file. JPEG is
encoded on the CPU (grey, about 10 ms an image on an Orin Nano) or, with `--encoding nvjpg`, on the
Jetson's hardware encoder (3-channel with flat chroma, format `bgr8; jpeg compressed bgr8`; the
recorder then takes about a quarter of the CPU). With `--encoding raw` the images are
`sensor_msgs/Image` (mono8) on `/zed/<eye>/image_raw`. Every frame's
`FrameTiming` has both eyes' exposure, gain and mid-exposure, where each exposure came from (the
eye's own sensor, set by hand, or the other eye's when the left sensor did not answer), and the
line time for rolling-shutter work. Stamps are `CLOCK_MONOTONIC` plus the system clock's offset at
the start, one offset for the whole file (an NTP step during recording shows on `/zed/clock`).
The file's `zed` metadata has the settings, its `stats` metadata what was recorded and lost.

```
zed_ros [--images rect|raw|both] [--exposure active|gradient|camera] [--mains 50|60]
        [--eyes both|left|right] [--rect FX,CX,CY] [--calibration SN.conf]
        [--no-jpeg] [--jpeg-quality 90] [--best-effort] [--depth N] [--no-imu]
        [--prefix /zed] [--seconds S]
```

Publishes the camera as ROS 2 topics, through [r2r](https://github.com/sequenceplanner/r2r):
both eyes rectified (`/zed/<eye>/image_rect`, `sensor_msgs/Image` mono8, with
`/zed/<eye>/camera_info`: no distortion, the right's P with the baseline, as the ZED wrapper's
rectified images), or raw (`image_raw`), and every IMU sample (`/zed/imu`). Each image also as a
JPEG from the Jetson's hardware encoder (NVJPG), on `<image topic>/nvjpg`
(`sensor_msgs/CompressedImage`, for recording; as mr-neuro-nav's `nvjpg_encoder.py` publishes
them), encoded in a thread of its own after the image went out (`--no-jpeg`: none). Rectification is
[Rudolf-V](https://github.com/twetto/Rudolf-V)'s from the camera's factory calibration (the file the
ZED SDK downloaded), onto the left raw camera's fx, cx, cy unless `--rect`: 3.8 ms a frame on an
Orin Nano. Both eyes carry the frame's stamp (the left image's mid-exposure, on the system
clock), so an exact time synchroniser pairs them. Images over Fast DDS's shared memory need a
segment larger than the default 512 KB on the publisher (`FASTRTPS_DEFAULT_PROFILES_FILE`): with
16 MB no frame was lost on the way, without it 0.4-2.6 % (docs/measurements.md). It is its own
crate, `crates/zed-capture-ros`, built with a ROS 2 environment sourced (Humble or newer). The
JPEGs need GStreamer's development files to build (`libgstreamer1.0-dev`,
`libgstreamer-plugins-base1.0-dev`) and JetPack's encoder to run; elsewhere build it with
`--no-default-features` (no JPEG topics; a build with them on a machine without NVJPG says so and
publishes the images alone):

```
source /opt/ros/humble/setup.bash
cd crates/zed-capture-ros && cargo build --release   # target/release/zed_ros
```

## uvcvideo with a longer queue (dkms/)

```
sudo dkms/install.sh      # sudo dkms/uninstall.sh puts the kernel's back
```

Builds the kernel's own uvcvideo with 32 transfers queued for a stream instead of 5 (~9.5 ms of
the camera's stream instead of ~1.5), through dkms: again for each new kernel. With it, nothing was
lost with jtop and htop running ([docs/measurements.md](docs/measurements.md)).

- The source is downloaded while building (`dkms/prepare.sh`): uvcvideo of the kernel's stable
  release of the same version (5.15.199-tegra: v5.15.199), from kernel.org or its GitHub mirror.
  It is used only if, built unchanged, it is the kernel's own module (the same srcversion); then
  `UVC_URBS` becomes 32 and the version gets `-urbs32`. A kernel with a patched uvcvideo builds
  nothing, and its own module stays.
- `install.sh` installs dkms if it is missing (without its recommended packages, which on a Jetson
  would be Ubuntu's kernel headers), builds the module, and loads it in place of the kernel's if no
  program has the camera open. It needs the kernel's headers (JetPack: `nvidia-l4t-kernel-headers`)
  and the internet.
- Its service `uvcvideo-zed.service` builds it at boot if it is missing for the running kernel
  (when a kernel update's dkms hook ran before the new headers were there), and loads it if the
  camera is not open yet.
- Loaded: `cat /sys/module/uvcvideo/version` says `1.1.1-urbs32`. The module is not signed, so the
  kernel logs that it is tainted; a kernel that requires signed modules would not load it (JetPack
  6 does not require them).

Measured on JetPack 6.2 (L4T 36.5.2, 5.15.199-tegra) only.

## Building for a Jetson (JetPack 6)

```
cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.35
cargo zigbuild --release --features record --target aarch64-unknown-linux-gnu.2.35  # zed_record too
```

`zed_record` with NVJPG links JetPack's GStreamer, so build it on the Jetson (Rust 1.92 or newer,
`libgstreamer1.0-dev` and `libgstreamer-plugins-base1.0-dev`):

```
cargo build --release --features nvjpg --bin zed_record
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
