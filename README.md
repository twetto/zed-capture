# zed-capture

The Stereolabs ZED 2 / 2i / Mini read without the ZED SDK, on Linux, in Rust.

- **Video** through V4L2: the side-by-side YUYV stream, split into left and right grey (the Y
  bytes) by a capture thread and kept in a ring of frames. A late reader finds its frames waiting
  instead of losing them, and the kernel's frame timestamps count the frames that were lost.
- **IMU** through hidraw: every sample at 400 Hz, read by a thread of its own. The camera MCU's
  clock counts the samples lost on the way.
- **`Camera`**: both together, on the IMU's clock: each frame with its sync pulse, its exposure
  and the middle of it, and the IMU samples up to it.

Why: the ZED SDK keeps only the newest frame and the newest IMU sample. Under load on a Jetson
Orin Nano it lost 2-4 % of frames and up to 6 % of IMU samples, while V4L2 alone lost none. See
mr-neuro-nav's `docs/idog/2026-11-03-30hz-recording-plan.md`.

What it doesn't do: the SDK's depth, tracking or rectification.

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

**Camera and IMU on one clock.** Each image sensor can send its frame-start pulse out; the
camera's MCU marks the IMU sample of each of the left sensor's pulses and counts them (with the
left's output off the marks stop, with the right's they go on). The pulse is off when the camera
powers up; `Camera` turns it on (one register bit in each sensor, through the camera's UVC
extension unit, as zed-open-capture's `ll_activate_sync`), and stops giving frames pulses a
second after the marks stop. A mark is a 400 Hz
sample, so alone it places its frame within 2.5 ms; a line through the marks (MCU time against
the MCU's frame count) places each frame's pulse within tens of µs, and the MCU's clock is
mapped onto the host's, its rate included (LabOrin's ZED 2i: its clock runs 0.57 % fast). From
about 3 s after opening, `Camera::next` gives each frame its pulse (`Frame::sync_time_ns`) and
the IMU samples up to it by the MCU's clock (`ImuSample::time_ns`).

On LabOrin (60 s each: idle, six cores spinning, a reader working 30 ms a frame, both), every
frame got its pulse; each frame's kernel stamp came 1.1-1.7 ms after it, and pulse to pulse was
33.335 ms (p10-p90 ±20 µs).

**When the image was exposed.** The sensors (OmniVision OV4689s) have a rolling shutter: HD720 at
30 Hz is 832 lines of 40.07 µs a frame, the image's 720 rows read over 29 ms. `Camera` reads both
sensors' exposure registers once a frame and gives each frame each eye's exposure
(`Frame::exposure_ns`) and the middle of the exposure of its image's middle row
(`Frame::mid_exposure_ns`): the pulse, plus 4.02 ms to the first row (the frame's blanking
lines, 4.33 ms, less 0.31 ms measured), plus the readout to that row, less half the eye's
exposure. The camera's auto exposure runs for each eye apart: seen from the same to 123 lines
(4.9 ms) apart. The left sensor's registers often don't answer while the camera streams (they
read 0xff or 0x00, for single reads or seconds on end; the right's always answered): its
exposure is its newest answer up to a second old, else none. The camera's ISP keeps no copy of
it in its registers that we found.
In the lab at night (30 ms exposure) that is 3.5 ms after the pulse; in daylight (a few ms)
~17 ms. Turning the camera by hand while it recorded (rotation from the images against the gyro,
in six bands of rows; 8 runs at 1.4, 16, 30 and 31 ms of exposure, HD720 at 30 Hz) found the top
row read first at 39.1-40.6 µs a row, and the image's middle row moving −0.502 ± 0.005 ms per ms
of exposure (half the exposure: the model's), all 8 within ±0.24 ms of the left's
`mid_exposure_ns`. One run analysed on both eyes (24.88 ms): the left's middle row −0.06 ms and
the right's −0.03 ms from their stamps, 39.75 and 39.63 µs a row: the sensors read out together.
Other modes are not measured. The kernel's stamp comes 1.4 ms after the pulse, before the first row
is read: the camera starts each frame's transfer during the blanking.

**Exposure by hand, and the mains.** `Video::set_manual_exposure` sets an exposure (ms, as whole
lines) and a raw gain on both sensors as zed-open-capture does: the camera's auto exposure off in
each eye's ISP (an OmniVision OV580), 5 frames' wait, each sensor written and checked; frames
then carry that exposure exactly. The auto exposure comes back on when the video is dropped (or
on a replug, if the process dies first). `VideoConfig::mains_hz` sets the camera's anti-flicker:
its auto exposure then keeps to whole periods of the lights' flicker. The camera powers up at
50 Hz; in Taiwan (60 Hz) that gave 29.93 ms at night against 24.88 ms at 60 Hz.

Only one process may use the camera's extension unit at a time: a request and its answer are
two calls, and another process's request between them takes the answer.

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
zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--mains 50|60]
         [--exposure-ms MS [--gain RAW]]
```

It saves raw left and right grey as PGM files, named by timestamp, and the camera's serial
number, which names its factory calibration (`/usr/local/zed/settings/SN<serial>.conf`). With
`--imu`, `frames.csv` (each frame's kernel stamp, sync pulse, MCU count, and each eye's exposure
and mid-exposure), `imu.csv` (every sample), `exposure.csv` (every reading of both exposures) and
`readout.csv` (the line time). `--exposure-ms` and `--gain` set a manual exposure for the run;
`--mains` the anti-flicker (`zed_count` takes it too).

`zed_count` also checks the frame sync: how steady each frame's arrival after its pulse is,
pulse to pulse, the marks against the fitted pulses, and the exposure and mid-exposure.

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
