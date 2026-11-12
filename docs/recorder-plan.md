# zed_record: plan

A standalone binary that records the ZED's images, IMU and frame timing through zed-capture into
one MCAP file, with stamps as `v4l2_trigger_camera` makes them: each image at the middle of its
exposure, its exposure published beside it with the same stamp.

## Usage

```
zed_record OUT.mcap [--eyes both|left|right] [--encoding jpeg|nvjpg|raw] [--quality 90]
                    [--every 1] [--mains 60] [--exposure-ms MS [--gain RAW]] [--seconds S]
```

- `--eyes`: both by default; one eye halves the encoding and the file when there is no budget for
  two. The other eye's exposure is still read and published (it costs no encoding).
- Recording starts once the frame marks are fitted (about 3 s after opening, printed), so every
  recorded frame has its pulse and its stamp. Frames before that are not recorded.
- Stops on SIGINT or SIGTERM, or after `--seconds`, and finishes the file (summary and index).

## Topics

ROS 2 message types, their definitions embedded in the file (MCAP's `ros2msg` schemas), so Python
(`mcap-ros2-support`), Foxglove and our tools read them without ROS. `ros2 bag play` reads the
standard ones; the custom ones need their package built where it runs.

| Topic | Type | `header.stamp` |
|---|---|---|
| `/zed/left/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG, grey) | left image's mid-exposure |
| `/zed/right/image_raw/compressed` | the same | right image's mid-exposure |
| `/zed/left/exposure` | `v4l2_trigger_camera/CameraExposure` | the left image's |
| `/zed/right/exposure` | the same | the right image's |
| `/zed/imu` | `sensor_msgs/Imu` (no orientation: its covariance[0] = −1) | the sample, by the MCU's clock |
| `/zed/frame_timing` | `zed_capture/FrameTiming` | the frame's pulse |
| `/zed/clock` | `zed_capture/ClockOffset` | once a second |

- `image_raw` means unrectified: the camera's factory calibration is attached to the file.
  `--encoding raw` writes `sensor_msgs/Image` (mono8) on `/zed/<eye>/image_raw` instead.
- **Stamps**: an eye's image = the pulse + the first row's delay + the readout to its middle row
  − half its exposure (`Frame::mid_exposure_ns`). Another row is earlier or later by its
  distance from the middle row × the line time (in `FrameTiming`).
- **`CameraExposure`** goes with each recorded image (same stamp), as `v4l2_trigger_camera`
  publishes it; every frame's exposure and gain for both eyes, recorded or not, are in
  `FrameTiming`. Its fields: `exposure_us`, `exposure_lines`,
  `analogue_gain` (the OV4689's raw gain register, 0-2047 in four bands; not a multiplier: the
  file's metadata says so),
  `exposure_measured` (true: from this eye's own sensor or set by hand; false: borrowed from the
  other eye, when this eye's sensor has not answered for a second), `auto_exposure`, and
  `mean_level` / `saturated_fraction` metered on the image (subsampled). `gradient_metric` 0.
- **`FrameTiming`** (new): the pulse, the MCU's frame count, the kernel stamp and sequence number,
  each eye's mid-exposure and exposure and where it came from, the line time, the first row's
  delay after the pulse, and the rows the sensor outputs before the image's first.
- A frame without an exposure for either eye (neither sensor answered for a second) is recorded
  stamped with the last known exposure and `exposure_measured` false, and counted.

## Clocks

zed-capture's times are `CLOCK_MONOTONIC`. ROS bags, like the dog's `/dog/*` topics, use the system
clock, which on the Orins (no RTC battery) is right only after NTP and can step. So the stamps are
monotonic + an offset measured when recording starts: one offset for the whole file, no steps
inside it. The offset, both clocks at start and the host are in the file's metadata, and
`/zed/clock` carries the offset measured again each second, so a later step (NTP) shows and can
be corrected. `FrameTiming` keeps the monotonic times.

## Also in the file

- Metadata `zed`: model, serial number, mode (1280 × 720 at 30 Hz), mains, exposure mode, the
  sensors' timing registers, zed-capture's version and git commit, the recorder's options, the
  clock offset.
- Attachment: the factory calibration (`/usr/local/zed/settings/SN<serial>.conf`), if present.
- Metadata `stats`, at the end: frames recorded, lost (by the kernel's stamps), dropped (ring,
  encoder queue), IMU samples missing, left-sensor readings unanswered, encoding time.

## Inside

- `Camera` (its capture, IMU and exposure threads, as now) → the main loop takes each frame with
  its IMU samples, writes the IMU, `FrameTiming` and exposure messages, and hands each eye's image
  to an encoder pool. The pool's queue is bounded: when it is full an image is dropped and
  counted, never the camera blocked.
- One writer thread owns the MCAP writer: chunked, no chunk compression (JPEG does not compress;
  zstd for `raw` later if wanted). If the process is killed rather than stopped, the chunks
  written are still recoverable (`mcap recover`).
- The recorder's dependencies (`mcap` without its default zstd/lz4, `jpeg-encoder`, both pure
  Rust) sit behind a cargo feature `record`, so the library stays at `libc` alone.

## Encoders

1. **JPEG on the CPU** (`jpeg-encoder`, grey, `--quality`): first, to have the whole path. To
   measure on the Orin: ms per image, and whether two eyes at 30 Hz fit.
2. **NVJPG** next: the chain mr-neuro-nav's `nvjpg_encoder.py` already runs (appsrc GRAY8 →
   nvvidconv → NVMM I420 → nvjpegenc), through GStreamer's Rust bindings behind a feature
   `nvjpg`, built on the Orin itself (LabOrin has cargo and GStreamer 1.20's development files).
   nvjpegenc writes 3-channel JPEG with flat chroma; consumers decode it as grey either way.
3. **Raw** (`sensor_msgs/Image`, mono8) for tests: 55 MB/s for two eyes.

## Steps

Done (2026-11-12): 0 and 1, with CPU JPEG and raw (results in `measurements.md`).

0. zed-capture reads each eye's exposure and gain in one request: the extension unit returns up
   to 13 consecutive registers when the request's count (bytes 9-10) says so (tested: 0x3500 and
   0x3800 blocks match single reads; 1.75 ms like one byte). Read twice and checked (the block's
   fixed registers too), both eyes: ~7 ms a frame, against 14 ms for the exposure alone now.
   `Frame` gets each eye's gain.
1. `zed_record` with CPU JPEG and raw, one or two eyes. On LabOrin: 60 s at 30 Hz with both eyes,
   no frame lost or dropped, the encoding cost; the file read back in Python through the embedded
   schemas; a shake run analysed from the MCAP gives the same stamps as from `zed_grab`.
2. NVJPG as the encoder; the same checks, and the CPU it saves.
3. On the dog, beside a ros2 bag of `/dog/*`: the time alignment between the two files.
4. Later: zed-capture into the node through PyO3, which then records in-process.

## Decided

- Topic prefix `/zed/...` (the ZED wrapper's are `/zed/zed_node/...`; different names keep its
  rectified images apart from these raw ones).
- JPEG quality 90 by default, as the current recordings.
- Gain stored raw for now; a conversion to a multiplier once measured (brightness against gain at
  a fixed exposure).
