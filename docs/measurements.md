# Measurements on a ZED 2i

What the code's timing, sync and exposure handling rests on, measured on one ZED 2i on a Jetson
Orin Nano ("LabOrin", JetPack 6), HD720 at 30 Hz, November 2026. Other camera models and video
modes are not measured.

## Frame loss: the ZED SDK against V4L2

Under load on the Orin Nano the ZED SDK lost 2-4 % of frames and up to 6 % of IMU samples (it keeps
only the newest of each); V4L2 alone lost none. Details: mr-neuro-nav's
`docs/idog/2026-11-03-30hz-recording-plan.md`.

## Frame sync

- Each image sensor can send its frame-start pulse (VSYNC) out: register 0x3002, bit 7, off at
  power-up. The camera's MCU marks the IMU sample after each pulse and counts the frames.
- **Only the left sensor's pulse makes the marks.** Switching each sensor's output while recording:

  | Sync output | Marks per s | MCU frame count |
  |---|---|---|
  | both on | 30.0 | counting |
  | left off, right on | 0 | stopped |
  | right off, left on | 30.1 | counting |
  | both off | 0 | stopped |
  | both on again | 30.2 | counting |

- A mark is a 400 Hz sample, so it lands 0-2.5 ms after its pulse. A line through ~300 marks (MCU
  time against the MCU's count, along their lower edge) places each pulse to about 20 µs: pulse to
  pulse 33.335 ms, p10-p90 ±20 µs.
- The MCU's clock runs 5660 ppm fast against the host's (the IMU is really 402.3 Hz), so its time
  is mapped with a rate as well as an offset.
- Every frame got its pulse in 60 s runs under four loads: idle, six cores spinning, a reader
  working 30 ms a frame, and both.
- The kernel's frame stamp comes 1.1-1.7 ms after the pulse, before the first row is read (below):
  the camera starts each frame's USB transfer during the blanking.

## The image sensors

- OmniVision OV4689 (chip id 0x4688), read through the camera's UVC extension unit as
  zed-open-capture does (unit 4, selector 2; left sensor 0xa3, right 0xa5, I2C address 0x6c).
- HD720 at 30 Hz: 4808 pixel clocks a line, 832 lines a frame, 1280 × 724 output (the camera sends
  720 rows). A line is 33.335 ms / 832 = 40.07 µs.
- Exposure: registers 0x3500-0x3502, in 1/16 lines. Gain: 0x3507-0x3509, raw, in four ranges
  (0-255, 378-511, 890-1023, 1914-2047).

### The left sensor's registers often don't answer

- While streaming with the IMU on, the left sensor's reads return 0xff or 0x00 in every register:
  0xff spans of ~0.6 s every few seconds, 0x00 runs of up to 10 s, and single bad reads among good
  ones. With the IMU off, every left read returned 0xff. Not streaming, both sensors read 0xff.
- The right sensor answered every read (thousands).
- The camera's answer looks the same either way: there is no status byte. So every value is read
  twice and range-checked, and for one-off reads the chip id is checked before and after.
- zed-open-capture notes the same: "After STREAMON the sensors ignore register writes and reads
  return 0 for ~150 ms"; it checks the chip id three times in a row at open.
- In the per-eye exposure runs, the left gave no good reading for over a second at a time often
  enough that 11-16 % of frames had no left exposure.

### Reading the left sensor disturbs it

Two walks along the same route on a battery, from the lab into a dark room and back, both eyes
recorded:

| Walk | Exposure reads | Length | Dark (right image's mean < 15) | Swings (> 30 in 0.1 s) | Left image near black alone |
|---|---|---|---|---|---|
| 1 | both sensors, every frame | 224 s | 12 s | 62 | 2: 0.67 s, 1.6 s |
| 2 | none | 263 s | 54 s | 61 | 0 |

Each time the left sensor's exposure and gain jumped in one step to 115 lines (4.6 ms) or 1 line
at 1× gain (the auto exposure moves in small steps), and the auto exposure took 0.7-1.6 s to bring
it back, while the right eye's held. The frame marks went on, so the sensor kept its other
settings: its exposure and gain were overwritten, most likely a write of the camera's own colliding
with our read on the left sensor's bus (the one whose reads fail). Two events against none is not
proof; a static lights test with six swings and the reads on had none. Since then only the right
sensor is read.

### One process on the extension unit at a time

A read is two calls: the request, then its answer. A second process's request between them takes
the answer. A test script run alongside zed-capture's exposure thread read the exposure's middle
byte (0x30) as register 0x3002 and wrote it back with bit 7 set (0xb0). It was restored to 0x80.

### USB link errors: lost frames, and the camera stuck once

The kernel logs `uvcvideo: Non-zero status (-71) in video completion handler` (EPROTO, a USB
protocol error) every few minutes while the camera streams: 7-66 an hour, 142 in one day, with
the camera behind the dev kit's USB hub. Each one costs a frame: the two frames lost in recording
runs (below) were lost at the second of such an error. Once, a burst of six in three seconds was
followed by the camera stopping altogether, until a replug (even the ZED SDK reported "camera not
detected"); the next 55 starts of the same code were clean. The same happened on 2026-11-05 with
only the ZED SDK running: errors more and more often, then a dead camera. Check with
`journalctl -k | grep -c "status (-71)"`.

## When the image is exposed

### Method

The camera is turned by hand (about ±10-15°, once a second) at a scene a few metres away. For each
pair of consecutive frames, the rotation comes from tracked features (left or right image,
undistorted, Kabsch on bearings) and is compared with the gyro integrated between the two frames'
times shifted by δ. The rotation between the IMU's axes and the camera's is fitted, then δ. The
features in each of six bands of rows give their own δ, and a line through the bands gives the
line time (its slope) and the image's middle row (its value at row 359.5). The whole image's δ alone
would be that of its features' mean row, which the scene's texture sets.

Checked on a synthetic shake (a still recording warped through a known rotation, rows 40.07 µs
apart, the middle row 3.00 ms after the stamp): the line gave 3.11 ms and 39.17 µs a row.

### Runs

The middle row's time after the pulse:

| Exposure | How set | Middle row after the pulse | Line time |
|---|---|---|---|
| 29.93 ms | auto | 3.73 ms | 39.74 µs |
| 31.09 ms | manual (ZED SDK) | 2.72 ms | 40.03 µs |
| 1.44 ms | manual, 3 runs | 17.82-17.93 ms | 39.09-39.73 µs |
| 1.44 ms | manual, gain 2047 | 17.79 ms | 40.63 µs |
| 1.44 ms | manual, gain 408 (dark) | 17.59 ms | 40.14 µs |
| 16.03 ms | manual | 10.47 ms | 40.21 µs |
| 24.88 ms | auto, 60 Hz mains, left image | 6.00 ms | 39.75 µs |
| 24.88 ms | the same run, right image | 6.03 ms | 39.63 µs |

A line through the first eight runs: the middle row at the pulse + 18.52 (± 0.08) ms
− 0.502 (± 0.005) × the exposure. That is the sensor's top row read first at about 40 µs a row,
4.02 ms after the pulse (14.48 ms more to the middle row), with half the exposure before it. With
the slope fixed at −0.5, all eight runs are within ±0.24 ms. The frame's blanking (832 − 724 = 108
lines) is 4.33 ms; the measured 4.02 is 0.31 ms less. Whether that is the pulse coming a few lines
into the blanking or a delay in the IMU's path, one mode cannot tell.

The two eyes of one run agree to 0.03 ms: the sensors read out together.

So `Frame::mid_exposure_ns` = the pulse + 4.02 ms + 14.48 ms − half the eye's exposure (HD720 at
30 Hz): 3.5 ms after the pulse at a 30 ms exposure, ~17.5 ms at 2 ms.

## Exposure control

- Writing the sensors' exposure takes effect only with the camera's auto exposure off: each eye's
  ISP (an OmniVision OV580) control register, 0x80181033 left and 0x80181833 right, bit 1 (0x0b on,
  0x09 off). zed-open-capture waits 5 frames after turning it off: the ISP can still apply an update
  it had pending.
- With it off, image brightness follows the exposure written (camera still, gain 408 left / 128
  right as the auto exposure left them):

  | Exposure | Left brightness | Right brightness |
  |---|---|---|
  | 50 lines (2.0 ms) | 14.6 | 11.4 |
  | 100 lines (4.0 ms) | 22.9 | 17.8 |
  | 200 lines (8.0 ms) | 41.9 | 31.6 |
  | 400 lines (16.0 ms) | 70.5 | 53.1 |
  | 800 lines (32.1 ms) | 106.5 | 87.5 |
  | 200 lines again | 39.7 | 30.1 |
  | auto back on | 102.5 | 100.6 |

- 800 lines kept 30 Hz with no frame lost.
- A 2 ms manual exposure through `Video::set_manual_exposure`: every frame carried 2.003 ms, and
  the middle row at the pulse + 17.50 ms.

### Gain

With the auto exposure off and the camera still, the brightness at raw gain 128 against exposure
(2 to 32 ms) gives the image's response to known light; at 2 ms, each raw gain's brightness read
back through it gives the gain's multiplier:

| Raw gain | 160 | 192 | 224 | 255 | 378 | 440 | 511 | 890 | 960 | 1023 | 1914 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Left | 1.23 | 1.48 | 1.66 | 1.84 | 1.97 | 2.92 | 4.00 | 4.21 | 6.08 | 7.56 | 8.29 |
| Right | 1.18 | 1.40 | 1.64 | 1.81 | 1.98 | 2.93 | 4.06 | 4.35 | 6.12 | 8.04 | 8.15 |
| 2^band × low byte / 128 | 1.25 | 1.50 | 1.75 | 1.99 | 1.91 | 2.88 | 3.98 | 3.81 | 6.00 | 7.97 | 7.63 |

So the multiplier is about 2^band × (low byte) / 128, the bands ×1, ×2, ×4, ×8 for 0x0xx, 0x1xx,
0x3xx, 0x7xx: within ~5 % inside each band; each band's first value read 3-14 % high, in noisy
steps (`gain_multiplier`). Gains under 128 lower the image too (not measured: too dark); the top
ones (1980, 2047) clipped. The steps' frames vary with the lights' flicker: these exposures are not
whole periods of it.

At the same exposure and gain the right image was 40-50 % brighter than the left (about 1.6-1.8×
in light, through the response): the left eye gets about half the light, and the auto exposure
gives it about twice the right's gain (440 against 192). A dirty lens, or the sensors differ.

### The auto exposure runs for each eye apart

Seen: the left 779 and the right 747 lines for 4 s after the stream started; 747 and 768 lines at
gains 408 and 128; 498 and 621 lines at gains 440 and 200 (4.9 ms apart); often the same.

The ISP's register pages (0x80181000-0x80181FFF) hold no copy of either eye's exposure or gain:
searched for 1-3 bytes in either byte order, in lines and in 1/16 lines, and for a left/right
ratio matching the exposures'. The only matches were image statistics, which changed between
dumps while the exposures held. So the left eye's exposure can only come from its unreliable sensor.

### How the auto exposure moves

Lights on, the lens covered by hand three times, the room's lights off, on again (60 Hz mains):

- It changes the exposure and gain every 3 frames (0.1 s), in steps.
- Into the dark, exposure first: a little gain at 16.6 ms, then the exposure to 24.9 ms (3 × 8.33,
  the longest whole number of flicker periods in a 33 ms frame) with the gain back at 1×, then the
  gain up to its top, 15× (raw ~2032): 1.1-2.0 s from light to the top. Back into light: 15× to 1×
  in 0.5 s, then the exposure down again.
- In the light the eyes differed in exposure too, not only in gain: the left 24.9 ms at 1.5-1.6×,
  the right 16.6 ms at 1.0-1.4× (the left gets less light, above).
- With the lights off the image's mean was 6.5-7 of 255 at 24.9 ms and 15×, with 5-10 trackable
  corners (600 in the light): too dark to track, and too dark to place the images in time against
  the gyro (229 of 929 frame pairs gave a rotation; the fit had no minimum).
- No frame was lost over the 200 s, the six swings included.

Once, the video stopped by itself, 60 s into a recording on a battery while the camera was carried
into a dark corridor, a second after the gain went to 12×: no USB error, the camera still enumerated,
its frame sync still on (so not powered off), and it streamed again when opened. The test above did
not repeat it.

### Mains frequency

The camera powers up with its anti-flicker at 50 Hz (V4L2's power line frequency control). At
night under the lab's lights the auto exposure held 747 lines (29.93 ms, three 10 ms flicker
periods) at 50 Hz and 621 lines (24.88 ms, three 8.33 ms periods) at 60 Hz, Taiwan's mains.

## Active exposure (zed-capture's own)

### When a write takes effect

Exposures written to both sensors at times spread over the frame, the camera still (LabOrin,
HD720 at 30 Hz): a write that ends before pulse k shows in frame k + 1, wherever in frame k − 1 it
falls; frame k still has the setting before. So frame n's setting is the newest one written before
pulse n − 1, which the controller looks up for each frame's stamps. It writes each new setting
1-18 ms after a pulse (both eyes' four registers take ~8 ms, the right's first), so both eyes take
it on the same frame, then reads it back.

### Mean mode

The mean mode of `v4l2_trigger_camera`'s auto exposure (target 100, 12 % a step at most,
saturation and floor rules), one exposure for both eyes, each its own gain, past one flicker
period in whole periods, at most 25 ms.

- Still, lights on: from the camera's own setting to 25.0 ms, left 1.47× and right 2.30×, means
  105 and 98, in about 1 s; 7 writes, all read back.
- 200 s of light swings (lens covered 3 times, the room's lights off, twisting in the dark,
  lights on): 6000 frames, 1 lost (a USB error), no stall. 244 writes; the right read back every
  time, the left 49 times not (each written again; 36 frames' left setting unconfirmed).
- Lit, both eyes held 105/105, at different gains (left 1.8×, right 2.6×): the eyes see different
  parts of the room.
- Lights off: 25 ms and 15.94× on both, means 10 (left) and 21 (right), the sensor's limit; the
  camera's own auto exposure stops at the same.
- After uncovering the lens the image stayed washed out (mean > 180) 0.63 s each time: from 15.94×
  back down at 12 % a step is ~19 steps. The camera's own auto exposure took 0-0.6 s in another run.
- A left image going dark for 7 frames at 53 s was the hand reaching the left lens first: the
  right followed 33-100 ms later.

### The tone curve

The gradient mode predicts the image after a step, so it needs the camera's response to light.
Measured on a still scene (the lab at night): both eyes through 15 exposures at gain 1 (8 to 623
lines) and 10 gains at 208 lines (one flicker period: no flicker), 6 frames averaged a step, every
write read back. From pixel pairs a step apart, the slope h(z) = dz / d ln(exposure × gain) at each
output level z (the median, binned by level):

| Level z | 10 | 18 | 34 | 50 | 66 | 82 | 98 | 114 | 130 | 154 | 178 | 202 | 218 | 234 | 250 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| h from exposure (left) | 4.8 | 13.4 | 27.9 | 42.0 | 56.4 | 71.5 | 88.5 | 85.4 | 86.2 | 101.9 | 106.4 | 119.1 | 94.1 | | |
| h from gain (left) | 5.6 | 13.9 | 28.2 | 42.7 | 54.8 | 67.4 | 86.6 | 85.1 | 87.4 | 101.5 | 111.6 | 105.4 | 108.3 | 60.5 | 1.9 |

- Exposure and gain give the same slope: one curve, after the gain.
- The two eyes' curves agree within 0.05 in log light: it is the ISP's, not the sensors'.
- About linear above a black level of 5 up to level ~100 (h ≈ 0.85-0.9 z), then compressed
  (h ≈ 0.6-0.65 z up to ~210), then a soft shoulder into the clip.

`ZED_TONE_SLOPE` is h every 8 levels, averaged over both eyes and both sweeps. Predicting each
step's image from another's at ratios 1.05-1.6 (flicker-free pairs), mean |predicted − real| per
pixel:

| | The curve | Power law 0.76 | Linear (`v4l2_trigger_camera`'s) |
|---|---|---|---|
| Left | 1.51 | 3.19 | 5.77 |
| Left, real > 120 | 2.76 | 9.30 | 18.8 |
| Right | 1.34 | 2.05 | 3.55 |
| Right, real > 120 | 4.29 | 7.53 | 15.6 |

At ratios 2.5-4.5 all three miss more (the curve 5.8-7.1). The curve was measured at the camera's
default image controls (gamma, contrast); another ZED 2i's ISP likely has the same, other models or
settings not. The controller checks it on every recording: each frame after a step predicted from
the frame stepped before (`zed_record`'s `tone_curve_check`).

### Gradient mode

`v4l2_trigger_camera`'s gradient mode (after Zhang et al., ICRA 2017), each eye apart, its
predictions through the tone curve, gradients at every 4th pixel each way.

- Still, lights on (20 s): from the camera's setting (24.9 ms, 1.81×; means 107/84) it settled in
  1 s at 16.7 ms, left 1.36× and right 1.92×, means 73/66, and held; 7 writes, all read back. The
  dim room needs gain past 25 ms, and the mode sheds gain down to a mean of ~66 (it raises gain only
  below 60): less gain and a darker image than the mean mode.
- The controller took 1.9 ms a frame (both eyes; at most 6.7 ms) on LabOrin.
- `tone_curve_check`: 1.03 levels mean error with the curve, 3.62 linear, 5.90 for no change.

### Mean against gradient: tracking

Scored as a VIO front end would (GFTT corners topped up to 500 at half resolution, pyramidal KLT
frame to frame, kept if the backward track returns within 0.5 px): the features tracked a frame
and the share surviving each frame, left eye. The runs are by hand, one each, so a few % is noise.

The 200 s light swings, each mode:

| | Mean | Gradient |
|---|---|---|
| Lit, still (settings) | 25 ms, means ~105 | 16.7 ms, less gain, means 67-75 |
| Lit, still: tracked | 319-327 | 329-338 |
| Covered and uncovered | 112-215 | 117-230 |
| Lights off (both 25 ms, ~15.9×) | 122-218 | 93-195 (the room ~16 % darker that run: means 9.1 against 10.8 at the same setting) |
| Washed out after uncovering | 0.63 s | 0.70-0.73 s |
| All 200 s: tracked, survival | 255, 65.2 % | 239, 63.1 % |

40 s in the lit room, 20 s of it twisting ±15° about twice a second, with the camera's own auto
exposure as a third:

| | Mean | Gradient | Camera's own |
|---|---|---|---|
| Exposure; image mean | 25 ms; 95-100 | 16.7-20.8 ms; 67-75 | 21-25 ms; 78-90 |
| Survival while twisting (three 5-s windows) | 96.8, 88.6, 94.0 % | 97.5, 88.8, 88.5 % | 95.5, 87.2, 96.5 % |
| All 40 s: tracked, survival | 394, 97.2 % | 367, 97.0 % | 382, 97.5 % |

The three track alike: the gradient mode's shorter exposure did not show in the twist. Its
controller takes 1.9 ms a frame against the mean mode's 0.18 ms. In the twist the gradient mode
lengthened its exposure (16.7 → 20.8 ms) as motion blurred the image.

## Cost of reading the exposure

Each read is a request and its answer through the extension unit, about 1.75 ms. Four reads per
eye (two bytes, twice) take about 7 ms; both eyes about 14 ms, once a frame. No frame or IMU sample
was lost with it.

## Recording (`zed_record`)

60 s at 30 Hz on LabOrin, auto exposure at 60 Hz mains:

| Recording | Frames lost | Encoding an image | Recorder's CPU | File |
|---|---|---|---|---|
| both eyes, JPEG q90 (2 × 60 s, 30 s) | 1, 1 of 1801; 0 of 901 | 10.0-10.3 ms (CPU) | 65 % of a core | 6.2 MB/s, 91-107 KiB an image |
| both eyes, NVJPG q90 (60 s, 30 s) | 1 of 1800; 0 of 901 | 5.5-6.9 ms (mostly waiting) | 17 % of a core | 6.7 MB/s, 95-117 KiB an image |
| left eye, JPEG q90 (20 s) | 0 of 601 | 10.3 ms | | 3.5 MB/s |
| both eyes, raw (10 s) | 0 of 302 | 0.3 ms | | 56 MB/s |

Each lost frame was lost in the kernel (the MCU's count and the kernel's sequence both jumped) at
the second of a USB protocol error (above), not by the recorder. NVJPG's JPEGs have three channels
(flat chroma): 3-4 % larger than the CPU's grey ones at the same quality. Reading
each eye's exposure and gain in one request (registers 0x3500-0x3509) took 6.9 ms a frame for
both eyes, read twice, against 13.9 ms for the exposure alone byte by byte.

## From Python

`zed_capture.Camera(exposure="active", mains_hz=60)` on LabOrin, read one frame at a time with
25 ms of busy Python work after each (as a node would), 60 s:

- 1800 frames (30.00 a second), none lost, no IMU sample missing (13.4 a frame).
- An image's age when `next()` returned it, from its stamp (the middle row's mid-exposure):
  median 25.2 ms, at most 26.7 ms. Most of it is the rest of the readout (the bottom row is read
  14.5 ms after the middle one) and the USB transfer.
- `next()` waited 8.0 ms (median): the frame period less the work. The process took 80 % of a core,
  75 % of it the work: capture and bindings about 5 %.
- Opening took 2.8 s, most of it waiting for the frame marks.

## ROS 2 topics (`zed_ros`)

`zed_ros` (active exposure, both eyes, raw mono8, reliable keep-last 1) on LabOrin (Humble, Fast
DDS 2.6), subscribers in other processes on the same machine, best effort keep-last 1 as
pipeline_node's; 60 s a case. Frames lost by the stamps' gaps:

| Case | Lost by the subscribers | Lost by the camera |
|---|---|---|
| One plain subscriber | 0 of 1801 | 0 |
| A plain one and a node-like one (exact left/right pairs, 25 ms work a pair) | 0 of 1801, pairs too | 0 |
| The same and `ros2 bag record` of both images | the camera's 6 (the bag: all 1802 published) | 6 |
| The same under 4 busy loops | the camera's 60 | 61 |
| No shared-memory profile on the publisher (512 KB segment) | node-like 2.55 %, plain 0.39 % left, 1.28 % right | 0 |

With the 16 MB segment (mr-neuro-nav's `fastdds_shm16.xml`) the subscribers got every frame the
camera gave. Publishing took 0.6-0.9 ms an image; images reached the subscribers 28-31 ms after
their mid-exposure.

Rectified (`--images rect`, Rudolf-V's maps applied byte to byte): the same as rudolf_v's own
rectification of the raw pair, every pixel (0 of 921600 differ, both eyes). 6.8 ms a frame with a
thread an eye, 3.8 ms with two; 8-9 ms on two pinned CPUs beside the node.

The pipeline node (mr-neuro-nav) in shadow on LabOrin (60 s, Light ESS on both eyes, zed_ros or
the ZED wrapper on CPUs 0-1, the node on 2-5, a stand-in dog stream; the same view):

| | zed_ros | ZED wrapper (depth off) |
|---|---|---|
| Node | 29.9 Hz | 29.8 Hz |
| Image age at the node (from each one's stamp) | 36 ms median | 38 ms median |
| Camera frames lost | 0.3 % | 0.6 % |
| Light ESS: valid depth, blind bins | 99.8 %, 0 % | 99.8 %, 0 % |

zed_ros stamps the mid-exposure, the wrapper later in the frame: the age gain is larger than the
2 ms shown. The dog's walk script in shadow on zed_ros (mr-neuro-nav walk_to_checkerboard.sh
--stereo ess --record --health, 60 s): node 29.6 fps, capture to command 59 ms (p90 64), camera
frames lost 0.5 %, the recording 8.0 MB/s with NVJPG.

### The camera lost frames under CPU load: monitoring tools delaying uvcvideo

Every build lost frames while the CPUs were partly busy (in 40 s: 1 with 2 busy threads, 11-25
with 4, 0-22 with 6, none idle), from the kernel: uvcvideo's sequence skipping 2 a frame, no bad
buffer, no USB error logged then. Not the power (a power bank and the wall supply alike), not
frame sync, the IMU or the busy threads' priority or CPUs.

How (uvcvideo's frame and statistics traces and ftrace's workqueue and scheduler events, root,
LabOrin 2026-11-13): the camera itself aborts the frame. Part way through (after 10-85 of a
frame's 113 payloads of ~32 KB) it sends a payload with the UVC error bit set; uvcvideo marks the
frame bad and drops it, and a one-payload stub after it. For this bulk stream uvcvideo keeps 5
transfers of 32 KB queued (~1.5 ms of data) and queues each again only after its copy work has
run in a workqueue (an unbound, high-priority kernel worker). That work started within 0.004 ms
(median; p99 0.013 ms), but now and then waited 1.5-6 ms, and 105 of the 119 waits over 1 ms came
within 40 ms before an error bit: with no transfer queued, the camera's buffer overflows. During
the long waits the worker's CPU ran ordinary tasks: jtop (109 ms in all), systemd (22),
systemd-udevd (17), htop (8); the busy threads hardly at all.

With the jtop service stopped and jtop's and htop's windows closed, the same tests lost nothing:
0 of 4800 frames with 2, 4 and 6 busy threads (with the service alone still running: 1-2 in 40 s).
The pipeline node on zed_ros with its recorder and health logger then lost 1 frame in 81 s (0.04
%, against 0.5 % with jtop and htop running). A stand-in that wakes every 50 ms for 3 ms of plain
user-mode work cost 2 frames in 40 s, and loops re-reading sysfs or procfs files 0-5 in 20 s: what
jtop and htop do that holds the worker off is theirs (likely their system calls), not load as
such. Not pinned down further. The ZED SDK reads the camera through the same uvcvideo.

So: no jtop (service or window) or htop while recording or driving. A longer queue in uvcvideo
(more transfers: a rebuilt module) or a real-time worker would make it robust; neither is done.

## Not measured

- Other video modes and frame rates (the 0.31 ms is measured at HD720 at 30 Hz only).
- Other ZED models.
- Which frame the camera's own auto exposure takes effect on (zed-capture's writes: above). The
  code assumes the frame after the one it was set during; while the exposure changes, a frame's
  may be a frame off.
