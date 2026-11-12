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

### Mains frequency

The camera powers up with its anti-flicker at 50 Hz (V4L2's power line frequency control). At
night under the lab's lights the auto exposure held 747 lines (29.93 ms, three 10 ms flicker
periods) at 50 Hz and 621 lines (24.88 ms, three 8.33 ms periods) at 60 Hz, Taiwan's mains.

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

## Not measured

- Other video modes and frame rates (the 0.31 ms is measured at HD720 at 30 Hz only).
- Other ZED models.
- Which frame a new auto exposure takes effect on. The code assumes the frame after the one it
  was set during; while the exposure changes, a frame's may be a frame off.
