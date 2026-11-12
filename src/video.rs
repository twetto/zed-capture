//! The camera's video through V4L2: YUYV, both eyes side by side, into a ring of grey frames.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::mem::size_of;
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use std::{io, ptr};

use crate::sys::{self, READ, WRITE, ioc};
use crate::sensor::{EYES, Sensors};
use crate::xu;

// ---- The kernel's V4L2 structs (videodev2.h), 64-bit layouts ----

const BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
const MEMORY_MMAP: u32 = 1;
const FIELD_ANY: u32 = 0;
const PIX_FMT_YUYV: u32 = u32::from_le_bytes(*b"YUYV");
const CAP_VIDEO_CAPTURE: u32 = 0x0000_0001;
const CAP_STREAMING: u32 = 0x0400_0000;
const CAP_DEVICE_CAPS: u32 = 0x8000_0000;
const BUF_FLAG_ERROR: u32 = 0x0000_0040;

#[repr(C)]
struct Capability {
    driver: [u8; 16],
    card: [u8; 32],
    bus_info: [u8; 32],
    version: u32,
    capabilities: u32,
    device_caps: u32,
    reserved: [u32; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct PixFormat {
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    bytesperline: u32,
    sizeimage: u32,
    colorspace: u32,
    priv_: u32,
    flags: u32,
    ycbcr_enc: u32,
    quantization: u32,
    xfer_func: u32,
}

#[repr(C, align(8))]
union FormatUnion {
    pix: PixFormat,
    raw: [u8; 200],
}

#[repr(C)]
struct Format {
    type_: u32,
    fmt: FormatUnion,
}

#[repr(C)]
struct RequestBuffers {
    count: u32,
    type_: u32,
    memory: u32,
    capabilities: u32,
    flags: u8,
    reserved: [u8; 3],
}

#[repr(C)]
struct Timecode {
    type_: u32,
    flags: u32,
    frames: u8,
    seconds: u8,
    minutes: u8,
    hours: u8,
    userbits: [u8; 4],
}

#[repr(C)]
struct Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    field: u32,
    timestamp: libc::timeval,
    timecode: Timecode,
    sequence: u32,
    memory: u32,
    m: u64, // union: the mmap offset (u32) in the low bytes
    length: u32,
    reserved2: u32,
    request_fd: i32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct Fract {
    numerator: u32,
    denominator: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct CaptureParm {
    capability: u32,
    capturemode: u32,
    timeperframe: Fract,
    extendedmode: u32,
    readbuffers: u32,
    reserved: [u32; 4],
}

#[repr(C)]
union ParmUnion {
    capture: CaptureParm,
    raw: [u8; 200],
}

#[repr(C)]
struct StreamParm {
    type_: u32,
    parm: ParmUnion,
}

const _: () = assert!(size_of::<Capability>() == 104);
const _: () = assert!(size_of::<Format>() == 208);
const _: () = assert!(size_of::<RequestBuffers>() == 20);
const _: () = assert!(size_of::<Buffer>() == 88);
const _: () = assert!(size_of::<StreamParm>() == 204);

const VIDIOC_QUERYCAP: libc::c_ulong = ioc(READ, b'V', 0, size_of::<Capability>());
const VIDIOC_S_FMT: libc::c_ulong = ioc(READ | WRITE, b'V', 5, size_of::<Format>());
const VIDIOC_REQBUFS: libc::c_ulong = ioc(READ | WRITE, b'V', 8, size_of::<RequestBuffers>());
const VIDIOC_QUERYBUF: libc::c_ulong = ioc(READ | WRITE, b'V', 9, size_of::<Buffer>());
const VIDIOC_QBUF: libc::c_ulong = ioc(READ | WRITE, b'V', 15, size_of::<Buffer>());
const VIDIOC_DQBUF: libc::c_ulong = ioc(READ | WRITE, b'V', 17, size_of::<Buffer>());
const VIDIOC_STREAMON: libc::c_ulong = ioc(WRITE, b'V', 18, size_of::<libc::c_int>());
const VIDIOC_STREAMOFF: libc::c_ulong = ioc(WRITE, b'V', 19, size_of::<libc::c_int>());
const VIDIOC_S_PARM: libc::c_ulong = ioc(READ | WRITE, b'V', 22, size_of::<StreamParm>());
const VIDIOC_G_CTRL: libc::c_ulong = ioc(READ | WRITE, b'V', 27, size_of::<Control>());
const VIDIOC_S_CTRL: libc::c_ulong = ioc(READ | WRITE, b'V', 28, size_of::<Control>());

/// `struct v4l2_control`.
#[repr(C)]
struct Control {
    id: u32,
    value: i32,
}

/// V4L2_CID_POWER_LINE_FREQUENCY: 0 off, 1 50 Hz, 2 60 Hz.
const CID_POWER_LINE_FREQUENCY: u32 = 0x0098_0918;

fn set_mains(fd: RawFd, hz: u32) -> io::Result<()> {
    let value = match hz {
        0 => 0,
        50 => 1,
        60 => 2,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("mains of {hz} Hz: 50, 60 or 0 (off)"),
            ));
        }
    };
    let mut c = Control {
        id: CID_POWER_LINE_FREQUENCY,
        value,
    };
    sys::ioctl(fd, VIDIOC_S_CTRL, &mut c).map_err(|e| context(e, "the power line frequency"))
}

/// No good frame for this long stops the capture with an error.
const STALL: Duration = Duration::from_secs(3);

fn zeroed<T>() -> T {
    // SAFETY: only used for the plain-data structs above, for which all zeros is valid.
    unsafe { std::mem::zeroed() }
}

fn context(e: io::Error, what: &str) -> io::Error {
    io::Error::new(e.kind(), format!("{what}: {e}"))
}

// ---- Public types ----

/// What to ask the camera for. The defaults are HD720 at 30 Hz.
#[derive(Clone, Debug)]
pub struct VideoConfig {
    /// One eye's width; the stream is twice as wide (2208, 1920, 1280 or 672).
    pub eye_width: u32,
    /// 1242, 1080, 720 or 376.
    pub height: u32,
    pub fps: u32,
    /// mmap buffers the driver fills; the capture thread returns each at once.
    pub buffers: u32,
    /// Frames kept for the reader. When it is full, the oldest is dropped and counted.
    pub ring: usize,
    /// The mains frequency of the lights (Hz: 50, 60, or 0 for none), set at open (see
    /// [`Video::set_mains_hz`]); None leaves the camera's (50 Hz at power-up).
    pub mains_hz: Option<u32>,
}

impl Default for VideoConfig {
    fn default() -> Self {
        Self {
            eye_width: 1280,
            height: 720,
            fps: 30,
            buffers: 8,
            ring: 8,
            mains_hz: None,
        }
    }
}

/// One stereo frame: left and right grey (the Y bytes), each `width × height`, row-major.
pub struct Frame {
    /// The kernel's frame counter: a gap is a frame the camera sent and the driver lost.
    pub sequence: u32,
    /// `CLOCK_MONOTONIC` nanoseconds, from the kernel (uvcvideo: the frame's start).
    pub timestamp_ns: u64,
    pub width: usize,
    pub height: usize,
    pub left: Vec<u8>,
    pub right: Vec<u8>,
    /// The frame's sync pulse on the host's clock, by the camera MCU's clock: the time of the IMU
    /// samples ([`ImuSample::time_ns`](crate::ImuSample::time_ns)). Set by
    /// [`Camera::next`](crate::Camera::next) once the frame marks are fitted, else None.
    pub sync_time_ns: Option<u64>,
    /// The MCU's count of this frame, with `sync_time_ns`.
    pub mcu_frame_count: Option<u32>,
    /// Each eye's exposure (ns; left, right), with `sync_time_ns`: the camera's auto exposure
    /// runs for each eye apart (LabOrin: the same to 123 lines, 4.9 ms, apart). None for an eye
    /// whose sensor has not answered for a second (the left's often does not):
    /// [`Camera::next`](crate::Camera::next) says which reading.
    pub exposure_ns: [Option<u32>; 2],
    /// The middle of the exposure of each eye's image's middle row (host ns), with its
    /// `exposure_ns`: see [`Camera::next`](crate::Camera::next). Rolling shutter: the other
    /// rows' come up to half a readout (HD720: 14.5 ms) before and after it.
    pub mid_exposure_ns: [Option<u64>; 2],
    pool: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl Drop for Frame {
    fn drop(&mut self) {
        if let Ok(mut pool) = self.pool.lock() {
            pool.push(std::mem::take(&mut self.left));
            pool.push(std::mem::take(&mut self.right));
        }
    }
}

/// Counts since [`Video::open`] or the last [`Video::reset_stats`].
#[derive(Clone, Debug, Default)]
pub struct VideoStats {
    /// Frames dequeued from the driver.
    pub frames: u64,
    /// Gaps in the kernel's sequence numbers. uvcvideo skips 2 numbers for each frame it
    /// loses (ZED 2i, L4T 36): `missing_by_time` is the count of frames.
    pub missing_by_sequence: u64,
    /// Frames missing by the kernel's timestamps (gaps of 1.5 periods or more).
    pub missing_by_time: u64,
    /// Frames dropped from a full ring: the reader was late.
    pub ring_drops: u64,
    /// Buffers with the error flag or the wrong size. A ZED 2i returns one per buffer right after
    /// STREAMON, stamped 0; they count neither as frames missing by time nor in the longest interval.
    pub bad_buffers: u64,
    /// The longest time between two frames' timestamps.
    pub longest_interval_ns: u64,
    /// Time the capture thread spent splitting YUYV into grey, in total.
    pub split_ns: u64,
}

struct Shared {
    queue: Mutex<VecDeque<Frame>>,
    ready: Condvar,
    stats: Mutex<VideoStats>,
    stop: AtomicBool,
    /// A good frame has come.
    started: AtomicBool,
    failed: Mutex<Option<String>>,
}

struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: the mapping is only read, by the capture thread, between DQBUF and QBUF.
unsafe impl Send for Mapping {}
unsafe impl Sync for Mapping {}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: mapped in Video::open with this length.
        unsafe { libc::munmap(self.ptr.cast(), self.len) };
    }
}

/// The video stream, read by a thread of its own from [`Video::open`] until dropped.
pub struct Video {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
    config: VideoConfig,
    fps: f64,
    buffers: u32,
    // Unmapped before the device is closed: fields drop in this order.
    _maps: Arc<Vec<Mapping>>,
    file: File,
    sensors: Sensors,
    /// The manual exposure set here, if any.
    manual: Mutex<Option<ManualExposure>>,
    /// The camera's auto exposure was turned off here: turned back on when dropped.
    turned_auto_off: AtomicBool,
}

/// A manual exposure as set ([`Video::set_manual_exposure`]).
#[derive(Clone, Copy, Debug)]
pub struct ManualExposure {
    /// In the sensor's lines…
    pub lines: u32,
    /// …and in ns, by the line time below.
    pub exposure_ns: f64,
    /// The frame period by the driver's frame rate over the sensor's lines per frame.
    pub line_ns: f64,
    /// The gain both sensors have (raw).
    pub gain: [Option<u16>; 2],
    /// `CLOCK_MONOTONIC` ns when both sensors had it.
    pub since_ns: u64,
}

/// The longest manual exposure: the frame's lines less these (800 at HD720, tried at 30 Hz with
/// no frame lost); longer, the sensor would stretch the frame.
const EXPOSURE_MARGIN_LINES: u32 = 32;

impl Video {
    /// Opens the V4L2 capture device (e.g. `/dev/video0`) and starts streaming. Turn the
    /// [`Imu`](crate::Imu)'s stream on only after the first frame
    /// ([`Video::wait_for_first_frame`]; [`Camera`](crate::Camera) does): turned on before, or
    /// even just after STREAMON, the video's buffers come back empty with the error flag and
    /// then stop coming (ZED 2i, LabOrin, 2026-11-08: 2 of 2 before, 1 of 5 just after). The
    /// capture thread stops with an error after 3 s without a good frame.
    pub fn open(path: &Path, config: VideoConfig) -> io::Result<Video> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)
            .map_err(|e| context(e, &format!("open {}", path.display())))?;
        let fd = file.as_raw_fd();

        let mut cap: Capability = zeroed();
        sys::ioctl(fd, VIDIOC_QUERYCAP, &mut cap).map_err(|e| context(e, "VIDIOC_QUERYCAP"))?;
        let caps = if cap.capabilities & CAP_DEVICE_CAPS != 0 {
            cap.device_caps
        } else {
            cap.capabilities
        };
        if caps & CAP_VIDEO_CAPTURE == 0 || caps & CAP_STREAMING == 0 {
            return Err(io::Error::other(format!(
                "{} is not a streaming capture device (the camera's metadata node?)",
                path.display()
            )));
        }

        let width = 2 * config.eye_width;
        let mut fmt: Format = zeroed();
        fmt.type_ = BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix = PixFormat {
            width,
            height: config.height,
            pixelformat: PIX_FMT_YUYV,
            field: FIELD_ANY,
            ..zeroed()
        };
        sys::ioctl(fd, VIDIOC_S_FMT, &mut fmt).map_err(|e| context(e, "VIDIOC_S_FMT"))?;
        // SAFETY: the driver filled in the pix member of a capture format.
        let pix = unsafe { fmt.fmt.pix };
        if (pix.width, pix.height, pix.pixelformat) != (width, config.height, PIX_FMT_YUYV) {
            return Err(io::Error::other(format!(
                "asked for YUYV {}x{}, the driver gave {:?} {}x{}",
                width,
                config.height,
                String::from_utf8_lossy(&pix.pixelformat.to_le_bytes()),
                pix.width,
                pix.height
            )));
        }

        let mut parm: StreamParm = zeroed();
        parm.type_ = BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture = CaptureParm {
            timeperframe: Fract {
                numerator: 1,
                denominator: config.fps,
            },
            ..zeroed()
        };
        sys::ioctl(fd, VIDIOC_S_PARM, &mut parm).map_err(|e| context(e, "VIDIOC_S_PARM"))?;
        if let Some(hz) = config.mains_hz {
            set_mains(fd, hz)?;
        }
        // SAFETY: the driver filled in the capture member.
        let tpf = unsafe { parm.parm.capture.timeperframe };
        let fps = if tpf.numerator > 0 {
            tpf.denominator as f64 / tpf.numerator as f64
        } else {
            config.fps as f64
        };

        let mut req: RequestBuffers = zeroed();
        req.count = config.buffers;
        req.type_ = BUF_TYPE_VIDEO_CAPTURE;
        req.memory = MEMORY_MMAP;
        sys::ioctl(fd, VIDIOC_REQBUFS, &mut req).map_err(|e| context(e, "VIDIOC_REQBUFS"))?;
        if req.count < 2 {
            return Err(io::Error::other(format!(
                "the driver gave {} buffers",
                req.count
            )));
        }

        let mut maps = Vec::with_capacity(req.count as usize);
        for index in 0..req.count {
            let mut buf: Buffer = zeroed();
            buf.index = index;
            buf.type_ = BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = MEMORY_MMAP;
            sys::ioctl(fd, VIDIOC_QUERYBUF, &mut buf).map_err(|e| context(e, "VIDIOC_QUERYBUF"))?;
            // SAFETY: the offset and length the driver gave for this buffer.
            let ptr = unsafe {
                libc::mmap(
                    ptr::null_mut(),
                    buf.length as usize,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    fd,
                    (buf.m & 0xffff_ffff) as libc::off_t,
                )
            };
            if ptr == libc::MAP_FAILED {
                return Err(context(io::Error::last_os_error(), "mmap"));
            }
            maps.push(Mapping {
                ptr: ptr.cast(),
                len: buf.length as usize,
            });
            sys::ioctl(fd, VIDIOC_QBUF, &mut buf).map_err(|e| context(e, "VIDIOC_QBUF"))?;
        }
        let maps = Arc::new(maps);

        let mut kind: libc::c_int = BUF_TYPE_VIDEO_CAPTURE as libc::c_int;
        sys::ioctl(fd, VIDIOC_STREAMON, &mut kind).map_err(|e| context(e, "VIDIOC_STREAMON"))?;

        let shared = Arc::new(Shared {
            queue: Mutex::new(VecDeque::with_capacity(config.ring + 1)),
            ready: Condvar::new(),
            stats: Mutex::new(VideoStats::default()),
            stop: AtomicBool::new(false),
            started: AtomicBool::new(false),
            failed: Mutex::new(None),
        });
        let capture = Capture {
            fd,
            maps: maps.clone(),
            shared: shared.clone(),
            eye_width: config.eye_width as usize,
            height: config.height as usize,
            ring: config.ring.max(1),
            period_ns: 1e9 / fps,
            pool: Arc::new(Mutex::new(Vec::new())),
        };
        let thread = std::thread::Builder::new()
            .name("zed-video".into())
            .spawn(move || capture.run())?;

        Ok(Video {
            shared,
            thread: Some(thread),
            config,
            fps,
            buffers: req.count,
            _maps: maps,
            sensors: Sensors::new(file.try_clone()?),
            file,
            manual: Mutex::new(None),
            turned_auto_off: AtomicBool::new(false),
        })
    }

    /// The oldest frame not yet taken, waiting up to `timeout` for one. Err once the capture
    /// thread has stopped on an error (the camera unplugged, say).
    pub fn next(&self, timeout: Duration) -> io::Result<Option<Frame>> {
        let deadline = Instant::now() + timeout;
        let mut queue = self.shared.queue.lock().unwrap();
        loop {
            if let Some(frame) = queue.pop_front() {
                return Ok(Some(frame));
            }
            if let Some(e) = self.shared.failed.lock().unwrap().as_ref() {
                return Err(io::Error::other(e.clone()));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            queue = self
                .shared
                .ready
                .wait_timeout(queue, deadline - now)
                .unwrap()
                .0;
        }
    }

    /// Waits up to `timeout` for the first good frame (without taking it). Err if the capture
    /// thread stopped or no frame came.
    pub fn wait_for_first_frame(&self, timeout: Duration) -> io::Result<()> {
        let deadline = Instant::now() + timeout;
        while !self.shared.started.load(Ordering::Acquire) {
            if let Some(e) = self.shared.failed.lock().unwrap().as_ref() {
                return Err(io::Error::other(e.clone()));
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "no frame within {:.1} s of starting the stream",
                        timeout.as_secs_f64()
                    ),
                ));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        Ok(())
    }

    /// Frames waiting in the ring.
    pub fn pending(&self) -> usize {
        self.shared.queue.lock().unwrap().len()
    }

    pub fn stats(&self) -> VideoStats {
        self.shared.stats.lock().unwrap().clone()
    }

    pub fn reset_stats(&self) {
        *self.shared.stats.lock().unwrap() = VideoStats::default();
    }

    pub fn config(&self) -> &VideoConfig {
        &self.config
    }

    /// The frame rate the driver agreed to.
    pub fn fps(&self) -> f64 {
        self.fps
    }

    /// The mmap buffers the driver gave.
    pub fn buffers(&self) -> u32 {
        self.buffers
    }

    /// Has image sensor `eye` send its frame-start pulse (VSYNC) out to the camera's MCU. The
    /// MCU marks the IMU sample of each of the left sensor's pulses
    /// ([`ImuSample::frame_sync`](crate::ImuSample::frame_sync)) and counts them: each frame's
    /// time on the IMU's clock (LabOrin's ZED 2i, 2026-11-12: with the left's output off the
    /// marks stop, with the right's they go on). Off when the camera powers up; the ZED SDK
    /// turns both on, and they stay on until the camera is unplugged. Returns whether it was
    /// already on. Each read is checked, and a sensor that does not answer (see
    /// [`crate::sensor`]) asked again for up to `wait`.
    pub fn enable_frame_sync(&self, eye: usize, wait: Duration) -> io::Result<bool> {
        let what = |e: io::Error| context(e, &format!("the {} sensor's sync output", EYES[eye]));
        let s = &self.sensors;
        let v = s.read_checked(eye, xu::REG_SYNC_OUTPUT, wait).map_err(what)?;
        if v & xu::SYNC_OUTPUT_ON != 0 {
            return Ok(true);
        }
        s.write(eye, xu::REG_SYNC_OUTPUT, v | xu::SYNC_OUTPUT_ON).map_err(what)?;
        let back = s.read_checked(eye, xu::REG_SYNC_OUTPUT, wait).map_err(what)?;
        if back & xu::SYNC_OUTPUT_ON == 0 {
            return Err(io::Error::other(format!(
                "the {} sensor's sync output: register {:#06x} reads {back:#04x} after writing {:#04x}",
                EYES[eye],
                xu::REG_SYNC_OUTPUT,
                v | xu::SYNC_OUTPUT_ON
            )));
        }
        Ok(false)
    }

    /// The image sensors' registers (a handle other threads can keep).
    pub fn sensors(&self) -> &Sensors {
        &self.sensors
    }

    /// Sets the mains frequency of the lights (50 or 60 Hz, or 0 for none): the camera's auto
    /// exposure keeps to whole periods of their flicker (twice the mains). LabOrin's ZED 2i at
    /// night: 747 lines (29.93 ms, 3 × 10 ms) at 50 Hz, 621 (24.88 ms, 3 × 8.33 ms) at 60 Hz.
    /// The camera powers up at 50 Hz; Taiwan's mains are 60 Hz.
    pub fn set_mains_hz(&self, hz: u32) -> io::Result<()> {
        set_mains(self.file.as_raw_fd(), hz)
    }

    /// The camera's mains frequency (Hz; 0: none).
    pub fn mains_hz(&self) -> io::Result<u32> {
        let mut c = Control {
            id: CID_POWER_LINE_FREQUENCY,
            value: 0,
        };
        sys::ioctl(self.file.as_raw_fd(), VIDIOC_G_CTRL, &mut c)
            .map_err(|e| context(e, "the power line frequency"))?;
        Ok(match c.value {
            1 => 50,
            2 => 60,
            _ => 0,
        })
    }

    /// Whether the camera's auto exposure and gain is on, per eye.
    pub fn auto_exposure(&self) -> io::Result<[bool; 2]> {
        Ok([self.sensors.auto_exposure(0)?, self.sensors.auto_exposure(1)?])
    }

    /// Turns the camera's auto exposure and gain back on, both eyes.
    pub fn set_auto_exposure(&self) -> io::Result<()> {
        self.sensors.set_auto_exposure(0, true)?;
        self.sensors.set_auto_exposure(1, true)?;
        *self.manual.lock().unwrap() = None;
        self.turned_auto_off.store(false, Ordering::Relaxed);
        Ok(())
    }

    /// The manual exposure set by [`Video::set_manual_exposure`], until the auto exposure is
    /// back on.
    pub fn manual_exposure(&self) -> Option<ManualExposure> {
        *self.manual.lock().unwrap()
    }

    /// A manual exposure, the same on both sensors, as zed-open-capture sets it: the camera's
    /// auto exposure and gain off (each eye's ISP), 5 frames' wait if it was on (the ISP can
    /// still apply an update it had pending), then `exposure` in whole lines (at least 1, at
    /// most the frame's lines less 32) and `gain` (raw, in [`crate::sensor::GAIN_RANGES`]; None
    /// keeps the gain the auto exposure left) written to each sensor and checked. Each frame's
    /// exposure ([`Frame::exposure_ns`]) then is this. The auto exposure comes back on when the
    /// video is dropped, or with [`Video::set_auto_exposure`]; if the process dies first, it
    /// stays off until the camera is unplugged.
    pub fn set_manual_exposure(
        &self,
        exposure: Duration,
        gain: Option<u16>,
    ) -> io::Result<ManualExposure> {
        let wait = Duration::from_secs(2);
        let timing = self
            .sensors
            .timing(1, wait)
            .or_else(|_| self.sensors.timing(0, wait))?;
        let line_ns = 1e9 / self.fps / timing.frame_length as f64;
        let max = (timing.frame_length as u32).saturating_sub(EXPOSURE_MARGIN_LINES);
        let lines = ((exposure.as_nanos() as f64 / line_ns).round() as u32).clamp(1, max);
        let was_on = self.auto_exposure()?;
        *self.manual.lock().unwrap() = None;
        self.turned_auto_off.store(true, Ordering::Relaxed);
        for eye in 0..2 {
            self.sensors.set_auto_exposure(eye, false)?;
        }
        if was_on.contains(&true) {
            std::thread::sleep(Duration::from_secs_f64(5.0 / self.fps));
        }
        for (eye, name) in EYES.iter().enumerate() {
            let what = |e: io::Error| context(e, &format!("the {name} sensor's exposure"));
            self.sensors
                .set_exposure_lines(eye, lines, max, wait)
                .map_err(what)?;
            if let Some(g) = gain {
                self.sensors.set_gain(eye, g, wait).map_err(what)?;
            }
        }
        let m = ManualExposure {
            lines,
            exposure_ns: lines as f64 * line_ns,
            line_ns,
            gain: [self.sensors.gain(0)?, self.sensors.gain(1)?],
            since_ns: crate::monotonic_ns(),
        };
        *self.manual.lock().unwrap() = Some(m);
        Ok(m)
    }
}

impl Drop for Video {
    fn drop(&mut self) {
        if self.turned_auto_off.load(Ordering::Relaxed) {
            let _ = self.set_auto_exposure();
        }
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        let mut kind: libc::c_int = BUF_TYPE_VIDEO_CAPTURE as libc::c_int;
        let _ = sys::ioctl(self.file.as_raw_fd(), VIDIOC_STREAMOFF, &mut kind);
    }
}

struct Capture {
    fd: RawFd,
    maps: Arc<Vec<Mapping>>,
    shared: Arc<Shared>,
    eye_width: usize,
    height: usize,
    ring: usize,
    period_ns: f64,
    pool: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl Capture {
    fn run(self) {
        if let Err(e) = self.capture() {
            *self.shared.failed.lock().unwrap() = Some(format!("video capture stopped: {e}"));
            self.shared.ready.notify_all();
        }
    }

    fn capture(&self) -> io::Result<()> {
        let frame_bytes = self.eye_width * 2 * 2 * self.height;
        let eye_bytes = self.eye_width * self.height;
        // The previous buffer's sequence number, and the previous timestamp. The buffers that come
        // back bad right after STREAMON (one per buffer on a ZED 2i) are stamped 0: no time.
        let mut last_sequence: Option<u32> = None;
        let mut last_time: Option<u64> = None;
        let mut last_good = Instant::now();
        while !self.shared.stop.load(Ordering::Relaxed) {
            if last_good.elapsed() > STALL {
                return Err(io::Error::other(format!(
                    "no frame for {} s (the IMU's stream turned on before the first frame stops the video)",
                    STALL.as_secs()
                )));
            }
            if !sys::wait_readable(self.fd, 200)? {
                continue;
            }
            let mut buf: Buffer = zeroed();
            buf.type_ = BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = MEMORY_MMAP;
            match sys::ioctl(self.fd, VIDIOC_DQBUF, &mut buf) {
                Ok(()) => {}
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(context(e, "VIDIOC_DQBUF")),
            }
            // Read before QBUF, which writes the struct back.
            let sequence = buf.sequence;
            let timestamp_ns =
                buf.timestamp.tv_sec as u64 * 1_000_000_000 + buf.timestamp.tv_usec as u64 * 1000;
            let good = buf.flags & BUF_FLAG_ERROR == 0
                && buf.bytesused as usize == frame_bytes
                && (buf.index as usize) < self.maps.len();

            let mut split_ns = 0;
            let frame = if good {
                let t0 = Instant::now();
                let (mut left, mut right) = {
                    let mut pool = self.pool.lock().unwrap();
                    (
                        pool.pop().unwrap_or_default(),
                        pool.pop().unwrap_or_default(),
                    )
                };
                left.resize(eye_bytes, 0);
                right.resize(eye_bytes, 0);
                let map = &self.maps[buf.index as usize];
                // SAFETY: the driver filled this buffer and hands it back only after QBUF below.
                let src = unsafe { std::slice::from_raw_parts(map.ptr, frame_bytes) };
                split_yuyv(src, self.eye_width, self.height, &mut left, &mut right);
                split_ns = t0.elapsed().as_nanos() as u64;
                Some(Frame {
                    sequence,
                    timestamp_ns,
                    width: self.eye_width,
                    height: self.height,
                    left,
                    right,
                    sync_time_ns: None,
                    mcu_frame_count: None,
                    exposure_ns: [None; 2],
                    mid_exposure_ns: [None; 2],
                    pool: self.pool.clone(),
                })
            } else {
                None
            };
            sys::ioctl(self.fd, VIDIOC_QBUF, &mut buf).map_err(|e| context(e, "VIDIOC_QBUF"))?;

            {
                let mut s = self.shared.stats.lock().unwrap();
                s.frames += 1;
                s.split_ns += split_ns;
                if !good {
                    s.bad_buffers += 1;
                }
                if let Some(seq) = last_sequence {
                    s.missing_by_sequence += sequence.wrapping_sub(seq).saturating_sub(1) as u64;
                }
                if let (Some(t), true) = (last_time, timestamp_ns > 0) {
                    let dt = timestamp_ns.saturating_sub(t);
                    s.longest_interval_ns = s.longest_interval_ns.max(dt);
                    let periods = (dt as f64 / self.period_ns).round() as u64;
                    s.missing_by_time += periods.saturating_sub(1);
                }
            }
            last_sequence = Some(sequence);
            if timestamp_ns > 0 {
                last_time = Some(timestamp_ns);
            }
            if good {
                last_good = Instant::now();
                self.shared.started.store(true, Ordering::Release);
            }

            if let Some(frame) = frame {
                let dropped = {
                    let mut queue = self.shared.queue.lock().unwrap();
                    queue.push_back(frame);
                    if queue.len() > self.ring {
                        queue.pop_front()
                    } else {
                        None
                    }
                };
                if dropped.is_some() {
                    self.shared.stats.lock().unwrap().ring_drops += 1;
                }
                self.shared.ready.notify_one();
            }
        }
        Ok(())
    }
}

/// Splits one side-by-side YUYV frame into left and right grey (each row: left eye's pixels,
/// then the right eye's; Y in the even bytes).
pub fn split_yuyv(src: &[u8], eye_width: usize, height: usize, left: &mut [u8], right: &mut [u8]) {
    let row = eye_width * 4;
    assert!(
        src.len() >= row * height
            && left.len() >= eye_width * height
            && right.len() >= eye_width * height
    );
    for y in 0..height {
        let (l, r) = src[y * row..(y + 1) * row].split_at(eye_width * 2);
        for (d, p) in left[y * eye_width..(y + 1) * eye_width]
            .iter_mut()
            .zip(l.as_chunks::<2>().0)
        {
            *d = p[0];
        }
        for (d, p) in right[y * eye_width..(y + 1) * eye_width]
            .iter_mut()
            .zip(r.as_chunks::<2>().0)
        {
            *d = p[0];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_numbers_match_the_kernel_headers() {
        assert_eq!(VIDIOC_QUERYCAP, 0x8068_5600);
        assert_eq!(VIDIOC_S_FMT, 0xc0d0_5605);
        assert_eq!(VIDIOC_REQBUFS, 0xc014_5608);
        assert_eq!(VIDIOC_QUERYBUF, 0xc058_5609);
        assert_eq!(VIDIOC_QBUF, 0xc058_560f);
        assert_eq!(VIDIOC_DQBUF, 0xc058_5611);
        assert_eq!(VIDIOC_STREAMON, 0x4004_5612);
        assert_eq!(VIDIOC_STREAMOFF, 0x4004_5613);
        assert_eq!(VIDIOC_S_PARM, 0xc0cc_5616);
    }

    #[test]
    fn split_takes_each_eyes_y_bytes() {
        let (w, h) = (3, 2);
        // Row y: left Y = 10y+x, right Y = 100+10y+x; U/V bytes 255.
        let mut src = Vec::new();
        for y in 0..h {
            for eye in [0u8, 100] {
                for x in 0..w {
                    src.extend([eye + 10 * y as u8 + x as u8, 255]);
                }
            }
        }
        let (mut left, mut right) = (vec![0; w * h], vec![0; w * h]);
        split_yuyv(&src, w, h, &mut left, &mut right);
        assert_eq!(left, [0, 1, 2, 10, 11, 12]);
        assert_eq!(right, [100, 101, 102, 110, 111, 112]);
    }
}
