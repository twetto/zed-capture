//! zed-capture's Python bindings: `zed_capture.Camera`, its `Frame`s and the clock helpers.
//!
//! A `Camera` opens the one ZED attached (or the nodes given) and waits for the frame marks, so
//! its frames come with their pulse and each eye's mid-exposure stamp. `next()` waits without
//! the GIL; the capture threads run on regardless, so a late caller finds its frames in the ring
//! (`VideoConfig::ring`) instead of losing them. After 3 s without a frame (the camera went
//! silent, or was unplugged) `next()` opens the camera again by itself, as `zed_record` does, and
//! counts the stall.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use numpy::ndarray::Array2;
use numpy::{PyArray1, PyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::{PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use zed_capture::{
    AutoExposureConfig, Camera, ExposureSource, GradientConfig, ImuSample, VideoConfig,
};

/// After this long without a frame the camera is opened again (`zed_record`'s).
const STALL: Duration = Duration::from_secs(3);
/// While the camera is gone, opening it is tried at most this often.
const RETRY: Duration = Duration::from_secs(1);
/// `next()` gives the GIL back this often to check for signals (Ctrl-C).
const SLICE: Duration = Duration::from_millis(200);

/// What a camera is opened with, every time.
#[derive(Clone)]
struct Options {
    config: VideoConfig,
    /// Manual exposure: ms and raw gain.
    manual: Option<(f64, Option<u16>)>,
    imu: bool,
    video: Option<PathBuf>,
    hid: Option<PathBuf>,
    open_timeout: Duration,
}

/// Opens the camera and waits for its frames to carry their pulse (the frame marks fitted:
/// about 3 s); the frames before are dropped. Without the IMU there are no pulses: no wait.
fn open(o: &Options) -> Result<(Camera, Option<String>), String> {
    let (video, hid, serial) = match &o.video {
        Some(v) => (v.clone(), o.hid.clone(), None),
        None => {
            let d = zed_capture::find().map_err(|e| format!("finding the ZED: {e}"))?;
            (d.video, Some(d.hid), d.serial)
        }
    };
    let hid = hid.filter(|_| o.imu);
    let camera = Camera::open(&video, hid.as_deref(), o.config.clone())
        .map_err(|e| format!("opening {}: {e}", video.display()))?;
    if let Some((ms, gain)) = o.manual {
        camera
            .video()
            .set_manual_exposure(Duration::from_secs_f64(ms / 1e3), gain)
            .map_err(|e| format!("the manual exposure: {e}"))?;
    }
    if camera.imu().is_some() {
        let start = Instant::now();
        loop {
            if start.elapsed() > o.open_timeout {
                return Err(format!(
                    "no frame sync after {:.0} s ({})",
                    o.open_timeout.as_secs_f64(),
                    match camera.frame_sync() {
                        Ok(_) => "the marks were turned on".to_string(),
                        Err(e) => format!("the marks could not be turned on: {e}"),
                    }
                ));
            }
            match camera.next(Duration::from_millis(500)) {
                Ok(Some((f, _))) if f.sync_time_ns.is_some() && camera.readout().is_some() => {
                    break;
                }
                Ok(_) => {}
                Err(e) => return Err(format!("the camera: {e}")),
            }
        }
    }
    // The counts are of the frames the caller gets: not of the ones dropped while opening (the
    // ring fills while the sensors are set up)
    camera.video().reset_stats();
    if let Some(imu) = camera.imu() {
        imu.reset_stats();
    }
    Ok((camera, serial))
}

/// Counts over every time the camera was opened.
#[derive(Default)]
struct Totals {
    frames: u64,
    lost: u64,
    ring_drops: u64,
    imu_samples: u64,
    imu_missing: u64,
}

impl Totals {
    fn add(&mut self, camera: &Camera) {
        let v = camera.video().stats();
        self.frames += v.frames;
        self.lost += v.missing_by_time;
        self.ring_drops += v.ring_drops;
        if let Some(imu) = camera.imu() {
            let s = imu.stats();
            self.imu_samples += s.samples;
            self.imu_missing += s.missing;
        }
    }
}

struct State {
    camera: Option<Arc<Camera>>,
    serial: Option<String>,
    /// When a frame came last (or the camera was opened).
    last_frame: Instant,
    /// When opening was last tried, while the camera is gone.
    last_try: Option<Instant>,
    stalls: Vec<String>,
    open_error: Option<String>,
    totals: Totals,
    closed: bool,
}

/// The ZED, opened: its stereo frames, each with its timing and the IMU samples up to it.
///
/// ``exposure``: ``"camera"`` (the camera's own auto exposure, each eye apart; only the right
/// sensor's exposure is read), ``"active"`` (zed-capture's own, mean mode: one exposure for
/// both eyes, each its own gain, every frame's setting known), ``"gradient"`` (zed-capture's,
/// gradient mode) or ``"manual"`` (``exposure_ms`` and the raw ``gain``). ``mains_hz``: the
/// lights' mains (50, 60; 0 for none), for the anti-flicker; the camera powers up at 50.
/// ``video``/``hid``: the V4L2 and hidraw nodes, else the one ZED attached. ``imu=False``: no
/// IMU, so no frame pulses nor exposure stamps. Opening waits up to ``open_timeout`` s for the
/// frame sync. Raises OSError when the camera cannot be opened, ValueError for bad settings.
#[pyclass(name = "Camera", frozen)]
struct PyCamera {
    options: Options,
    exposure: &'static str,
    state: Mutex<State>,
}

fn os_error(e: impl std::fmt::Display) -> PyErr {
    PyOSError::new_err(e.to_string())
}

impl PyCamera {
    /// The camera to read from: the open one, or, after a stall or an error, a new one (None
    /// while it cannot be opened). Called without the GIL.
    fn camera(&self, error: Option<String>) -> Option<Arc<Camera>> {
        let mut s = self.state.lock().unwrap();
        let stalled = error.is_some() || s.last_frame.elapsed() > STALL;
        if let Some(c) = &s.camera {
            if !stalled {
                return Some(c.clone());
            }
            let reason = error.unwrap_or_else(|| {
                format!("no frame for {:.1} s", s.last_frame.elapsed().as_secs_f64())
            });
            s.stalls.push(reason);
            if let Some(old) = s.camera.take() {
                s.totals.add(&old);
                // The last reference (but a stats() call's): stops its threads, gives the
                // camera its auto exposure back
                drop(old);
            }
        }
        if s.last_try.is_some_and(|t| t.elapsed() < RETRY) {
            return None;
        }
        s.last_try = Some(Instant::now());
        drop(s);
        let opened = open(&self.options);
        let mut s = self.state.lock().unwrap();
        match opened {
            Ok((camera, serial)) => {
                let camera = Arc::new(camera);
                if s.closed {
                    return None;
                }
                s.camera = Some(camera.clone());
                s.serial = serial.or(s.serial.take());
                s.last_frame = Instant::now();
                s.last_try = None;
                s.open_error = None;
                Some(camera)
            }
            Err(e) => {
                s.open_error = Some(e);
                None
            }
        }
    }

    fn current(&self) -> Option<Arc<Camera>> {
        self.state.lock().unwrap().camera.clone()
    }
}

#[pymethods]
impl PyCamera {
    #[new]
    #[pyo3(signature = (
        exposure="camera",
        mains_hz=None,
        exposure_ms=None,
        gain=None,
        read_exposure=true,
        read_left_exposure=false,
        imu=true,
        video=None,
        hid=None,
        open_timeout=15.0,
    ))]
    fn new(
        py: Python<'_>,
        exposure: &str,
        mains_hz: Option<u32>,
        exposure_ms: Option<f64>,
        gain: Option<u16>,
        read_exposure: bool,
        read_left_exposure: bool,
        imu: bool,
        video: Option<PathBuf>,
        hid: Option<PathBuf>,
        open_timeout: f64,
    ) -> PyResult<Self> {
        let exposure: &'static str = match exposure {
            "camera" => "camera",
            "active" => "active",
            "gradient" => "gradient",
            "manual" => "manual",
            other => {
                return Err(PyValueError::new_err(format!(
                    "exposure '{other}': expected camera, active, gradient or manual"
                )));
            }
        };
        if exposure == "manual" {
            match exposure_ms {
                Some(ms) if ms > 0.0 && ms.is_finite() => {}
                _ => {
                    return Err(PyValueError::new_err(
                        "exposure 'manual' needs exposure_ms > 0",
                    ));
                }
            }
        } else if exposure_ms.is_some() || gain.is_some() {
            return Err(PyValueError::new_err(
                "exposure_ms and gain are for exposure='manual'",
            ));
        }
        if let Some(g) = gain
            && zed_capture::gain_multiplier(g).is_none()
        {
            return Err(PyValueError::new_err(format!(
                "gain {g}: not one of the sensor's raw gains (zed_capture.gain_multiplier)"
            )));
        }
        if let Some(h) = mains_hz
            && ![0, 50, 60].contains(&h)
        {
            return Err(PyValueError::new_err(format!(
                "mains_hz {h}: expected 50, 60 or 0"
            )));
        }
        if !(open_timeout > 0.0 && open_timeout.is_finite()) {
            return Err(PyValueError::new_err("open_timeout must be > 0"));
        }
        if video.is_none() && hid.is_some() {
            return Err(PyValueError::new_err("hid is given with video"));
        }
        // Past one flicker period of the lights, whole periods: twice the mains
        let active = matches!(exposure, "active" | "gradient").then(|| AutoExposureConfig {
            flicker_hz: mains_hz.filter(|&h| h > 0).map(|h| 2.0 * h as f64),
            gradient: (exposure == "gradient").then(GradientConfig::default),
            ..Default::default()
        });
        let options = Options {
            config: VideoConfig {
                mains_hz,
                read_exposure,
                read_left_exposure,
                active_exposure: active,
                ..Default::default()
            },
            manual: exposure_ms.map(|ms| (ms, gain)),
            imu,
            video,
            hid,
            open_timeout: Duration::from_secs_f64(open_timeout),
        };
        let (camera, serial) = py.allow_threads(|| open(&options)).map_err(os_error)?;
        Ok(PyCamera {
            options,
            exposure,
            state: Mutex::new(State {
                camera: Some(Arc::new(camera)),
                serial,
                last_frame: Instant::now(),
                last_try: None,
                stalls: Vec::new(),
                open_error: None,
                totals: Totals::default(),
                closed: false,
            }),
        })
    }

    /// The oldest frame not yet taken, waiting up to ``timeout`` s; None if none came. While the
    /// camera is stalled or gone it is opened again here (each time at most once a second), and
    /// None is returned until it is back.
    #[pyo3(signature = (timeout=1.0))]
    fn next(&self, py: Python<'_>, timeout: f64) -> PyResult<Option<PyFrame>> {
        if !(timeout >= 0.0 && timeout.is_finite()) {
            return Err(PyValueError::new_err("timeout must be >= 0"));
        }
        if self.state.lock().unwrap().closed {
            return Err(PyValueError::new_err("the camera is closed"));
        }
        let deadline = Instant::now() + Duration::from_secs_f64(timeout);
        let mut error = None;
        loop {
            let slice = deadline
                .saturating_duration_since(Instant::now())
                .min(SLICE);
            let got = py.allow_threads(|| match self.camera(error.take()) {
                Some(c) => c.next(slice).map_err(|e| format!("the camera: {e}")),
                None => {
                    std::thread::sleep(slice);
                    Ok(None)
                }
            });
            match got {
                Ok(Some((frame, imu))) => {
                    self.state.lock().unwrap().last_frame = Instant::now();
                    return Ok(Some(PyFrame::new(py, frame, imu)));
                }
                Ok(None) => {}
                Err(e) => error = Some(e),
            }
            py.check_signals()?;
            if Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }

    /// Counts since the camera was first opened (from its frame sync on, each time): frames,
    /// frames lost (the kernel's), frames
    /// dropped from a full ring (the reader was late), IMU samples and those missing, stalls
    /// (and why), the last error opening it again; zed-capture's own auto exposure's counts and
    /// its tone-curve check (since the camera was last opened).
    fn stats<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let (camera, mut totals, stalls, open_error) = {
            let s = self.state.lock().unwrap();
            let t = &s.totals;
            (
                s.camera.clone(),
                Totals {
                    frames: t.frames,
                    lost: t.lost,
                    ring_drops: t.ring_drops,
                    imu_samples: t.imu_samples,
                    imu_missing: t.imu_missing,
                },
                s.stalls.clone(),
                s.open_error.clone(),
            )
        };
        let d = PyDict::new(py);
        let active = camera.as_ref().and_then(|c| {
            totals.add(c);
            c.active_exposure_stats()
        });
        d.set_item("frames", totals.frames)?;
        d.set_item("frames_lost", totals.lost)?;
        d.set_item("ring_drops", totals.ring_drops)?;
        d.set_item("imu_samples", totals.imu_samples)?;
        d.set_item("imu_missing", totals.imu_missing)?;
        d.set_item("stalls", stalls.len())?;
        d.set_item("stall_reasons", stalls)?;
        d.set_item("open_error", open_error)?;
        match active {
            Some(a) => {
                let x = PyDict::new(py);
                x.set_item("steps", a.steps)?;
                x.set_item("writes", a.writes)?;
                x.set_item("unverified", (a.unverified[0], a.unverified[1]))?;
                x.set_item("write_errors", a.write_errors)?;
                x.set_item("last_error", a.last_error)?;
                x.set_item(
                    "controller_ms_mean",
                    a.step_ns_total as f64 / a.steps.max(1) as f64 / 1e6,
                )?;
                x.set_item("controller_ms_max", a.step_ns_max as f64 / 1e6)?;
                d.set_item("active_exposure", x)?;
                let c = PyDict::new(py);
                c.set_item("pixels", a.prediction.pixels)?;
                if let Some([curve, linear, unchanged]) = a.prediction.means() {
                    c.set_item("curve", curve)?;
                    c.set_item("linear", linear)?;
                    c.set_item("unchanged", unchanged)?;
                }
                d.set_item("tone_curve_check", c)?;
            }
            None => {
                d.set_item("active_exposure", py.None())?;
                d.set_item("tone_curve_check", py.None())?;
            }
        }
        Ok(d)
    }

    /// Stops the capture and gives the camera its own auto exposure back. Later ``next()``
    /// calls raise.
    fn close(&self, py: Python<'_>) {
        let camera = {
            let mut s = self.state.lock().unwrap();
            s.closed = true;
            s.camera.take()
        };
        py.allow_threads(|| drop(camera));
    }

    fn __enter__(slf: Py<Self>) -> Py<Self> {
        slf
    }

    #[pyo3(signature = (*_args))]
    fn __exit__(&self, py: Python<'_>, _args: &Bound<'_, pyo3::types::PyTuple>) -> bool {
        self.close(py);
        false
    }

    /// The camera's serial number (from its IMU's USB interface), if known.
    #[getter]
    fn serial(&self) -> Option<String> {
        self.state.lock().unwrap().serial.clone()
    }

    /// The factory calibration the ZED SDK downloaded for this camera
    /// (``/usr/local/zed/settings/SN<serial>.conf``), if it is there: for
    /// ``rudolf_v.StereoRectification.from_zed_conf``.
    #[getter]
    fn calibration_file(&self) -> Option<String> {
        let serial = self.serial()?;
        let path = format!("/usr/local/zed/settings/SN{serial}.conf");
        Path::new(&path).is_file().then_some(path)
    }

    #[getter]
    fn width(&self) -> u32 {
        self.options.config.eye_width
    }

    #[getter]
    fn height(&self) -> u32 {
        self.options.config.height
    }

    #[getter]
    fn fps(&self) -> u32 {
        self.options.config.fps
    }

    /// ``"camera"``, ``"active"``, ``"gradient"`` or ``"manual"``.
    #[getter]
    fn exposure(&self) -> &'static str {
        self.exposure
    }

    /// One sensor line (ns), once the frame marks are fitted: row r of an image is read
    /// ``(r - (height - 1) / 2) * line_ns`` after its middle row (rolling shutter).
    #[getter]
    fn line_ns(&self) -> Option<f64> {
        self.current()?.readout().map(|r| r.line_ns)
    }

    fn __repr__(&self) -> String {
        let s = self.state.lock().unwrap();
        format!(
            "Camera(serial={}, exposure='{}', {}x{} at {} Hz, {})",
            s.serial.as_deref().unwrap_or("?"),
            self.exposure,
            self.options.config.eye_width,
            self.options.config.height,
            self.options.config.fps,
            if s.closed {
                "closed"
            } else if s.camera.is_some() {
                "open"
            } else {
                "reopening"
            }
        )
    }
}

fn source_name(s: ExposureSource) -> &'static str {
    match s {
        ExposureSource::None => "none",
        ExposureSource::Read => "read",
        ExposureSource::Manual => "manual",
        ExposureSource::Active => "active",
        ExposureSource::ActiveUnchecked => "active_unchecked",
    }
}

/// One stereo frame: both eyes' grey images (uint8, height × width, unrectified), the frame's
/// timing and the IMU samples since the previous frame, all on ``CLOCK_MONOTONIC``
/// (``zed_capture.realtime_offset_ns()`` turns it into the system clock).
///
/// ``stamp_ns``: the left image's time, the middle of its middle row's exposure; with the camera's
/// own auto exposure the left's exposure is not read, so the right's (close to it). Which one is
/// in ``stamp_source`` (``"left"``, ``"right"``, or ``"kernel"``: the kernel's frame stamp,
/// before the frame marks are fitted or without the IMU). ``mid_exposure_ns``,
/// ``exposure_ns``, ``gain`` (raw: ``zed_capture.gain_multiplier``) and ``exposure_source`` are
/// per eye (left, right), None where unknown. ``pulse_ns``: the frame's sync pulse.
/// ``imu_time_ns`` (int64, N), ``gyro`` (rad/s) and ``accel`` (m/s²) (float64, N × 3): every IMU
/// sample up to this frame's pulse, in the IMU's own axes.
#[pyclass(name = "Frame", frozen)]
struct PyFrame {
    #[pyo3(get)]
    left: Py<PyArray2<u8>>,
    #[pyo3(get)]
    right: Py<PyArray2<u8>>,
    #[pyo3(get)]
    stamp_ns: u64,
    #[pyo3(get)]
    stamp_source: &'static str,
    #[pyo3(get)]
    mid_exposure_ns: (Option<u64>, Option<u64>),
    #[pyo3(get)]
    exposure_ns: (Option<u32>, Option<u32>),
    #[pyo3(get)]
    gain: (Option<u16>, Option<u16>),
    #[pyo3(get)]
    exposure_source: (&'static str, &'static str),
    #[pyo3(get)]
    pulse_ns: Option<u64>,
    #[pyo3(get)]
    kernel_ns: u64,
    #[pyo3(get)]
    sequence: u32,
    #[pyo3(get)]
    mcu_frame_count: Option<u32>,
    #[pyo3(get)]
    imu_time_ns: Py<PyArray1<i64>>,
    #[pyo3(get)]
    gyro: Py<PyArray2<f64>>,
    #[pyo3(get)]
    accel: Py<PyArray2<f64>>,
}

/// A `rows × 3` float64 array of each sample's vector.
fn vectors<'py>(
    py: Python<'py>,
    samples: &[ImuSample],
    f: impl Fn(&ImuSample) -> [f32; 3],
) -> Bound<'py, PyArray2<f64>> {
    let data = samples
        .iter()
        .flat_map(|s| f(s).map(|v| v as f64))
        .collect::<Vec<f64>>();
    PyArray2::from_owned_array(
        py,
        Array2::from_shape_vec((samples.len(), 3), data).unwrap(),
    )
}

impl PyFrame {
    fn new(py: Python<'_>, mut f: zed_capture::Frame, imu: Vec<ImuSample>) -> PyFrame {
        let (w, h) = (f.width, f.height);
        // The frame's buffers become the arrays: no copy
        let image = |data: Vec<u8>| {
            PyArray2::from_owned_array(py, Array2::from_shape_vec((h, w), data).unwrap()).unbind()
        };
        let left = image(std::mem::take(&mut f.left));
        let right = image(std::mem::take(&mut f.right));
        let (stamp_ns, stamp_source) = match f.mid_exposure_ns {
            [Some(t), _] => (t, "left"),
            [None, Some(t)] => (t, "right"),
            [None, None] => (f.timestamp_ns, "kernel"),
        };
        PyFrame {
            left,
            right,
            stamp_ns,
            stamp_source,
            mid_exposure_ns: f.mid_exposure_ns.into(),
            exposure_ns: f.exposure_ns.into(),
            gain: f.gain.into(),
            exposure_source: f.exposure_source.map(source_name).into(),
            pulse_ns: f.sync_time_ns,
            kernel_ns: f.timestamp_ns,
            sequence: f.sequence,
            mcu_frame_count: f.mcu_frame_count,
            imu_time_ns: PyArray1::from_vec(py, imu.iter().map(|s| s.time_ns as i64).collect())
                .unbind(),
            gyro: vectors(py, &imu, |s| s.gyro).unbind(),
            accel: vectors(py, &imu, |s| s.accel).unbind(),
        }
    }
}

#[pymethods]
impl PyFrame {
    fn __repr__(&self, py: Python<'_>) -> String {
        format!(
            "Frame(sequence={}, stamp_ns={} ({}), exposure_ns={:?}, {} IMU samples)",
            self.sequence,
            self.stamp_ns,
            self.stamp_source,
            self.exposure_ns,
            PyUntypedArrayMethods::len(self.imu_time_ns.bind(py))
        )
    }
}

/// ``CLOCK_MONOTONIC`` now, ns: the clock of the frames and IMU samples.
#[pyfunction]
fn monotonic_ns() -> u64 {
    zed_capture::monotonic_ns()
}

/// The system clock (``CLOCK_REALTIME``) less ``CLOCK_MONOTONIC``, ns, now: add it to a stamp
/// for the system clock's time (ROS stamps). It moves when the system clock is set (NTP).
#[pyfunction]
fn realtime_offset_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let before = zed_capture::monotonic_ns() as i64;
    // SAFETY: a valid clock id and a valid pointer.
    unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) };
    let after = zed_capture::monotonic_ns() as i64;
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec - (before + after) / 2
}

/// The gain multiplier of one of the sensor's raw gains (``Frame.gain``): about
/// 2^band × low byte / 128; None for a value the sensor does not take.
#[pyfunction]
fn gain_multiplier(raw: u16) -> Option<f64> {
    zed_capture::gain_multiplier(raw).map(|g| g as f64)
}

#[pymodule]
fn _zed_capture(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCamera>()?;
    m.add_class::<PyFrame>()?;
    m.add_function(wrap_pyfunction!(monotonic_ns, m)?)?;
    m.add_function(wrap_pyfunction!(realtime_offset_ns, m)?)?;
    m.add_function(wrap_pyfunction!(gain_multiplier, m)?)?;
    Ok(())
}
