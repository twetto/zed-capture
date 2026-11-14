//! The ZED's frames and IMU as ROS 2 topics, from zed-capture (no ZED SDK), through r2r.
//!
//!   zed_ros [--images rect|raw|both] [--exposure active|gradient|camera] [--mains 50|60]
//!           [--eyes both|left|right] [--rect FX,CX,CY] [--calibration SN.conf]
//!           [--no-jpeg] [--jpeg-quality 90] [--best-effort] [--depth N] [--no-imu]
//!           [--prefix /zed] [--seconds S]
//!
//! Topics, under --prefix (default /zed), for each eye published:
//!   <eye>/image_rect     sensor_msgs/Image, mono8, rectified (--images rect, the default, or both)
//!   <eye>/camera_info    sensor_msgs/CameraInfo of the rectified images: no distortion, K the
//!                        rectified camera, the right's P with the baseline (Tx = -fx B), as the
//!                        ZED wrapper publishes for its rectified images
//!   <eye>/image_raw      sensor_msgs/Image, mono8, unrectified (--images raw or both)
//!   <image topic>/nvjpg  sensor_msgs/CompressedImage: each image's JPEG from the Jetson's hardware
//!                        encoder (NVJPG; zed-capture's `nvjpg`), as mr-neuro-nav's nvjpg_encoder.py
//!                        publishes it ("bgr8; jpeg compressed bgr8", the chroma flat; reliable,
//!                        keep-last 10), encoded in a thread of its own after the image went out;
//!                        unless --no-jpeg. Without NVJPG (not a Jetson, or built without the
//!                        feature nvjpg) there are none, and it says so
//!   imu                  sensor_msgs/Imu: every sample, the IMU's own axes, no orientation
//!
//! Rectification is Rudolf-V's, from the camera's factory calibration (the ZED SDK downloads it to
//! /usr/local/zed/settings/SN<serial>.conf; --calibration to name another), onto a pinhole camera
//! with fx = fy: by default the left raw camera's fx, cx and cy, --rect to choose.
//!
//! Both eyes' messages carry the frame's stamp, so that an exact time synchroniser pairs them: the
//! middle of the left image's middle row's exposure (with the camera's own auto exposure the
//! left's exposure is not read: the right's, close to it). Stamps are on the system clock: each
//! frame's and sample's `CLOCK_MONOTONIC` time plus the clocks' offset when it is published.
//! Images are published reliable, keep-last --depth (1), as the ZED wrapper's; --best-effort.
//! Every 10 s it logs the frames captured and published and what rectifying and publishing took.
//! After 3 s without a frame it opens the camera again. Stops on SIGINT or SIGTERM, or after
//! --seconds.
//!
//! For images over Fast DDS's shared memory, run it with a segment larger than the default
//! 512 KB (FASTRTPS_DEFAULT_PROFILES_FILE; mr-neuro-nav's scripts/idog/fastdds_shm16.xml): with it
//! subscribers got every frame, without it they lost 0.4-2.6 %.

mod jpeg;
mod rectify;

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use jpeg::{JpegCounts, JpegStats, JpegStream};
use r2r::QosProfile;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::Vector3;
use r2r::sensor_msgs::msg::{CameraInfo, Image, Imu};
use r2r::std_msgs::msg::Header;
use rectify::Rectifier;
use zed_capture::{AutoExposureConfig, Camera, GradientConfig, ImuSample, VideoConfig};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

const USAGE: &str = "usage: zed_ros [--images rect|raw|both] [--exposure active|gradient|camera] [--mains 50|60] [--eyes both|left|right] [--rect FX,CX,CY] [--calibration SN.conf] [--no-jpeg] [--jpeg-quality 90] [--best-effort] [--depth N] [--no-imu] [--prefix /zed] [--seconds S]";

/// After this long without a frame the camera is opened again.
const STALL: Duration = Duration::from_secs(3);
/// How long opening may wait for the frame sync.
const SYNC_WAIT: Duration = Duration::from_secs(15);
/// How often it logs.
const LOG_EVERY: Duration = Duration::from_secs(10);
const EYES: [&str; 2] = ["left", "right"];

struct Args {
    rect: bool,
    raw: bool,
    exposure: String,
    mains_hz: Option<u32>,
    eyes: [bool; 2],
    camera: Option<[f64; 3]>,
    calibration: Option<PathBuf>,
    jpeg: bool,
    jpeg_quality: u8,
    best_effort: bool,
    depth: usize,
    imu: bool,
    prefix: String,
    seconds: Option<f64>,
}

fn args() -> Result<Args, String> {
    let mut a = Args {
        rect: true,
        raw: false,
        exposure: "active".into(),
        mains_hz: None,
        eyes: [true, true],
        camera: None,
        calibration: None,
        jpeg: true,
        jpeg_quality: 90,
        best_effort: false,
        depth: 1,
        imu: true,
        prefix: "/zed".into(),
        seconds: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--best-effort" => {
                a.best_effort = true;
                continue;
            }
            "--no-imu" => {
                a.imu = false;
                continue;
            }
            "--no-jpeg" => {
                a.jpeg = false;
                continue;
            }
            "-h" | "--help" => return Err(USAGE.into()),
            _ => {}
        }
        let value = it.next().ok_or(format!("{arg} needs a value"))?;
        let bad = || format!("{arg}: bad value {value}");
        match arg.as_str() {
            "--images" => {
                (a.rect, a.raw) = match value.as_str() {
                    "rect" => (true, false),
                    "raw" => (false, true),
                    "both" => (true, true),
                    _ => return Err(bad()),
                }
            }
            "--exposure" if ["active", "gradient", "camera"].contains(&value.as_str()) => {
                a.exposure = value
            }
            "--mains" => a.mains_hz = Some(value.parse().map_err(|_| bad())?),
            "--eyes" => {
                a.eyes = match value.as_str() {
                    "both" => [true, true],
                    "left" => [true, false],
                    "right" => [false, true],
                    _ => return Err(bad()),
                }
            }
            "--rect" => {
                let v: Vec<f64> = value
                    .split(',')
                    .map(|x| x.trim().parse().map_err(|_| bad()))
                    .collect::<Result<_, _>>()?;
                let [fx, cx, cy] = v[..] else {
                    return Err(bad());
                };
                if fx <= 0.0 {
                    return Err(bad());
                }
                a.camera = Some([fx, cx, cy]);
            }
            "--calibration" => a.calibration = Some(value.into()),
            "--jpeg-quality" => {
                a.jpeg_quality = value
                    .parse()
                    .ok()
                    .filter(|q| (1..=100).contains(q))
                    .ok_or_else(bad)?
            }
            "--depth" => a.depth = value.parse().ok().filter(|&d| d > 0).ok_or_else(bad)?,
            "--prefix" => a.prefix = value.trim_end_matches('/').to_string(),
            "--seconds" => a.seconds = Some(value.parse().map_err(|_| bad())?),
            _ => return Err(format!("{arg} {value}?\n{USAGE}")),
        }
    }
    Ok(a)
}

/// `CLOCK_REALTIME` less `CLOCK_MONOTONIC`, now (ns).
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

fn time(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

/// Opens the ZED (with its serial number) and waits for its frames to carry their pulse (the
/// frame marks fitted, ~3 s); the counts start from there.
fn open(config: &VideoConfig) -> Result<(Camera, Option<String>), String> {
    let d = zed_capture::find().map_err(|e| format!("finding the ZED: {e}"))?;
    let camera = Camera::open(&d.video, Some(&d.hid), config.clone())
        .map_err(|e| format!("opening the ZED: {e}"))?;
    let start = Instant::now();
    loop {
        if STOP.load(Ordering::Relaxed) {
            return Err("stopped".into());
        }
        if start.elapsed() > SYNC_WAIT {
            return Err(format!("no frame sync after {} s", SYNC_WAIT.as_secs()));
        }
        match camera.next(Duration::from_millis(500)) {
            Ok(Some((f, _))) if f.sync_time_ns.is_some() && camera.readout().is_some() => break,
            Ok(_) => {}
            Err(e) => return Err(format!("the camera: {e}")),
        }
    }
    camera.video().reset_stats();
    if let Some(imu) = camera.imu() {
        imu.reset_stats();
    }
    Ok((camera, d.serial))
}

/// Counts over a log period, and in all.
#[derive(Default, Clone)]
struct Counts {
    frames: u64,
    images: u64,
    publish_ns: u64,
    publish_max_ns: u64,
    rectify_ns: u64,
    rectify_max_ns: u64,
    imu: u64,
    errors: u64,
}

impl Counts {
    fn add(&mut self, o: &Counts) {
        self.frames += o.frames;
        self.images += o.images;
        self.publish_ns += o.publish_ns;
        self.publish_max_ns = self.publish_max_ns.max(o.publish_max_ns);
        self.rectify_ns += o.rectify_ns;
        self.rectify_max_ns = self.rectify_max_ns.max(o.rectify_max_ns);
        self.imu += o.imu;
        self.errors += o.errors;
    }

    /// Publishes an image, counting it and its time.
    fn publish(&mut self, publisher: &r2r::Publisher<Image>, msg: &Image) {
        let t0 = Instant::now();
        match publisher.publish(msg) {
            Ok(()) => self.images += 1,
            Err(_) => self.errors += 1,
        }
        let ns = t0.elapsed().as_nanos() as u64;
        self.publish_ns += ns;
        self.publish_max_ns = self.publish_max_ns.max(ns);
    }

    /// Publishing an image, and rectifying a frame: mean and longest (ms).
    fn ms(&self) -> [f64; 4] {
        [
            self.publish_ns as f64 / self.images.max(1) as f64 / 1e6,
            self.publish_max_ns as f64 / 1e6,
            self.rectify_ns as f64 / self.frames.max(1) as f64 / 1e6,
            self.rectify_max_ns as f64 / 1e6,
        ]
    }
}

fn imu_message(s: &ImuSample, offset: i64) -> Imu {
    let v = |x: [f32; 3]| Vector3 {
        x: x[0] as f64,
        y: x[1] as f64,
        z: x[2] as f64,
    };
    let mut no_orientation = vec![0.0; 9];
    no_orientation[0] = -1.0;
    Imu {
        header: Header {
            stamp: time(s.time_ns as i64 + offset),
            frame_id: "zed_imu".into(),
        },
        orientation_covariance: no_orientation,
        angular_velocity: v(s.gyro),
        angular_velocity_covariance: vec![0.0; 9],
        linear_acceleration: v(s.accel),
        linear_acceleration_covariance: vec![0.0; 9],
        ..Default::default()
    }
}

/// The rectified camera's info for eye `eye` (0 left, 1 right).
fn camera_info(r: &Rectifier, eye: usize, header: Header) -> CameraInfo {
    let (fx, cx, cy) = (r.fx, r.cx, r.cy);
    let tx = if eye == 1 { -fx * r.baseline } else { 0.0 };
    CameraInfo {
        header,
        height: r.height as u32,
        width: r.width as u32,
        distortion_model: "plumb_bob".into(),
        d: vec![0.0; 5],
        k: vec![fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0],
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: vec![fx, 0.0, cx, tx, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        ..Default::default()
    }
}

/// One eye's publishers.
struct EyePubs {
    rect: Option<r2r::Publisher<Image>>,
    info: Option<r2r::Publisher<CameraInfo>>,
    raw: Option<r2r::Publisher<Image>>,
    /// The JPEGs of `rect`'s and `raw`'s images.
    jpeg: [Option<JpegStream>; 2],
}

/// The JPEG streams of the images published, by eye: rect's and raw's. All or none: an encoder
/// for each first, each tried once, then their publishers.
#[cfg(feature = "nvjpg")]
fn jpeg_streams(
    node: &mut r2r::Node,
    pubs: &mut [EyePubs],
    prefix: &str,
    (width, height): (usize, usize),
    quality: u8,
    stats: &Arc<JpegStats>,
) -> Result<(), String> {
    use r2r::sensor_msgs::msg::CompressedImage;
    let mut streams = Vec::new();
    for (eye, p) in pubs.iter().enumerate() {
        for (kind, (name, on)) in [
            ("image_rect", p.rect.is_some()),
            ("image_raw", p.raw.is_some()),
        ]
        .into_iter()
        .enumerate()
        {
            if on {
                streams.push((eye, kind, format!("{prefix}/{}/{name}/nvjpg", EYES[eye])));
            }
        }
    }
    // Each one tried on a blank image (Nvjpg::new)
    let encoders = streams
        .iter()
        .map(|_| zed_capture::nvjpg::Nvjpg::new(width, height, quality))
        .collect::<Result<Vec<_>, _>>()?;
    for ((eye, kind, topic), encoder) in streams.into_iter().zip(encoders) {
        let publisher = node
            .create_publisher::<CompressedImage>(
                &topic,
                QosProfile::default().keep_last(10).reliable(),
            )
            .map_err(|e| format!("ROS: {e}"))?;
        pubs[eye].jpeg[kind] = Some(JpegStream::start(encoder, publisher, stats.clone()));
    }
    Ok(())
}

#[cfg(not(feature = "nvjpg"))]
fn jpeg_streams(
    _: &mut r2r::Node,
    _: &mut [EyePubs],
    _: &str,
    _: (usize, usize),
    _: u8,
    _: &Arc<JpegStats>,
) -> Result<(), String> {
    Err("built without the feature nvjpg".into())
}

fn image(header: Header, width: usize, height: usize, data: Vec<u8>) -> Image {
    Image {
        header,
        height: height as u32,
        width: width as u32,
        encoding: "mono8".into(),
        is_bigendian: 0,
        step: width as u32,
        data,
    }
}

fn run(a: Args) -> Result<(), String> {
    let config = VideoConfig {
        mains_hz: a.mains_hz,
        active_exposure: (a.exposure != "camera").then(|| AutoExposureConfig {
            flicker_hz: a.mains_hz.filter(|&h| h > 0).map(|h| 2.0 * h as f64),
            gradient: (a.exposure == "gradient").then(GradientConfig::default),
            ..Default::default()
        }),
        ..Default::default()
    };
    // ROS and the JPEG encoders first (~0.3 s): the camera's frames would wait in its ring
    let ros = |e: r2r::Error| format!("ROS: {e}");
    let ctx = r2r::Context::create().map_err(ros)?;
    let mut node = r2r::Node::create(ctx, "zed_ros", "").map_err(ros)?;
    let qos = QosProfile::default().keep_last(a.depth);
    let qos = if a.best_effort {
        qos.best_effort()
    } else {
        qos.reliable()
    };
    let mut pubs = Vec::new();
    for (eye, name) in EYES.iter().enumerate() {
        let topic = |t: &str| format!("{}/{name}/{t}", a.prefix);
        let on = a.eyes[eye];
        pubs.push(EyePubs {
            rect: match on && a.rect {
                true => Some(
                    node.create_publisher::<Image>(&topic("image_rect"), qos.clone())
                        .map_err(ros)?,
                ),
                false => None,
            },
            info: match on && a.rect {
                true => Some(
                    node.create_publisher::<CameraInfo>(&topic("camera_info"), qos.clone())
                        .map_err(ros)?,
                ),
                false => None,
            },
            raw: match on && a.raw {
                true => Some(
                    node.create_publisher::<Image>(&topic("image_raw"), qos.clone())
                        .map_err(ros)?,
                ),
                false => None,
            },
            jpeg: [None, None],
        });
    }
    let jpeg_stats = Arc::new(JpegStats::default());
    let size = (config.eye_width as usize, config.height as usize);
    let jpeg = a.jpeg
        && match jpeg_streams(
            &mut node,
            &mut pubs,
            &a.prefix,
            size,
            a.jpeg_quality,
            &jpeg_stats,
        ) {
            Ok(()) => true,
            Err(e) => {
                eprintln!("zed_ros: no JPEG topics: {e}");
                for p in &mut pubs {
                    p.jpeg = [None, None];
                }
                false
            }
        };
    let imu_pub = match a.imu {
        true => Some(
            node.create_publisher::<Imu>(
                &format!("{}/imu", a.prefix),
                QosProfile::default().keep_last(100).reliable(),
            )
            .map_err(ros)?,
        ),
        false => None,
    };
    eprintln!("zed_ros: opening the ZED ({} exposure)", a.exposure);
    let (cam, serial) = open(&config)?;
    let mut camera = Some(cam);
    let rectifier = match a.rect {
        true => {
            let path = a
                .calibration
                .clone()
                .or_else(|| {
                    serial
                        .as_ref()
                        .map(|s| format!("/usr/local/zed/settings/SN{s}.conf").into())
                })
                .ok_or(
                    "the camera's serial number is unknown: name its calibration (--calibration)",
                )?;
            let (w, h) = (config.eye_width as usize, config.height as usize);
            let r = Rectifier::from_zed_conf(&path, w, h, a.camera)?;
            eprintln!(
                "zed_ros: rectifying by {} onto fx {:.3}, cx {:.3}, cy {:.3}, baseline {:.5} m",
                path.display(),
                r.fx,
                r.cx,
                r.cy,
                r.baseline
            );
            Some(r)
        }
        false => None,
    };
    let eyes: Vec<&str> = (0..2).filter(|&e| a.eyes[e]).map(|e| EYES[e]).collect();
    let kinds: Vec<&str> = [(a.rect, "image_rect,camera_info"), (a.raw, "image_raw")]
        .iter()
        .filter(|(on, _)| *on)
        .map(|(_, k)| *k)
        .collect();
    eprintln!(
        "zed_ros: publishing {}/{{{}}}/{{{}}}{}{}",
        a.prefix,
        eyes.join(","),
        kinds.join(","),
        if jpeg {
            format!(
                ", their images' JPEGs (.../nvjpg, quality {})",
                a.jpeg_quality
            )
        } else {
            String::new()
        },
        if a.imu { " and imu" } else { "" }
    );

    let started = Instant::now();
    let (mut period, mut total) = (Counts::default(), Counts::default());
    let mut jpeg_total = JpegCounts::default();
    let mut last_log = Instant::now();
    let mut last_frame = Instant::now();
    let (mut stalls, mut lost, mut ring_drops) = (0u64, 0u64, 0u64);
    while !STOP.load(Ordering::Relaxed)
        && a.seconds
            .is_none_or(|s| started.elapsed().as_secs_f64() < s)
    {
        // After a stall, the camera again
        let Some(cam) = camera.as_ref() else {
            match open(&config) {
                Ok((c, _)) => {
                    eprintln!("zed_ros: the camera is back");
                    camera = Some(c);
                    last_frame = Instant::now();
                }
                Err(e) => {
                    eprintln!("zed_ros: {e}; trying again");
                    std::thread::sleep(Duration::from_secs(1));
                }
            }
            continue;
        };
        let got = match cam.next(Duration::from_millis(200)) {
            Ok(g) => g,
            Err(e) => {
                eprintln!("zed_ros: the camera: {e}");
                None
            }
        };
        let Some((mut frame, samples)) = got else {
            if last_frame.elapsed() > STALL {
                stalls += 1;
                eprintln!(
                    "zed_ros: no frame for {} s: opening the camera again",
                    STALL.as_secs()
                );
                let v = cam.video().stats();
                lost += v.missing_by_time;
                ring_drops += v.ring_drops;
                camera = None;
            }
            continue;
        };
        last_frame = Instant::now();
        period.frames += 1;
        let offset = realtime_offset_ns();
        let stamp_ns = frame.mid_exposure_ns[0]
            .or(frame.mid_exposure_ns[1])
            .unwrap_or(frame.timestamp_ns);
        let header = |eye: usize| Header {
            stamp: time(stamp_ns as i64 + offset),
            frame_id: format!("zed_{}_camera_optical_frame", EYES[eye]),
        };
        let (w, h) = (frame.width, frame.height);
        let rectified = rectifier.as_ref().map(|r| {
            let t0 = Instant::now();
            let pair = r.rectify_pair([&frame.left, &frame.right]);
            let ns = t0.elapsed().as_nanos() as u64;
            period.rectify_ns += ns;
            period.rectify_max_ns = period.rectify_max_ns.max(ns);
            pair
        });
        let mut raw = [
            std::mem::take(&mut frame.left),
            std::mem::take(&mut frame.right),
        ];
        let mut rectified = rectified.map(|[l, r]| [Some(l), Some(r)]);
        for (eye, p) in pubs.iter().enumerate() {
            if let (Some(publisher), Some(data)) =
                (&p.rect, rectified.as_mut().and_then(|r| r[eye].take()))
            {
                let msg = image(header(eye), w, h, data);
                period.publish(publisher, &msg);
                if let Some(j) = &p.jpeg[0] {
                    j.send(msg.header, msg.data);
                }
            }
            if let (Some(publisher), Some(r)) = (&p.info, &rectifier)
                && publisher
                    .publish(&camera_info(r, eye, header(eye)))
                    .is_err()
            {
                period.errors += 1;
            }
            if let Some(publisher) = &p.raw {
                let data = std::mem::take(&mut raw[eye]);
                let msg = image(header(eye), w, h, data);
                period.publish(publisher, &msg);
                if let Some(j) = &p.jpeg[1] {
                    j.send(msg.header, msg.data);
                }
            }
        }
        if let Some(p) = &imu_pub {
            for s in &samples {
                match p.publish(&imu_message(s, offset)) {
                    Ok(()) => period.imu += 1,
                    Err(_) => period.errors += 1,
                }
            }
        }
        if last_log.elapsed() >= LOG_EVERY {
            let v = cam.video().stats();
            let subs: Vec<String> = pubs
                .iter()
                .flat_map(|p| p.rect.iter().chain(p.raw.iter()))
                .map(|p| {
                    p.get_inter_process_subscription_count()
                        .map(|n| n.to_string())
                        .unwrap_or("?".into())
                })
                .collect();
            let [publish, publish_max, rect, rect_max] = period.ms();
            let jpegs = jpeg_stats.take();
            jpeg_total.add(&jpegs);
            eprintln!(
                "zed_ros: {:.0} s: {} frames ({:.2}/s), rectified in {rect:.2} ms (at most \
                 {rect_max:.2}), {} images published in {publish:.2} ms each (at most \
                 {publish_max:.2}), {} IMU samples, {} errors{}; lost {} by the camera ({} bad \
                 buffers, {} sequence numbers skipped, longest {:.1} ms between frames), {} from \
                 the ring; image subscribers {}",
                started.elapsed().as_secs_f64(),
                period.frames,
                period.frames as f64 / last_log.elapsed().as_secs_f64(),
                period.images,
                period.imu,
                period.errors,
                if jpeg {
                    format!("; {}", jpegs.summary())
                } else {
                    String::new()
                },
                lost + v.missing_by_time,
                v.bad_buffers,
                v.missing_by_sequence,
                v.longest_interval_ns as f64 / 1e6,
                ring_drops + v.ring_drops,
                subs.join("/")
            );
            total.add(&period);
            period = Counts::default();
            last_log = Instant::now();
        }
    }
    total.add(&period);
    // The JPEG threads finish what waits
    for p in &mut pubs {
        p.jpeg = [None, None];
    }
    jpeg_total.add(&jpeg_stats.take());
    if let Some(cam) = &camera {
        let v = cam.video().stats();
        lost += v.missing_by_time;
        ring_drops += v.ring_drops;
    }
    let [publish, publish_max, rect, rect_max] = total.ms();
    eprintln!(
        "zed_ros: done after {:.1} s: {} frames (rectified in {rect:.2} ms, at most \
         {rect_max:.2}), {} images published ({publish:.2} ms each, at most {publish_max:.2}), \
         {} IMU samples, {} errors{}; lost {lost} by the camera, {ring_drops} from the ring; \
         {stalls} stalls",
        started.elapsed().as_secs_f64(),
        total.frames,
        total.images,
        total.imu,
        total.errors,
        if jpeg {
            format!("; {}", jpeg_total.summary())
        } else {
            String::new()
        },
    );
    Ok(())
}

fn main() -> ExitCode {
    let a = match args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    // SAFETY: the handler only stores to an atomic.
    unsafe {
        let handler = on_signal as extern "C" fn(libc::c_int) as *const () as libc::sighandler_t;
        libc::signal(libc::SIGINT, handler);
        libc::signal(libc::SIGTERM, handler);
    }
    match run(a) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("zed_ros: {e}");
            ExitCode::FAILURE
        }
    }
}
