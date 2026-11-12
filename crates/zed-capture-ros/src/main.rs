//! The ZED's frames and IMU as ROS 2 topics, from zed-capture (no ZED SDK), through r2r.
//!
//!   zed_ros [--exposure active|gradient|camera] [--mains 50|60] [--eyes both|left|right]
//!           [--best-effort] [--depth N] [--no-imu] [--prefix /zed] [--seconds S]
//!
//! Topics, under --prefix (default /zed):
//!   left/image_raw, right/image_raw   sensor_msgs/Image, mono8, unrectified
//!   imu                               sensor_msgs/Imu: every sample, the IMU's own axes, no
//!                                     orientation
//!
//! Both eyes' images carry the frame's stamp, so that an exact time synchroniser pairs them: the
//! middle of the left image's middle row's exposure (with the camera's own auto exposure the
//! left's exposure is not read: the right's, close to it). Stamps are on the system clock: each
//! frame's and sample's `CLOCK_MONOTONIC` time plus the clocks' offset when it is published.
//! Images are published reliable, keep-last --depth (1), as the ZED wrapper's; --best-effort.
//! Every 10 s it logs the frames captured and published and what publishing took. After 3 s
//! without a frame it opens the camera again. Stops on SIGINT or SIGTERM, or after --seconds.
//!
//! For images over Fast DDS's shared memory, run it with a segment larger than the default
//! 512 KB (FASTRTPS_DEFAULT_PROFILES_FILE; mr-neuro-nav's scripts/idog/fastdds_shm16.xml).

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use r2r::QosProfile;
use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::Vector3;
use r2r::sensor_msgs::msg::{Image, Imu};
use r2r::std_msgs::msg::Header;
use zed_capture::{AutoExposureConfig, Camera, GradientConfig, ImuSample, VideoConfig};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

const USAGE: &str = "usage: zed_ros [--exposure active|gradient|camera] [--mains 50|60] [--eyes both|left|right] [--best-effort] [--depth N] [--no-imu] [--prefix /zed] [--seconds S]";

/// After this long without a frame the camera is opened again.
const STALL: Duration = Duration::from_secs(3);
/// How long opening may wait for the frame sync.
const SYNC_WAIT: Duration = Duration::from_secs(15);
/// How often it logs.
const LOG_EVERY: Duration = Duration::from_secs(10);

struct Args {
    exposure: String,
    mains_hz: Option<u32>,
    eyes: [bool; 2],
    best_effort: bool,
    depth: usize,
    imu: bool,
    prefix: String,
    seconds: Option<f64>,
}

fn args() -> Result<Args, String> {
    let mut a = Args {
        exposure: "active".into(),
        mains_hz: None,
        eyes: [true, true],
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
            "-h" | "--help" => return Err(USAGE.into()),
            _ => {}
        }
        let value = it.next().ok_or(format!("{arg} needs a value"))?;
        let bad = || format!("{arg}: bad value {value}");
        match arg.as_str() {
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

/// Opens the ZED and waits for its frames to carry their pulse (the frame marks fitted, ~3 s);
/// the counts start from there.
fn open(config: &VideoConfig) -> Result<Camera, String> {
    let camera = Camera::find(config.clone()).map_err(|e| format!("opening the ZED: {e}"))?;
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
    Ok(camera)
}

/// Counts over a log period, and in all.
#[derive(Default, Clone)]
struct Counts {
    frames: u64,
    images: u64,
    publish_ns: u64,
    publish_max_ns: u64,
    imu: u64,
    errors: u64,
}

impl Counts {
    fn add(&mut self, o: &Counts) {
        self.frames += o.frames;
        self.images += o.images;
        self.publish_ns += o.publish_ns;
        self.publish_max_ns = self.publish_max_ns.max(o.publish_max_ns);
        self.imu += o.imu;
        self.errors += o.errors;
    }

    fn publish_ms(&self) -> (f64, f64) {
        (
            self.publish_ns as f64 / self.images.max(1) as f64 / 1e6,
            self.publish_max_ns as f64 / 1e6,
        )
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
    let ros = |e: r2r::Error| format!("ROS: {e}");
    let ctx = r2r::Context::create().map_err(ros)?;
    let mut node = r2r::Node::create(ctx, "zed_ros", "").map_err(ros)?;
    let qos = QosProfile::default().keep_last(a.depth);
    let qos = if a.best_effort {
        qos.best_effort()
    } else {
        qos.reliable()
    };
    const EYES: [&str; 2] = ["left", "right"];
    let mut image_pubs = Vec::new();
    for (eye, name) in EYES.iter().enumerate() {
        image_pubs.push(match a.eyes[eye] {
            true => Some(
                node.create_publisher::<Image>(
                    &format!("{}/{name}/image_raw", a.prefix),
                    qos.clone(),
                )
                .map_err(ros)?,
            ),
            false => None,
        });
    }
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
    let mut camera = Some(open(&config)?);
    let published: Vec<&str> = (0..2).filter(|&e| a.eyes[e]).map(|e| EYES[e]).collect();
    eprintln!(
        "zed_ros: publishing {}/{{{}}}/image_raw{}",
        a.prefix,
        published.join(","),
        if a.imu { " and imu" } else { "" }
    );
    let started = Instant::now();
    let (mut period, mut total) = (Counts::default(), Counts::default());
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
                Ok(c) => {
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
        let offset = realtime_offset_ns();
        let stamp_ns = frame.mid_exposure_ns[0]
            .or(frame.mid_exposure_ns[1])
            .unwrap_or(frame.timestamp_ns);
        let stamp = time(stamp_ns as i64 + offset);
        period.frames += 1;
        let (w, h) = (frame.width as u32, frame.height as u32);
        let images = [
            std::mem::take(&mut frame.left),
            std::mem::take(&mut frame.right),
        ];
        for ((publisher, data), name) in image_pubs.iter().zip(images).zip(EYES) {
            let Some(publisher) = publisher else {
                continue;
            };
            let msg = Image {
                header: Header {
                    stamp: stamp.clone(),
                    frame_id: format!("zed_{name}_camera"),
                },
                height: h,
                width: w,
                encoding: "mono8".into(),
                is_bigendian: 0,
                step: w,
                data,
            };
            let t0 = Instant::now();
            match publisher.publish(&msg) {
                Ok(()) => period.images += 1,
                Err(_) => period.errors += 1,
            }
            let ns = t0.elapsed().as_nanos() as u64;
            period.publish_ns += ns;
            period.publish_max_ns = period.publish_max_ns.max(ns);
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
            let subs: Vec<String> = image_pubs
                .iter()
                .flatten()
                .map(|p| {
                    p.get_inter_process_subscription_count()
                        .map(|n| n.to_string())
                        .unwrap_or("?".into())
                })
                .collect();
            let (mean, max) = period.publish_ms();
            eprintln!(
                "zed_ros: {:.0} s: {} frames ({:.2}/s), {} images published in {mean:.2} ms \
                 each (at most {max:.2}), {} IMU samples, {} errors; lost {} by the camera \
                 ({} bad buffers, {} sequence numbers skipped, longest {:.1} ms between \
                 frames), {} from the ring; subscribers {}",
                started.elapsed().as_secs_f64(),
                period.frames,
                period.frames as f64 / last_log.elapsed().as_secs_f64(),
                period.images,
                period.imu,
                period.errors,
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
    if let Some(cam) = &camera {
        let v = cam.video().stats();
        lost += v.missing_by_time;
        ring_drops += v.ring_drops;
    }
    let (mean, max) = total.publish_ms();
    eprintln!(
        "zed_ros: done after {:.1} s: {} frames, {} images published ({mean:.2} ms each, at \
         most {max:.2}), {} IMU samples, {} errors; lost {lost} by the camera, {ring_drops} \
         from the ring; {stalls} stalls",
        started.elapsed().as_secs_f64(),
        total.frames,
        total.images,
        total.imu,
        total.errors,
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
