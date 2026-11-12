//! Records the ZED's images, IMU and frame timing into one MCAP file (ROS 2 messages, their
//! definitions in the file).
//!
//!   zed_record OUT.mcap [--eyes both|left|right] [--encoding jpeg|raw] [--quality 90] [--every 1]
//!                       [--mains 50|60] [--exposure-ms MS [--gain RAW]] [--seconds S] [--encoders N]
//!
//! Each image is stamped at its eye's middle row's mid-exposure, its `CameraExposure` beside it
//! with the same stamp; `/zed/frame_timing` has every frame's pulse, both eyes' exposure and gain,
//! and the readout's timing; `/zed/imu` every IMU sample. Stamps are `CLOCK_MONOTONIC` plus the
//! system clock's offset measured when recording starts (see the file's `zed` metadata).
//! Recording starts once the frame marks are fitted (~3 s after opening) and stops on SIGINT or
//! SIGTERM, or after --seconds. --every N records every Nth frame's images (the timing and the
//! IMU always); --encoders the threads encoding images (default: one per eye recorded).

mod cdr;
mod msgs;

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use msgs::{Exposure, FrameTiming, Source};
use zed_capture::{Camera, VideoConfig, find, monotonic_ns};

const EYES: [&str; 2] = ["left", "right"];
const USAGE: &str = "usage: zed_record OUT.mcap [--eyes both|left|right] [--encoding jpeg|raw] [--quality 90] [--every 1] [--mains 50|60] [--exposure-ms MS [--gain RAW]] [--seconds S] [--encoders N]";

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Encoding {
    Jpeg,
    Raw,
}

struct Args {
    out: PathBuf,
    eyes: [bool; 2],
    encoding: Encoding,
    quality: u8,
    every: usize,
    mains_hz: Option<u32>,
    exposure_ms: Option<f64>,
    gain: Option<u16>,
    seconds: Option<f64>,
    encoders: Option<usize>,
}

fn args() -> Result<Args, String> {
    let mut a = Args {
        out: PathBuf::new(),
        eyes: [true, true],
        encoding: Encoding::Jpeg,
        quality: 90,
        every: 1,
        mains_hz: None,
        exposure_ms: None,
        gain: None,
        seconds: None,
        encoders: None,
    };
    let mut out = None;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        if !arg.starts_with("--") {
            if out.replace(PathBuf::from(&arg)).is_some() {
                return Err(format!("one output file, not also {arg}"));
            }
            continue;
        }
        let value = it.next().ok_or(format!("{arg} needs a value"))?;
        let bad = || format!("{arg}: bad value {value}");
        match arg.as_str() {
            "--eyes" => {
                a.eyes = match value.as_str() {
                    "both" => [true, true],
                    "left" => [true, false],
                    "right" => [false, true],
                    _ => return Err(bad()),
                }
            }
            "--encoding" => {
                a.encoding = match value.as_str() {
                    "jpeg" => Encoding::Jpeg,
                    "raw" => Encoding::Raw,
                    "nvjpg" => return Err("--encoding nvjpg is not built yet".into()),
                    _ => return Err(bad()),
                }
            }
            "--quality" => a.quality = value.parse().map_err(|_| bad())?,
            "--every" => a.every = value.parse::<usize>().map_err(|_| bad())?.max(1),
            "--mains" => a.mains_hz = Some(value.parse().map_err(|_| bad())?),
            "--exposure-ms" => a.exposure_ms = Some(value.parse().map_err(|_| bad())?),
            "--gain" => a.gain = Some(value.parse().map_err(|_| bad())?),
            "--seconds" => a.seconds = Some(value.parse().map_err(|_| bad())?),
            "--encoders" => a.encoders = Some(value.parse::<usize>().map_err(|_| bad())?.max(1)),
            _ => return Err(format!("unknown option {arg}\n{USAGE}")),
        }
    }
    a.out = out.ok_or(USAGE)?;
    if a.gain.is_some() && a.exposure_ms.is_none() {
        return Err("--gain goes with --exposure-ms".into());
    }
    if !(1..=100).contains(&a.quality) {
        return Err("--quality: 1 to 100".into());
    }
    Ok(a)
}

fn realtime_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: a valid clock id and a valid pointer.
    unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) };
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec
}

/// The system clock less `CLOCK_MONOTONIC` (ns), from the tightest of a few readings, with the
/// monotonic and system times it was read at.
fn clock_offset() -> (i64, i64, i64) {
    let mut best = (i64::MAX, 0, 0, 0);
    for _ in 0..8 {
        let m0 = monotonic_ns() as i64;
        let r = realtime_ns();
        let m1 = monotonic_ns() as i64;
        let m = m0 + (m1 - m0) / 2;
        if m1 - m0 < best.0 {
            best = (m1 - m0, r - m, m, r);
        }
    }
    (best.1, best.2, best.3)
}

fn hostname() -> String {
    let mut b = [0u8; 256];
    // SAFETY: a buffer of its stated length.
    if unsafe { libc::gethostname(b.as_mut_ptr().cast(), b.len()) } != 0 {
        return String::new();
    }
    let n = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..n]).into_owned()
}

// ---- The writer: one thread owns the MCAP file ----

enum Out {
    Message {
        channel: u16,
        time_ns: u64,
        data: Vec<u8>,
    },
    Metadata(String, BTreeMap<String, String>),
}

struct Channels {
    image: [Option<u16>; 2],
    exposure: [u16; 2],
    imu: u16,
    timing: u16,
    clock: u16,
}

fn open_file(
    a: &Args,
    zed: BTreeMap<String, String>,
    calibration: Option<(String, Vec<u8>)>,
    start_ns: u64,
) -> mcap::McapResult<(mcap::Writer<BufWriter<File>>, Channels)> {
    let file = BufWriter::with_capacity(1 << 20, File::create(&a.out)?);
    let mut w = mcap::WriteOptions::new()
        .compression(None)
        .profile("ros2")
        .library(format!(
            "zed_record (zed-capture {})",
            env!("CARGO_PKG_VERSION")
        ))
        .chunk_size(Some(8 << 20))
        .create(file)?;
    // Before the channels: a metadata record or an attachment closes the chunk open, and a
    // chunk of channels without messages has no message index, which rosbag2's reader refuses
    w.write_metadata(&mcap::records::Metadata {
        name: "zed".into(),
        metadata: zed,
    })?;
    if let Some((name, data)) = calibration {
        w.attach(&mcap::Attachment {
            log_time: start_ns,
            create_time: start_ns,
            name,
            media_type: "text/plain".into(),
            data: Cow::Owned(data),
        })?;
    }
    let mut add = |schema: msgs::Schema, topic: &str| -> mcap::McapResult<u16> {
        let id = w.add_schema(schema.name, "ros2msg", schema.text.as_bytes())?;
        w.add_channel(id, topic, "cdr", &BTreeMap::new())
    };
    let mut image = [None; 2];
    let mut exposure = [0; 2];
    for (eye, name) in EYES.iter().enumerate() {
        if a.eyes[eye] {
            image[eye] = Some(match a.encoding {
                Encoding::Jpeg => add(
                    msgs::compressed_image_schema(),
                    &format!("/zed/{name}/image_raw/compressed"),
                )?,
                Encoding::Raw => add(msgs::image_schema(), &format!("/zed/{name}/image_raw"))?,
            });
            exposure[eye] = add(
                msgs::camera_exposure_schema(),
                &format!("/zed/{name}/exposure"),
            )?;
        }
    }
    let channels = Channels {
        image,
        exposure,
        imu: add(msgs::imu_schema(), "/zed/imu")?,
        timing: add(msgs::frame_timing_schema(), "/zed/frame_timing")?,
        clock: add(msgs::clock_offset_schema(), "/zed/clock")?,
    };
    Ok((w, channels))
}

/// Writes what it is sent until the senders are gone, then finishes the file. Its messages and
/// bytes, or the first error (after which it keeps taking messages and drops them).
fn spawn_writer(
    mut w: mcap::Writer<BufWriter<File>>,
    rx: Receiver<Out>,
) -> io::Result<JoinHandle<Result<(u64, u64), String>>> {
    std::thread::Builder::new()
        .name("zed-record-write".into())
        .spawn(move || {
            let mut sequence: HashMap<u16, u32> = HashMap::new();
            let (mut messages, mut bytes) = (0u64, 0u64);
            let mut error = None;
            for out in rx {
                if error.is_some() {
                    continue;
                }
                let r = match out {
                    Out::Message {
                        channel,
                        time_ns,
                        data,
                    } => {
                        let s = sequence.entry(channel).or_insert(0);
                        *s += 1;
                        messages += 1;
                        bytes += data.len() as u64;
                        w.write_to_known_channel(
                            &mcap::records::MessageHeader {
                                channel_id: channel,
                                sequence: *s,
                                log_time: time_ns,
                                publish_time: time_ns,
                            },
                            &data,
                        )
                    }
                    Out::Metadata(name, metadata) => {
                        w.write_metadata(&mcap::records::Metadata { name, metadata })
                    }
                };
                if let Err(e) = r {
                    eprintln!("zed_record: writing: {e}; stopping");
                    STOP.store(true, Ordering::Relaxed);
                    error = Some(e.to_string());
                }
            }
            if let Some(e) = error {
                return Err(e);
            }
            w.finish().map_err(|e| e.to_string())?;
            Ok((messages, bytes))
        })
}

// ---- The encoders ----

struct Job {
    eye: usize,
    stamp_ns: i64,
    width: u32,
    height: u32,
    data: Vec<u8>,
    exposure: Exposure,
}

#[derive(Default)]
struct EncodeStats {
    images: [AtomicU64; 2],
    ns: [AtomicU64; 2],
    bytes: [AtomicU64; 2],
}

/// The mean level and the fraction at 255 of every 4th pixel of every 4th row.
fn meter(data: &[u8], width: usize) -> (f32, f32) {
    let (mut sum, mut clipped, mut n) = (0u64, 0u64, 0u64);
    for row in data.chunks_exact(width).step_by(4) {
        for &v in row.iter().step_by(4) {
            sum += v as u64;
            clipped += (v == 255) as u64;
            n += 1;
        }
    }
    let n = n.max(1) as f32;
    (sum as f32 / n, clipped as f32 / n)
}

fn encode(job: &Job, encoding: Encoding, quality: u8, frame_id: &str) -> Result<Vec<u8>, String> {
    Ok(match encoding {
        Encoding::Jpeg => {
            let mut jpeg = Vec::with_capacity(job.data.len() / 4);
            jpeg_encoder::Encoder::new(&mut jpeg, quality)
                .encode(
                    &job.data,
                    job.width as u16,
                    job.height as u16,
                    jpeg_encoder::ColorType::Luma,
                )
                .map_err(|e| e.to_string())?;
            msgs::compressed_image(
                job.stamp_ns,
                frame_id,
                "mono8; jpeg compressed mono8",
                &jpeg,
            )
        }
        Encoding::Raw => {
            msgs::image_mono8(job.stamp_ns, frame_id, job.width, job.height, &job.data)
        }
    })
}

fn spawn_encoders(
    n: usize,
    encoding: Encoding,
    quality: u8,
    channels: &Channels,
    out: SyncSender<Out>,
    stats: Arc<EncodeStats>,
) -> io::Result<(SyncSender<Job>, Vec<JoinHandle<()>>)> {
    let (tx, rx) = mpsc::sync_channel::<Job>(2 * n);
    let rx = Arc::new(Mutex::new(rx));
    let image = channels.image;
    let exposure = channels.exposure;
    let mut handles = Vec::new();
    for i in 0..n {
        let (rx, out, stats) = (rx.clone(), out.clone(), stats.clone());
        handles.push(
            std::thread::Builder::new()
                .name(format!("zed-record-enc{i}"))
                .spawn(move || {
                    loop {
                        let Ok(mut job) = rx.lock().unwrap().recv() else {
                            return;
                        };
                        let t0 = Instant::now();
                        let e = job.eye;
                        let frame_id = format!("zed_{}", EYES[e]);
                        (job.exposure.mean_level, job.exposure.saturated_fraction) =
                            meter(&job.data, job.width as usize);
                        let msg = match encode(&job, encoding, quality, &frame_id) {
                            Ok(m) => m,
                            Err(err) => {
                                eprintln!("zed_record: encoding: {err}");
                                continue;
                            }
                        };
                        stats.images[e].fetch_add(1, Ordering::Relaxed);
                        stats.ns[e].fetch_add(t0.elapsed().as_nanos() as u64, Ordering::Relaxed);
                        stats.bytes[e].fetch_add(msg.len() as u64, Ordering::Relaxed);
                        let time_ns = job.stamp_ns.max(0) as u64;
                        let ex = msgs::camera_exposure(job.stamp_ns, &frame_id, &job.exposure);
                        let sent = out
                            .send(Out::Message {
                                channel: image[e].expect("a channel for each eye recorded"),
                                time_ns,
                                data: msg,
                            })
                            .and_then(|_| {
                                out.send(Out::Message {
                                    channel: exposure[e],
                                    time_ns,
                                    data: ex,
                                })
                            });
                        if sent.is_err() {
                            return;
                        }
                    }
                })?,
        );
    }
    Ok((tx, handles))
}

// ---- Recording ----

#[derive(Default, Debug)]
struct Counts {
    frames: u64,
    without_pulse: u64,
    without_exposure: u64,
    images_dropped: [u64; 2],
    exposure_from: BTreeMap<String, u64>,
    imu_samples: u64,
}

fn context(what: &'static str) -> impl Fn(io::Error) -> String {
    move |e| format!("{what}: {e}")
}

fn run(a: Args) -> Result<(), String> {
    let d = find().map_err(context("the camera"))?;
    let config = VideoConfig {
        mains_hz: a.mains_hz,
        ..Default::default()
    };
    let camera =
        Camera::open(&d.video, Some(&d.hid), config.clone()).map_err(context("opening"))?;
    if let Err(m) = camera.frame_sync() {
        eprintln!("zed_record: the frame marks were not turned on ({m}); waiting for them anyway");
    }
    let manual = match a.exposure_ms {
        Some(ms) => Some(
            camera
                .video()
                .set_manual_exposure(Duration::from_secs_f64(ms / 1e3), a.gain)
                .map_err(context("the manual exposure"))?,
        ),
        None => None,
    };
    let timing = camera
        .sensor_timing()
        .map_err(|m| format!("the sensors' timing: {m}"))?;

    // Recording starts with the first frame that has its pulse
    let waiting = Instant::now();
    let first = loop {
        if STOP.load(Ordering::Relaxed) {
            return Ok(());
        }
        if waiting.elapsed() > Duration::from_secs(15) {
            return Err("no frame sync after 15 s".into());
        }
        let Some((frame, _)) = camera
            .next(Duration::from_secs(1))
            .map_err(context("the camera"))?
        else {
            continue;
        };
        if frame.sync_time_ns.is_some() && camera.readout().is_some() {
            break frame;
        }
    };
    let readout = camera.readout().expect("checked above");
    let (offset_ns, start_mono, start_real) = clock_offset();

    let mut zed = BTreeMap::new();
    let mut put = |k: &str, v: String| {
        zed.insert(k.to_string(), v);
    };
    put("model", d.model.to_string());
    put("serial", d.serial.clone().unwrap_or_default());
    put("video_device", d.video.display().to_string());
    put(
        "mode",
        format!(
            "{}x{} per eye at {:.2} Hz",
            config.eye_width,
            config.height,
            camera.video().fps()
        ),
    );
    put(
        "mains_hz",
        camera
            .video()
            .mains_hz()
            .map(|h| h.to_string())
            .unwrap_or_else(|e| format!("unknown ({e})")),
    );
    put(
        "exposure",
        match manual {
            Some(m) => format!(
                "manual: {} lines = {:.3} ms, gain {:?}",
                m.lines,
                m.exposure_ns / 1e6,
                m.gain
            ),
            None => "the camera's auto exposure, each eye's own".into(),
        },
    );
    put(
        "sensor",
        format!(
            "chip {:#06x}, {} pixel clocks a line, {} lines a frame, output {}x{}",
            timing.chip_id,
            timing.line_length,
            timing.frame_length,
            timing.output_width,
            timing.output_height
        ),
    );
    put("line_us", format!("{:.4}", readout.line_ns / 1e3));
    put(
        "first_row_us",
        format!("{:.1}", readout.pulse_to_readout_ns / 1e3),
    );
    put(
        "stamps",
        "images and their exposure: the eye's middle row's mid-exposure (the pulse + first_row_us \
         + the readout to the middle row - half the exposure); frame_timing: the pulse; imu: the \
         sample. All CLOCK_MONOTONIC + clock_offset_ns."
            .into(),
    );
    put("clock_offset_ns", offset_ns.to_string());
    put("clock_start_monotonic_ns", start_mono.to_string());
    put("clock_start_realtime_ns", start_real.to_string());
    put(
        "gain",
        "the OV4689's raw gain register (0-255, 378-511, 890-1023, 1914-2047): not a multiplier"
            .into(),
    );
    put("imu_axes", "the IMU's own, not the cameras'".into());
    put("host", hostname());
    put("recorder", std::env::args().collect::<Vec<_>>().join(" "));
    let calibration = d.serial.as_ref().and_then(|s| {
        let name = format!("SN{s}.conf");
        let data = std::fs::read(format!("/usr/local/zed/settings/{name}")).ok()?;
        Some((name, data))
    });
    put(
        "calibration",
        match &calibration {
            Some((n, _)) => format!("attached: {n} (the factory calibration)"),
            None => "not found in /usr/local/zed/settings".into(),
        },
    );

    let (w, channels) = open_file(&a, zed, calibration, start_real as u64)
        .map_err(|e| format!("{}: {e}", a.out.display()))?;
    let (out_tx, out_rx) = mpsc::sync_channel::<Out>(4096);
    let writer = spawn_writer(w, out_rx).map_err(context("the writer"))?;
    let eyes_recorded = a.eyes.iter().filter(|&&r| r).count();
    let enc_stats = Arc::new(EncodeStats::default());
    let (jobs, encoders) = spawn_encoders(
        a.encoders.unwrap_or(eyes_recorded),
        a.encoding,
        a.quality,
        &channels,
        out_tx.clone(),
        enc_stats.clone(),
    )
    .map_err(context("the encoders"))?;
    eprintln!(
        "zed_record: recording {} to {} ({:?}{}); stop with Ctrl-C",
        match a.eyes {
            [true, true] => "both eyes",
            [true, false] => "the left eye",
            _ => "the right eye",
        },
        a.out.display(),
        a.encoding,
        if a.encoding == Encoding::Jpeg {
            format!(" q{}", a.quality)
        } else {
            String::new()
        }
    );

    let send = |channel: u16, stamp_ns: i64, data: Vec<u8>| {
        out_tx
            .send(Out::Message {
                channel,
                time_ns: stamp_ns.max(0) as u64,
                data,
            })
            .map_err(|_| "the writer stopped".to_string())
    };
    let mut counts = Counts::default();
    let mut last_known: [Option<(f64, u16)>; 2] = [None; 2];
    let started = Instant::now();
    let mut next_clock = Instant::now();
    let mut seen = 0usize;
    let mut pending = Some((first, Vec::new()));
    let mut result = Ok(());
    camera.video().reset_stats();
    if let Some(imu) = camera.imu() {
        imu.reset_stats();
    }
    while result.is_ok()
        && !STOP.load(Ordering::Relaxed)
        && a.seconds
            .is_none_or(|s| started.elapsed().as_secs_f64() < s)
    {
        let (mut f, samples) = match pending.take() {
            Some(x) => x,
            None => match camera.next(Duration::from_secs(1)) {
                Ok(Some(x)) => x,
                Ok(None) => continue,
                Err(err) => {
                    result = Err(format!("the camera: {err}"));
                    break;
                }
            },
        };
        for s in &samples {
            let t = s.time_ns as i64 + offset_ns;
            send(channels.imu, t, msgs::imu(t, "zed_imu", s.gyro, s.accel))?;
        }
        counts.imu_samples += samples.len() as u64;
        if Instant::now() >= next_clock {
            next_clock += Duration::from_secs(1);
            let (_, m, r) = clock_offset();
            send(channels.clock, r, msgs::clock_offset(r, m, offset_ns))?;
        }
        counts.frames += 1;
        let Some(pulse) = f.sync_time_ns else {
            counts.without_pulse += 1;
            continue;
        };
        let readout = camera.readout().unwrap_or(readout);
        let middle = pulse as f64 + readout.middle_row_ns(f.height);
        // Each eye's exposure (ns) and gain, and where they came from
        let mut eye_exposure = [None; 2];
        for eye in 0..2 {
            if let (Some(ex), Some(g)) = (f.exposure_ns[eye], f.gain[eye]) {
                let source = if manual.is_some() {
                    Source::Manual
                } else {
                    Source::Own
                };
                eye_exposure[eye] = Some((ex as f64, g, source));
                last_known[eye] = Some((ex as f64, g));
            }
        }
        for eye in 0..2 {
            if eye_exposure[eye].is_some() {
                continue;
            }
            let gain = last_known[eye].map(|(_, g)| g).unwrap_or(0);
            eye_exposure[eye] = match (eye_exposure[1 - eye], last_known[eye]) {
                (Some((ex, _, Source::Own | Source::Manual)), _) => {
                    Some((ex, gain, Source::OtherEye))
                }
                (_, Some((ex, _))) => Some((ex, gain, Source::LastKnown)),
                _ => None,
            };
        }
        let [Some(left), Some(right)] = eye_exposure else {
            counts.without_exposure += 1;
            continue;
        };
        let both = [left, right];
        for (eye, (_, _, s)) in both.iter().enumerate() {
            *counts
                .exposure_from
                .entry(format!("{}_{s:?}", EYES[eye]))
                .or_insert(0) += 1;
        }
        let mid = [0, 1].map(|eye| {
            f.mid_exposure_ns[eye]
                .map(|m| m as f64)
                .unwrap_or(middle - both[eye].0 / 2.0) as i64
                + offset_ns
        });
        send(
            channels.timing,
            pulse as i64 + offset_ns,
            msgs::frame_timing(&FrameTiming {
                pulse_ns: pulse as i64 + offset_ns,
                mcu_frame_count: f.mcu_frame_count.unwrap_or(0),
                sequence: f.sequence,
                kernel_ns: f.timestamp_ns as i64 + offset_ns,
                mid_exposure_ns: mid,
                exposure_us: both.map(|(ex, _, _)| (ex / 1e3) as f32),
                gain: both.map(|(_, g, _)| g as u32),
                source: both.map(|(_, _, s)| s),
                line_us: (readout.line_ns / 1e3) as f32,
                first_row_us: (readout.pulse_to_readout_ns / 1e3) as f32,
                rows_before_image: readout.rows_before_image as f32,
            }),
        )?;
        seen += 1;
        if !(seen - 1).is_multiple_of(a.every) {
            continue;
        }
        for (eye, &(ex, gain, source)) in both.iter().enumerate() {
            if !a.eyes[eye] {
                continue;
            }
            let data = std::mem::take(if eye == 0 { &mut f.left } else { &mut f.right });
            let job = Job {
                eye,
                stamp_ns: mid[eye],
                width: f.width as u32,
                height: f.height as u32,
                data,
                exposure: Exposure {
                    exposure_us: (ex / 1e3) as f32,
                    exposure_lines: (ex / readout.line_ns).round() as u32,
                    analogue_gain: gain as u32,
                    exposure_measured: matches!(source, Source::Own | Source::Manual),
                    auto_exposure: manual.is_none(),
                    ..Default::default()
                },
            };
            match jobs.try_send(job) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) => counts.images_dropped[eye] += 1,
                Err(TrySendError::Disconnected(_)) => {
                    result = Err("the encoders stopped".into());
                }
            }
        }
    }
    let seconds = started.elapsed().as_secs_f64();
    drop(jobs);
    for h in encoders {
        let _ = h.join();
    }

    let v = camera.video().stats();
    let mut stats = BTreeMap::new();
    let mut put = |k: &str, v: String| {
        stats.insert(k.to_string(), v);
    };
    put("seconds", format!("{seconds:.3}"));
    put("frames", counts.frames.to_string());
    put("frames_without_pulse", counts.without_pulse.to_string());
    put(
        "frames_without_exposure",
        counts.without_exposure.to_string(),
    );
    put("frames_lost_by_time", v.missing_by_time.to_string());
    put("ring_drops", v.ring_drops.to_string());
    put("imu_samples", counts.imu_samples.to_string());
    if let Some(imu) = camera.imu() {
        put("imu_missing", imu.stats().missing.to_string());
    }
    if let Some(x) = camera.exposure_stats() {
        put(
            "exposure_readings_unanswered",
            format!(
                "left {} right {} of {}",
                x.unanswered[0], x.unanswered[1], x.readings
            ),
        );
    }
    for (k, n) in &counts.exposure_from {
        put(&format!("exposure_from_{k}"), n.to_string());
    }
    for (eye, name) in EYES.iter().enumerate() {
        if !a.eyes[eye] {
            continue;
        }
        let n = enc_stats.images[eye].load(Ordering::Relaxed);
        put(&format!("{name}_images"), n.to_string());
        put(
            &format!("{name}_images_dropped"),
            counts.images_dropped[eye].to_string(),
        );
        put(
            &format!("{name}_encode_ms"),
            format!(
                "{:.2}",
                enc_stats.ns[eye].load(Ordering::Relaxed) as f64 / n.max(1) as f64 / 1e6
            ),
        );
        put(
            &format!("{name}_kib_per_image"),
            format!(
                "{:.1}",
                enc_stats.bytes[eye].load(Ordering::Relaxed) as f64 / n.max(1) as f64 / 1024.0
            ),
        );
    }
    let summary = stats
        .iter()
        .map(|(k, v)| format!("{k} {v}"))
        .collect::<Vec<_>>()
        .join("; ");
    let _ = out_tx.send(Out::Metadata("stats".into(), stats));
    drop(out_tx);
    let written = writer
        .join()
        .map_err(|_| "the writer panicked".to_string())?
        .map_err(|e| format!("{}: {e}", a.out.display()))?;
    eprintln!(
        "zed_record: {}: {} messages, {:.1} MB; {summary}",
        a.out.display(),
        written.0,
        written.1 as f64 / 1e6
    );
    result
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
            eprintln!("zed_record: {e}");
            ExitCode::FAILURE
        }
    }
}
