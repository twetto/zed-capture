//! Saves raw frames: left and right grey (the Y bytes) as 8-bit PGM files, and with --imu every
//! IMU sample and each frame's time on the IMU's clock.
//!
//!   zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--video /dev/videoN]
//!                [--mains 50|60] [--exposure-ms MS [--gain RAW]]
//!
//! `OUT/left/<ns>.pgm` and `OUT/right/<ns>.pgm`, named by the frame's kernel timestamp
//! (CLOCK_MONOTONIC ns), one frame in every --every, after --skip seconds for the auto exposure to
//! settle (and, with --imu, for the frame marks to be fitted). OUT/serial.txt holds the camera's
//! serial number, which names its factory calibration (`/usr/local/zed/settings/SN<serial>.conf`).
//!
//! --mains: the lights' mains frequency, for the camera's anti-flicker (Taiwan: 60). --exposure-ms:
//! a manual exposure on both sensors (the camera's auto exposure off; back on at exit), with the
//! raw --gain (0-255, 378-511, 890-1023 or 1914-2047; else the gain the auto exposure left).
//!
//! --imu: through `Camera`, with the frame marks on. OUT/frames.csv, one line per frame after
//! --skip: its kernel stamp, its sync pulse on the IMU's clock (empty before the marks are
//! fitted), the MCU's count of it, whether its images were saved, each eye's exposure and the
//! middle of the exposure of its image's middle row (`Frame::mid_exposure_ns`; empty for an eye
//! whose sensor has not answered for a second). OUT/imu.csv:
//! every sample after --skip, its time on the host's clock by the MCU's (`time_ns`), its MCU and
//! read times, gyro (rad/s), accel (m/s²), the frame mark and the MCU's frame count.
//! OUT/exposure.csv: every reading of both sensors' exposure registers, when it began (host ns),
//! how long it took, each eye's exposure in lines (empty when it did not answer).
//! OUT/readout.csv: the line time and the rows the mid-exposure times were figured with
//! (`Readout`) and the sensor's lines per frame.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use zed_capture::{Camera, Frame, ImuSample, Video, VideoConfig, find};

struct Args {
    out: PathBuf,
    frames: usize,
    every: usize,
    skip: f64,
    left_only: bool,
    imu: bool,
    video: Option<PathBuf>,
    mains_hz: Option<u32>,
    exposure_ms: Option<f64>,
    gain: Option<u16>,
}

const USAGE: &str = "usage: zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--video /dev/videoN] [--mains 50|60] [--exposure-ms MS [--gain RAW]]";

fn args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut a = Args {
        out: PathBuf::new(),
        frames: 5,
        every: 15,
        skip: 3.0,
        left_only: false,
        imu: false,
        video: None,
        mains_hz: None,
        exposure_ms: None,
        gain: None,
    };
    let mut out = None;
    while let Some(arg) = it.next() {
        if !arg.starts_with("--") {
            if out.replace(PathBuf::from(&arg)).is_some() {
                return Err(format!("one output directory, not also {arg}"));
            }
            continue;
        }
        match arg.as_str() {
            "--left-only" => {
                a.left_only = true;
                continue;
            }
            "--imu" => {
                a.imu = true;
                continue;
            }
            _ => {}
        }
        let value = it.next().ok_or(format!("{arg} needs a value"))?;
        let bad = || format!("{arg}: bad value {value}");
        match arg.as_str() {
            "--frames" => a.frames = value.parse().map_err(|_| bad())?,
            "--every" => a.every = value.parse::<usize>().map_err(|_| bad())?.max(1),
            "--skip" => a.skip = value.parse().map_err(|_| bad())?,
            "--video" => a.video = Some(PathBuf::from(value)),
            "--mains" => a.mains_hz = Some(value.parse().map_err(|_| bad())?),
            "--exposure-ms" => a.exposure_ms = Some(value.parse().map_err(|_| bad())?),
            "--gain" => a.gain = Some(value.parse().map_err(|_| bad())?),
            _ => return Err(format!("unknown option {arg}\n{USAGE}")),
        }
    }
    a.out = out.ok_or(USAGE)?;
    if a.imu && a.video.is_some() {
        return Err("--imu finds the camera itself: no --video".into());
    }
    if a.gain.is_some() && a.exposure_ms.is_none() {
        return Err("--gain goes with --exposure-ms".into());
    }
    Ok(a)
}

fn write_pgm(path: &Path, width: usize, height: usize, grey: &[u8]) -> io::Result<()> {
    let mut f = io::BufWriter::new(fs::File::create(path)?);
    write!(f, "P5\n{width} {height}\n255\n")?;
    f.write_all(grey)?;
    f.flush()
}

/// Where the frames come from: the video alone, or the camera with its IMU.
enum Source {
    Video(Video),
    Camera(Camera),
}

impl Source {
    fn next(&self) -> io::Result<(Frame, Vec<ImuSample>)> {
        let timeout = Duration::from_secs(3);
        let got = match self {
            Source::Video(v) => v.next(timeout)?.map(|f| (f, Vec::new())),
            Source::Camera(c) => c.next(timeout)?,
        };
        got.ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "no frame in 3 s"))
    }

    fn video(&self) -> &Video {
        match self {
            Source::Video(v) => v,
            Source::Camera(c) => c.video(),
        }
    }
}

fn run(a: Args) -> io::Result<()> {
    let config = VideoConfig {
        mains_hz: a.mains_hz,
        ..Default::default()
    };
    let (source, serial) = match (&a.video, a.imu) {
        (Some(v), _) => (Source::Video(Video::open(v, config)?), None),
        (None, false) => {
            let d = find()?;
            (Source::Video(Video::open(&d.video, config)?), d.serial)
        }
        (None, true) => {
            let d = find()?;
            let camera = Camera::open(&d.video, Some(&d.hid), config)?;
            if let Err(e) = camera.frame_sync() {
                eprintln!("zed_grab: no frame marks ({e}): frames.csv without sync times");
            }
            (Source::Camera(camera), d.serial)
        }
    };
    if let Some(ms) = a.exposure_ms {
        let m = source
            .video()
            .set_manual_exposure(Duration::from_secs_f64(ms / 1e3), a.gain)?;
        eprintln!(
            "zed_grab: manual exposure {} lines = {:.3} ms (line {:.3} µs), gain {:?}",
            m.lines,
            m.exposure_ns / 1e6,
            m.line_ns / 1e3,
            m.gain
        );
    }
    let eyes: &[&str] = if a.left_only {
        &["left"]
    } else {
        &["left", "right"]
    };
    for eye in eyes {
        fs::create_dir_all(a.out.join(eye))?;
    }
    if let Some(s) = &serial {
        fs::write(a.out.join("serial.txt"), format!("{s}\n"))?;
    }
    let mut csv = if a.imu {
        let mut frames = io::BufWriter::new(fs::File::create(a.out.join("frames.csv"))?);
        writeln!(
            frames,
            "kernel_ns,sync_ns,mcu_count,saved,exposure_left_ns,exposure_right_ns,mid_left_ns,mid_right_ns"
        )?;
        let mut imu = io::BufWriter::new(fs::File::create(a.out.join("imu.csv"))?);
        writeln!(
            imu,
            "time_ns,mcu_ns,read_ns,gx,gy,gz,ax,ay,az,frame_sync,frame_count"
        )?;
        let mut exposure = io::BufWriter::new(fs::File::create(a.out.join("exposure.csv"))?);
        writeln!(exposure, "time_ns,read_ns,left_lines,right_lines")?;
        Some((frames, imu, exposure))
    } else {
        None
    };

    let t_start = Instant::now();
    let (mut seen, mut saved, mut synced) = (0usize, 0usize, 0usize);
    while saved < a.frames {
        let (frame, samples) = source.next()?;
        if t_start.elapsed().as_secs_f64() < a.skip {
            continue;
        }
        seen += 1;
        let save = (seen - 1) % a.every == 0;
        if let (Some((frames, imu, exposure)), Source::Camera(camera)) = (&mut csv, &source) {
            let opt = |v: Option<u64>| v.map(|v| v.to_string()).unwrap_or_default();
            let e = frame.exposure_ns.map(|e| opt(e.map(u64::from)));
            let m = frame.mid_exposure_ns.map(opt);
            writeln!(
                frames,
                "{},{},{},{},{},{},{},{}",
                frame.timestamp_ns,
                opt(frame.sync_time_ns),
                opt(frame.mcu_frame_count.map(u64::from)),
                save as u8,
                e[0],
                e[1],
                m[0],
                m[1]
            )?;
            for r in camera.drain_exposure() {
                let lines = r
                    .lines
                    .map(|l| l.map(|l| format!("{l:.4}")).unwrap_or_default());
                writeln!(
                    exposure,
                    "{},{},{},{}",
                    r.time_ns, r.read_ns, lines[0], lines[1]
                )?;
            }
            synced += frame.sync_time_ns.is_some() as usize;
            for s in &samples {
                writeln!(
                    imu,
                    "{},{},{},{:.6},{:.6},{:.6},{:.5},{:.5},{:.5},{},{}",
                    s.time_ns,
                    s.mcu_time_ns,
                    s.host_time_ns,
                    s.gyro[0],
                    s.gyro[1],
                    s.gyro[2],
                    s.accel[0],
                    s.accel[1],
                    s.accel[2],
                    s.frame_sync as u8,
                    s.frame_sync_count
                )?;
            }
        }
        if !save {
            continue;
        }
        let name = format!("{}.pgm", frame.timestamp_ns);
        for eye in eyes {
            let grey = if *eye == "left" {
                &frame.left
            } else {
                &frame.right
            };
            write_pgm(
                &a.out.join(eye).join(&name),
                frame.width,
                frame.height,
                grey,
            )?;
        }
        saved += 1;
    }
    if let Some((mut frames, mut imu, mut exposure)) = csv {
        frames.flush()?;
        imu.flush()?;
        exposure.flush()?;
    }
    if let Source::Camera(camera) = &source
        && let (Some(r), Ok(timing)) = (camera.readout(), camera.sensor_timing())
    {
        fs::write(
            a.out.join("readout.csv"),
            format!(
                "line_ns,rows_before_image,pulse_to_readout_ns,frame_length,output_height\n{:.3},{},{},{},{}\n",
                r.line_ns,
                r.rows_before_image,
                r.pulse_to_readout_ns,
                timing.frame_length,
                timing.output_height
            ),
        )?;
    }
    let s = source.video().stats();
    println!(
        "{saved} frames to {} ({}x{}, camera S/N {}); {} buffers read, {} bad, {} frames lost by timestamp, {} ring drops{}",
        a.out.display(),
        VideoConfig::default().eye_width,
        VideoConfig::default().height,
        serial.as_deref().unwrap_or("unknown"),
        s.frames,
        s.bad_buffers,
        s.missing_by_time,
        s.ring_drops,
        if a.imu {
            format!("; {synced} of {seen} frames with a sync time")
        } else {
            String::new()
        }
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
    match run(a) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("zed_grab: {e}");
            ExitCode::FAILURE
        }
    }
}
