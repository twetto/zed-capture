//! Saves raw frames: left and right grey (the Y bytes) as 8-bit PGM files, and with --imu every
//! IMU sample and each frame's time on the IMU's clock.
//!
//!   zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--video /dev/videoN]
//!
//! `OUT/left/<ns>.pgm` and `OUT/right/<ns>.pgm`, named by the frame's kernel timestamp
//! (CLOCK_MONOTONIC ns), one frame in every --every, after --skip seconds for the auto exposure to
//! settle (and, with --imu, for the frame marks to be fitted). OUT/serial.txt holds the camera's
//! serial number, which names its factory calibration (`/usr/local/zed/settings/SN<serial>.conf`).
//!
//! --imu: through `Camera`, with the frame marks on. OUT/frames.csv, one line per frame after
//! --skip: its kernel stamp, its sync pulse on the IMU's clock (empty before the marks are
//! fitted), the MCU's count of it, whether its images were saved. OUT/imu.csv: every sample after
//! --skip, its time on the host's clock by the MCU's (`time_ns`), its MCU and read times, gyro
//! (rad/s), accel (m/s²), the frame mark and the MCU's frame count.

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
}

const USAGE: &str = "usage: zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--left-only] [--imu] [--video /dev/videoN]";

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
            _ => return Err(format!("unknown option {arg}\n{USAGE}")),
        }
    }
    a.out = out.ok_or(USAGE)?;
    if a.imu && a.video.is_some() {
        return Err("--imu finds the camera itself: no --video".into());
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
    let (source, serial) = match (&a.video, a.imu) {
        (Some(v), _) => (Source::Video(Video::open(v, VideoConfig::default())?), None),
        (None, false) => {
            let d = find()?;
            (
                Source::Video(Video::open(&d.video, VideoConfig::default())?),
                d.serial,
            )
        }
        (None, true) => {
            let d = find()?;
            let camera = Camera::open(&d.video, Some(&d.hid), VideoConfig::default())?;
            if let Err(e) = camera.frame_sync() {
                eprintln!("zed_grab: no frame marks ({e}): frames.csv without sync times");
            }
            (Source::Camera(camera), d.serial)
        }
    };
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
        writeln!(frames, "kernel_ns,sync_ns,mcu_count,saved")?;
        let mut imu = io::BufWriter::new(fs::File::create(a.out.join("imu.csv"))?);
        writeln!(
            imu,
            "time_ns,mcu_ns,read_ns,gx,gy,gz,ax,ay,az,frame_sync,frame_count"
        )?;
        Some((frames, imu))
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
        if let Some((frames, imu)) = &mut csv {
            let opt = |v: Option<u64>| v.map(|v| v.to_string()).unwrap_or_default();
            writeln!(
                frames,
                "{},{},{},{}",
                frame.timestamp_ns,
                opt(frame.sync_time_ns),
                opt(frame.mcu_frame_count.map(u64::from)),
                save as u8
            )?;
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
    if let Some((mut frames, mut imu)) = csv {
        frames.flush()?;
        imu.flush()?;
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
