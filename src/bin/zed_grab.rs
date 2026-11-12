//! Saves raw frames: left and right grey (the Y bytes) as 8-bit PGM files.
//!
//!   zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--video /dev/videoN]
//!
//! OUT/left/<ns>.pgm and OUT/right/<ns>.pgm, named by the frame's timestamp (CLOCK_MONOTONIC ns),
//! one frame in every --every, after --skip seconds for the auto exposure to settle. OUT/serial.txt
//! holds the camera's serial number, which names its factory calibration
//! (/usr/local/zed/settings/SN<serial>.conf).

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use zed_capture::{Video, VideoConfig, find};

struct Args {
    out: PathBuf,
    frames: usize,
    every: usize,
    skip: f64,
    video: Option<PathBuf>,
}

fn args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut a = Args {
        out: PathBuf::new(),
        frames: 5,
        every: 15,
        skip: 3.0,
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
        let value = it.next().ok_or(format!("{arg} needs a value"))?;
        let bad = || format!("{arg}: bad value {value}");
        match arg.as_str() {
            "--frames" => a.frames = value.parse().map_err(|_| bad())?,
            "--every" => a.every = value.parse::<usize>().map_err(|_| bad())?.max(1),
            "--skip" => a.skip = value.parse().map_err(|_| bad())?,
            "--video" => a.video = Some(PathBuf::from(value)),
            _ => return Err(format!("unknown option {arg}")),
        }
    }
    a.out = out
        .ok_or("usage: zed_grab OUT [--frames 5] [--every 15] [--skip 3] [--video /dev/videoN]")?;
    Ok(a)
}

fn write_pgm(path: &Path, width: usize, height: usize, grey: &[u8]) -> io::Result<()> {
    let mut f = io::BufWriter::new(fs::File::create(path)?);
    write!(f, "P5\n{width} {height}\n255\n")?;
    f.write_all(grey)?;
    f.flush()
}

fn run(a: Args) -> io::Result<()> {
    let (video_path, serial) = match &a.video {
        Some(v) => (v.clone(), None),
        None => {
            let d = find()?;
            (d.video, d.serial)
        }
    };
    for eye in ["left", "right"] {
        fs::create_dir_all(a.out.join(eye))?;
    }
    if let Some(s) = &serial {
        fs::write(a.out.join("serial.txt"), format!("{s}\n"))?;
    }
    let video = Video::open(&video_path, VideoConfig::default())?;
    let t_start = Instant::now();
    let (mut seen, mut saved) = (0usize, 0usize);
    while saved < a.frames {
        let Some(frame) = video.next(Duration::from_secs(3))? else {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "no frame in 3 s"));
        };
        if t_start.elapsed().as_secs_f64() < a.skip {
            continue;
        }
        seen += 1;
        if (seen - 1) % a.every != 0 {
            continue;
        }
        let name = format!("{}.pgm", frame.timestamp_ns);
        write_pgm(
            &a.out.join("left").join(&name),
            frame.width,
            frame.height,
            &frame.left,
        )?;
        write_pgm(
            &a.out.join("right").join(&name),
            frame.width,
            frame.height,
            &frame.right,
        )?;
        saved += 1;
    }
    let s = video.stats();
    println!(
        "{saved} frames to {} ({}x{}, camera S/N {}); {} buffers read, {} bad, {} frames lost by timestamp",
        a.out.display(),
        VideoConfig::default().eye_width,
        VideoConfig::default().height,
        serial.as_deref().unwrap_or("unknown"),
        s.frames,
        s.bad_buffers,
        s.missing_by_time
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
