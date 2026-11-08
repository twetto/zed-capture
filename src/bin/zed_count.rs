//! Reads the ZED's video and IMU for a while and counts what was lost, and where.
//!
//!   zed_count [--seconds 60] [--skip 3] [--work-ms 0] [--hogs 0] [--buffers 8] [--ring 8] [--no-imu] [--gaps]
//!             [--video /dev/videoN --hid /dev/hidrawN]
//!
//! --work-ms: CPU work per frame in the reading thread (a consumer's stand-in).
//! --hogs: threads spinning on the CPU meanwhile.
//! --gaps: print each pair of frames taken whose sequence numbers or timestamps are not adjacent.
//! The first --skip seconds are not counted (start-up).

use std::hint::black_box;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use zed_capture::{Camera, VideoConfig, find};

struct Args {
    seconds: f64,
    skip: f64,
    work_ms: f64,
    hogs: usize,
    buffers: u32,
    ring: usize,
    video: Option<PathBuf>,
    hid: Option<PathBuf>,
    no_imu: bool,
    gaps: bool,
}

fn args() -> Result<Args, String> {
    let mut a = Args {
        seconds: 60.0,
        skip: 3.0,
        work_ms: 0.0,
        hogs: 0,
        buffers: 8,
        ring: 8,
        video: None,
        hid: None,
        no_imu: false,
        gaps: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        if flag == "--no-imu" || flag == "--gaps" {
            a.no_imu |= flag == "--no-imu";
            a.gaps |= flag == "--gaps";
            continue;
        }
        let value = it.next().ok_or(format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--seconds" => a.seconds = parsed(&flag, &value)?,
            "--skip" => a.skip = parsed(&flag, &value)?,
            "--work-ms" => a.work_ms = parsed(&flag, &value)?,
            "--hogs" => a.hogs = parsed(&flag, &value)?,
            "--buffers" => a.buffers = parsed(&flag, &value)?,
            "--ring" => a.ring = parsed(&flag, &value)?,
            "--video" => a.video = Some(value.into()),
            "--hid" => a.hid = Some(value.into()),
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(a)
}

fn parsed<T: std::str::FromStr>(flag: &str, value: &str) -> Result<T, String> {
    value.parse().map_err(|_| format!("{flag}: {value:?}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("zed_count: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let a = args()?;
    let (video_path, hid_path) = match (a.video.clone(), a.hid.clone()) {
        (Some(v), Some(h)) => (v, h),
        _ => {
            let d = find().map_err(|e| e.to_string())?;
            println!(
                "{} serial {}: {} and {}",
                d.model,
                d.serial.as_deref().unwrap_or("?"),
                d.video.display(),
                d.hid.display()
            );
            (d.video, d.hid)
        }
    };

    let stop = Arc::new(AtomicBool::new(false));
    let hogs: Vec<_> = (0..a.hogs)
        .map(|_| {
            let stop = stop.clone();
            std::thread::spawn(move || {
                let mut x = 0u64;
                while !stop.load(Ordering::Relaxed) {
                    x = black_box(x.wrapping_mul(6364136223846793005).wrapping_add(1));
                }
            })
        })
        .collect();

    let config = VideoConfig {
        buffers: a.buffers,
        ring: a.ring,
        ..Default::default()
    };
    let hid = (!a.no_imu).then_some(hid_path.as_path());
    let camera = Camera::open(&video_path, hid, config).map_err(|e| e.to_string())?;
    let (video, imu) = (camera.video(), camera.imu());
    println!(
        "{}x{} per eye at {:.2} fps, {} buffers, ring {}; {} hogs, {} ms work a frame; {} s after {} s",
        video.config().eye_width,
        video.config().height,
        video.fps(),
        video.buffers(),
        video.config().ring,
        a.hogs,
        a.work_ms,
        a.seconds,
        a.skip
    );

    let work = Duration::from_secs_f64(a.work_ms / 1e3);
    let start = Instant::now();
    let mut counting = false;
    let (mut taken, mut imu_taken, mut first_seq, mut last_seq) = (0u64, 0u64, None, 0u32);
    let mut most_pending = 0;
    let mut last_ts = 0u64;
    let (mut fewest, mut most, mut accel_norm_sum) = (usize::MAX, 0, 0.0f64);
    loop {
        let t = start.elapsed().as_secs_f64();
        if !counting && t >= a.skip {
            video.reset_stats();
            counting = true;
            if let Some(imu) = imu {
                imu.reset_stats();
            }
        }
        if t >= a.skip + a.seconds {
            break;
        }
        let Some((frame, samples)) = camera
            .next(Duration::from_millis(500))
            .map_err(|e| e.to_string())?
        else {
            continue;
        };
        if counting {
            fewest = fewest.min(samples.len());
            most = most.max(samples.len());
            accel_norm_sum += samples
                .iter()
                .map(|s| s.accel.iter().map(|a| (a * a) as f64).sum::<f64>().sqrt())
                .sum::<f64>();
            taken += 1;
            imu_taken += samples.len() as u64;
            if a.gaps
                && let Some(prev) = first_seq.map(|_| (last_seq, last_ts))
            {
                let dt = frame.timestamp_ns.saturating_sub(prev.1);
                if frame.sequence.wrapping_sub(prev.0) != 1 || dt > 50_000_000 {
                    println!(
                        "gap at {:.2} s: sequence {} -> {}, {:.1} ms",
                        t,
                        prev.0,
                        frame.sequence,
                        dt as f64 / 1e6
                    );
                }
            }
            first_seq.get_or_insert(frame.sequence);
            last_seq = frame.sequence;
            last_ts = frame.timestamp_ns;
            most_pending = most_pending.max(video.pending());
        }
        let busy_until = Instant::now() + work;
        while Instant::now() < busy_until {
            black_box(&frame.left);
        }
    }
    stop.store(true, Ordering::Relaxed);
    for h in hogs {
        let _ = h.join();
    }

    let v = video.stats();
    let secs = a.seconds;
    let span = first_seq
        .map(|f| last_seq.wrapping_sub(f) as u64 + 1)
        .unwrap_or(0);
    println!(
        "video: {} frames ({:.2}/s), {} taken over sequence numbers {} apart; camera frames missing {} by sequence, {} by time; ring drops {}, most waiting {}; bad buffers {}; longest interval {:.1} ms; split {:.2} ms a frame",
        v.frames,
        v.frames as f64 / secs,
        taken,
        span,
        v.missing_by_sequence,
        v.missing_by_time,
        v.ring_drops,
        most_pending,
        v.bad_buffers,
        v.longest_interval_ns as f64 / 1e6,
        if v.frames > 0 {
            v.split_ns as f64 / v.frames as f64 / 1e6
        } else {
            0.0
        }
    );
    if let Some(imu) = imu {
        let i = imu.stats();
        println!(
            "imu per frame: {} to {} samples; mean |accel| {:.3} m/s²",
            if most > 0 { fewest } else { 0 },
            most,
            accel_norm_sum / imu_taken.max(1) as f64
        );
        let mcu_frames = i
            .first_frame_count
            .map(|f| i.last_frame_count.wrapping_sub(f));
        println!(
            "imu: {} samples ({:.1} Hz), {} taken; missing {} by the MCU clock ({:.2} %); gaps over 5 ms {}, longest {:.1} ms; longest read gap {:.1} ms; not valid {}, bad reports {}, queue drops {}; frame marks {}, MCU frame count +{}",
            i.samples,
            i.samples as f64 / secs,
            imu_taken,
            i.missing,
            100.0 * i.missing as f64 / (i.samples + i.missing).max(1) as f64,
            i.gaps_over_5ms,
            i.longest_gap_ns as f64 / 1e6,
            i.longest_read_gap_ns as f64 / 1e6,
            i.not_valid,
            i.bad_reports,
            i.queue_drops,
            i.frame_syncs,
            mcu_frames.map(|n| n.to_string()).unwrap_or("?".into())
        );
    }
    Ok(())
}
