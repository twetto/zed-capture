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

    if imu.is_some() {
        match camera.frame_sync() {
            Ok(was) => println!(
                "frame marks: on (the left sensor's sync output was {})",
                if was { "already on" } else { "off: turned on" }
            ),
            Err(e) => println!("frame marks: not turned on: {e}"),
        }
    }
    match camera.sensor_timing() {
        Ok(t) => println!(
            "sensors: chip {:#06x}, {} pixel clocks a line, {} lines a frame, output {}x{}",
            t.chip_id, t.line_length, t.frame_length, t.output_width, t.output_height
        ),
        Err(e) => println!("sensors: {e}"),
    }

    let work = Duration::from_secs_f64(a.work_ms / 1e3);
    let mut sync = SyncCheck::default();
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
            sync.add(&frame, &samples);
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
            "imu: {} samples ({:.1} Hz), {} taken; missing {} by the MCU clock ({:.2} %); gaps over 5 ms {}, longest {:.1} ms; longest read gap {:.1} ms; not valid {}, bad reports {}, queue drops {}; frame marks {} ({} repeated, {} skipping, {} off the fit's line, {} fit restarts), MCU frame count +{}",
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
            i.mark_repeats,
            i.mark_skips,
            i.marks_off_line,
            i.mark_fit_restarts,
            mcu_frames.map(|n| n.to_string()).unwrap_or("?".into())
        );
    }
    if let Some(imu) = imu {
        sync.report(imu.frame_clock(), camera.arrival_periods());
    }
    if let Some(e) = camera.exposure_stats() {
        println!(
            "exposure (the right sensor's): read {} times ({:.1}/s since opening), not answered {}, {} failed{}; a read {:.2} ms on average, at most {:.2} ms; line {}",
            e.readings,
            e.readings as f64 / (a.skip + a.seconds),
            e.unanswered,
            e.failures,
            e.last_error
                .map(|m| format!(" (last: {m})"))
                .unwrap_or_default(),
            e.total_read_ns as f64 / e.readings.max(1) as f64 / 1e6,
            e.longest_read_ns as f64 / 1e6,
            camera
                .readout()
                .map(|r| format!("{:.3} µs", r.line_ns / 1e3))
                .unwrap_or("–".into())
        );
        sync.report_exposure();
    }
    Ok(())
}

fn ms(ns: i64) -> String {
    format!("{:.3}", ns as f64 / 1e6)
}

fn quantiles(v: &mut [i64]) -> String {
    if v.is_empty() {
        return "–".into();
    }
    v.sort();
    let p = |f: f64| v[((v.len() - 1) as f64 * f).round() as usize];
    format!(
        "min {} p10 {} median {} p90 {} max {} ms",
        ms(v[0]),
        ms(p(0.1)),
        ms(p(0.5)),
        ms(p(0.9)),
        ms(*v.last().unwrap())
    )
}

/// Each frame's pulse (by the MCU's clock) against its kernel stamp, and the marks against the
/// pulses.
#[derive(Default)]
struct SyncCheck {
    frames: u64,
    synced: u64,
    /// Kernel stamp − pulse.
    arrival: Vec<i64>,
    /// Pulse − the previous frame's pulse, when the frames are consecutive by the kernel.
    interval: Vec<i64>,
    /// The MCU's count from one frame to the next, when not 1 (by the kernel's sequence: 1).
    count_steps_off: u64,
    /// A mark's time − the pulse of the frame with its count.
    mark_after_pulse: Vec<i64>,
    pulses: std::collections::HashMap<u32, u64>,
    pending_marks: Vec<(u32, u64)>,
    prev: Option<(u32, u64, u32)>,
    /// Each frame's exposure.
    exposure: Vec<i64>,
    /// Frames with a pulse and without an exposure.
    exposure_missing: u64,
    /// Mid-exposure − pulse.
    mid_after_pulse: Vec<i64>,
}

impl SyncCheck {
    fn add(&mut self, frame: &zed_capture::Frame, samples: &[zed_capture::ImuSample]) {
        self.frames += 1;
        self.pending_marks.extend(
            samples
                .iter()
                .filter(|x| x.frame_sync)
                .map(|x| (x.frame_sync_count, x.time_ns)),
        );
        let (Some(t), Some(count)) = (frame.sync_time_ns, frame.mcu_frame_count) else {
            self.prev = None;
            return;
        };
        self.synced += 1;
        self.arrival.push(frame.timestamp_ns as i64 - t as i64);
        match frame.exposure_ns {
            Some(e) => self.exposure.push(e as i64),
            None => self.exposure_missing += 1,
        }
        if let Some(mid) = frame.mid_exposure_ns {
            self.mid_after_pulse.push(mid as i64 - t as i64);
        }
        if let Some((seq, pt, pcount)) = self.prev
            && frame.sequence.wrapping_sub(seq) == 1
        {
            self.interval.push(t as i64 - pt as i64);
            self.count_steps_off += (count.wrapping_sub(pcount) != 1) as u64;
        }
        self.prev = Some((frame.sequence, t, count));
        self.pulses.insert(count, t);
        let pulses = &self.pulses;
        let mut kept = Vec::new();
        for (c, mt) in self.pending_marks.drain(..) {
            match pulses.get(&c) {
                Some(&p) => self.mark_after_pulse.push(mt as i64 - p as i64),
                None => kept.push((c, mt)),
            }
        }
        self.pending_marks = kept;
    }

    fn report_exposure(&mut self) {
        println!(
            "  frames with a mid-exposure {}, with a pulse and no exposure {}; exposure {}",
            self.mid_after_pulse.len(),
            self.exposure_missing,
            quantiles(&mut self.exposure)
        );
        println!(
            "  middle row's mid-exposure after the pulse: {}",
            quantiles(&mut self.mid_after_pulse)
        );
    }

    fn report(&mut self, clock: Option<zed_capture::FrameClock>, arrival: Option<f64>) {
        let q = quantiles;
        match clock {
            Some(c) => println!(
                "sync: {} of {} frames on the IMU's clock; fit of {} marks: period {:.4} ms ({:.4} by the MCU, whose clock runs {:+.0} ppm), marks up to {:.3} ms above the line; arrival learned {}",
                self.synced,
                self.frames,
                c.marks,
                c.period_ns / 1e6,
                c.mcu_period_ns / 1e6,
                (1.0 / c.clock_rate - 1.0) * 1e6,
                c.spread_ns / 1e6,
                arrival
                    .map(|a| format!("{:.2} ms", a * c.period_ns / 1e6))
                    .unwrap_or("–".into())
            ),
            None => println!("sync: no frame marks fitted ({} frames)", self.frames),
        }
        println!("  kernel stamp after the pulse: {}", q(&mut self.arrival));
        println!(
            "  pulse to pulse, consecutive frames: {}; MCU count steps other than 1: {}",
            q(&mut self.interval),
            self.count_steps_off
        );
        println!(
            "  each mark after its frame's pulse (within one IMU interval, 2.5 ms, if matched): {}",
            q(&mut self.mark_after_pulse)
        );
    }
}
