//! zed-capture's own auto exposure on the camera ([`crate::auto_exposure`]) in place of the
//! camera's: each frame that already shows the newest setting is metered and steps the
//! controller; a thread of its own writes the new setting to both sensors just after a frame's
//! pulse (so both eyes take it on the same frame), reads it back, and keeps when it was written.
//!
//! A setting written before a frame's pulse is the one the frame after that takes (measured: a
//! write ending before pulse k lands on frame k + 1, for any write within the frame). So frame
//! n's setting is the newest written before pulse n − 1, and the stamps follow it exactly.

use std::collections::VecDeque;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::auto_exposure::{self, AutoExposureConfig, Controller, PredictionCheck, Setting};
use crate::sensor::{EYES, Sensors};
use crate::{gain_multiplier, monotonic_ns};

/// Writes start at least this long after a pulse (ns)…
const WRITE_AFTER_PULSE_NS: f64 = 1.0e6;
/// …and no later than this (ns), so that both eyes' writes (~8 ms) end well before the next.
const WRITE_BY_NS: f64 = 18.0e6;
/// Settings kept for frames to look up.
const HISTORY: usize = 64;
/// A setting that does not read back is written again this many times, then taken as written.
const RETRIES: u8 = 3;

/// A setting as written: when the writes ended, and whether each eye read it back (one that
/// did not after its retries is taken as written: the frames say it is unconfirmed).
#[derive(Clone, Copy, Debug)]
pub struct Applied {
    pub written_ns: u64,
    pub setting: Setting,
    pub raw_gain: [u16; 2],
    pub verified: [bool; 2],
}

/// What the writer is asked to do: a setting, and a pulse and frame period to time the writes by.
#[derive(Clone, Copy)]
struct Request {
    setting: Setting,
    pulse_ns: f64,
    period_ns: f64,
    retries: u8,
}

#[derive(Default)]
struct Shared {
    request: Option<Request>,
    history: VecDeque<Applied>,
    /// The newest setting asked for (written or not).
    commanded: Option<Setting>,
    stats: ActiveStats,
}

/// Counts since the controller started.
#[derive(Clone, Debug, Default)]
pub struct ActiveStats {
    pub steps: u64,
    pub writes: u64,
    /// Writes that did not read back, per eye (written again).
    pub unverified: [u64; 2],
    pub write_errors: u64,
    pub last_error: Option<String>,
    /// The controller's time on the frames it stepped, ns: in all and the longest.
    pub step_ns_total: u64,
    pub step_ns_max: u64,
    /// How well the tone curve predicted each frame after a step from the one before.
    pub prediction: PredictionCheck,
}

pub(crate) struct ActiveExposure {
    /// Locked only by the frames' thread (in [`ActiveExposure::frame`]) and for the stats.
    controller: Mutex<Controller>,
    max_lines: u32,
    shared: Arc<(Mutex<Shared>, Condvar)>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

fn write_setting(sensors: &Sensors, eye: usize, lines: u32, raw_gain: u16) -> io::Result<()> {
    let v = lines * 16;
    for (register, value) in [
        (0x3501, (v >> 8) as u8),
        (0x3502, v as u8),
        (0x3508, (raw_gain >> 8) as u8),
        (0x3509, raw_gain as u8),
    ] {
        sensors.write(eye, register, value)?;
    }
    Ok(())
}

impl ActiveExposure {
    /// Starts from `start` (written at once, untimed). The camera's own auto exposure must be
    /// off already.
    pub(crate) fn start(
        sensors: Sensors,
        config: AutoExposureConfig,
        line_ns: f64,
        max_lines: u32,
        start: Setting,
    ) -> io::Result<ActiveExposure> {
        let shared = Arc::new((Mutex::new(Shared::default()), Condvar::new()));
        let stop = Arc::new(AtomicBool::new(false));
        {
            let mut s = shared.0.lock().unwrap();
            s.commanded = Some(start);
            s.request = Some(Request {
                setting: start,
                pulse_ns: 0.0,
                period_ns: 0.0,
                retries: 0,
            });
        }
        let (sh, st) = (shared.clone(), stop.clone());
        let thread = std::thread::Builder::new()
            .name("zed-active-exposure".into())
            .spawn(move || writer(sensors, max_lines, sh, st))?;
        Ok(ActiveExposure {
            controller: Mutex::new(Controller::new(config, line_ns)),
            max_lines,
            shared,
            stop,
            thread: Some(thread),
        })
    }

    /// The setting a frame of pulse `pulse_ns` took: the newest written before the previous
    /// frame's pulse (`period_ns` earlier).
    pub(crate) fn applied_at(&self, pulse_ns: f64, period_ns: f64) -> Option<Applied> {
        let s = self.shared.0.lock().unwrap();
        s.history
            .iter()
            .rev()
            .find(|a| (a.written_ns as f64) < pulse_ns - period_ns)
            .copied()
    }

    /// Meters a frame that took `applied` and, if it is the newest setting asked for, steps the
    /// controller; a new setting goes to the writer, timed by this frame's pulse.
    pub(crate) fn frame(
        &self,
        applied: &Applied,
        left: &[u8],
        right: &[u8],
        width: usize,
        pulse_ns: f64,
        period_ns: f64,
    ) {
        // Only a frame that shows the newest setting, with nothing still being written. The
        // writer makes no request of its own while none is pending, so this holds while the
        // controller runs unlocked (the gradient mode takes a few ms).
        {
            let s = self.shared.0.lock().unwrap();
            if s.commanded != Some(applied.setting) || s.request.is_some() {
                return;
            }
        }
        let started = Instant::now();
        let next = self
            .controller
            .lock()
            .unwrap()
            .step(&applied.setting, [left, right], width);
        let took = started.elapsed().as_nanos() as u64;
        let next = Setting {
            lines: next.lines.min(self.max_lines),
            ..next
        };
        let mut s = self.shared.0.lock().unwrap();
        s.stats.steps += 1;
        s.stats.step_ns_total += took;
        s.stats.step_ns_max = s.stats.step_ns_max.max(took);
        if next != applied.setting {
            s.commanded = Some(next);
            s.request = Some(Request {
                setting: next,
                pulse_ns,
                period_ns,
                retries: 0,
            });
            self.shared.1.notify_one();
        }
    }

    pub(crate) fn stats(&self) -> ActiveStats {
        let prediction = self.controller.lock().unwrap().prediction_check();
        ActiveStats {
            prediction,
            ..self.shared.0.lock().unwrap().stats.clone()
        }
    }
}

/// Writes each request just after a pulse, reads it back, and writes again what did not take.
fn writer(
    sensors: Sensors,
    max_lines: u32,
    shared: Arc<(Mutex<Shared>, Condvar)>,
    stop: Arc<AtomicBool>,
) {
    let (lock, cv) = &*shared;
    loop {
        let request = {
            let mut s = lock.lock().unwrap();
            loop {
                if stop.load(Ordering::Relaxed) {
                    return;
                }
                if let Some(r) = s.request {
                    break r;
                }
                s = cv.wait_timeout(s, Duration::from_millis(100)).unwrap().0;
            }
        };
        // Into the next window after a pulse: [pulse + 1 ms, pulse + 18 ms]
        if request.period_ns > 0.0 {
            let now = monotonic_ns() as f64;
            let since = (now - request.pulse_ns).rem_euclid(request.period_ns);
            let wait = if since < WRITE_AFTER_PULSE_NS {
                WRITE_AFTER_PULSE_NS - since
            } else if since > WRITE_BY_NS {
                request.period_ns - since + WRITE_AFTER_PULSE_NS
            } else {
                0.0
            };
            std::thread::sleep(Duration::from_nanos(wait as u64));
        }
        let setting = Setting {
            lines: request.setting.lines.min(max_lines),
            ..request.setting
        };
        let raw_gain = setting.gain.map(auto_exposure::gain_raw);
        let mut result = Ok(());
        for eye in [1, 0] {
            if let Err(e) = write_setting(&sensors, eye, setting.lines, raw_gain[eye]) {
                result = Err(format!("the {} sensor: {e}", EYES[eye]));
            }
        }
        let written_ns = monotonic_ns();
        // Read back: the exposure and the gain, each eye
        let verified = [0, 1].map(|eye| {
            matches!(sensors.exposure_and_gain(eye, max_lines),
                Ok(Some((e, g))) if e == setting.lines * 16 && g == raw_gain[eye])
        });
        let mut s = lock.lock().unwrap();
        s.stats.writes += 1;
        if let Err(e) = result {
            s.stats.write_errors += 1;
            s.stats.last_error = Some(e);
        }
        for (count, v) in s.stats.unverified.iter_mut().zip(verified) {
            *count += !v as u64;
        }
        if s.history.len() == HISTORY {
            s.history.pop_front();
        }
        s.history.push_back(Applied {
            written_ns,
            setting,
            raw_gain,
            verified,
        });
        // Done, unless a newer request came meanwhile; what did not read back is written again
        if s.request.map(|r| r.setting) == Some(request.setting) {
            s.request =
                (verified != [true, true] && request.retries < RETRIES).then_some(Request {
                    retries: request.retries + 1,
                    ..request
                });
        }
    }
}

impl Drop for ActiveExposure {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.shared.1.notify_one();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// The setting the camera has now, from the right sensor's registers, both eyes at its gain
/// (the controller balances them from there); None if it did not answer.
pub(crate) fn current_setting(sensors: &Sensors, max_lines: u32) -> Option<Setting> {
    let (e, g) = sensors.exposure_and_gain(1, max_lines).ok()??;
    let gain = gain_multiplier(g)? as f64;
    Some(Setting {
        lines: (e / 16).max(1),
        gain: [gain; 2],
    })
}
