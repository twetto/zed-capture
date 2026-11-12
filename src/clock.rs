//! The camera MCU's clock onto the host's, and the frames' times from the MCU's frame marks.
//!
//! Each IMU sample carries the MCU's clock (39.0625 µs ticks). With the image sensors' sync
//! output on ([`Video::enable_frame_sync`](crate::Video::enable_frame_sync)) the MCU marks the
//! sample of each frame and counts the frames. A mark is a 400 Hz sample, so it places its frame
//! only within 2.5 ms; but a frame period is no multiple of 2.5 ms, so successive marks fall at
//! different places on the samples' grid, and a line through them (time against the MCU's frame
//! count) places every frame much closer: its slope is the frame period, and it runs along the
//! marks' lower edge, which is the pulse if the MCU marks the first sample after it.

use std::collections::VecDeque;

/// The MCU's clock onto the host's: a line along the lower edge of (read time − MCU time)
/// against MCU time, since the read latency only adds. One point per one-second window, its
/// least-latency read, over the last `max_windows` windows; the slope by least squares, placed
/// under all the points. The MCU's clock does not keep the host's rate (LabOrin's ZED 2i: 0.4 %
/// fast, its ticks taken as 39.0625 µs), so an offset alone would drift.
#[derive(Debug)]
pub(crate) struct ClockMap {
    /// (MCU ns, read − MCU ns) of each window's least.
    windows: VecDeque<(u64, i64)>,
    max_windows: usize,
    current: Option<(u64, i64)>,
    window_start: Option<u64>,
}

impl ClockMap {
    pub(crate) fn new(max_windows: usize) -> Self {
        Self {
            windows: VecDeque::with_capacity(max_windows),
            max_windows,
            current: None,
            window_start: None,
        }
    }

    pub(crate) fn add(&mut self, mcu_ns: u64, host_ns: u64) {
        let start = *self.window_start.get_or_insert(host_ns);
        if host_ns.saturating_sub(start) >= 1_000_000_000 {
            if let Some(c) = self.current.take() {
                if self.windows.len() == self.max_windows {
                    self.windows.pop_front();
                }
                self.windows.push_back(c);
            }
            self.window_start = Some(host_ns);
        }
        let d = host_ns as i64 - mcu_ns as i64;
        if self.current.is_none_or(|(_, cd)| d < cd) {
            self.current = Some((mcu_ns, d));
        }
    }

    /// (read − MCU) = d0 + slope·(mcu − m0) along the lower edge: (m0, d0, slope).
    fn line(&self) -> Option<(f64, f64, f64)> {
        let points = || self.windows.iter().chain(self.current.iter());
        let &(m0, _) = points().next()?;
        let xy = || points().map(move |&(m, d)| ((m as f64 - m0 as f64) * 1e-9, d as f64));
        let slope = if self.windows.len() >= 2 {
            // Least squares over the closed windows (s → ns per s)
            let n = self.windows.len() as f64;
            let closed = || {
                self.windows
                    .iter()
                    .map(|&(m, d)| ((m as f64 - m0 as f64) * 1e-9, d as f64))
            };
            let (sx, sy) = closed().fold((0.0, 0.0), |(a, b), (x, y)| (a + x, b + y));
            let (mx, my) = (sx / n, sy / n);
            let (sxx, sxy) = closed().fold((0.0, 0.0), |(a, b), (x, y)| {
                (a + (x - mx) * (x - mx), b + (x - mx) * (y - my))
            });
            if sxx > 0.0 { sxy / sxx } else { 0.0 }
        } else {
            0.0
        };
        let d0 = xy()
            .map(|(x, y)| y - slope * x)
            .fold(f64::INFINITY, f64::min);
        Some((m0 as f64, d0, slope * 1e-9))
    }

    /// The host time (`CLOCK_MONOTONIC` ns) of an MCU time, the least read latency included.
    pub(crate) fn to_host(&self, mcu_ns: f64) -> Option<f64> {
        let (m0, d0, slope) = self.line()?;
        Some(mcu_ns + d0 + slope * (mcu_ns - m0))
    }

    /// Windows closed: the rate is known from 2.
    pub(crate) fn closed_windows(&self) -> usize {
        self.windows.len()
    }

    /// Host ns per MCU ns.
    pub(crate) fn rate(&self) -> f64 {
        self.line().map_or(1.0, |(_, _, slope)| 1.0 + slope)
    }
}

/// Each frame's sync pulse on the host's clock, fitted to the MCU's frame marks.
#[derive(Clone, Copy, Debug)]
pub struct FrameClock {
    /// A frame count of the MCU's (the newest mark's)…
    pub count: u32,
    /// …its pulse on the MCU clock (ns)…
    pub mcu_ns: f64,
    /// …and on the host's (`CLOCK_MONOTONIC` ns).
    pub host_ns: f64,
    /// The frame period by the host's clock (ns)…
    pub period_ns: f64,
    /// …and by the MCU's.
    pub mcu_period_ns: f64,
    /// Host ns per MCU ns.
    pub clock_rate: f64,
    /// Marks in the fit.
    pub marks: usize,
    /// How far above the line the marks lie (ns), the highest: under one sample interval
    /// (2.5 ms) when the line fits.
    pub spread_ns: f64,
}

impl FrameClock {
    /// The pulse of frame `count + frames` on the host's clock.
    pub fn host_ns_after(&self, frames: i64) -> f64 {
        self.host_ns + frames as f64 * self.period_ns
    }

    /// Frames from `count` to the host time `host_ns`, fractional.
    pub fn frames_to(&self, host_ns: f64) -> f64 {
        (host_ns - self.host_ns) / self.period_ns
    }
}

/// A line through the last `max_marks` marks: the slope by least squares, placed along the
/// marks' lower edge.
#[derive(Debug)]
pub(crate) struct MarkFit {
    marks: VecDeque<(u32, u64)>,
    max_marks: usize,
    /// Marks in a row left out as off the line.
    off_line: usize,
}

/// What became of a mark.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Mark {
    Fitted,
    /// The previous mark's count again: left out.
    Repeat,
    /// Off the line by more than a sample interval or so: left out.
    OffLine,
    /// The fit started over with it: the count went back or jumped, the MCU's clock went back,
    /// or `RESTART_AFTER` marks in a row were off the line.
    Restart,
}

/// Marks before the line is used to leave marks out.
const CHECK_AFTER: usize = 10;
/// How far from the line a mark may be: the marks lie on the line's upper side, within one
/// sample interval (2.5 ms), with some margin.
const OFF_LINE_NS: std::ops::RangeInclusive<f64> = -2.0e6..=5.0e6;
const RESTART_AFTER: usize = 5;

impl MarkFit {
    pub(crate) fn new(max_marks: usize) -> Self {
        Self {
            marks: VecDeque::with_capacity(max_marks),
            max_marks,
            off_line: 0,
        }
    }

    /// A mark: the MCU's frame count and the marked sample's MCU time.
    pub(crate) fn add(&mut self, count: u32, mcu_ns: u64) -> Mark {
        let mut restart = false;
        if let Some(&(c, t)) = self.marks.back() {
            let step = count.wrapping_sub(c);
            if step == 0 {
                return Mark::Repeat;
            }
            restart = step > 1_000 || mcu_ns <= t;
            if !restart
                && self.marks.len() >= CHECK_AFTER
                && let Some((c_line, t_line, period, _)) = self.line()
            {
                let off = mcu_ns as f64 - (t_line + count.wrapping_sub(c_line) as f64 * period);
                if OFF_LINE_NS.contains(&off) {
                    self.off_line = 0;
                } else {
                    self.off_line += 1;
                    if self.off_line < RESTART_AFTER {
                        return Mark::OffLine;
                    }
                    restart = true;
                }
            }
        }
        if restart {
            self.marks.clear();
            self.off_line = 0;
        }
        if self.marks.len() == self.max_marks {
            self.marks.pop_front();
        }
        self.marks.push_back((count, mcu_ns));
        if restart { Mark::Restart } else { Mark::Fitted }
    }

    /// (the newest mark's count, the line's MCU time there, the period, the marks' spread above
    /// the line); None with fewer than 3 marks.
    fn line(&self) -> Option<(u32, f64, f64, f64)> {
        let n = self.marks.len();
        if n < 3 {
            return None;
        }
        let &(c_last, t_last) = self.marks.back().unwrap();
        // x: frames before the newest mark (≤ 0); y: ns before it
        let xy = || {
            self.marks
                .iter()
                .map(move |&(c, t)| (-(c_last.wrapping_sub(c) as f64), t as f64 - t_last as f64))
        };
        let (sx, sy) = xy().fold((0.0, 0.0), |(a, b), (x, y)| (a + x, b + y));
        let (mx, my) = (sx / n as f64, sy / n as f64);
        let (sxx, sxy) = xy().fold((0.0, 0.0), |(a, b), (x, y)| {
            (a + (x - mx) * (x - mx), b + (x - mx) * (y - my))
        });
        if sxx <= 0.0 {
            return None;
        }
        let period = sxy / sxx;
        let residuals = || xy().map(|(x, y)| y - period * x);
        let low = residuals().fold(f64::INFINITY, f64::min);
        let high = residuals().fold(f64::NEG_INFINITY, f64::max);
        Some((c_last, t_last as f64 + low, period, high - low))
    }

    /// The line, onto the host's clock with `map`; None with fewer than 3 marks.
    pub(crate) fn clock(&self, map: &ClockMap) -> Option<FrameClock> {
        let (count, mcu_ns, period, spread) = self.line()?;
        let rate = map.rate();
        Some(FrameClock {
            count,
            mcu_ns,
            host_ns: map.to_host(mcu_ns)?,
            period_ns: period * rate,
            mcu_period_ns: period,
            clock_rate: rate,
            marks: self.marks.len(),
            spread_ns: spread,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An MCU clock `ppm` fast, reads 0.3-2.3 ms late (the least at every 7th), one per 2.5 ms
    /// of MCU time for `secs`.
    fn feed(map: &mut ClockMap, ppm: f64, secs: u64) -> impl Fn(f64) -> f64 + use<> {
        let host_of = move |mcu: f64| 7e9 + mcu / (1.0 + ppm * 1e-6);
        for k in 0..secs * 400 {
            let mcu = k as f64 * 2.5e6;
            let late = 0.3e6
                + if k % 7 == 0 {
                    0.0
                } else {
                    ((k * 37) % 20) as f64 * 0.1e6
                };
            map.add(mcu as u64, (host_of(mcu) + late) as u64);
        }
        host_of
    }

    #[test]
    fn the_map_follows_a_clock_that_runs_fast_or_slow() {
        for ppm in [4300.0, -4300.0, 0.0] {
            let mut map = ClockMap::new(20);
            let host_of = feed(&mut map, ppm, 30);
            for mcu in [20e9, 29e9, 30e9] {
                let err = map.to_host(mcu).unwrap() - host_of(mcu);
                // The least latency (0.3 ms) is in the map; what is left: a few µs
                assert!((err - 0.3e6).abs() < 20e3, "{ppm} ppm at {mcu}: {err}");
            }
            assert!(
                (map.rate() - 1.0 / (1.0 + ppm * 1e-6)).abs() < 2e-6,
                "{ppm}: {}",
                map.rate()
            );
        }
    }

    #[test]
    fn one_window_gives_the_offset() {
        let mut map = ClockMap::new(20);
        assert!(map.to_host(0.0).is_none());
        map.add(1_000, 5_000_001_500);
        map.add(2_000, 5_000_002_200);
        assert_eq!(map.to_host(10_000.0), Some(5_000_010_200.0));
        assert_eq!(map.rate(), 1.0);
    }

    /// Marks: the first 400 Hz sample at or after each pulse, as the MCU's clock ticks.
    fn marks(period_ns: f64, first_pulse_ns: f64, n: u32) -> Vec<(u32, u64)> {
        (0..n)
            .map(|k| {
                let pulse = first_pulse_ns + k as f64 * period_ns;
                let sample = (pulse / 2.5e6).ceil() * 2.5e6;
                (100 + k, sample as u64)
            })
            .collect()
    }

    #[test]
    fn the_line_finds_the_pulses_between_the_samples() {
        let (period, first) = (33_380_000.0, 1_234_567.0);
        let mut fit = MarkFit::new(300);
        for (c, t) in marks(period, first, 300) {
            fit.add(c, t);
        }
        let mut map = ClockMap::new(20);
        map.add(0, 1_000);
        let clock = fit.clock(&map).unwrap();
        assert_eq!(clock.count, 399);
        assert!((clock.mcu_period_ns - period).abs() < 200.0, "{clock:?}");
        let pulse = first + 299.0 * period;
        assert!((clock.mcu_ns - pulse).abs() < 100_000.0, "{clock:?}");
        assert!((clock.host_ns - clock.mcu_ns - 1_000.0).abs() < 1e-3);
        assert!(
            clock.spread_ns < 2.5e6 && clock.spread_ns > 2.0e6,
            "{clock:?}"
        );
        assert!((clock.frames_to(clock.host_ns_after(-7)) + 7.0).abs() < 1e-9);
    }

    #[test]
    fn marks_off_the_line_are_left_out_and_five_in_a_row_restart() {
        let period = 33.4e6;
        let mut fit = MarkFit::new(300);
        let m = marks(period, 0.0, 60);
        for &(c, t) in &m[..40] {
            fit.add(c, t);
        }
        // The same count again, then a count marked a frame late
        assert_eq!(fit.add(139, m[39].1 + 2_500_000), Mark::Repeat);
        assert_eq!(fit.add(140, m[41].1), Mark::OffLine);
        assert_eq!(fit.add(141, m[41].1), Mark::Fitted);
        // A new phase: five off the line, then the fit follows it
        for k in 0..4 {
            assert_eq!(
                fit.add(142 + k, m[42 + k as usize].1 + 10_000_000),
                Mark::OffLine
            );
        }
        assert_eq!(fit.add(146, m[46].1 + 10_000_000), Mark::Restart);
    }

    #[test]
    fn a_count_going_back_starts_over() {
        let mut fit = MarkFit::new(300);
        for (c, t) in marks(33.3e6, 0.0, 10) {
            fit.add(c, t);
        }
        fit.add(5, 1_000_000_000);
        let mut map = ClockMap::new(20);
        map.add(0, 0);
        assert!(fit.clock(&map).is_none());
    }
}
