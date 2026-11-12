//! Auto exposure for feature tracking, run by zed-capture instead of the camera's: the "mean"
//! mode of `v4l2_trigger_camera`'s `auto_exposure.hpp`, for the ZED's two sensors.
//!
//! What tracking needs from it (as there): a tracked patch's brightness should barely change from
//! frame to frame, so each step changes exposure × gain by at most `max_step_ratio` and small
//! errors are held (`deadband`); exposure before gain (gain amplifies noise); clipped highlights
//! push the exposure down, but never below the floor of the unclipped pixels' level.
//!
//! For the ZED:
//! - Both eyes get one exposure, so their images share their timing (the stamps) and their blur;
//!   each eye gets its own gain, so their brightness matches (the left gets about half the
//!   right's light). The exposure is the one the dimmer-wanting eye can take.
//! - Exposures past one flicker period of the lights keep to whole periods (8.33 ms at 60 Hz
//!   mains), as the camera's own auto exposure does: under mains lights other exposures flicker
//!   from frame to frame and band the rolling shutter's rows. Gain makes up the rest.
//! - The camera's output is not linear in light (its tone curve); like the original, the loop
//!   only needs brightness to rise with exposure × gain, and it closes every step.
//!
//! The gradient mode ([`GradientConfig`]) is the original's too, after Zhang, Forster and
//! Scaramuzza, "Active Exposure Control for Robust Visual Odometry in HDR Environments" (ICRA
//! 2017): each eye's exposure × gain goes to whichever of a step down, none and a step up gives the
//! most image gradient (a soft percentile of the squared gradients), judged on the frame as it
//! would look after the step. The original predicts that image as linear in light; the ZED's tone
//! curve is far from it (a knee at about level 100), so here it goes through the measured curve
//! ([`ToneCurve::zed`]): at a 12 % step, linear mispredicts the brighter pixels by ~17 levels, the
//! curve by ~3.

/// Settings of the controller.
#[derive(Clone, Debug)]
pub struct AutoExposureConfig {
    /// The metered mean wanted, 0-255.
    pub target_level: f64,
    /// The output at no light.
    pub black_level: f64,
    /// |log error| below this: hold.
    pub deadband: f64,
    /// The fraction of the log error corrected per step.
    pub correction: f64,
    /// Exposure × gain changes by at most this ratio per step.
    pub max_step_ratio: f64,
    /// Above this fraction of clipped pixels darken; brighten again only below
    /// `saturation_resume`.
    pub saturation_limit: f64,
    pub saturation_resume: f64,
    /// Darkening for clipping stops where the unclipped pixels' mean would go below this.
    pub saturation_floor_level: f64,
    /// The longest exposure (ns): the motion-blur cap. Rounded down to whole flicker periods.
    pub exposure_max_ns: f64,
    /// The lights' flicker (twice the mains frequency), Hz; None: exposures are not kept to it.
    pub flicker_hz: Option<f64>,
    /// The highest gain multiplier (the OV4689's: 15.94).
    pub gain_max: f64,
    /// The gradient mode, with its settings; None: the mean mode.
    pub gradient: Option<GradientConfig>,
}

impl Default for AutoExposureConfig {
    fn default() -> Self {
        AutoExposureConfig {
            target_level: 100.0,
            black_level: 0.0,
            deadband: 0.06,
            correction: 0.5,
            max_step_ratio: 1.12,
            saturation_limit: 0.05,
            saturation_resume: 0.025,
            saturation_floor_level: 60.0,
            // Three 60 Hz flicker periods: the longest that fits in a 33 ms frame
            exposure_max_ns: 25.0e6,
            flicker_hz: Some(120.0),
            gain_max: MAX_GAIN,
            gradient: None,
        }
    }
}

/// Settings of the gradient mode (the original's defaults, but `step`).
#[derive(Clone, Debug)]
pub struct GradientConfig {
    /// The soft percentile's p and k (Zhang et al.'s eq. (10); they suggest p 0.7-0.8, k 5).
    pub percentile: f64,
    pub k: i32,
    /// Gradients are taken at every `step`-th pixel of every `step`-th row (the original: 2, for
    /// one eye; two eyes here).
    pub step: usize,
    /// A step is taken only if the metric gains this ratio or more.
    pub min_improvement: f64,
    /// Below this mean the frame is too dark to judge: brighten, past the exposure cap by gain.
    /// Gain past the cap is shed while a step down keeps the mean 10 % above it. Gain is never
    /// raised for the metric alone: it amplifies noise, which the prediction does not show.
    pub gain_floor_level: f64,
    /// The camera's tone curve, to predict the frame after a step.
    pub tone: ToneCurve,
}

impl Default for GradientConfig {
    fn default() -> Self {
        GradientConfig {
            percentile: 0.75,
            k: 5,
            step: 4,
            min_improvement: 1.01,
            gain_floor_level: 60.0,
            tone: ToneCurve::zed(),
        }
    }
}

/// The highest gain: 8 × 255 / 128.
pub const MAX_GAIN: f64 = 8.0 * 255.0 / 128.0;

/// Both sensors' setting: one exposure, each eye's gain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Setting {
    /// Exposure, in the sensor's lines.
    pub lines: u32,
    /// Each eye's gain, a multiplier (1 to [`MAX_GAIN`]), one the sensor can take
    /// ([`gain_raw`]).
    pub gain: [f64; 2],
}

/// The sensor's raw gain for a multiplier (1 to [`MAX_GAIN`]): 2^band × low byte / 128, the
/// bands 0x0xx, 0x1xx, 0x3xx, 0x7xx (see [`crate::gain_multiplier`]), the low byte 128 to 255.
pub fn gain_raw(multiplier: f64) -> u16 {
    let m = multiplier.clamp(1.0, MAX_GAIN);
    let band = (m.log2().floor() as usize).min(3);
    let low = (m / (1u32 << band) as f64 * 128.0)
        .round()
        .clamp(128.0, 255.0) as u16;
    [0x000, 0x100, 0x300, 0x700][band] | low
}

/// The multiplier [`gain_raw`] gives.
pub fn gain_of_raw(raw: u16) -> f64 {
    crate::gain_multiplier(raw).unwrap_or(1.0) as f64
}

/// One eye's metering: its mean (clipped pixels count as 255) and its fraction at 255.
#[derive(Clone, Copy, Debug, Default)]
pub struct Meter {
    pub mean: f64,
    pub saturated: f64,
}

/// The mean and the clipped fraction of every `step`-th pixel of every `step`-th row.
pub fn meter(grey: &[u8], width: usize, step: usize) -> Meter {
    let (mut sum, mut clipped, mut n) = (0u64, 0u64, 0u64);
    for row in grey.chunks_exact(width).step_by(step) {
        for &v in row.iter().step_by(step) {
            sum += v as u64;
            clipped += (v == 255) as u64;
            n += 1;
        }
    }
    let n = n.max(1) as f64;
    Meter {
        mean: sum as f64 / n,
        saturated: clipped as f64 / n,
    }
}

/// The mean of the unclipped pixels, from the mean (clipped counted as 255) and the fraction
/// clipped.
fn unclipped_mean(m: Meter) -> f64 {
    if m.saturated >= 1.0 {
        255.0
    } else {
        (m.mean - 255.0 * m.saturated) / (1.0 - m.saturated)
    }
}

/// Clipping against the floor on the rest of the frame: whether the unclipped pixels are below
/// `saturation_floor_level` (starved), whether a step down would leave them at or above it, and
/// the log ratio up to it.
struct ClipFloor {
    starved: bool,
    may_darken: bool,
    log_to_floor: f64,
}

fn clip_floor(c: &AutoExposureConfig, m: Meter) -> ClipFloor {
    let floor = (c.saturation_floor_level - c.black_level).max(1.0);
    let clear = (unclipped_mean(m) - c.black_level).max(1.0);
    ClipFloor {
        starved: clear < floor,
        may_darken: clear / c.max_step_ratio >= floor,
        log_to_floor: (floor / clear).ln(),
    }
}

/// One eye's wanted change of exposure × gain, as a log ratio (0: hold), from its metering.
fn wanted_log_step(c: &AutoExposureConfig, m: Meter) -> f64 {
    let signal = (m.mean - c.black_level).max(1.0);
    let wanted = (c.target_level - c.black_level).max(1.0);
    let mut log_err = (wanted / signal).ln();
    let max_log = c.max_step_ratio.ln();
    // Clipping: darken at least a step, unless the unclipped pixels would fall below the floor;
    // below the floor, brighten towards it whatever the clipping (a lamp clips at any exposure)
    let f = clip_floor(c, m);
    if m.saturated > c.saturation_limit && f.may_darken {
        log_err = log_err.min(-max_log);
    } else if f.starved {
        log_err = log_err.max(f.log_to_floor);
        if log_err < c.deadband {
            return 0.0;
        }
    } else if log_err.abs() < c.deadband
        || m.saturated > c.saturation_limit
        || (log_err > 0.0 && m.saturated > c.saturation_resume)
    {
        return 0.0;
    }
    (c.correction * log_err).clamp(-max_log, max_log)
}

/// Exposure × gain (in lines × multiplier) of an eye.
fn product(s: &Setting, eye: usize) -> f64 {
    s.lines as f64 * s.gain[eye]
}

/// The exposure (lines) for an exposure × gain, exposure first: up to the cap, past one flicker
/// period in whole periods.
fn exposure_for(c: &AutoExposureConfig, line_ns: f64, wanted_product: f64) -> u32 {
    // A part of a line over the cap is rounding (three 60 Hz periods are 623.97 lines and the
    // 25 ms cap 623.97 too): whole periods within it
    let cap = (c.exposure_max_ns / line_ns + 0.01).max(1.0);
    let mut lines = wanted_product.clamp(1.0, cap);
    if let Some(hz) = c.flicker_hz {
        let period = 1e9 / hz / line_ns;
        if lines >= period {
            lines = ((lines + 0.01) / period).floor() * period;
        }
    }
    (lines.floor() as u32).max(1)
}

/// One step for both eyes from their metering: each eye's exposure × gain moved by its own
/// wanted step, the common exposure the smaller eye's, each eye's gain the rest. `current` is
/// returned when neither eye wants a change.
pub fn step(
    c: &AutoExposureConfig,
    line_ns: f64,
    current: &Setting,
    meters: [Meter; 2],
) -> Setting {
    let steps = meters.map(|m| wanted_log_step(c, m));
    if steps == [0.0, 0.0] && current.lines <= exposure_for(c, line_ns, f64::MAX) {
        return *current;
    }
    split(
        c,
        line_ns,
        [0, 1].map(|eye| product(current, eye) * steps[eye].exp()),
    )
}

/// Both eyes' setting for their wanted exposure × gain: the exposure the smaller one's (gain
/// does not go below 1), each eye's gain the rest.
fn split(c: &AutoExposureConfig, line_ns: f64, wanted: [f64; 2]) -> Setting {
    let lines = exposure_for(c, line_ns, wanted[0].min(wanted[1]));
    let gain = wanted.map(|w| gain_of_raw(gain_raw(w / lines as f64)).min(c.gain_max));
    Setting { lines, gain }
}

// ---- The tone curve ----------------------------------------------------------------------------

/// The ZED 2i's tone curve, both eyes alike (within 0.05 in log light: the ISP's, not the
/// sensors'), at the camera's default image controls: its slope h(z) = dz / d ln(exposure × gain)
/// at output levels z, every 8 levels. Measured on a still scene over exposure and gain steps,
/// which agree (2026-11-13, docs/measurements.md): about linear above a black level of 5 up to
/// level 100, compressed above, a soft shoulder from ~215 into the clip. Only its shape matters
/// here (a more or less sensitive sensor shifts the light, not the curve); other firmware or image
/// controls (gamma, contrast) may change it.
pub const ZED_TONE_SLOPE: [(u8, f64); 33] = [
    (0, 1.0),
    (8, 3.0),
    (16, 11.7),
    (24, 19.9),
    (32, 26.6),
    (40, 33.2),
    (48, 40.7),
    (56, 47.4),
    (64, 54.2),
    (72, 61.2),
    (80, 67.4),
    (88, 76.1),
    (96, 86.3),
    (104, 89.4),
    (112, 89.0),
    (120, 85.9),
    (128, 87.0),
    (136, 91.4),
    (144, 94.4),
    (152, 99.7),
    (160, 104.7),
    (168, 106.6),
    (176, 107.3),
    (184, 111.9),
    (192, 114.2),
    (200, 112.4),
    (208, 111.2),
    (216, 108.0),
    (224, 81.6),
    (232, 71.0),
    (240, 42.5),
    (248, 9.8),
    (255, 1.0),
];

/// A camera's tone curve: the log of the light (× exposure × gain) of each output level, 0 at
/// level 100.
#[derive(Clone, Debug)]
pub struct ToneCurve {
    log_light: Vec<f64>,
}

impl ToneCurve {
    /// From the curve's slope dz / d ln(light) at two or more levels (ascending, the first at 0,
    /// the last at 255), linear between them, at least 1.
    pub fn from_slope(knots: &[(u8, f64)]) -> ToneCurve {
        assert!(knots.len() >= 2, "a tone curve needs two slopes or more");
        let h: Vec<f64> = (0..256)
            .map(|z| {
                let z = z as f64;
                let i = knots
                    .partition_point(|k| k.0 as f64 <= z)
                    .clamp(1, knots.len() - 1);
                let (a, b) = (knots[i - 1], knots[i]);
                let t = ((z - a.0 as f64) / (b.0 as f64 - a.0 as f64).max(1.0)).clamp(0.0, 1.0);
                (a.1 + t * (b.1 - a.1)).max(1.0)
            })
            .collect();
        let mut log_light = vec![0.0; 256];
        for z in 1..256 {
            log_light[z] = log_light[z - 1] + (1.0 / h[z - 1] + 1.0 / h[z]) / 2.0;
        }
        let at = log_light[100];
        log_light.iter_mut().for_each(|g| *g -= at);
        ToneCurve { log_light }
    }

    /// [`ZED_TONE_SLOPE`]'s.
    pub fn zed() -> ToneCurve {
        ToneCurve::from_slope(&ZED_TONE_SLOPE)
    }

    /// The output level (fractional) of a log light.
    fn level(&self, x: f64) -> f64 {
        let g = &self.log_light;
        if x <= g[0] {
            return 0.0;
        }
        if x >= g[255] {
            return 255.0;
        }
        let i = g.partition_point(|&v| v <= x);
        (i - 1) as f64 + (x - g[i - 1]) / (g[i] - g[i - 1])
    }

    /// The level each level would read at `ratio` times the exposure × gain. A clipped pixel
    /// (255) stays clipped: what it hides is unknown, so darkening earns no credit for it.
    pub fn map(&self, ratio: f64) -> [u8; 256] {
        let d = ratio.ln();
        let mut lut = [255u8; 256];
        for (z, out) in lut.iter_mut().enumerate().take(255) {
            *out = self.level(self.log_light[z] + d).round().clamp(0.0, 255.0) as u8;
        }
        lut
    }
}

// ---- The gradient mode -------------------------------------------------------------------------

/// |dI/dx| + |dI/dy| of 8-bit central differences: 0 to 510.
const BINS: usize = 511;

/// One eye's look at a frame for the gradient mode: the metric at exposure × gain × 1/s, 1 and s,
/// its metering, its clipped fraction after a step up and its mean after a step down.
struct GradientLook {
    metric: [f64; 3],
    meter: Meter,
    clipped_up: f64,
    mean_down: f64,
}

/// Zhang et al.'s M_softperc (eq. (9), (10)): the squared gradients' mean, weighted by their
/// rank, the weight peaking at the `percentile`-th. A histogram of gradient magnitude stands in
/// for sorting (the original's); each bin's mean squared gradient takes the weight of the ranks
/// it spans.
#[derive(Clone)]
struct GradientMetric {
    samples: usize,
    percentile: f64,
    k: i32,
    /// Cumulative rank weights, normalised.
    cum: Vec<f64>,
    count: [[u32; BINS]; 3],
    sum: [[u64; BINS]; 3],
}

impl GradientMetric {
    fn new() -> GradientMetric {
        GradientMetric {
            samples: 0,
            percentile: -1.0,
            k: -1,
            cum: Vec::new(),
            count: [[0; BINS]; 3],
            sum: [[0; BINS]; 3],
        }
    }

    /// The rank weights, for the number of samples of a `width × height` image.
    fn prepare(&mut self, width: usize, height: usize, g: &GradientConfig) {
        let n = |len: usize| len.saturating_sub(2).div_ceil(g.step.max(1));
        let samples = n(width) * n(height);
        if samples == self.samples && g.percentile == self.percentile && g.k == self.k {
            return;
        }
        (self.samples, self.percentile, self.k) = (samples, g.percentile, g.k);
        let s = samples as f64;
        let ps = (g.percentile * s).floor();
        let w: Vec<f64> = (0..samples)
            .map(|i| {
                let x = i as f64;
                let v = if x <= ps {
                    (std::f64::consts::FRAC_PI_2 / ps.max(1.0) * x).sin()
                } else {
                    (std::f64::consts::FRAC_PI_2 * (1.0 - (x - ps) / (s - ps).max(1.0))).sin()
                };
                v.max(0.0).powi(g.k)
            })
            .collect();
        let total: f64 = w.iter().sum::<f64>().max(f64::MIN_POSITIVE);
        self.cum = Vec::with_capacity(samples + 1);
        self.cum.push(0.0);
        let mut acc = 0.0;
        for x in w {
            acc += x / total;
            self.cum.push(acc);
        }
    }

    /// One pass over a grey image: the metric after each of `luts` (a step down, none, a step
    /// up), the metering, the clipping after the step up and the mean after the step down.
    fn look(
        &mut self,
        grey: &[u8],
        width: usize,
        g: &GradientConfig,
        luts: &[[u8; 256]; 3],
    ) -> GradientLook {
        let height = grey.len() / width.max(1);
        self.prepare(width, height, g);
        self.count = [[0; BINS]; 3];
        self.sum = [[0; BINS]; 3];
        let (mut sum, mut clipped, mut clipped_up, mut sum_down, mut n) =
            (0u64, 0u64, 0u64, 0u64, 0u64);
        let step = g.step.max(1);
        for y in (1..height.saturating_sub(1)).step_by(step) {
            let up = &grey[(y - 1) * width..y * width];
            let row = &grey[y * width..(y + 1) * width];
            let dn = &grey[(y + 1) * width..(y + 2) * width];
            for x in (1..width - 1).step_by(step) {
                let v = row[x] as usize;
                sum += v as u64;
                clipped += (v == 255) as u64;
                clipped_up += (luts[2][v] == 255) as u64;
                sum_down += luts[0][v] as u64;
                n += 1;
                for (i, lut) in luts.iter().enumerate() {
                    let gx = lut[row[x + 1] as usize] as i32 - lut[row[x - 1] as usize] as i32;
                    let gy = lut[dn[x] as usize] as i32 - lut[up[x] as usize] as i32;
                    let bin = (gx.abs() + gy.abs()) as usize;
                    self.count[i][bin] += 1;
                    self.sum[i][bin] += (gx * gx + gy * gy) as u64;
                }
            }
        }
        let metric = [0, 1, 2].map(|i| {
            let (mut m, mut rank) = (0.0, 0usize);
            for b in 0..BINS {
                let c = self.count[i][b] as usize;
                if c == 0 {
                    continue;
                }
                let end = (rank + c).min(self.samples);
                let from = rank.min(self.samples);
                m += self.sum[i][b] as f64 / c as f64 * (self.cum[end] - self.cum[from]);
                rank += c;
            }
            m
        });
        let n = n.max(1) as f64;
        GradientLook {
            metric,
            meter: Meter {
                mean: sum as f64 / n,
                saturated: clipped as f64 / n,
            },
            clipped_up: clipped_up as f64 / n,
            mean_down: sum_down as f64 / n,
        }
    }
}

/// One eye's step in the gradient mode, as a ratio of its exposure × gain: 1/s, 1 or s (s the
/// `max_step_ratio`). `cap` is the longest exposure, lines.
fn gradient_ratio(
    c: &AutoExposureConfig,
    g: &GradientConfig,
    product: f64,
    cap: f64,
    look: &GradientLook,
) -> f64 {
    let s = c.max_step_ratio;
    let m = look.meter;
    // A last step short of the limits goes to them (`split` clamps to 1 line, 1× and the top gain)
    let can_down = product > 1.0 + 1e-6;
    let can_up = product < cap * c.gain_max * (1.0 - 1e-6);
    // Too much clipped: darken whatever the metric says (it cannot see what the clipped pixels
    // hide), unless that would take the unclipped pixels below the floor
    if m.saturated > c.saturation_limit && can_down && clip_floor(c, m).may_darken {
        return 1.0 / s;
    }
    let [down, hold, up] = look.metric;
    // Too dark to judge (a signal of a level or two scales to the same levels): brighten
    let dark = m.mean - c.black_level < g.gain_floor_level - c.black_level;
    // Past the exposure cap a step up is gain alone: only for a dark frame
    let past_cap = product * s > cap + 0.5;
    // Never up into clipping past saturation_resume (the override would take it back: a cycle)
    let mut best = (1.0, hold);
    if can_up
        && (dark || (up > hold * g.min_improvement && !past_cap))
        && look.clipped_up <= c.saturation_resume
    {
        best = (s, up);
    }
    if !dark && can_down && down > best.1 * g.min_improvement {
        return 1.0 / s;
    }
    // Gain past the cap is shed while a step down keeps the frame 10 % above the floor
    if best.0 == 1.0 && !dark && product > cap + 0.5 && look.mean_down > 1.1 * g.gain_floor_level {
        return 1.0 / s;
    }
    best.0
}

// ---- The controller ----------------------------------------------------------------------------

/// Every how many pixels a frame is metered in the mean mode, and sampled for the prediction
/// check, in each direction.
pub(crate) const METER_STEP: usize = 8;

/// How well the tone curve predicts a frame from the one before a step: summed |predicted −
/// real| over the pixels compared (unclipped in both, subsampled), for the curve, for a linear
/// camera (the original's assumption) and for no change at all. Motion adds to all three alike.
#[derive(Clone, Debug, Default)]
pub struct PredictionCheck {
    pub pixels: u64,
    pub curve: f64,
    pub linear: f64,
    pub unchanged: f64,
}

impl PredictionCheck {
    /// The mean errors: the curve's, linear's, no change's.
    pub fn means(&self) -> Option<[f64; 3]> {
        let n = self.pixels as f64;
        (self.pixels > 0).then(|| [self.curve / n, self.linear / n, self.unchanged / n])
    }
}

/// The frame a step was taken on: its setting and its subsampled images.
struct Stepped {
    setting: Setting,
    samples: [Vec<u8>; 2],
}

/// Either mode, with what it keeps between frames.
pub struct Controller {
    config: AutoExposureConfig,
    line_ns: f64,
    /// The gradient mode's tone curve, or the ZED's (for the check in the mean mode).
    tone: ToneCurve,
    /// The tone curve's maps for a step down, none, a step up.
    luts: [[u8; 256]; 3],
    metric: [GradientMetric; 2],
    last: Option<Stepped>,
    check: PredictionCheck,
}

impl Controller {
    pub fn new(config: AutoExposureConfig, line_ns: f64) -> Controller {
        let s = config.max_step_ratio;
        let tone = config
            .gradient
            .as_ref()
            .map(|g| g.tone.clone())
            .unwrap_or_else(ToneCurve::zed);
        Controller {
            luts: [1.0 / s, 1.0, s].map(|r| tone.map(r)),
            tone,
            config,
            line_ns,
            metric: [GradientMetric::new(), GradientMetric::new()],
            last: None,
            check: PredictionCheck::default(),
        }
    }

    /// The next setting, from a frame (both eyes' grey images, `width` wide) that took `current`.
    pub fn step(&mut self, current: &Setting, images: [&[u8]; 2], width: usize) -> Setting {
        self.check_prediction(current, images, width);
        let c = &self.config;
        match &c.gradient {
            None => step(
                c,
                self.line_ns,
                current,
                images.map(|g| meter(g, width, METER_STEP)),
            ),
            Some(g) => {
                let cap = exposure_for(c, self.line_ns, f64::MAX) as f64;
                let mut ratios = [1.0; 2];
                for (eye, ratio) in ratios.iter_mut().enumerate() {
                    let look = self.metric[eye].look(images[eye], width, g, &self.luts);
                    *ratio = gradient_ratio(c, g, product(current, eye), cap, &look);
                }
                if ratios == [1.0, 1.0] && current.lines as f64 <= cap {
                    return *current;
                }
                split(
                    c,
                    self.line_ns,
                    [0, 1].map(|eye| product(current, eye) * ratios[eye]),
                )
            }
        }
    }

    /// The tone curve's prediction of this frame from the one stepped before, if the setting
    /// changed between them.
    fn check_prediction(&mut self, current: &Setting, images: [&[u8]; 2], width: usize) {
        let samples = images.map(|g| {
            g.chunks_exact(width)
                .step_by(METER_STEP)
                .flat_map(|row| row.iter().step_by(METER_STEP).copied())
                .collect::<Vec<u8>>()
        });
        if let Some(last) = &self.last
            && last.setting != *current
        {
            for (eye, (before, now)) in last.samples.iter().zip(&samples).enumerate() {
                let r = product(current, eye) / product(&last.setting, eye);
                if (r - 1.0).abs() < 1e-3 || before.len() != now.len() {
                    continue;
                }
                let lut = self.tone.map(r);
                for (&p, &q) in before.iter().zip(now) {
                    if p == 255 || q == 255 {
                        continue;
                    }
                    let q = q as f64;
                    self.check.pixels += 1;
                    self.check.curve += (lut[p as usize] as f64 - q).abs();
                    self.check.linear += ((p as f64 * r).min(255.0) - q).abs();
                    self.check.unchanged += (p as f64 - q).abs();
                }
            }
        }
        self.last = Some(Stepped {
            setting: *current,
            samples,
        });
    }

    pub fn prediction_check(&self) -> PredictionCheck {
        self.check.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINE_NS: f64 = 40_066.0;

    #[test]
    fn gains_round_trip() {
        for (m, raw) in [
            (1.0, 128),
            (1.5, 192),
            (2.875, 0x100 | 184),
            (8.0, 0x700 | 128),
        ] {
            assert_eq!(gain_raw(m), raw, "{m}");
            assert!((gain_of_raw(raw) - m).abs() < 0.01);
        }
        assert_eq!(gain_raw(100.0), 2047);
        assert_eq!(gain_raw(0.3), 128);
    }

    #[test]
    fn exposure_keeps_to_flicker_periods_and_the_cap() {
        let c = AutoExposureConfig::default();
        // Under one period (208 lines at 120 Hz): as wanted
        assert_eq!(exposure_for(&c, LINE_NS, 150.0), 150);
        // Past one period: whole periods (207.98 lines each), at most the cap (25 ms: three)
        assert_eq!(exposure_for(&c, LINE_NS, 300.0), 207);
        assert_eq!(exposure_for(&c, LINE_NS, 500.0), 415);
        assert_eq!(exposure_for(&c, LINE_NS, 5000.0), 623);
        let open = AutoExposureConfig {
            flicker_hz: None,
            ..c
        };
        assert_eq!(exposure_for(&open, LINE_NS, 5000.0), 623);
    }

    /// A camera whose brightness goes as (light × exposure × gain)^0.76, clipped, per eye.
    fn simulate(light: [f64; 2], steps: usize) -> (Setting, [Meter; 2]) {
        let c = AutoExposureConfig::default();
        let mut s = Setting {
            lines: 100,
            gain: [1.0, 1.0],
        };
        let mut m = [Meter::default(); 2];
        for _ in 0..steps {
            m = [0, 1].map(|eye| Meter {
                mean: (light[eye] * product(&s, eye)).powf(0.76).min(255.0),
                saturated: 0.0,
            });
            s = step(&c, LINE_NS, &s, m);
        }
        (s, m)
    }

    #[test]
    fn both_eyes_reach_the_target_one_exposure_their_own_gains() {
        // The left gets half the right's light
        let (s, m) = simulate([0.05, 0.1], 200);
        for (eye, me) in m.iter().enumerate() {
            assert!((me.mean - 100.0).abs() < 8.0, "{eye}: {}", me.mean);
        }
        assert!(s.gain[0] > 1.6 * s.gain[1], "{s:?}");
    }

    #[test]
    fn steps_are_small_and_dark_scenes_use_gain_after_the_cap() {
        let c = AutoExposureConfig::default();
        let mut s = Setting {
            lines: 200,
            gain: [1.0, 1.0],
        };
        let dark = [Meter {
            mean: 10.0,
            saturated: 0.0,
        }; 2];
        let mut last = product(&s, 0);
        for _ in 0..60 {
            s = step(&c, LINE_NS, &s, dark);
            let p = product(&s, 0);
            assert!(p / last <= 1.12 * 1.01, "{} -> {}", last, p);
            last = p;
        }
        assert_eq!(s.lines, 623);
        assert!(s.gain[0] > 5.0, "{s:?}");
    }

    #[test]
    fn clipping_darkens_unless_the_rest_would_go_dark() {
        let c = AutoExposureConfig::default();
        let s = Setting {
            lines: 400,
            gain: [2.0, 2.0],
        };
        let clipped = Meter {
            mean: 120.0,
            saturated: 0.10,
        };
        assert!(product(&step(&c, LINE_NS, &s, [clipped; 2]), 0) < product(&s, 0));
        // A lamp over a dark room: the unclipped pixels at ~16, below the floor of 60
        let lamp = Meter {
            mean: 40.0,
            saturated: 0.10,
        };
        assert!(product(&step(&c, LINE_NS, &s, [lamp; 2]), 0) > product(&s, 0));
    }

    #[test]
    fn the_tone_curve_maps_levels() {
        let t = ToneCurve::zed();
        let same = t.map(1.0);
        assert!((0..256).all(|z| same[z] as usize == z));
        // About linear below the knee, compressed above: h(50) ~ 42, h(150) ~ 98
        let up = t.map(1.12);
        assert_eq!(up[50], 55);
        assert!((161..=162).contains(&up[150]), "{}", up[150]);
        assert_eq!(t.map(1.0 / 1.12)[50], 45);
        assert!(up.windows(2).all(|w| w[0] <= w[1]));
        assert_eq!(t.map(0.5)[255], 255);
    }

    #[test]
    fn the_metric_goes_as_the_squared_gradients() {
        let g = GradientConfig {
            step: 1,
            ..Default::default()
        };
        let (w, h) = (64, 48);
        let img: Vec<u8> = (0..w * h)
            .map(|i| (20 + (i % w) + (i / w) % 7 * 3) as u8)
            .collect();
        let mut double = [255u8; 256];
        let mut half = [0u8; 256];
        for z in 0..256 {
            double[z] = (z * 2).min(255) as u8;
            half[z] = (z / 2) as u8;
        }
        let ident: [u8; 256] = std::array::from_fn(|z| z as u8);
        let mut m = GradientMetric::new();
        let look = m.look(&img, w, &g, &[half, ident, double]);
        let [lo, mid, hi] = look.metric;
        assert!((hi / mid - 4.0).abs() < 0.05, "{hi} {mid}");
        assert!((mid / lo - 4.0).abs() < 0.6, "{mid} {lo}");
        let flat = vec![100u8; w * h];
        assert_eq!(
            m.look(&flat, w, &g, &[half, ident, double]).metric,
            [0.0; 3]
        );
    }

    /// A still scene for each eye: 8 × 8 blocks of reflectance spread over e^4, under light
    /// falling off e^2 across the image, `offsets` the eyes' log light at a product of 1.
    fn scene(width: usize, height: usize, offsets: [f64; 2]) -> [Vec<f64>; 2] {
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let blocks: Vec<f64> = (0..(width / 8) * (height / 8))
            .map(|_| -2.0 + 4.0 * rnd())
            .collect();
        let base: Vec<f64> = (0..width * height)
            .map(|i| {
                let (x, y) = (i % width, i / width);
                blocks[(y / 8) * (width / 8) + x / 8] + 2.0 * x as f64 / width as f64 - 1.0
            })
            .collect();
        offsets.map(|o| base.iter().map(|v| v + o).collect())
    }

    /// The gradient mode in closed loop on [`scene`] through the ZED's tone curve: the last
    /// setting, each eye's last mean and clipped fraction, and how often the setting changed in
    /// the last 40 steps.
    fn run_gradient(offsets: [f64; 2], steps: usize) -> (Setting, [Meter; 2], usize) {
        let (w, h) = (160, 120);
        let t = ToneCurve::zed();
        let light = scene(w, h, offsets);
        let mut ctl = Controller::new(
            AutoExposureConfig {
                gradient: Some(GradientConfig {
                    step: 2,
                    ..Default::default()
                }),
                ..Default::default()
            },
            LINE_NS,
        );
        let mut s = Setting {
            lines: 100,
            gain: [1.0, 1.0],
        };
        let (mut changes, mut meters) = (0, [Meter::default(); 2]);
        for i in 0..steps {
            let images = [0, 1].map(|eye| {
                let p = product(&s, eye).ln();
                light[eye]
                    .iter()
                    .map(|l| t.level(l + p).round() as u8)
                    .collect::<Vec<u8>>()
            });
            meters = [0, 1].map(|eye| meter(&images[eye], w, 1));
            let next = ctl.step(&s, [&images[0], &images[1]], w);
            changes += (i + 40 >= steps && next != s) as usize;
            s = next;
        }
        (s, meters, changes)
    }

    #[test]
    fn the_gradient_mode_settles_unclipped() {
        // The median block at level 100 near a product of 300; the right eye brighter
        let (s, m, changes) = run_gradient([-(300f64.ln()), 0.5 - 300f64.ln()], 150);
        assert!(
            changes <= 4,
            "{changes} changes in the last 40 steps: {s:?}"
        );
        for me in m {
            assert!(
                me.saturated <= 0.05 && (30.0..=220.0).contains(&me.mean),
                "{m:?} {s:?}"
            );
        }
        assert!(product(&s, 1) < product(&s, 0), "{s:?}");
    }

    #[test]
    fn the_gradient_mode_uses_gain_only_in_the_dark() {
        // Dim: at the exposure cap, gain only until the mean is up to ~60
        let dim = -(623f64.ln()) - 1.3;
        let (s, m, _) = run_gradient([dim; 2], 150);
        assert_eq!(s.lines, 623, "{s:?}");
        assert!(s.gain[0] > 1.2 && s.gain[0] < 4.0, "{s:?} {m:?}");
        assert!((55.0..=90.0).contains(&m[0].mean), "{m:?} {s:?}");
        // Dark: all the gain there is
        let (s, _, _) = run_gradient([-(1e5f64.ln()) - 3.0; 2], 150);
        assert_eq!(s.lines, 623);
        assert!(s.gain[0] > MAX_GAIN - 0.01, "{s:?}");
    }
}
