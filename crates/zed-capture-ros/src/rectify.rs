//! The raw pair rectified onto one pinhole camera, from the ZED's factory calibration: Rudolf-V's
//! rectification (Bouguet's rotations, the calibration's rational distortion; checked against the
//! ZED SDK's rectified images, rows within 0.02 px), its bilinear maps applied here byte to byte.

use std::path::Path;

use rudolf_v::camera::{DistortionModel, StereoRig};
use rudolf_v::rectify::{RemapTap, StereoRectification};

pub struct Rectifier {
    /// Per eye, each rectified pixel's bilinear tap in the raw image, row-major.
    maps: [Vec<RemapTap>; 2],
    pub width: usize,
    pub height: usize,
    /// The rectified camera, both eyes': fx = fy, cx, cy (pixels).
    pub fx: f64,
    pub cx: f64,
    pub cy: f64,
    /// The right camera's distance along the rectified x-axis (m).
    pub baseline: f64,
}

impl Rectifier {
    /// From a ZED factory calibration file (`/usr/local/zed/settings/SN<serial>.conf`) at the
    /// resolution of `width × height`, onto the pinhole camera `camera` (fx = fy, cx, cy); by
    /// default the left raw camera's fx, cx and cy, which keeps the left image's framing.
    pub fn from_zed_conf(
        path: &Path,
        width: usize,
        height: usize,
        camera: Option<[f64; 3]>,
    ) -> Result<Rectifier, String> {
        let resolution = match (width, height) {
            (2208, 1242) => "2K",
            (1920, 1080) => "FHD",
            (1280, 720) => "HD",
            (672, 376) => "VGA",
            _ => return Err(format!("no ZED resolution is {width}x{height}")),
        };
        let rig = StereoRig::from_zed_conf(path, resolution, DistortionModel::Rational)?;
        let [fx, cx, cy] = camera.unwrap_or([rig.cam0.fx, rig.cam0.cx, rig.cam0.cy]);
        let r = StereoRectification::new(&rig, fx, cx, cy);
        let baseline = r.rig().baseline_meters();
        if !(0.01..1.0).contains(&baseline) {
            return Err(format!("{}: a baseline of {baseline} m", path.display()));
        }
        Ok(Rectifier {
            maps: [r.map(0).to_vec(), r.map(1).to_vec()],
            width,
            height,
            fx,
            cx,
            cy,
            baseline,
        })
    }

    /// Both eyes' raw grey images (`width × height`, row-major) rectified: bilinear, rounded; 0
    /// where a rectified pixel sees no raw one. Each eye in bands of rows, each on a thread of its
    /// own: [`threads_per_eye`].
    pub fn rectify_pair(&self, raw: [&[u8]; 2]) -> [Vec<u8>; 2] {
        let n = self.width * self.height;
        for r in raw {
            assert_eq!(r.len(), n, "raw image size");
        }
        let mut out = [vec![0; n], vec![0; n]];
        let band = self.height.div_ceil(threads_per_eye()) * self.width;
        std::thread::scope(|s| {
            for (eye, o) in out.iter_mut().enumerate() {
                for (map, o) in self.maps[eye].chunks(band).zip(o.chunks_mut(band)) {
                    let (raw, w) = (raw[eye], self.width);
                    s.spawn(move || remap(map, raw, w, o));
                }
            }
        });
        out
    }
}

/// Threads an eye is rectified on: half the CPUs this process may run on, one or two. On LabOrin
/// (6 cores) one an eye took 6.8 ms a frame, two 3.8 ms; four threads pinned to 2 cores, 9.1 ms.
fn threads_per_eye() -> usize {
    std::thread::available_parallelism().map_or(1, |n| (n.get() / 2).clamp(1, 2))
}

/// Samples `raw` (rows `width` wide) at each tap of `map` into `out`.
fn remap(map: &[RemapTap], raw: &[u8], width: usize, out: &mut [u8]) {
    for (o, t) in out.iter_mut().zip(map) {
        if t.pos == u32::MAX {
            *o = 0;
            continue;
        }
        let j = (t.pos >> 16) as usize * width + (t.pos & 0xffff) as usize;
        let (a, b) = (raw[j] as f32, raw[j + 1] as f32);
        let (c, d) = (raw[j + width] as f32, raw[j + width + 1] as f32);
        let v = (1.0 - t.wy) * ((1.0 - t.wx) * a + t.wx * b) + t.wy * ((1.0 - t.wx) * c + t.wx * d);
        *o = (v + 0.5) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tap(x: u32, y: u32, wx: f32, wy: f32) -> RemapTap {
        RemapTap {
            pos: x | y << 16,
            wx,
            wy,
        }
    }

    #[test]
    fn taps_sample_bilinearly() {
        // 4 × 3: the level is 10 x + 100 y
        let raw: Vec<u8> = (0..12)
            .map(|i| (10 * (i % 4) + 100 * (i / 4)) as u8)
            .collect();
        let map = [
            tap(0, 0, 0.0, 0.0),
            tap(1, 0, 0.5, 0.0),
            tap(2, 1, 0.25, 0.5),
            RemapTap::NONE,
        ];
        let mut out = [9; 4];
        remap(&map, &raw, 4, &mut out);
        assert_eq!(out, [0, 15, 173, 0]);
    }
}
