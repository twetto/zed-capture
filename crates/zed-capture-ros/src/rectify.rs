//! The raw pair rectified onto one pinhole camera, from the ZED's factory calibration: Rudolf-V's
//! rectification (Bouguet's rotations, the calibration's rational distortion; checked against the
//! ZED SDK's rectified images, rows within 0.02 px), its bilinear maps applied byte to byte, on
//! the GPU by Rudolf-V's `GpuRectification` (feature gpu: Vulkan) or here on the CPU. The GPU
//! does the CPU's arithmetic; its fused multiply-adds may round the other way on a few pixels in
//! a million (one grey level): it is checked against the CPU when made.

use std::path::Path;

use rudolf_v::camera::{DistortionModel, StereoRig};
use rudolf_v::rectify::{RemapTap, StereoRectification};
#[cfg(feature = "gpu")]
use rudolf_v::{gpu::device::GpuDevice, gpu::rectify::GpuRectification, image::Image};

pub struct Rectifier {
    /// Per eye, each rectified pixel's bilinear tap in the raw image, row-major.
    maps: [Vec<RemapTap>; 2],
    #[cfg(feature = "gpu")]
    gpu: Option<Gpu>,
    pub width: usize,
    pub height: usize,
    /// The rectified camera, both eyes': fx = fy, cx, cy (pixels).
    pub fx: f64,
    pub cx: f64,
    pub cy: f64,
    /// The right camera's distance along the rectified x-axis (m).
    pub baseline: f64,
}

/// Rudolf-V's rectification on the GPU, with its images kept between frames.
#[cfg(feature = "gpu")]
struct Gpu {
    device: GpuDevice,
    rect: GpuRectification,
    raw: [Image<u8>; 2],
    out: [Image<u8>; 2],
}

/// The GPU's rectified pixels may differ from the CPU's by one grey level on this share of the
/// pixels at most (Rudolf-V: a few in a million); more, and the CPU rectifies.
#[cfg(feature = "gpu")]
const GPU_DIFFERENT_MAX: f64 = 1e-4;

impl Rectifier {
    /// From a ZED factory calibration file (`/usr/local/zed/settings/SN<serial>.conf`) at the
    /// resolution of `width × height`, onto the pinhole camera `camera` (fx = fy, cx, cy); by
    /// default the left raw camera's fx, cx and cy, which keeps the left image's framing. On the
    /// GPU if `gpu` and there is one (Vulkan), else on the CPU.
    pub fn from_zed_conf(
        path: &Path,
        width: usize,
        height: usize,
        camera: Option<[f64; 3]>,
        gpu: bool,
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
        #[cfg(feature = "gpu")]
        let gpu_rect = match gpu {
            true => match GpuDevice::new() {
                Ok(device) => Some(Gpu {
                    rect: GpuRectification::new(&device, &r),
                    device,
                    raw: [Image::new(width, height), Image::new(width, height)],
                    out: [Image::new(width, height), Image::new(width, height)],
                }),
                Err(e) => {
                    eprintln!("zed_ros: no GPU to rectify on ({e}): on the CPU");
                    None
                }
            },
            false => None,
        };
        #[cfg(not(feature = "gpu"))]
        if gpu {
            eprintln!("zed_ros: built without the feature gpu: rectifying on the CPU");
        }
        #[allow(unused_mut)]
        let mut rectifier = Rectifier {
            maps: [r.map(0).to_vec(), r.map(1).to_vec()],
            #[cfg(feature = "gpu")]
            gpu: gpu_rect,
            width,
            height,
            fx,
            cx,
            cy,
            baseline,
        };
        #[cfg(feature = "gpu")]
        rectifier.check_gpu();
        Ok(rectifier)
    }

    /// The GPU's rectification of a noise pair against the CPU's (also its first use, which
    /// costs most): too far apart, and the CPU rectifies.
    #[cfg(feature = "gpu")]
    fn check_gpu(&mut self) {
        let Some(g) = self.gpu.as_mut() else { return };
        let n = self.width * self.height;
        let mut state = 0x2545_f491_u32;
        let mut noise = || {
            (0..n)
                .map(|_| {
                    state ^= state << 13;
                    state ^= state >> 17;
                    state ^= state << 5;
                    (state >> 24) as u8
                })
                .collect::<Vec<u8>>()
        };
        let raw = [noise(), noise()];
        for (image, r) in g.raw.iter_mut().zip(&raw) {
            image.as_mut_slice().copy_from_slice(r);
        }
        g.rect
            .rectify_into(&g.device, &g.raw[0], &g.raw[1], &mut g.out);
        let gpu = [g.out[0].as_slice().to_vec(), g.out[1].as_slice().to_vec()];
        let cpu = self.cpu_pair([&raw[0], &raw[1]]);
        let (mut different, mut most) = (0usize, 0u8);
        for (a, b) in gpu.iter().zip(&cpu) {
            for (x, y) in a.iter().zip(b) {
                let d = x.abs_diff(*y);
                different += (d > 0) as usize;
                most = most.max(d);
            }
        }
        eprintln!(
            "zed_ros: the GPU's rectification of a noise pair against the CPU's: {different} of {} \
             pixels differ, by at most {most}",
            2 * n
        );
        if most > 1 || different as f64 / (2 * n) as f64 > GPU_DIFFERENT_MAX {
            eprintln!("zed_ros: too far from the CPU's: rectifying on the CPU");
            self.gpu = None;
        }
    }

    /// Where it rectifies: "on the GPU (its name)" or "on the CPU (N threads an eye)".
    pub fn place(&self) -> String {
        #[cfg(feature = "gpu")]
        if let Some(g) = &self.gpu {
            return format!("on the GPU ({})", g.device.adapter_info.name);
        }
        format!("on the CPU ({} threads an eye)", threads_per_eye())
    }

    /// Both eyes' raw grey images (`width × height`, row-major) rectified: bilinear, rounded; 0
    /// where a rectified pixel sees no raw one.
    pub fn rectify_pair(&mut self, raw: [&[u8]; 2]) -> [Vec<u8>; 2] {
        let n = self.width * self.height;
        for r in raw {
            assert_eq!(r.len(), n, "raw image size");
        }
        #[cfg(feature = "gpu")]
        if let Some(g) = self.gpu.as_mut() {
            for (image, r) in g.raw.iter_mut().zip(raw) {
                image.as_mut_slice().copy_from_slice(r);
            }
            g.rect
                .rectify_into(&g.device, &g.raw[0], &g.raw[1], &mut g.out);
            return [g.out[0].as_slice().to_vec(), g.out[1].as_slice().to_vec()];
        }
        self.cpu_pair(raw)
    }

    /// On the CPU: each eye in bands of rows, each on a thread of its own: [`threads_per_eye`].
    fn cpu_pair(&self, raw: [&[u8]; 2]) -> [Vec<u8>; 2] {
        let n = self.width * self.height;
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
