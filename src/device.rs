//! The camera's devices, found through sysfs.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const VENDOR: u16 = 0x2b03;
/// (video product, hidraw product, model). The original ZED has no IMU.
const MODELS: [(u16, u16, &str); 3] = [
    (0xf880, 0xf881, "ZED 2i"),
    (0xf780, 0xf781, "ZED 2"),
    (0xf682, 0xf681, "ZED Mini"),
];

/// One camera's devices.
#[derive(Clone, Debug)]
pub struct Devices {
    pub model: &'static str,
    /// The V4L2 capture node.
    pub video: PathBuf,
    /// The IMU's hidraw node.
    pub hid: PathBuf,
    /// The camera's serial number, from the IMU's USB interface (the video's is not the camera's).
    pub serial: Option<String>,
}

/// The one ZED attached. Err if there is none, or more than one (then name the devices).
pub fn find() -> io::Result<Devices> {
    let videos = videos()?;
    let hids = hids()?;
    let mut found = Vec::new();
    for (video_product, video) in &videos {
        let Some(&(_, hid_product, model)) = MODELS.iter().find(|m| m.0 == *video_product) else {
            continue;
        };
        for (product, hid, serial) in &hids {
            if *product == hid_product {
                found.push(Devices {
                    model,
                    video: video.clone(),
                    hid: hid.clone(),
                    serial: serial.clone(),
                });
            }
        }
    }
    match found.len() {
        1 => Ok(found.remove(0)),
        0 => Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!(
                "no ZED with an IMU found (video nodes {}, hidraw nodes {})",
                videos.len(),
                hids.len()
            ),
        )),
        _ => Err(io::Error::other(format!(
            "{} ZEDs found: name the devices: {found:?}",
            found.len()
        ))),
    }
}

fn read_hex(path: &Path) -> Option<u16> {
    u16::from_str_radix(fs::read_to_string(path).ok()?.trim(), 16).ok()
}

/// (product, /dev/videoN) of each Stereolabs capture node (V4L2 index 0; 1 is the metadata).
fn videos() -> io::Result<Vec<(u16, PathBuf)>> {
    let mut out = Vec::new();
    for entry in fs::read_dir("/sys/class/video4linux")? {
        let dir = entry?.path();
        let usb = dir.join("device/..");
        if read_hex(&usb.join("idVendor")) != Some(VENDOR)
            || fs::read_to_string(dir.join("index"))
                .map(|s| s.trim() != "0")
                .unwrap_or(true)
        {
            continue;
        }
        if let Some(product) = read_hex(&usb.join("idProduct")) {
            out.push((product, Path::new("/dev").join(dir.file_name().unwrap())));
        }
    }
    out.sort();
    Ok(out)
}

/// (product, /dev/hidrawN, serial) of each Stereolabs hidraw node.
fn hids() -> io::Result<Vec<(u16, PathBuf, Option<String>)>> {
    let mut out = Vec::new();
    let Ok(entries) = fs::read_dir("/sys/class/hidraw") else {
        return Ok(out);
    };
    for entry in entries {
        let dir = entry?.path();
        let Ok(uevent) = fs::read_to_string(dir.join("device/uevent")) else {
            continue;
        };
        let (mut id, mut serial) = (None, None);
        for line in uevent.lines() {
            if let Some(v) = line.strip_prefix("HID_ID=") {
                // bus:vendor:product, e.g. 0003:00002B03:0000F881
                let mut parts = v
                    .split(':')
                    .skip(1)
                    .map(|p| u32::from_str_radix(p, 16).ok());
                id = parts.next().flatten().zip(parts.next().flatten());
            } else if let Some(v) = line.strip_prefix("HID_UNIQ=") {
                serial = Some(v.to_string()).filter(|s| !s.is_empty());
            }
        }
        if let Some((vendor, product)) = id
            && vendor == VENDOR as u32
        {
            out.push((
                product as u16,
                Path::new("/dev").join(dir.file_name().unwrap()),
                serial,
            ));
        }
    }
    out.sort();
    Ok(out)
}
