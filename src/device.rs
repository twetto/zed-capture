//! The camera's devices, found through sysfs.

use std::fs;
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::sys;

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
///
/// When a ZED's video is there but its hidraw node is not, the IMU's USB interface has likely lost
/// its kernel driver to the ZED SDK, which detaches it and leaves it so when it closes the camera:
/// this gives it back (see [`reattach_hid`]) and waits for the node.
pub fn find() -> io::Result<Devices> {
    let videos = videos()?;
    let mut hids = hid_nodes()?;
    if missing_hid(&videos, &hids) && reattach_hid()? > 0 {
        let t0 = Instant::now();
        loop {
            hids = hid_nodes()?;
            // udev sets the node's mode just after it appears: wait until it opens
            let ready = !missing_hid(&videos, &hids)
                && hids.iter().all(|(_, path, _)| {
                    fs::OpenOptions::new()
                        .read(true)
                        .write(true)
                        .open(path)
                        .is_ok()
                });
            if ready || t0.elapsed() > Duration::from_secs(3) {
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }
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

/// A ZED's video whose IMU has no hidraw node.
fn missing_hid(videos: &[(u16, PathBuf)], hids: &[(u16, PathBuf, Option<String>)]) -> bool {
    videos.iter().any(|(video_product, _)| {
        MODELS
            .iter()
            .find(|m| m.0 == *video_product)
            .is_some_and(|m| !hids.iter().any(|h| h.0 == m.1))
    })
}

/// `struct usbdevfs_ioctl` (linux/usbdevice_fs.h).
#[repr(C)]
struct UsbdevfsIoctl {
    ifno: libc::c_int,
    ioctl_code: libc::c_int,
    data: *mut libc::c_void,
}

const USBDEVFS_IOCTL: libc::c_ulong =
    sys::ioc(sys::READ | sys::WRITE, b'U', 18, size_of::<UsbdevfsIoctl>());
const USBDEVFS_CONNECT: libc::c_ulong = sys::ioc(0, b'U', 23, 0);

/// Gives every ZED IMU interface without a kernel driver back to the kernel (usbhid, which makes
/// its hidraw node), as libusb's `libusb_attach_kernel_driver` does. The ZED SDK detaches the
/// driver to talk to the IMU itself and does not attach it again when it closes the camera; until
/// then, or a replug, there is no hidraw node. Needs write access to the camera's
/// `/dev/bus/usb/BBB/DDD` node, which Stereolabs' udev rule (`99-slabs.rules`) gives everyone.
/// Returns how many interfaces it gave back.
pub fn reattach_hid() -> io::Result<usize> {
    let mut done = 0;
    let Ok(entries) = fs::read_dir("/sys/bus/usb/devices") else {
        return Ok(0);
    };
    for entry in entries {
        let dev = entry?.path();
        if read_hex(&dev.join("idVendor")) != Some(VENDOR)
            || !read_hex(&dev.join("idProduct")).is_some_and(|p| MODELS.iter().any(|m| m.1 == p))
        {
            continue;
        }
        let name = dev.file_name().unwrap().to_string_lossy().into_owned();
        // Its interfaces: <device>:<configuration>.<interface>
        for intf in fs::read_dir(&dev)? {
            let intf = intf?.path();
            let is_interface = intf
                .file_name()
                .is_some_and(|f| f.to_string_lossy().starts_with(&format!("{name}:")));
            if !is_interface
                || read_hex(&intf.join("bInterfaceClass")) != Some(0x03)
                || intf.join("driver").exists()
            {
                continue;
            }
            let Some(ifno) = read_hex(&intf.join("bInterfaceNumber")) else {
                continue;
            };
            let number = |f: &str| -> io::Result<u32> {
                fs::read_to_string(dev.join(f))?
                    .trim()
                    .parse()
                    .map_err(|_| {
                        io::Error::other(format!("{}: not a number", dev.join(f).display()))
                    })
            };
            let node = format!(
                "/dev/bus/usb/{:03}/{:03}",
                number("busnum")?,
                number("devnum")?
            );
            let file = fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(&node)
                .map_err(|e| {
                    io::Error::new(
                        e.kind(),
                        format!(
                            "the ZED's IMU has no kernel driver (the ZED SDK detached it), and {node} \
                             can't be opened to attach it again ({e}): replug the camera, or install \
                             Stereolabs' udev rules"
                        ),
                    )
                })?;
            let mut request = UsbdevfsIoctl {
                ifno: ifno.into(),
                ioctl_code: USBDEVFS_CONNECT as libc::c_int,
                data: std::ptr::null_mut(),
            };
            sys::ioctl(file.as_raw_fd(), USBDEVFS_IOCTL, &mut request).map_err(|e| {
                io::Error::new(
                    e.kind(),
                    format!("attaching the kernel's driver to the ZED's IMU ({node}): {e}"),
                )
            })?;
            done += 1;
        }
    }
    Ok(done)
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
fn hid_nodes() -> io::Result<Vec<(u16, PathBuf, Option<String>)>> {
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
