//! The camera's vendor requests through its UVC extension unit, as zed-open-capture's
//! `ll_VendorControl`: a 384-byte request written with SET_CUR (64 bytes over USB 2) and, for a
//! read, the answer read back with GET_CUR. Used here to reach the image sensors' registers over
//! the camera's I2C.

use std::io;
use std::mem::size_of;
use std::os::fd::RawFd;
use std::time::Duration;

use crate::sys::{self, READ, WRITE, ioc};

/// `struct uvc_xu_control_query` (linux/uvcvideo.h).
#[repr(C)]
struct XuQuery {
    unit: u8,
    selector: u8,
    query: u8,
    size: u16,
    data: *mut u8,
}

const _: () = assert!(size_of::<XuQuery>() == 16);
const UVCIOC_CTRL_QUERY: libc::c_ulong = ioc(READ | WRITE, b'u', 0x21, size_of::<XuQuery>());

/// The camera's extension unit and its one control.
const UNIT: u8 = 0x04;
const SELECTOR: u8 = 0x02;
const UVC_SET_CUR: u8 = 0x01;
const UVC_GET_CUR: u8 = 0x81;
const UVC_GET_LEN: u8 = 0x85;
const REQUEST_LEN: usize = 384;

const TASK_SET: u8 = 0x50;
const TASK_GET: u8 = 0x51;
/// The I2C bus of the left and of the right image sensor, and the sensors' address on it.
const I2C: [u8; 2] = [0xa3, 0xa5];
const SENSOR_ADDRESS: u8 = 0x6c;

/// Each image sensor's register whose bit 7 sends its frame-start pulse (VSYNC) out, to the
/// camera's MCU: the MCU then marks the IMU sample of each frame (zed-open-capture's
/// `ll_activate_sync`). Off when the camera powers up.
pub const REG_SYNC_OUTPUT: u16 = 0x3002;
pub const SYNC_OUTPUT_ON: u8 = 0x80;

fn query(fd: RawFd, request: u8, data: &mut [u8]) -> io::Result<()> {
    let mut q = XuQuery {
        unit: UNIT,
        selector: SELECTOR,
        query: request,
        size: data.len() as u16,
        data: data.as_mut_ptr(),
    };
    sys::ioctl(fd, UVCIOC_CTRL_QUERY, &mut q)
}

/// The length of the extension unit's requests (384 bytes over USB 3, 64 over USB 2).
pub fn request_len(fd: RawFd) -> io::Result<usize> {
    let mut len = [0u8; 2];
    query(fd, UVC_GET_LEN, &mut len)
        .map_err(|e| io::Error::new(e.kind(), format!("the camera's extension unit: {e}")))?;
    let len = u16::from_le_bytes(len) as usize;
    if !(18..=REQUEST_LEN).contains(&len) {
        return Err(io::Error::other(format!(
            "the camera's extension unit takes {len} bytes"
        )));
    }
    Ok(len)
}

fn request(fd: RawFd, len: usize, buf: &mut [u8; REQUEST_LEN], read: bool) -> io::Result<()> {
    query(fd, UVC_SET_CUR, &mut buf[..len])?;
    std::thread::sleep(Duration::from_micros(300));
    if read {
        query(fd, UVC_GET_CUR, &mut buf[..len])?;
    }
    Ok(())
}

fn sensor_request(task: u8, eye: usize, register: u16) -> [u8; REQUEST_LEN] {
    let mut b = [0u8; REQUEST_LEN];
    b[0] = task;
    b[1] = I2C[eye];
    b[2] = SENSOR_ADDRESS;
    b[3] = 2; // register address bytes
    b[4] = 1; // data bytes
    b[5..9].copy_from_slice(&(register as u32).to_be_bytes());
    b[9] = 0x90; // one byte (high bits of the count), with zed-open-capture's page flags
    b[10] = 0x01;
    b[12] = 0x01;
    b
}

/// One byte of an image sensor's register (`eye` 0 left, 1 right), with requests of `len`
/// ([`request_len`]). 0xff when the sensor does not answer: when it is not streaming, and for
/// spans of ~0.6 s every few seconds on the left sensor of LabOrin's ZED 2i while it streams
/// (2026-11-12; the right's always answered).
pub fn read_sensor_register(fd: RawFd, len: usize, eye: usize, register: u16) -> io::Result<u8> {
    let mut b = sensor_request(TASK_GET, eye, register);
    request(fd, len, &mut b, true)?;
    Ok(b[17])
}

pub fn write_sensor_register(
    fd: RawFd,
    len: usize,
    eye: usize,
    register: u16,
    value: u8,
) -> io::Result<()> {
    let mut b = sensor_request(TASK_SET, eye, register);
    b[16] = value;
    request(fd, len, &mut b, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_number_and_request_layout() {
        assert_eq!(UVCIOC_CTRL_QUERY, 0xc010_7521);
        let b = sensor_request(TASK_GET, 1, REG_SYNC_OUTPUT);
        assert_eq!(
            &b[..13],
            &[0x51, 0xa5, 0x6c, 2, 1, 0, 0, 0x30, 0x02, 0x90, 1, 0, 1]
        );
    }
}
