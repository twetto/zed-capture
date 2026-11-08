//! ioctl numbers and the calls the two devices share.

use std::io;
use std::os::fd::RawFd;

pub const WRITE: u32 = 1;
pub const READ: u32 = 2;

/// `_IOC(dir, type, nr, size)` as in the kernel's asm-generic ioctl.h (x86-64 and arm64).
pub const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> libc::c_ulong {
    ((dir << 30) | ((size as u32) << 16) | ((ty as u32) << 8) | nr as u32) as libc::c_ulong
}

/// An ioctl, retried when a signal interrupts it.
pub fn ioctl<T>(fd: RawFd, request: libc::c_ulong, arg: *mut T) -> io::Result<()> {
    loop {
        // SAFETY: the caller passes a request whose size is that of *arg.
        if unsafe { libc::ioctl(fd, request as _, arg) } != -1 {
            return Ok(());
        }
        let e = io::Error::last_os_error();
        if e.kind() != io::ErrorKind::Interrupted {
            return Err(e);
        }
    }
}

/// Waits up to `timeout_ms` for `fd` to be readable. Err when the device is gone.
pub fn wait_readable(fd: RawFd, timeout_ms: i32) -> io::Result<bool> {
    let mut p = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    // SAFETY: one valid pollfd.
    let n = unsafe { libc::poll(&mut p, 1, timeout_ms) };
    if n < 0 {
        let e = io::Error::last_os_error();
        return if e.kind() == io::ErrorKind::Interrupted {
            Ok(false)
        } else {
            Err(e)
        };
    }
    if p.revents & libc::POLLIN != 0 {
        return Ok(true);
    }
    if p.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
        return Err(io::Error::other(format!(
            "poll: revents {:#x} (unplugged?)",
            p.revents
        )));
    }
    Ok(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioc_matches_the_kernel_headers() {
        // VIDIOC_QUERYCAP, VIDIOC_DQBUF on 64-bit, HIDIOCSFEATURE(2)
        assert_eq!(ioc(READ, b'V', 0, 104), 0x8068_5600);
        assert_eq!(ioc(READ | WRITE, b'V', 17, 88), 0xc058_5611);
        assert_eq!(ioc(READ | WRITE, b'H', 0x06, 2), 0xc002_4806);
    }
}
