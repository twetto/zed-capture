//! ROS 2's message encoding: CDR, little-endian (the encapsulation header `00 01 00 00`), each
//! primitive aligned to its size counted from after the header.

pub struct Cdr {
    buf: Vec<u8>,
}

const HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

impl Cdr {
    pub fn with_capacity(n: usize) -> Cdr {
        let mut buf = Vec::with_capacity(n + 64);
        buf.extend_from_slice(&HEADER);
        Cdr { buf }
    }

    fn align(&mut self, n: usize) {
        let at = self.buf.len() - HEADER.len();
        let pad = (n - at % n) % n;
        self.buf.resize(self.buf.len() + pad, 0);
    }

    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.buf.push(v);
        self
    }

    pub fn bool(&mut self, v: bool) -> &mut Self {
        self.u8(v as u8)
    }

    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    pub fn i32(&mut self, v: i32) -> &mut Self {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    pub fn i64(&mut self, v: i64) -> &mut Self {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    pub fn f64(&mut self, v: f64) -> &mut Self {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// A string: its length with the terminating 0, its bytes, the 0.
    pub fn string(&mut self, s: &str) -> &mut Self {
        self.u32(s.len() as u32 + 1);
        self.buf.extend_from_slice(s.as_bytes());
        self.buf.push(0);
        self
    }

    /// A `uint8[]`: its length, its bytes.
    pub fn bytes(&mut self, b: &[u8]) -> &mut Self {
        self.u32(b.len() as u32);
        self.buf.extend_from_slice(b);
        self
    }

    /// A `builtin_interfaces/Time` from ns (seconds rounded down, nanoseconds 0 to 1e9 − 1).
    pub fn time(&mut self, ns: i64) -> &mut Self {
        self.i32(ns.div_euclid(1_000_000_000) as i32)
            .u32(ns.rem_euclid(1_000_000_000) as u32)
    }

    /// A `std_msgs/Header`.
    pub fn header(&mut self, stamp_ns: i64, frame_id: &str) -> &mut Self {
        self.time(stamp_ns).string(frame_id)
    }

    pub fn finish(self) -> Vec<u8> {
        self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alignment_counts_from_after_the_header() {
        let mut c = Cdr::with_capacity(0);
        c.u8(7).u32(1).u8(2).f64(1.5);
        let b = c.finish();
        assert_eq!(&b[..4], &HEADER);
        // u8 at 0, u32 at 4, u8 at 8, f64 at 16 (after the header)
        assert_eq!(b[4], 7);
        assert_eq!(&b[8..12], &1u32.to_le_bytes());
        assert_eq!(b[12], 2);
        assert_eq!(&b[20..28], &1.5f64.to_le_bytes());
        assert_eq!(b.len(), 28);
    }

    #[test]
    fn header_and_strings() {
        let mut c = Cdr::with_capacity(0);
        c.header(-1, "ab");
        let b = c.finish();
        // −1 ns: −1 s + 999999999 ns; "ab" as 3 bytes with its 0
        assert_eq!(&b[4..8], &(-1i32).to_le_bytes());
        assert_eq!(&b[8..12], &999_999_999u32.to_le_bytes());
        assert_eq!(&b[12..16], &3u32.to_le_bytes());
        assert_eq!(&b[16..19], b"ab\0");
    }
}
