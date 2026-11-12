//! The ROS 2 messages zed_record writes: their definitions (MCAP's `ros2msg` schemas, the message
//! with every type it uses after it) and their CDR.

use crate::cdr::Cdr;

const SEPARATOR: &str =
    "\n================================================================================\nMSG: ";

const TIME: &str = "int32 sec\nuint32 nanosec\n";
const HEADER: &str = "builtin_interfaces/Time stamp\nstring frame_id\n";
const VECTOR3: &str = "float64 x\nfloat64 y\nfloat64 z\n";
const QUATERNION: &str = "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";

/// A schema: the message type's name (`pkg/msg/Type`) and its definition.
pub struct Schema {
    pub name: &'static str,
    pub text: String,
}

fn schema(name: &'static str, definition: &str, uses: &[(&str, &str)]) -> Schema {
    let mut text = definition.to_string();
    for (n, d) in uses {
        text.push_str(SEPARATOR);
        text.push_str(n);
        text.push('\n');
        text.push_str(d);
    }
    Schema { name, text }
}

const WITH_HEADER: [(&str, &str); 2] = [
    ("std_msgs/Header", HEADER),
    ("builtin_interfaces/Time", TIME),
];

pub fn compressed_image_schema() -> Schema {
    schema(
        "sensor_msgs/msg/CompressedImage",
        "std_msgs/Header header\nstring format\nuint8[] data\n",
        &WITH_HEADER,
    )
}

pub fn image_schema() -> Schema {
    schema(
        "sensor_msgs/msg/Image",
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\n",
        &WITH_HEADER,
    )
}

pub fn imu_schema() -> Schema {
    schema(
        "sensor_msgs/msg/Imu",
        "std_msgs/Header header\n\
         geometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n\
         geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n\
         geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n",
        &[
            ("std_msgs/Header", HEADER),
            ("builtin_interfaces/Time", TIME),
            ("geometry_msgs/Quaternion", QUATERNION),
            ("geometry_msgs/Vector3", VECTOR3),
        ],
    )
}

/// v4l2_trigger_camera's: the exposure in effect for one image, with the image's stamp.
pub fn camera_exposure_schema() -> Schema {
    schema(
        "v4l2_trigger_camera/msg/CameraExposure",
        "std_msgs/Header header\n\
         float32 exposure_us\nuint32 exposure_lines\nuint32 analogue_gain\n\
         bool exposure_measured\nbool auto_exposure\n\
         float32 mean_level\nfloat32 saturated_fraction\nfloat32 gradient_metric\n",
        &WITH_HEADER,
    )
}

pub fn frame_timing_schema() -> Schema {
    schema(
        "zed_capture/msg/FrameTiming",
        "# A ZED frame's timing (zed-capture). header.stamp: the frame's sync pulse (the left\n\
         # sensor's frame start), on the IMU's clock; kernel_stamp: the kernel's frame stamp;\n\
         # mid_exposure: each eye's middle row's mid-exposure (its image's stamp). Row r of an\n\
         # eye's image is read (r - (720 - 1) / 2) * line_us later than its middle row.\n\
         uint8 EXPOSURE_NONE=0\nuint8 EXPOSURE_OWN=1\nuint8 EXPOSURE_MANUAL=2\n\
         uint8 EXPOSURE_OTHER_EYE=3\nuint8 EXPOSURE_LAST_KNOWN=4\n\
         std_msgs/Header header\nuint32 mcu_frame_count\nuint32 sequence\n\
         builtin_interfaces/Time kernel_stamp\nbuiltin_interfaces/Time[2] mid_exposure\n\
         float32[2] exposure_us\nuint32[2] gain\nuint8[2] exposure_source\n\
         float32 line_us\nfloat32 first_row_us\nfloat32 rows_before_image\n",
        &WITH_HEADER,
    )
}

pub fn clock_offset_schema() -> Schema {
    schema(
        "zed_capture/msg/ClockOffset",
        "# The system clock against CLOCK_MONOTONIC, measured now. The file's stamps are\n\
         # CLOCK_MONOTONIC + file_offset_ns, measured once when recording started.\n\
         std_msgs/Header header\nint64 monotonic_ns\nint64 realtime_ns\nint64 file_offset_ns\n",
        &WITH_HEADER,
    )
}

pub fn compressed_image(stamp_ns: i64, frame_id: &str, format: &str, data: &[u8]) -> Vec<u8> {
    let mut c = Cdr::with_capacity(data.len());
    c.header(stamp_ns, frame_id).string(format).bytes(data);
    c.finish()
}

pub fn image_mono8(stamp_ns: i64, frame_id: &str, width: u32, height: u32, data: &[u8]) -> Vec<u8> {
    let mut c = Cdr::with_capacity(data.len());
    c.header(stamp_ns, frame_id)
        .u32(height)
        .u32(width)
        .string("mono8")
        .u8(0)
        .u32(width)
        .bytes(data);
    c.finish()
}

/// An IMU sample without orientation (its covariance's first element −1), the rest's covariance
/// unknown (zeros).
pub fn imu(stamp_ns: i64, frame_id: &str, gyro: [f32; 3], accel: [f32; 3]) -> Vec<u8> {
    let mut c = Cdr::with_capacity(400);
    c.header(stamp_ns, frame_id);
    for v in [0.0, 0.0, 0.0, 1.0] {
        c.f64(v);
    }
    c.f64(-1.0);
    for _ in 1..9 {
        c.f64(0.0);
    }
    for v in gyro {
        c.f64(v as f64);
    }
    for _ in 0..9 {
        c.f64(0.0);
    }
    for v in accel {
        c.f64(v as f64);
    }
    for _ in 0..9 {
        c.f64(0.0);
    }
    c.finish()
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Exposure {
    pub exposure_us: f32,
    pub exposure_lines: u32,
    pub analogue_gain: u32,
    pub exposure_measured: bool,
    pub auto_exposure: bool,
    pub mean_level: f32,
    pub saturated_fraction: f32,
}

pub fn camera_exposure(stamp_ns: i64, frame_id: &str, e: &Exposure) -> Vec<u8> {
    let mut c = Cdr::with_capacity(64);
    c.header(stamp_ns, frame_id)
        .f32(e.exposure_us)
        .u32(e.exposure_lines)
        .u32(e.analogue_gain)
        .bool(e.exposure_measured)
        .bool(e.auto_exposure)
        .f32(e.mean_level)
        .f32(e.saturated_fraction)
        .f32(0.0);
    c.finish()
}

/// Where an eye's exposure came from (`FrameTiming.exposure_source`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    None = 0,
    Own = 1,
    Manual = 2,
    OtherEye = 3,
    LastKnown = 4,
}

pub struct FrameTiming {
    pub pulse_ns: i64,
    pub mcu_frame_count: u32,
    pub sequence: u32,
    pub kernel_ns: i64,
    pub mid_exposure_ns: [i64; 2],
    pub exposure_us: [f32; 2],
    pub gain: [u32; 2],
    pub source: [Source; 2],
    pub line_us: f32,
    pub first_row_us: f32,
    pub rows_before_image: f32,
}

pub fn frame_timing(t: &FrameTiming) -> Vec<u8> {
    let mut c = Cdr::with_capacity(128);
    c.header(t.pulse_ns, "zed")
        .u32(t.mcu_frame_count)
        .u32(t.sequence)
        .time(t.kernel_ns);
    for m in t.mid_exposure_ns {
        c.time(m);
    }
    for e in t.exposure_us {
        c.f32(e);
    }
    for g in t.gain {
        c.u32(g);
    }
    for s in t.source {
        c.u8(s as u8);
    }
    c.f32(t.line_us)
        .f32(t.first_row_us)
        .f32(t.rows_before_image);
    c.finish()
}

pub fn clock_offset(realtime_ns: i64, monotonic_ns: i64, file_offset_ns: i64) -> Vec<u8> {
    let mut c = Cdr::with_capacity(64);
    c.header(realtime_ns, "")
        .i64(monotonic_ns)
        .i64(realtime_ns)
        .i64(file_offset_ns);
    c.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schemas_name_what_they_use() {
        let s = imu_schema();
        assert!(s.text.starts_with("std_msgs/Header header\n"));
        for t in [
            "MSG: std_msgs/Header\n",
            "MSG: builtin_interfaces/Time\n",
            "MSG: geometry_msgs/Quaternion\n",
            "MSG: geometry_msgs/Vector3\n",
        ] {
            assert!(s.text.contains(t), "{t}");
        }
    }

    #[test]
    fn compressed_image_layout() {
        let b = compressed_image(1_500_000_000, "zed_left", "jpeg", &[1, 2, 3]);
        // stamp 1 s 500000000 ns; "zed_left" (9 with its 0) at 12; "jpeg" (5) at 28; data at 40
        assert_eq!(&b[4..8], &1i32.to_le_bytes());
        assert_eq!(&b[8..12], &500_000_000u32.to_le_bytes());
        assert_eq!(&b[12..16], &9u32.to_le_bytes());
        assert_eq!(&b[16..25], b"zed_left\0");
        assert_eq!(&b[28..32], &5u32.to_le_bytes());
        assert_eq!(&b[32..37], b"jpeg\0");
        assert_eq!(&b[40..44], &3u32.to_le_bytes());
        assert_eq!(&b[44..], &[1, 2, 3]);
    }
}
