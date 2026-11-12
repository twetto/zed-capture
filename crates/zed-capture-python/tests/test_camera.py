"""zed_capture's Python bindings. The capture itself is tested in zed-capture; these check what
the bindings add: settings refused before the camera is touched, the clock helpers, and, with a
ZED attached and ZED_CAPTURE_CAMERA=1 (the tests take the camera for a few seconds), frames as
arrays with their timing and IMU samples, the stats, and closing."""

from __future__ import annotations

import os
import time

import numpy as np
import pytest

from zed_capture import Camera, gain_multiplier, monotonic_ns, realtime_offset_ns

needs_camera = pytest.mark.skipif(
    os.environ.get("ZED_CAPTURE_CAMERA") != "1",
    reason="needs a ZED and ZED_CAPTURE_CAMERA=1",
)


def test_gain_multiplier():
    assert gain_multiplier(128) == pytest.approx(1.0)
    assert gain_multiplier(0x100 | 192) == pytest.approx(3.0)
    assert gain_multiplier(0x7FF) == pytest.approx(8 * 255 / 128)
    assert gain_multiplier(0x200) is None


def test_clocks():
    a = monotonic_ns()
    b = monotonic_ns()
    assert 0 < a <= b
    # The system clock is past 2020, and the offset holds between two calls
    offset = realtime_offset_ns()
    assert (monotonic_ns() + offset) / 1e9 > 1.58e9
    assert abs(realtime_offset_ns() - offset) < 1_000_000
    assert abs((monotonic_ns() + offset) / 1e9 - time.time()) < 0.01


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"exposure": "auto"}, "expected camera, active, gradient or manual"),
        ({"exposure": "manual"}, "needs exposure_ms"),
        ({"exposure": "manual", "exposure_ms": -1.0}, "needs exposure_ms"),
        ({"exposure": "active", "exposure_ms": 5.0}, "for exposure='manual'"),
        ({"gain": 128}, "for exposure='manual'"),
        ({"exposure": "manual", "exposure_ms": 5.0, "gain": 0x200}, "raw gains"),
        ({"mains_hz": 55}, "expected 50, 60 or 0"),
        ({"open_timeout": 0.0}, "open_timeout"),
        ({"hid": "/dev/hidraw0"}, "hid is given with video"),
    ],
)
def test_bad_settings_are_refused_before_opening(kwargs, message):
    with pytest.raises(ValueError, match=message):
        Camera(**kwargs)


def test_a_missing_camera_is_an_os_error(tmp_path):
    with pytest.raises(OSError, match="opening"):
        Camera(video=str(tmp_path / "video99"), imu=False)


def take(cam, n):
    frames = []
    while len(frames) < n:
        f = cam.next(timeout=2.0)
        assert f is not None, cam.stats()
        frames.append(f)
    return frames


@needs_camera
def test_active_exposure_frames():
    with Camera(exposure="active", mains_hz=60) as cam:
        assert cam.exposure == "active"
        assert cam.line_ns == pytest.approx(40_066, rel=0.01)
        frames = take(cam, 60)
        stats = cam.stats()
    for f in frames:
        assert f.left.shape == f.right.shape == (cam.height, cam.width)
        assert f.left.dtype == np.uint8 and f.left.flags["C_CONTIGUOUS"]
        assert f.stamp_source == "left"
        assert set(f.exposure_source) <= {"active", "active_unchecked"}
        assert f.exposure_ns[0] == f.exposure_ns[1]
        assert f.mid_exposure_ns[0] == f.stamp_ns
        assert f.pulse_ns < f.stamp_ns < f.pulse_ns + 33_400_000
        assert f.gyro.shape == f.accel.shape == (len(f.imu_time_ns), 3)
    # Every frame's IMU samples, up to its pulse, none twice
    times = np.concatenate([f.imu_time_ns for f in frames[1:]])
    assert np.all(np.diff(times) > 0)
    assert 11 * 59 < len(times) < 16 * 59
    assert all(f.imu_time_ns[-1] <= f.pulse_ns for f in frames if len(f.imu_time_ns))
    # Gravity, about
    accel = np.concatenate([f.accel for f in frames])
    assert 9.0 < np.linalg.norm(accel.mean(axis=0)) < 10.5
    gaps = np.diff([f.pulse_ns for f in frames]) / 1e6
    assert np.all((gaps > 30) & (gaps < 70)), gaps
    assert stats["frames"] >= 60 and stats["stalls"] == 0
    assert stats["active_exposure"]["writes"] >= 1
    with pytest.raises(ValueError, match="closed"):
        cam.next()


@needs_camera
def test_the_cameras_own_exposure_stamps_by_the_right_eye():
    with Camera(mains_hz=60) as cam:
        assert cam.serial
        frames = take(cam, 30)
    f = frames[-1]
    assert f.stamp_source == "right"
    assert f.exposure_source[1] == "read" and f.exposure_ns[0] is None
    assert cam.stats()["active_exposure"] is None
