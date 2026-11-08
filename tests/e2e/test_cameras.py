"""Slice 6: body cameras through the driver's image publisher.

Needs the stack started with IMAGES=true (rendering in the sim, image
publishers in the driver); skipped otherwise. Works in any world: the checks
rely on the ground plane and the sky only.
"""

import os
import typing

import numpy as np
import pytest
import rclpy.time
from rclpy.duration import Duration
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from conftest import SPOT_NAME, STARTUP_TIMEOUT_SEC, RosHarness

pytestmark = pytest.mark.skipif(
    os.environ.get("IMAGES", "false").lower() not in ("1", "true"), reason="needs IMAGES=true"
)

CAMERAS = ["frontleft", "frontright", "left", "right", "back"]
WIDTH, HEIGHT = 640, 480


def frame(name: str) -> str:
    return f"{SPOT_NAME}/{name}" if SPOT_NAME else name


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def depth_array(msg: Image) -> np.ndarray:
    assert msg.encoding == "16UC1"
    return np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]


def collect(ros: RosHarness, topic: str, count: int, timeout_sec: float) -> typing.List[Image]:
    messages: typing.List[Image] = []
    sub = ros.node.create_subscription(Image, topic, messages.append, 10)
    try:
        ros.wait_until(lambda: len(messages) >= count, timeout_sec, f"{count} images on {topic}")
    finally:
        ros.node.destroy_subscription(sub)
    return messages


@pytest.mark.parametrize("camera", CAMERAS)
def test_rgb_image_and_camera_info(ros: RosHarness, camera: str) -> None:
    image = ros.wait_for_message(Image, ros.topic(f"camera/{camera}/image"), STARTUP_TIMEOUT_SEC)
    assert (image.width, image.height) == (WIDTH, HEIGHT)
    assert image.encoding == "bgr8"
    assert image.header.frame_id == frame(f"{camera}_fisheye")

    info = ros.wait_for_message(CameraInfo, ros.topic(f"camera/{camera}/camera_info"), STARTUP_TIMEOUT_SEC)
    assert (info.width, info.height) == (WIDTH, HEIGHT)
    assert info.k[0] > 100.0 and info.k[4] > 100.0
    assert info.k[2] == pytest.approx(WIDTH / 2, abs=1.0)
    assert info.k[5] == pytest.approx(HEIGHT / 2, abs=1.0)
    assert info.header.frame_id == image.header.frame_id


@pytest.mark.parametrize("camera", CAMERAS)
def test_depth_images(ros: RosHarness, camera: str) -> None:
    depth = ros.wait_for_message(Image, ros.topic(f"depth/{camera}/image"), STARTUP_TIMEOUT_SEC)
    assert (depth.width, depth.height) == (WIDTH, HEIGHT)
    assert depth.header.frame_id == frame(camera)
    pixels = depth_array(depth)
    assert np.count_nonzero(pixels) > 0.1 * pixels.size, "the ground should be in view"

    registered = ros.wait_for_message(Image, ros.topic(f"depth_registered/{camera}/image"), STARTUP_TIMEOUT_SEC)
    assert registered.header.frame_id == frame(f"{camera}_fisheye")
    assert (registered.width, registered.height) == (WIDTH, HEIGHT)


def test_front_camera_sees_ground_below_and_sky_above(ros: RosHarness) -> None:
    """Checks the camera orientation: tilted down, image upright."""
    pixels = depth_array(ros.wait_for_message(Image, ros.topic("depth/frontleft/image"), STARTUP_TIMEOUT_SEC))
    bottom = pixels[-10:, WIDTH // 2 - 10 : WIDTH // 2 + 10]
    top = pixels[:10, WIDTH // 2 - 10 : WIDTH // 2 + 10]
    # Ground close below the nose (millimeters; the robot sits or stands).
    assert np.all(bottom > 100) and np.all(bottom < 2000), bottom
    # Looking up past the horizon: nothing within range (0 = no return), unless
    # a world puts something right in front of the robot.
    assert np.count_nonzero(top) < top.size or np.median(top) > np.median(bottom)


def test_images_are_stamped_with_simulation_time_and_stream(ros: RosHarness) -> None:
    images = collect(ros, ros.topic("camera/frontleft/image"), 10, STARTUP_TIMEOUT_SEC)
    clock = ros.wait_for_message(Clock, "/clock", STARTUP_TIMEOUT_SEC)
    stamps = [stamp_sec(m.header.stamp) for m in images]
    assert stamps == sorted(stamps)
    assert len(set(stamps)) > 5, "images should be new renders, not repeats"
    # Recent: within a few seconds of simulation time.
    assert stamp_sec(clock.clock) - stamps[-1] < 5.0


def test_camera_frames_are_in_tf(ros: RosHarness) -> None:
    buffer = Buffer()
    listener = TransformListener(buffer, ros.node)
    try:
        for camera in CAMERAS:
            ros.wait_until(
                lambda: buffer.can_transform(frame("body"), frame(f"{camera}_fisheye"), rclpy.time.Time()),
                STARTUP_TIMEOUT_SEC,
                f"TF body -> {camera}_fisheye",
            )
            t = buffer.lookup_transform(frame("body"), frame(f"{camera}_fisheye"), rclpy.time.Time(), Duration())
            # On the body surface.
            assert 0.05 < abs(t.transform.translation.x) + abs(t.transform.translation.y) < 0.6
            assert buffer.can_transform(frame(f"{camera}_fisheye"), frame(camera), rclpy.time.Time())
    finally:
        del listener
