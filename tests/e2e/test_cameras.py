"""Slices 6 and 10: body cameras (and, with ARM=true, the hand camera)
through the driver's image publisher.

Needs the stack started with IMAGES=true (rendering in the sim, image
publishers in the driver); skipped otherwise. Works in any world: the checks
rely on the ground plane and the sky only.
"""

import math
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
from driving import assert_success, claim_power_on_and_stand, power_off_and_release, wait_for_body_height

pytestmark = pytest.mark.skipif(
    os.environ.get("IMAGES", "false").lower() not in ("1", "true"), reason="needs IMAGES=true"
)

BODY_CAMERAS = ["frontleft", "frontright", "left", "right", "back"]
ARM = os.environ.get("ARM", "false").lower() in ("1", "true")
CAMERAS = BODY_CAMERAS + (["hand"] if ARM else [])
# Like the robot: 640x480 visual cameras, 424x240 depth sensors on the body,
# a 224x171 time-of-flight sensor in the hand. Registered depth (and the
# point clouds made from it) has the visual resolution.
WIDTH, HEIGHT = 640, 480
DEPTH_SIZE = {**{camera: (424, 240) for camera in BODY_CAMERAS}, "hand": (224, 171)}
# Frames of the visual and the depth images.
VISUAL_FRAME = {**{camera: f"{camera}_fisheye" for camera in BODY_CAMERAS}, "hand": "hand_color_image_sensor"}
DEPTH_FRAME = {**{camera: camera for camera in BODY_CAMERAS}, "hand": "hand_depth_sensor"}
# The driver's rgb_cameras parameter (config/driver_launch.sh); the hand camera is always in color.
RGB_CAMERAS = os.environ.get("RGB_CAMERAS", "true").lower() in ("1", "true")


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
    assert image.encoding == ("bgr8" if RGB_CAMERAS or camera == "hand" else "mono8")
    assert image.header.frame_id == frame(VISUAL_FRAME[camera])

    info = ros.wait_for_message(CameraInfo, ros.topic(f"camera/{camera}/camera_info"), STARTUP_TIMEOUT_SEC)
    assert (info.width, info.height) == (WIDTH, HEIGHT)
    assert info.k[0] > 100.0 and info.k[4] > 100.0
    assert info.k[2] == pytest.approx(WIDTH / 2, abs=1.0)
    assert info.k[5] == pytest.approx(HEIGHT / 2, abs=1.0)
    assert info.header.frame_id == image.header.frame_id


@pytest.mark.parametrize("camera", CAMERAS)
def test_depth_images(ros: RosHarness, camera: str) -> None:
    depth = ros.wait_for_message(Image, ros.topic(f"depth/{camera}/image"), STARTUP_TIMEOUT_SEC)
    assert (depth.width, depth.height) == DEPTH_SIZE[camera]
    assert depth.header.frame_id == frame(DEPTH_FRAME[camera])
    pixels = depth_array(depth)

    registered = ros.wait_for_message(Image, ros.topic(f"depth_registered/{camera}/image"), STARTUP_TIMEOUT_SEC)
    assert registered.header.frame_id == frame(VISUAL_FRAME[camera])
    assert (registered.width, registered.height) == (WIDTH, HEIGHT)
    if camera == "hand":
        return  # the closed gripper blocks the view: test_hand_camera_sees_once_the_gripper_opens
    assert np.count_nonzero(pixels) > 0.1 * pixels.size, "the ground should be in view"
    assert np.count_nonzero(depth_array(registered)) > 0.1 * WIDTH * HEIGHT, "registered depth is empty"


def test_front_camera_sees_ground_below_and_sky_above(ros: RosHarness) -> None:
    """Checks the camera orientation: tilted down, image upright."""
    pixels = depth_array(ros.wait_for_message(Image, ros.topic("depth/frontleft/image"), STARTUP_TIMEOUT_SEC))
    center = pixels.shape[1] // 2
    bottom = pixels[-10:, center - 10 : center + 10]
    top = pixels[:10, center - 10 : center + 10]
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
            visual = VISUAL_FRAME[camera]
            ros.wait_until(
                lambda: buffer.can_transform(frame("body"), frame(visual), rclpy.time.Time()),
                STARTUP_TIMEOUT_SEC,
                f"TF body -> {visual}",
            )
            t = buffer.lookup_transform(frame("body"), frame(visual), rclpy.time.Time(), Duration())
            if camera == "hand":
                # In the gripper, through the arm's TF (arm_link_wr1): next to the hand frame.
                h = buffer.lookup_transform(frame("hand"), frame(visual), rclpy.time.Time(), Duration())
                p = h.transform.translation
                assert math.sqrt(p.x**2 + p.y**2 + p.z**2) < 0.1
            else:
                # On the body surface.
                assert 0.05 < abs(t.transform.translation.x) + abs(t.transform.translation.y) < 0.6
            assert buffer.can_transform(frame(visual), frame(DEPTH_FRAME[camera]), rclpy.time.Time())
    finally:
        del listener


@pytest.mark.skipif(
    os.environ.get("POINT_CLOUDS", "false").lower() not in ("1", "true"), reason="needs POINT_CLOUDS=true"
)
def test_point_clouds_put_the_ground_at_zero(ros: RosHarness) -> None:
    """The driver's depth_image_proc clouds, moved into odom: ground points lie at z = 0."""
    from sensor_msgs.msg import PointCloud2

    buffer = Buffer()
    listener = TransformListener(buffer, ros.node)
    try:
        for camera in CAMERAS:
            cloud = ros.wait_for_message(PointCloud2, ros.topic(f"depth_registered/{camera}/points"), STARTUP_TIMEOUT_SEC)
            assert cloud.header.frame_id == frame(VISUAL_FRAME[camera])
            assert (cloud.width, cloud.height) == (WIDTH, HEIGHT)
            offsets = {f.name: f.offset for f in cloud.fields}
            assert {"x", "y", "z", "rgb"} <= offsets.keys()
            raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
            xyz = np.stack(
                [raw[:, offsets[a] : offsets[a] + 4].copy().view(np.float32)[:, 0] for a in ("x", "y", "z")], axis=1
            ).reshape(HEIGHT, WIDTH, 3)
            # Below the horizon, the ground dominates. (The depth sensor's
            # narrower vertical field of view leaves the bottom rows empty.)
            ground = xyz[HEIGHT // 2 :, WIDTH // 4 : 3 * WIDTH // 4].reshape(-1, 3)
            if camera == "hand":
                # The hand looks straight ahead (through the gripper, closed or
                # open): not a ground check. test_hand_camera_sees_once_the_gripper_opens.
                continue
            ground = ground[np.isfinite(ground).all(axis=1)]
            assert len(ground) > 100, f"{camera}: no ground points"

            ros.wait_until(
                lambda: buffer.can_transform(frame("odom"), cloud.header.frame_id, rclpy.time.Time()),
                STARTUP_TIMEOUT_SEC,
                f"TF odom -> {cloud.header.frame_id}",
            )
            t = buffer.lookup_transform(frame("odom"), cloud.header.frame_id, rclpy.time.Time(), Duration()).transform
            q = t.rotation
            rotation = np.array(
                [
                    [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                    [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.x * q.w)],
                    [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x**2 + q.y**2)],
                ]
            )
            in_odom = ground @ rotation.T + np.array([t.translation.x, t.translation.y, t.translation.z])
            assert np.median(np.abs(in_odom[:, 2])) < 0.02, f"{camera}: ground at z={np.median(in_odom[:, 2]):.3f}"
    finally:
        del listener


def gripper_blocked_fraction(image: Image) -> float:
    """Share of the image's central half covered by the dark grey gripper."""
    pixels = np.frombuffer(bytes(image.data), np.uint8).reshape(image.height, image.step)[:, : image.width * 3]
    pixels = pixels.reshape(image.height, image.width, 3)
    center = pixels[image.height // 4 : 3 * image.height // 4, image.width // 4 : 3 * image.width // 4]
    return float(np.mean(np.all(center < 70, axis=2)))


@pytest.mark.skipif(not ARM, reason="needs ARM=true")
def test_hand_camera_sees_once_the_gripper_opens(ros: RosHarness) -> None:
    """Like on the robot, the camera in the palm looks through the jaws: closed, they block the view."""
    closed = ros.wait_for_message(Image, ros.topic("camera/hand/image"), STARTUP_TIMEOUT_SEC)
    assert gripper_blocked_fraction(closed) > 0.5

    claim_power_on_and_stand(ros)
    try:
        assert_success(ros.trigger("open_gripper"))
        stamp = stamp_sec(ros.wait_for_message(Clock, "/clock", STARTUP_TIMEOUT_SEC).clock)
        after = lambda msg: stamp_sec(msg.header.stamp) > stamp + 0.5  # noqa: E731 - rendered with the jaw open
        opened = ros.wait_for_message(Image, ros.topic("camera/hand/image"), STARTUP_TIMEOUT_SEC, predicate=after)
        assert gripper_blocked_fraction(opened) < 0.1
        depth = depth_array(ros.wait_for_message(Image, ros.topic("depth/hand/image"), STARTUP_TIMEOUT_SEC, predicate=after))
        assert np.count_nonzero(depth) > 0.2 * depth.size, "the scene should be in view"
        # The driver's default registration (depth_image_proc, no hole filling)
        # maps each depth pixel to one visual pixel: the 224x171 sensor,
        # magnified ~4x per axis, fills only a few percent of the 640x480 image.
        registered = ros.wait_for_message(
            Image, ros.topic("depth_registered/hand/image"), STARTUP_TIMEOUT_SEC, predicate=after
        )
        assert np.count_nonzero(depth_array(registered)) > 0.01 * WIDTH * HEIGHT
        assert_success(ros.trigger("close_gripper"))
    finally:
        assert_success(ros.trigger("sit"))
        wait_for_body_height(ros, 0.20, tolerance=0.04)
        power_off_and_release(ros)
