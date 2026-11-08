"""Slice 5: body height, body pose and velocity limit through the driver.

- set_stand_height: a stand command with a body height offset.
- body_pose (topic): updates the driver's mobility params, which take effect
  with the next stand or cmd_vel.
- max_velocity: velocity limit in the mobility params of cmd_vel.

The tests run in order and share the robot; the last one restores the
driver's defaults and leaves the robot sitting, powered off and released.
"""

import math

import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.srv import SetStandHeight, SetVelocity

from conftest import RosHarness
from driving import (
    MOTION_TIMEOUT_SEC,
    STAND_HEIGHT,
    Drive,
    after,
    assert_success,
    claim_power_on_and_stand,
    pitch,
    planar_speed,
    power_off_and_release,
    stamp_sec,
    twist,
    wait_for_body_height,
    wait_for_odometry,
    yaw,
)

ANGLE_TOLERANCE = 0.02


def set_stand_height(ros: RosHarness, height: float) -> None:
    request = SetStandHeight.Request()
    request.height = height
    assert_success(ros.call(SetStandHeight, "set_stand_height", request))


def set_max_velocity(ros: RosHarness, v_x: float, v_y: float, v_rot: float) -> None:
    request = SetVelocity.Request()
    request.velocity_limit = twist(v_x, v_y, v_rot)
    assert_success(ros.call(SetVelocity, "max_velocity", request))


def publish_body_pose(ros: RosHarness, height: float = 0.0, body_pitch: float = 0.0) -> None:
    """Publish on body_pose and give the driver time to take it in."""
    msg = Pose()
    msg.position.z = height
    msg.orientation.w = math.cos(body_pitch / 2)
    msg.orientation.y = math.sin(body_pitch / 2)
    publisher = ros.node.create_publisher(Pose, ros.topic("body_pose"), 1)
    try:
        for _ in range(100):
            if publisher.get_subscription_count() > 0:
                break
            ros.executor.spin_once(timeout_sec=0.05)
        assert publisher.get_subscription_count() > 0, "the driver does not subscribe to body_pose"
        # Latest-wins parameter update; publish a few times in case one is dropped.
        for _ in range(5):
            publisher.publish(msg)
            ros.executor.spin_once(timeout_sec=0.05)
    finally:
        ros.node.destroy_publisher(publisher)


def knee(ros: RosHarness) -> float:
    msg = ros.wait_for_message(JointState, ros.topic("joint_states"), MOTION_TIMEOUT_SEC)
    return next(p for n, p in zip(msg.name, msg.position) if n.endswith("front_left_knee"))


def assert_feet_on_ground(ros: RosHarness) -> None:
    feet = ros.wait_for_message(FootStateArray, ros.topic("status/feet"), MOTION_TIMEOUT_SEC)
    assert all(f.contact == FootState.CONTACT_MADE for f in feet.states)


def test_stand_up(ros: RosHarness) -> None:
    claim_power_on_and_stand(ros)


def test_stand_height(ros: RosHarness) -> None:
    nominal_knee = knee(ros)

    set_stand_height(ros, -0.15)
    wait_for_body_height(ros, STAND_HEIGHT - 0.15)
    assert knee(ros) < nominal_knee - 0.2, "crouching bends the knees"
    assert_feet_on_ground(ros)

    set_stand_height(ros, 0.1)
    wait_for_body_height(ros, STAND_HEIGHT + 0.1)
    assert knee(ros) > nominal_knee + 0.2, "standing tall stretches the knees"
    assert_feet_on_ground(ros)

    # Requests beyond the robot's range are saturated.
    set_stand_height(ros, -1.0)
    wait_for_body_height(ros, STAND_HEIGHT - 0.15)

    set_stand_height(ros, 0.0)
    wait_for_body_height(ros, STAND_HEIGHT)


def test_body_pose_applies_with_next_stand(ros: RosHarness) -> None:
    publish_body_pose(ros, height=-0.1, body_pitch=0.25)
    assert_success(ros.trigger("stand"))
    posed = wait_for_odometry(
        ros,
        lambda msg: abs(pitch(msg) - 0.25) < ANGLE_TOLERANCE
        and abs(msg.pose.pose.position.z - (STAND_HEIGHT - 0.1)) < 0.01,
    )
    assert abs(posed.pose.pose.orientation.z) < 0.01, "no yaw requested"
    assert_feet_on_ground(ros)

    publish_body_pose(ros)
    assert_success(ros.trigger("stand"))
    wait_for_odometry(
        ros, lambda msg: abs(pitch(msg)) < ANGLE_TOLERANCE and abs(msg.pose.pose.position.z - STAND_HEIGHT) < 0.01
    )


def test_body_pose_applies_while_walking(ros: RosHarness) -> None:
    publish_body_pose(ros, height=-0.1)
    drive = Drive(ros)
    try:
        samples = drive.drive_for(twist(v_x=0.4), sim_seconds=2.0)
        drive.wait_until_still()
    finally:
        drive.close()
    walking = after(samples, 1.0)
    assert walking and all(abs(s.pose.pose.position.z - (STAND_HEIGHT - 0.1)) < 0.01 for s in walking)
    assert planar_speed(walking[-1]) == pytest.approx(0.4, rel=0.05)

    publish_body_pose(ros)
    assert_success(ros.trigger("stand"))
    wait_for_body_height(ros, STAND_HEIGHT)


def test_max_velocity_limits_cmd_vel(ros: RosHarness) -> None:
    set_max_velocity(ros, 0.3, 0.2, 0.25)
    drive = Drive(ros)
    try:
        samples = drive.drive_for(twist(v_x=1.0, v_rot=1.0), sim_seconds=2.5)
        drive.wait_until_still()
    finally:
        drive.close()
    steady = after(samples, 1.0)
    a, b = steady[0], steady[-1]
    assert planar_speed(b) == pytest.approx(0.3, rel=0.05)
    turned = math.remainder(yaw(b) - yaw(a), 2.0 * math.pi)
    assert turned / (stamp_sec(b) - stamp_sec(a)) == pytest.approx(0.25, rel=0.05)


def test_restore_defaults_and_power_off(ros: RosHarness) -> None:
    # Limits above the robot's own leave only those in effect.
    set_max_velocity(ros, 10.0, 10.0, 10.0)
    publish_body_pose(ros)
    power_off_and_release(ros)

