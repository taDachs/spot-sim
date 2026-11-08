"""Slice 4: Spot walks with cmd_vel, timed in simulation time.

Speeds are measured from odometry stamps, which follow the robot clock
(simulation time), so the same expectations hold at any real-time factor.
tests/run_time_matrix.sh runs this file at several RTFs and with and without
use_sim_time.

The tests run in order and share the robot; the last one leaves it sitting,
powered off and released, as the other test files expect.
"""

import math
import time
import typing

import pytest
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from spot_msgs.msg import FootState, FootStateArray

from conftest import STARTUP_TIMEOUT_SEC, RosHarness
from driving import (
    CMD_DURATION,
    MOTION_TIMEOUT_SEC,
    RTF,
    STAND_HEIGHT,
    Drive,
    after,
    claim_power_on_and_stand,
    planar_speed,
    power_off_and_release,
    stamp_sec,
    twist,
    yaw,
)

def test_simulation_runs_at_requested_rtf(ros: RosHarness) -> None:
    """Sanity check for the RTF matrix: the simulation really runs at RTF."""
    first = ros.wait_for_message(Clock, "/clock", STARTUP_TIMEOUT_SEC)
    wall_start = time.monotonic()
    time.sleep(3.0)
    last = ros.wait_for_message(Clock, "/clock", 5.0)
    wall = time.monotonic() - wall_start
    sim = (last.clock.sec - first.clock.sec) + (last.clock.nanosec - first.clock.nanosec) * 1e-9
    measured = sim / wall
    assert abs(measured - RTF) < 0.3 * RTF, f"simulation runs at RTF {measured:.2f}, expected {RTF}"


def test_stand_up_for_driving(ros: RosHarness) -> None:
    claim_power_on_and_stand(ros)


def test_walks_forward_at_commanded_speed(ros: RosHarness) -> None:
    drive = Drive(ros)
    try:
        start = drive.wait_until_still()
        samples = drive.drive_for(twist(v_x=0.5), sim_seconds=3.0)
    finally:
        drive.close()

    steady = after(samples, 1.0)
    assert len(steady) >= 5, "too few odometry samples while walking"
    a, b = steady[0], steady[-1]
    dt = stamp_sec(b) - stamp_sec(a)
    dx = b.pose.pose.position.x - a.pose.pose.position.x
    dy = b.pose.pose.position.y - a.pose.pose.position.y
    # Distance per second of robot time, independent of RTF and of the clock
    # the driver runs on. Commands that lapsed between cmd_vel messages
    # would show up as a lower speed.
    heading = yaw(start)
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    assert forward / dt == pytest.approx(0.5, rel=0.05)
    assert abs(-dx * math.sin(heading) + dy * math.cos(heading)) < 0.02
    # The reported body velocity matches the motion.
    assert planar_speed(b) == pytest.approx(0.5, rel=0.05)
    # Walking does not change the body height.
    assert abs(b.pose.pose.position.z - STAND_HEIGHT) < 0.02


def test_stops_after_cmd_vel_stops(ros: RosHarness) -> None:
    drive = Drive(ros)
    try:
        drive.drive_for(twist(v_x=0.5), sim_seconds=1.5)
        last_command = stamp_sec(drive.odometry[-1])
        still = drive.wait_until_still()
    finally:
        drive.close()
    # The last command expires after cmd_duration, then the base decelerates
    # (0.5 m/s at 2 m/s^2 takes 0.25 s). Allow for odometry latency.
    stopped_after = stamp_sec(still) - last_command
    assert stopped_after < CMD_DURATION + 0.25 + 0.5, f"still moving {stopped_after:.2f} s after the last command"


def test_turns_in_place(ros: RosHarness) -> None:
    drive = Drive(ros)
    try:
        start = drive.wait_until_still()
        samples = drive.drive_for(twist(v_rot=0.5), sim_seconds=3.0)
    finally:
        drive.close()
    steady = after(samples, 1.0)
    a, b = steady[0], steady[-1]
    turned = math.remainder(yaw(b) - yaw(a), 2.0 * math.pi)
    assert turned / (stamp_sec(b) - stamp_sec(a)) == pytest.approx(0.5, rel=0.05)
    assert b.twist.twist.angular.z == pytest.approx(0.5, rel=0.05)
    moved = math.hypot(
        b.pose.pose.position.x - start.pose.pose.position.x, b.pose.pose.position.y - start.pose.pose.position.y
    )
    assert moved < 0.05


def test_walks_sideways_in_body_frame(ros: RosHarness) -> None:
    """cmd_vel is in the body frame: after turning, sideways is not odom y."""
    drive = Drive(ros)
    try:
        start = drive.wait_until_still()
        samples = drive.drive_for(twist(v_y=0.3), sim_seconds=2.5)
    finally:
        drive.close()
    a, b = after(samples, 1.0)[0], samples[-1]
    heading = yaw(start)
    dx = b.pose.pose.position.x - a.pose.pose.position.x
    dy = b.pose.pose.position.y - a.pose.pose.position.y
    left = -dx * math.sin(heading) + dy * math.cos(heading)
    assert left / (stamp_sec(b) - stamp_sec(a)) == pytest.approx(0.3, rel=0.05)


def test_legs_trot_while_walking(ros: RosHarness) -> None:
    drive = Drive(ros)
    knees: typing.List[float] = []
    lifted: typing.Set[int] = set()

    def on_joints(msg: JointState) -> None:
        knees.extend(p for n, p in zip(msg.name, msg.position) if n.endswith("front_left_knee"))

    def on_feet(msg: FootStateArray) -> None:
        lifted.update(i for i, f in enumerate(msg.states) if f.contact == FootState.CONTACT_LOST)

    subs = [
        ros.node.create_subscription(JointState, ros.topic("joint_states"), on_joints, qos_profile_sensor_data),
        ros.node.create_subscription(FootStateArray, ros.topic("status/feet"), on_feet, qos_profile_sensor_data),
    ]
    try:
        drive.drive_for(twist(v_x=0.4), sim_seconds=2.0)
    finally:
        for sub in subs:
            ros.node.destroy_subscription(sub)
        drive.close()
    assert knees and max(knees) - min(knees) > 0.1, "front left knee did not move while walking"
    assert lifted == {0, 1, 2, 3}, f"only feet {sorted(lifted)} lifted off the ground"

    # Standing still again: legs back in the nominal stance.
    drive = Drive(ros)
    try:
        drive.wait_until_still()
    finally:
        drive.close()
    feet = ros.wait_for_message(
        FootStateArray,
        ros.topic("status/feet"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: all(f.contact == FootState.CONTACT_MADE for f in msg.states),
    )
    assert len(feet.states) == 4


def test_sit_and_power_off_after_driving(ros: RosHarness) -> None:
    power_off_and_release(ros)
