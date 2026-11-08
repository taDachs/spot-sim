"""Slice 4: Spot walks with cmd_vel, timed in simulation time.

Speeds are measured from odometry stamps, which follow the robot clock
(simulation time), so the same expectations hold at any real-time factor.
tests/run_time_matrix.sh runs this file at several RTFs and with and without
use_sim_time.

The tests run in order and share the robot; the last one leaves it sitting,
powered off and released, as the other test files expect.
"""

import math
import os
import time
import typing

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from spot_msgs.msg import FootState, FootStateArray, PowerState

from conftest import STARTUP_TIMEOUT_SEC, RosHarness

RTF = float(os.environ.get("RTF", "1.0"))
MOTION_TIMEOUT_SEC = 20.0
STAND_HEIGHT = 0.52
# The driver publishes cmd_vel as velocity commands that expire after
# cmd_duration (config/driver.yaml), in client time.
CMD_DURATION = 0.25
# Publish rate of the test, in wall time. At RTF 2 consecutive commands are
# 0.1 s of simulation time apart, well inside cmd_duration.
PUBLISH_PERIOD_SEC = 0.05


def stamp_sec(msg: Odometry) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def planar_speed(msg: Odometry) -> float:
    return math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)


def is_still(msg: Odometry) -> bool:
    return planar_speed(msg) < 0.01 and abs(msg.twist.twist.angular.z) < 0.01


class Drive:
    """Publishes a twist on cmd_vel while recording odometry."""

    def __init__(self, ros: RosHarness) -> None:
        self.ros = ros
        self.odometry: typing.List[Odometry] = []
        self.publisher = ros.node.create_publisher(Twist, ros.topic("cmd_vel"), 10)
        self.subscription = ros.node.create_subscription(
            Odometry, ros.topic("odometry"), self.odometry.append, qos_profile_sensor_data
        )

    def close(self) -> None:
        self.ros.node.destroy_publisher(self.publisher)
        self.ros.node.destroy_subscription(self.subscription)

    def sim_elapsed(self) -> float:
        return stamp_sec(self.odometry[-1]) - stamp_sec(self.odometry[0]) if self.odometry else 0.0

    def run(self, twist: typing.Optional[Twist], until: typing.Callable[["Drive"], bool], timeout_sec: float) -> None:
        """Publish `twist` (or nothing) and spin until `until(self)` holds."""
        deadline = time.monotonic() + timeout_sec
        next_publish = 0.0
        while not until(self):
            now = time.monotonic()
            assert now < deadline, "timed out while driving"
            if twist is not None and now >= next_publish:
                self.publisher.publish(twist)
                next_publish = now + PUBLISH_PERIOD_SEC
            self.ros.executor.spin_once(timeout_sec=0.01)

    def drive_for(self, twist: Twist, sim_seconds: float) -> typing.List[Odometry]:
        """Publish `twist` for `sim_seconds` of robot time; return the odometry received meanwhile."""
        self.odometry.clear()
        timeout = MOTION_TIMEOUT_SEC + sim_seconds / min(RTF, 1.0) * 2.0
        self.run(twist, lambda d: d.sim_elapsed() >= sim_seconds, timeout)
        return list(self.odometry)

    def wait_until_still(self, timeout_sec: float = MOTION_TIMEOUT_SEC) -> Odometry:
        self.odometry.clear()
        self.run(None, lambda d: bool(d.odometry) and is_still(d.odometry[-1]), timeout_sec)
        return self.odometry[-1]


def twist(v_x: float = 0.0, v_y: float = 0.0, v_rot: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = v_x
    msg.linear.y = v_y
    msg.angular.z = v_rot
    return msg


def after(samples: typing.List[Odometry], sim_seconds: float) -> typing.List[Odometry]:
    """Samples at least `sim_seconds` after the first one (skip the acceleration ramp)."""
    start = stamp_sec(samples[0])
    return [s for s in samples if stamp_sec(s) - start >= sim_seconds]


def assert_success(response) -> None:
    assert response.success, response.message


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
    assert_success(ros.trigger("claim", timeout_sec=STARTUP_TIMEOUT_SEC))
    assert_success(ros.trigger("power_on"))
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: msg.motor_power_state == PowerState.STATE_ON,
    )
    assert_success(ros.trigger("stand"))
    ros.wait_for_message(
        Odometry,
        ros.topic("odometry"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: abs(msg.pose.pose.position.z - STAND_HEIGHT) < 0.02,
    )


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
    assert_success(ros.trigger("power_off"))
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: msg.motor_power_state == PowerState.STATE_OFF,
    )
    assert_success(ros.trigger("release"))

