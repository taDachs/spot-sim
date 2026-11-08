"""Shared helpers for end-to-end tests that move the robot: cmd_vel, odometry, power."""

import math
import os
import time
import typing

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from spot_msgs.msg import LeaseArray, PowerState

from conftest import STARTUP_TIMEOUT_SEC, RosHarness

RTF = float(os.environ.get("RTF", "1.0"))
MOTION_TIMEOUT_SEC = 20.0
STAND_HEIGHT = 0.52  # body origin above the ground (spot_sim_core)
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


def pitch(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))


def assert_success(response) -> None:
    assert response.success, response.message


def wait_for_odometry(ros: RosHarness, predicate: typing.Callable[[Odometry], bool]) -> Odometry:
    return ros.wait_for_message(Odometry, ros.topic("odometry"), MOTION_TIMEOUT_SEC, predicate=predicate)


def wait_for_body_height(ros: RosHarness, height: float, tolerance: float = 0.01) -> Odometry:
    return wait_for_odometry(ros, lambda msg: abs(msg.pose.pose.position.z - height) < tolerance)


def wait_for_motor_state(ros: RosHarness, state: int) -> None:
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: msg.motor_power_state == state,
    )


def claim_power_on_and_stand(ros: RosHarness) -> None:
    # Wait for the driver's first lease poll: its lease-on-action re-claim
    # (e.g. in power_on) trips over a missing lease list right after startup
    # (spot_wrapper SpotLeash.claim; harmless, but it logs a traceback).
    ros.wait_for_message(LeaseArray, ros.topic("status/leases"), STARTUP_TIMEOUT_SEC)
    assert_success(ros.trigger("claim", timeout_sec=STARTUP_TIMEOUT_SEC))
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)
    assert_success(ros.trigger("stand"))
    wait_for_body_height(ros, STAND_HEIGHT, tolerance=0.02)


def power_off_and_release(ros: RosHarness) -> None:
    assert_success(ros.trigger("power_off"))
    wait_for_motor_state(ros, PowerState.STATE_OFF)
    assert_success(ros.trigger("release"))
