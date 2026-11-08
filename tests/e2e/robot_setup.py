"""Bringing the shared robot up and down in end-to-end tests: claim, power,
stand, release, and waiting for the robot to get there."""

import typing

from nav_msgs.msg import Odometry
from spot_msgs.msg import LeaseArray, PowerState

from conftest import STARTUP_TIMEOUT_SEC, RosHarness, assert_success

MOTION_TIMEOUT_SEC = 20.0
STAND_HEIGHT = 0.52  # body origin above the ground (spot_sim_core)


def wait_for_odometry(ros: RosHarness, predicate: typing.Callable[[Odometry], bool]) -> Odometry:
    return ros.wait_for_message(Odometry, ros.topic("odometry"), MOTION_TIMEOUT_SEC, predicate=predicate)


def wait_for_body_height(ros: RosHarness, height: float, tolerance: float = 0.01) -> Odometry:
    return wait_for_odometry(ros, lambda msg: abs(msg.pose.pose.position.z - height) < tolerance)


def wait_for_motor_state(ros: RosHarness, state: int, timeout_sec: float = MOTION_TIMEOUT_SEC) -> None:
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        timeout_sec,
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
