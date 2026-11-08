"""Slice 1: the unmodified driver connects to the simulator and publishes robot state."""

import time

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from spot_msgs.msg import BatteryStateArray, FootStateArray, PowerState
from tf2_msgs.msg import TFMessage

from conftest import STARTUP_TIMEOUT_SEC, USE_SIM_TIME, RosHarness, frame

LEG_JOINTS = {
    f"{leg}_{joint}"
    for leg in ("front_left", "front_right", "rear_left", "rear_right")
    for joint in ("hip_x", "hip_y", "knee")
}

# Wall-clock time is ~1.7e9 s since the epoch; simulation time starts at 0.
SIM_TIME_UPPER_BOUND_SEC = 1e8


def test_clock_is_bridged(ros: RosHarness) -> None:
    msg = ros.wait_for_message(Clock, "/clock", timeout_sec=30.0)
    assert msg.clock.sec < SIM_TIME_UPPER_BOUND_SEC


def test_joint_states_cover_all_legs(ros: RosHarness) -> None:
    msg = ros.wait_for_message(JointState, ros.topic("joint_states"), STARTUP_TIMEOUT_SEC)
    assert {frame(j) for j in LEG_JOINTS} <= set(msg.name)
    assert len(msg.position) == len(msg.name)


def test_state_is_stamped_with_robot_clock(ros: RosHarness) -> None:
    msg = ros.wait_for_message(JointState, ros.topic("joint_states"), STARTUP_TIMEOUT_SEC)
    stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if USE_SIM_TIME:
        # Robot clock == sim time, driver clock == sim time -> stamps are sim time.
        assert 0.0 < stamp_sec < SIM_TIME_UPPER_BOUND_SEC
    else:
        # Driver on wall time: time sync maps robot (sim) time onto wall time.
        assert abs(stamp_sec - time.time()) < 5.0


def test_tf_has_odom_to_body(ros: RosHarness) -> None:
    def has_odom_body(msg: TFMessage) -> bool:
        return any(
            t.header.frame_id == frame("odom") and t.child_frame_id == frame("body") for t in msg.transforms
        )

    ros.wait_for_message(TFMessage, "/tf", STARTUP_TIMEOUT_SEC, predicate=has_odom_body)


def test_status_topics(ros: RosHarness) -> None:
    power = ros.wait_for_message(PowerState, ros.topic("status/power_states"), STARTUP_TIMEOUT_SEC)
    assert power.motor_power_state == PowerState.STATE_OFF
    battery = ros.wait_for_message(BatteryStateArray, ros.topic("status/battery_states"), STARTUP_TIMEOUT_SEC)
    assert battery.battery_states
    feet = ros.wait_for_message(FootStateArray, ros.topic("status/feet"), STARTUP_TIMEOUT_SEC)
    assert len(feet.states) == 4
