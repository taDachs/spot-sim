"""Slice 3: Spot stands, sits and powers off safely, as Gazebo and the driver see it.

The tests run in order and share the robot.
"""

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from spot_msgs.msg import FootStateArray, FootState, PowerState

from conftest import STARTUP_TIMEOUT_SEC, SPOT_NAME, RosHarness

MOTION_TIMEOUT_SEC = 20.0

# Body origin above the ground (spot_sim_core::LegKinematics).
STAND_HEIGHT = 0.52
SIT_HEIGHT = 0.20
HEIGHT_TOLERANCE = 0.04


def assert_success(response) -> None:
    assert response.success, response.message


def wait_for_body_height(ros: RosHarness, height: float) -> Odometry:
    return ros.wait_for_message(
        Odometry,
        ros.topic("odometry"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: abs(msg.pose.pose.position.z - height) < HEIGHT_TOLERANCE,
    )


def wait_for_motor_state(ros: RosHarness, state: int) -> None:
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: msg.motor_power_state == state,
    )


def joint_positions(ros: RosHarness) -> dict:
    msg = ros.wait_for_message(JointState, ros.topic("joint_states"), MOTION_TIMEOUT_SEC)
    prefix = f"{SPOT_NAME}/" if SPOT_NAME else ""
    return {name.removeprefix(prefix): position for name, position in zip(msg.name, msg.position)}


def test_robot_starts_sitting_on_the_ground(ros: RosHarness) -> None:
    wait_for_body_height(ros, SIT_HEIGHT)
    feet = ros.wait_for_message(FootStateArray, ros.topic("status/feet"), STARTUP_TIMEOUT_SEC)
    assert all(f.contact == FootState.CONTACT_MADE for f in feet.states)


def test_stand(ros: RosHarness) -> None:
    assert_success(ros.trigger("claim", timeout_sec=STARTUP_TIMEOUT_SEC))
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)

    assert_success(ros.trigger("stand"))
    wait_for_body_height(ros, STAND_HEIGHT)
    joints = joint_positions(ros)
    # Legs extended compared to sitting: knees less bent.
    assert -2.0 < joints["front_left_knee"] < -1.0
    assert abs(joints["front_left_hip_x"]) < 1e-3


def test_sit(ros: RosHarness) -> None:
    assert_success(ros.trigger("sit"))
    wait_for_body_height(ros, SIT_HEIGHT)
    assert joint_positions(ros)["front_left_knee"] < -2.5


def test_safe_power_off_sits_then_cuts_power(ros: RosHarness) -> None:
    assert_success(ros.trigger("stand"))
    wait_for_body_height(ros, STAND_HEIGHT)
    assert_success(ros.trigger("power_off"))
    wait_for_body_height(ros, SIT_HEIGHT)
    wait_for_motor_state(ros, PowerState.STATE_OFF)


def test_gentle_estop_sits_before_cutting_power(ros: RosHarness) -> None:
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)
    assert_success(ros.trigger("stand"))
    wait_for_body_height(ros, STAND_HEIGHT)

    assert_success(ros.trigger("estop/gentle"))
    wait_for_body_height(ros, SIT_HEIGHT)
    wait_for_motor_state(ros, PowerState.STATE_OFF)
    assert_success(ros.trigger("estop/release"))
    assert_success(ros.trigger("release"))
