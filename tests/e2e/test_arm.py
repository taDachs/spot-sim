"""Slice 8: the Spot Arm through the driver.

Needs the stack started with ARM=true; skipped otherwise. The driver detects
the arm by itself (ManipulatorState in the robot state, gripper-camera-param
in the directory) and offers its arm services.

The tests run in order and share the robot; the last one leaves it sitting,
powered off and released.
"""

import os

import pytest
import rclpy.time
from bosdyn_api_msgs.msg import ApiGraspOverrideOverride, ManipulatorState, ManipulatorStateCarryState
from bosdyn_api_msgs.msg import ManipulatorStateStowState as StowState
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from spot_msgs.srv import GetGripperCameraParameters, OverrideGraspOrCarry, SetGripperAngle
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from conftest import STARTUP_TIMEOUT_SEC, RosHarness, assert_success, frame
from driving import (
    MOTION_TIMEOUT_SEC,
    STAND_HEIGHT,
    claim_power_on_and_stand,
    power_off_and_release,
    wait_for_body_height,
)

pytestmark = pytest.mark.skipif(os.environ.get("ARM", "false").lower() not in ("1", "true"), reason="needs ARM=true")

ARM_JOINTS = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"]
GRIPPER_OPEN = -1.57
SIT_HEIGHT = 0.20


def manipulator_state(ros: RosHarness, predicate=lambda _: True) -> ManipulatorState:
    return ros.wait_for_message(
        ManipulatorState, ros.topic("manipulation_state"), MOTION_TIMEOUT_SEC, predicate=predicate
    )


def joint_position(msg: JointState, name: str) -> float:
    return msg.position[list(msg.name).index(frame(name))]


def wait_for_joint(ros: RosHarness, name: str, value: float, tolerance: float = 0.01) -> JointState:
    return ros.wait_for_message(
        JointState,
        ros.topic("joint_states"),
        MOTION_TIMEOUT_SEC,
        predicate=lambda msg: frame(name) in msg.name and abs(joint_position(msg, name) - value) < tolerance,
    )


def hand_in_body(ros: RosHarness):
    """The hand frame's position in the body frame (TF from the driver's robot state)."""
    buffer = Buffer()
    listener = TransformListener(buffer, ros.node)
    try:
        ros.wait_until(
            lambda: buffer.can_transform(frame("body"), frame("hand"), rclpy.time.Time()),
            MOTION_TIMEOUT_SEC,
            "TF body -> hand",
        )
        # Drain a few updates so the transform is current.
        ros.wait_for_message(JointState, ros.topic("joint_states"), MOTION_TIMEOUT_SEC)
        ros.wait_for_message(JointState, ros.topic("joint_states"), MOTION_TIMEOUT_SEC)
        return buffer.lookup_transform(frame("body"), frame("hand"), rclpy.time.Time(), Duration()).transform.translation
    finally:
        del listener


def test_driver_detects_the_arm(ros: RosHarness) -> None:
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    has_arm = ros.wait_for_message(Bool, ros.topic("status/has_arm"), STARTUP_TIMEOUT_SEC, qos=latched)
    assert has_arm.data

    joints = ros.wait_for_message(JointState, ros.topic("joint_states"), STARTUP_TIMEOUT_SEC)
    assert {frame(name) for name in ARM_JOINTS} <= set(joints.name)

    state = manipulator_state(ros)
    assert state.stow_state.value == StowState.STOWSTATE_STOWED
    assert state.gripper_open_percentage == pytest.approx(0.0, abs=1.0)
    assert not state.is_gripper_holding_item

    # Stowed: the hand rests on top of the body, short of its front.
    hand = hand_in_body(ros)
    assert 0.3 < hand.x < 0.6 and 0.15 < hand.z < 0.35


def test_unstow_carry_and_stow(ros: RosHarness) -> None:
    claim_power_on_and_stand(ros)

    # The driver's services block until the arm arrives (block_until_arm_arrives).
    assert_success(ros.trigger("arm_unstow"))
    assert manipulator_state(ros).stow_state.value == StowState.STOWSTATE_DEPLOYED
    ready = hand_in_body(ros)
    assert ready.x > 0.75, "ready holds the hand out in front"

    assert_success(ros.trigger("arm_carry"))
    carry = hand_in_body(ros)
    assert 0.55 < carry.x < ready.x

    assert_success(ros.trigger("arm_stow"))
    assert manipulator_state(ros).stow_state.value == StowState.STOWSTATE_STOWED
    assert hand_in_body(ros).x < 0.6


def test_gripper(ros: RosHarness) -> None:
    assert_success(ros.trigger("open_gripper"))
    wait_for_joint(ros, "arm_f1x", GRIPPER_OPEN)
    assert manipulator_state(ros).gripper_open_percentage == pytest.approx(100.0, abs=1.0)

    response = ros.call(SetGripperAngle, "set_gripper_angle", SetGripperAngle.Request(gripper_angle=45.0))
    assert_success(response)
    wait_for_joint(ros, "arm_f1x", GRIPPER_OPEN / 2, tolerance=0.02)
    assert manipulator_state(ros, lambda m: abs(m.gripper_open_percentage - 50.0) < 2.0)

    assert_success(ros.trigger("close_gripper"))
    wait_for_joint(ros, "arm_f1x", 0.0)


def test_grasp_override_and_camera_parameters(ros: RosHarness) -> None:
    request = OverrideGraspOrCarry.Request()
    request.grasp_override.value = ApiGraspOverrideOverride.OVERRIDE_HOLDING
    request.carry_override.value = ManipulatorStateCarryState.CARRY_STATE_CARRIABLE
    assert_success(ros.call(OverrideGraspOrCarry, "override_grasp_or_carry", request))
    state = manipulator_state(ros, lambda m: m.is_gripper_holding_item)
    assert state.carry_state.value == ManipulatorStateCarryState.CARRY_STATE_CARRIABLE

    request.grasp_override.value = ApiGraspOverrideOverride.OVERRIDE_NOT_HOLDING
    request.carry_override.value = ManipulatorStateCarryState.CARRY_STATE_UNKNOWN
    assert_success(ros.call(OverrideGraspOrCarry, "override_grasp_or_carry", request))
    manipulator_state(ros, lambda m: not m.is_gripper_holding_item)

    response = ros.call(GetGripperCameraParameters, "get_gripper_camera_parameters", GetGripperCameraParameters.Request())
    assert_success(response)


def test_sitting_stows_the_arm_first(ros: RosHarness) -> None:
    assert_success(ros.trigger("arm_unstow"))
    assert_success(ros.trigger("sit"))
    manipulator_state(ros, lambda m: m.stow_state.value == StowState.STOWSTATE_STOWED)
    # Stowed while still standing: the body only starts down once the arm is in.
    odometry = ros.wait_for_message(Odometry, ros.topic("odometry"), MOTION_TIMEOUT_SEC)
    assert odometry.pose.pose.position.z > STAND_HEIGHT - 0.05
    wait_for_body_height(ros, SIT_HEIGHT, tolerance=0.04)
    power_off_and_release(ros)
