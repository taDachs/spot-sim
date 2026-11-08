"""Slice 9: moving the arm through the driver - joints, hand poses, velocity,
gaze, and the arm holding a pose in the world while the robot walks.

Needs the stack started with ARM=true; skipped otherwise. The tests run in
order and share the robot; the last one leaves it sitting, powered off and
released.
"""

import math
import os
import time
import typing

import pytest
import rclpy.time
from bosdyn.api import arm_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from bosdyn_msgs.msg import ArmVelocityCommandRequest
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from spot_msgs.action import RobotCommand
from tf2_ros import Buffer, TransformListener

from conftest import RosHarness, assert_success, frame
from driving import (
    MOTION_TIMEOUT_SEC,
    Drive,
    claim_power_on_and_stand,
    power_off_and_release,
    twist,
    wait_for_body_height,
)

pytestmark = pytest.mark.skipif(os.environ.get("ARM", "false").lower() not in ("1", "true"), reason="needs ARM=true")

POSITION_TOLERANCE = 0.01  # [m]
ANGLE_TOLERANCE = 0.02  # [rad]
SIT_HEIGHT = 0.20


class Tf:
    def __init__(self, ros: RosHarness) -> None:
        self.ros = ros
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, ros.node)

    def pose(self, parent: str, child: str) -> math_helpers.SE3Pose:
        self.ros.wait_until(
            lambda: self.buffer.can_transform(frame(parent), frame(child), rclpy.time.Time()),
            MOTION_TIMEOUT_SEC,
            f"TF {parent} -> {child}",
        )
        t = self.buffer.lookup_transform(frame(parent), frame(child), rclpy.time.Time(), Duration()).transform
        r = t.rotation
        return math_helpers.SE3Pose(
            t.translation.x, t.translation.y, t.translation.z, math_helpers.Quat(r.w, r.x, r.y, r.z)
        )

    def wait_for_pose(
        self, parent: str, child: str, expected: math_helpers.SE3Pose, timeout_sec: float = MOTION_TIMEOUT_SEC
    ) -> None:
        def close() -> bool:
            if not self.buffer.can_transform(frame(parent), frame(child), rclpy.time.Time()):
                return False
            return pose_error(self.pose(parent, child), expected) < (POSITION_TOLERANCE, ANGLE_TOLERANCE)

        try:
            self.ros.wait_until(close, timeout_sec, f"{child} at the target in {parent}")
        except BaseException:
            print(f"{child} in {parent}: {self.pose(parent, child)}, expected {expected}")
            raise


def pose_error(a: math_helpers.SE3Pose, b: math_helpers.SE3Pose) -> typing.Tuple[float, float]:
    delta = a.inverse() * b
    w = min(1.0, abs(delta.rot.w))
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)), 2.0 * math.acos(w)


@pytest.fixture(scope="module")
def tf(ros: RosHarness) -> typing.Iterator[Tf]:
    yield Tf(ros)


def send_robot_command(ros: RosHarness, command) -> RobotCommand.Result:
    """Sends a bosdyn RobotCommand proto through the driver's robot_command action; waits for the result."""
    client = ActionClient(ros.node, RobotCommand, ros.topic("robot_command"))
    try:
        ros.wait_until(lambda: client.server_is_ready(), MOTION_TIMEOUT_SEC, "robot_command action server")
        goal = RobotCommand.Goal()
        convert(command, goal.command)
        sent = client.send_goal_async(goal)
        ros.wait_until(sent.done, MOTION_TIMEOUT_SEC, "goal acceptance")
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        ros.wait_until(result.done, MOTION_TIMEOUT_SEC, "robot_command result")
        return result.result().result
    finally:
        client.destroy()


def publish(ros: RosHarness, msg_type: type, topic: str, msg, times: int = 1, period_sec: float = 0.1) -> None:
    """Publishes `msg` once the driver subscribes (and `times` times in all)."""
    publisher = ros.node.create_publisher(msg_type, ros.topic(topic), 10)
    try:
        ros.wait_until(lambda: publisher.get_subscription_count() > 0, MOTION_TIMEOUT_SEC, f"subscriber on {topic}")
        for _ in range(times):
            publisher.publish(msg)
            deadline = time.monotonic() + period_sec
            while time.monotonic() < deadline:
                ros.executor.spin_once(timeout_sec=0.05)
    finally:
        ros.node.destroy_publisher(publisher)


def test_stand_and_unstow(ros: RosHarness) -> None:
    claim_power_on_and_stand(ros)
    assert_success(ros.trigger("arm_unstow"))


def test_arm_joint_commands(ros: RosHarness) -> None:
    target = {"arm_sh0": 0.4, "arm_sh1": -1.2, "arm_el0": 1.9, "arm_el1": 0.3, "arm_wr0": -0.6, "arm_wr1": 0.5}
    publish(ros, JointState, "arm_joint_commands", JointState(name=list(target), position=list(target.values())))

    def at_target(msg: JointState) -> bool:
        positions = dict(zip(msg.name, msg.position))
        return all(abs(positions.get(frame(name), math.inf) - value) < 0.01 for name, value in target.items())

    ros.wait_for_message(JointState, ros.topic("joint_states"), MOTION_TIMEOUT_SEC, predicate=at_target)


def test_arm_pose_commands_in_body(ros: RosHarness, tf: Tf) -> None:
    target = math_helpers.SE3Pose(0.8, 0.1, 0.35, math_helpers.Quat.from_pitch(0.3))
    msg = PoseStamped()
    msg.header.frame_id = "body"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(target.x), float(target.y), float(target.z)
    q = target.rot
    msg.pose.orientation.w, msg.pose.orientation.x = float(q.w), float(q.x)
    msg.pose.orientation.y, msg.pose.orientation.z = float(q.y), float(q.z)
    publish(ros, PoseStamped, "arm_pose_commands", msg)
    tf.wait_for_pose("body", "hand", target)


def test_robot_command_hand_pose_in_odom_with_gripper(ros: RosHarness, tf: Tf) -> None:
    """Like spot_examples' arm_simple: a hand pose in odom plus a gripper command in one synchronized command."""
    odom_T_flat_body = tf.pose("odom", "body")
    odom_T_flat_body = math_helpers.SE3Pose(
        odom_T_flat_body.x, odom_T_flat_body.y, odom_T_flat_body.z, math_helpers.Quat.from_yaw(odom_T_flat_body.rot.to_yaw())
    )
    odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose(0.75, 0.0, 0.25, math_helpers.Quat())
    arm = RobotCommandBuilder.arm_pose_command(
        odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, 1.0, 0.0, 0.0, 0.0, "odom", 2.0
    )
    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
    result = send_robot_command(ros, RobotCommandBuilder.build_synchro_command(gripper, arm))
    assert result.success, result.message
    hand = tf.pose("odom", "hand")
    assert pose_error(hand, odom_T_hand)[0] < POSITION_TOLERANCE


def test_hand_stays_put_in_odom_while_walking(ros: RosHarness, tf: Tf) -> None:
    hand_before = tf.pose("odom", "hand")
    body_before = tf.pose("odom", "body")
    drive = Drive(ros)
    try:
        # Backward: walking toward the hand would soon bring it out of the arm's reach.
        drive.drive_for(twist(v_x=-0.3), sim_seconds=1.0)
        drive.wait_until_still()
    finally:
        drive.close()
    body_after = tf.pose("odom", "body")
    assert math.dist((body_before.x, body_before.y), (body_after.x, body_after.y)) > 0.15
    # Walking (a mobility-only command) leaves the arm command running.
    tf.wait_for_pose("odom", "hand", hand_before, timeout_sec=5.0)


def test_arm_velocity_commands(ros: RosHarness, tf: Tf) -> None:
    before = tf.pose("body", "hand")
    request = arm_command_pb2.ArmVelocityCommand.Request()
    request.cylindrical_velocity.linear_velocity.z = 0.1  # up; the driver runs it for arm_cmd_duration (1 s)
    msg = ArmVelocityCommandRequest()
    convert(request, msg)
    publish(ros, ArmVelocityCommandRequest, "arm_velocity_commands", msg)
    ros.wait_until(lambda: tf.pose("body", "hand").z - before.z > 0.05, MOTION_TIMEOUT_SEC, "hand moving up")

    # ... and stops at the end time: 0.1 m/s for 1 s.
    def spin(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ros.executor.spin_once(timeout_sec=0.05)

    spin(2.0)
    settled = tf.pose("body", "hand")
    spin(1.0)
    assert abs(tf.pose("body", "hand").z - settled.z) < 0.002
    assert settled.z - before.z == pytest.approx(0.1, abs=0.02)


def test_gaze(ros: RosHarness, tf: Tf) -> None:
    # From the ready pose; the hand stays in place and turns toward a point ahead-left, low.
    assert_success(ros.trigger("arm_unstow"))
    body = tf.pose("odom", "body")
    target = body * math_helpers.SE3Pose(2.0, 1.0, -0.5, math_helpers.Quat())
    result = send_robot_command(ros, RobotCommandBuilder.arm_gaze_command(target.x, target.y, target.z, "odom"))
    assert result.success, result.result.command.synchronized_feedback.arm_command_feedback.feedback.arm_gaze_feedback
    hand = tf.pose("odom", "hand")
    direction = (target.x - hand.x, target.y - hand.y, target.z - hand.z)
    norm = math.sqrt(sum(c * c for c in direction))
    x_axis = hand.rot.transform_point(1.0, 0.0, 0.0)
    assert sum(a * b / norm for a, b in zip(x_axis, direction)) > math.cos(0.05)


def test_stow_sit_and_power_off(ros: RosHarness) -> None:
    assert_success(ros.trigger("close_gripper"))
    assert_success(ros.trigger("arm_stow"))
    assert_success(ros.trigger("sit"))
    wait_for_body_height(ros, SIT_HEIGHT, tolerance=0.04)
    power_off_and_release(ros)
