"""Local planner: walking to goals (the driver's `trajectory` action) and
obstacle avoidance on cmd_vel, around the objects of the yard world.

Needs WORLD=yard; skipped otherwise. The tests run in order and share the
robot (claimed and standing for the module). Positions are in odom, which
is the world frame here: the robot boots at the origin facing +x.

With the kinematic base the robot has no collisions: only the planner
keeps it clear of obstacles, and with avoidance disabled it walks through
them (test_trajectory_without_avoidance_goes_straight). An RL base with
physics would stop at the box instead.
"""

import math
import os
import typing

import pytest
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from spot_msgs.action import Trajectory

from conftest import RosHarness
from driving import Drive, twist, yaw
from robot_setup import MOTION_TIMEOUT_SEC, claim_power_on_and_stand, power_off_and_release, wait_for_odometry

pytestmark = pytest.mark.skipif(os.environ.get("WORLD", "empty") != "yard", reason="needs WORLD=yard")

# The red box of yard.sdf: a 0.8 m cube at (3.2, 0), yawed 0.3 rad. Its
# farthest corner is 0.57 m from the centre.
BOX = (3.2, 0.0)
BOX_HALF_DIAGONAL = 0.4 * math.sqrt(2.0)
POSITION_TOLERANCE = 0.15  # planner tolerance 0.1 m, plus odometry delay
YAW_TOLERANCE = 0.15
TRAJECTORY_DURATION_SEC = 30


class Pose2(typing.NamedTuple):
    x: float
    y: float
    yaw: float


def pose_of(msg: Odometry) -> Pose2:
    return Pose2(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw(msg))


def current_pose(ros: RosHarness) -> Pose2:
    return pose_of(wait_for_odometry(ros, lambda msg: True))


def angle_difference(a: float, b: float) -> float:
    return math.remainder(a - b, 2.0 * math.pi)


def distance_to_box(pose: Pose2) -> float:
    return math.hypot(pose.x - BOX[0], pose.y - BOX[1])


class GoTo(typing.NamedTuple):
    success: bool
    message: str
    path: typing.List[Pose2]  # odometry received meanwhile


def go_to(ros: RosHarness, x: float, y: float, heading: float, avoid: bool = True) -> GoTo:
    """Sends the driver's `trajectory` action to the odom pose (x, y, heading)."""
    start = current_pose(ros)
    # The action takes the goal in the body frame.
    dx, dy = x - start.x, y - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    dyaw = angle_difference(heading, start.yaw)
    goal = Trajectory.Goal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = "body"
    goal.target_pose.pose.position.x = c * dx + s * dy
    goal.target_pose.pose.position.y = -s * dx + c * dy
    goal.target_pose.pose.orientation.z = math.sin(dyaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(dyaw / 2.0)
    goal.duration = DurationMsg(sec=TRAJECTORY_DURATION_SEC)
    goal.precise_positioning = True
    goal.disable_obstacle_avoidance = not avoid

    path: typing.List[Pose2] = []
    subscription = ros.node.create_subscription(
        Odometry, ros.topic("odometry"), lambda msg: path.append(pose_of(msg)), qos_profile_sensor_data
    )
    client = ActionClient(ros.node, Trajectory, ros.topic("trajectory"))
    try:
        ros.wait_until(lambda: client.server_is_ready(), MOTION_TIMEOUT_SEC, "trajectory action server")
        sent = client.send_goal_async(goal)
        ros.wait_until(sent.done, MOTION_TIMEOUT_SEC, "goal acceptance")
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        ros.wait_until(result.done, TRAJECTORY_DURATION_SEC + MOTION_TIMEOUT_SEC, "trajectory result")
        outcome = result.result().result
        return GoTo(outcome.success, outcome.message, path)
    finally:
        client.destroy()
        ros.node.destroy_subscription(subscription)


def assert_at(pose: Pose2, x: float, y: float, heading: float) -> None:
    assert math.hypot(pose.x - x, pose.y - y) < POSITION_TOLERANCE, pose
    assert abs(angle_difference(pose.yaw, heading)) < YAW_TOLERANCE, pose


@pytest.fixture(scope="module", autouse=True)
def standing_robot(ros: RosHarness) -> typing.Iterator[None]:
    claim_power_on_and_stand(ros)
    yield
    power_off_and_release(ros)


def test_walks_to_a_free_goal(ros: RosHarness) -> None:
    result = go_to(ros, 0.5, -0.8, math.pi / 2)
    assert result.success, result.message
    assert_at(current_pose(ros), 0.5, -0.8, math.pi / 2)


def test_walks_around_an_obstacle(ros: RosHarness) -> None:
    # Behind the box: the straight line from (0.5, -0.8) runs through it.
    result = go_to(ros, 4.7, 0.2, 0.0)
    assert result.success, result.message
    assert_at(current_pose(ros), 4.7, 0.2, 0.0)
    closest = min(distance_to_box(p) for p in result.path)
    # Body half-width 0.25 m plus the corner: anything less would touch.
    assert closest > BOX_HALF_DIAGONAL + 0.25, f"came within {closest:.2f} m of the box centre"


def test_goal_inside_an_obstacle_is_blocked(ros: RosHarness) -> None:
    result = go_to(ros, BOX[0], BOX[1], math.pi)
    assert not result.success
    assert result.message == "not at goal", result.message
    assert distance_to_box(current_pose(ros)) > BOX_HALF_DIAGONAL + 0.25


def test_cmd_vel_keeps_clear_of_an_obstacle(ros: RosHarness) -> None:
    # Face the box from behind, then drive straight at it: the robot stops in
    # front of it or slides past it, but never touches it.
    assert go_to(ros, 5.0, 0.0, math.pi).success
    drive = Drive(ros)
    try:
        samples = drive.drive_for(twist(v_x=0.5), sim_seconds=5.0)
        drive.wait_until_still()
    finally:
        drive.close()
    poses = [pose_of(msg) for msg in samples]
    assert poses[-1].x < 4.7, f"didn't drive: {poses[-1]}"
    closest = min(distance_to_box(p) for p in poses)
    assert closest > BOX_HALF_DIAGONAL + 0.25, f"came within {closest:.2f} m of the box centre"


def test_trajectory_without_avoidance_goes_straight(ros: RosHarness) -> None:
    # Avoidance off: straight through the box (the kinematic base has no
    # collisions; a physics base would stop at it instead).
    assert go_to(ros, 5.0, 0.0, math.pi).success
    result = go_to(ros, 2.2, 0.0, math.pi, avoid=False)
    assert result.success, result.message
    closest = min(distance_to_box(p) for p in result.path)
    assert closest < 0.3, f"went around the box ({closest:.2f} m) although avoidance was off"
