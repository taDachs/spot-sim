"""Start the Spot simulator: a Gazebo server whose world serves the Spot API, plus /clock for ROS."""

import os
import string
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def render_world(context: LaunchContext) -> str:
    """Fill the world template with launch arguments and return the path of the rendered file."""
    world = LaunchConfiguration("world").perform(context)
    template_path = world if os.path.isfile(world) else os.path.join(
        get_package_share_directory("spot_sim_bringup"), "worlds", f"{world}.sdf"
    )
    with open(template_path, encoding="utf-8") as f:
        template = string.Template(f.read())
    rendered = template.substitute(
        rtf=LaunchConfiguration("rtf").perform(context),
        step_size=LaunchConfiguration("step_size").perform(context),
        port=LaunchConfiguration("port").perform(context),
        username=os.environ.get("BOSDYN_CLIENT_USERNAME", "user"),
        password=os.environ.get("BOSDYN_CLIENT_PASSWORD", "password"),
    )
    out = tempfile.NamedTemporaryFile("w", prefix="spot_sim_", suffix=".sdf", delete=False)
    with out:
        out.write(rendered)
    return out.name


def launch_setup(context: LaunchContext):
    world_file = render_world(context)
    gz_args = ["gz", "sim", "-r", "-v", LaunchConfiguration("verbosity").perform(context)]
    if LaunchConfiguration("headless").perform(context).lower() in ("true", "1"):
        gz_args.append("-s")
    gz_args.append(world_file)

    return [
        ExecuteProcess(cmd=gz_args, output="screen", on_exit=Shutdown(reason="gz sim exited")),
        # Simulation time for ROS nodes running with use_sim_time:=true.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="empty", description="World name in spot_sim_bringup/worlds or a path"),
            DeclareLaunchArgument("rtf", default_value="1.0", description="Target real-time factor"),
            DeclareLaunchArgument("step_size", default_value="0.001", description="Physics step size [s]"),
            DeclareLaunchArgument("port", default_value="443", description="Spot API gRPC port"),
            DeclareLaunchArgument("headless", default_value="true", description="Run the Gazebo server without GUI"),
            DeclareLaunchArgument("verbosity", default_value="3", description="gz sim verbosity (0-4)"),
            OpaqueFunction(function=launch_setup),
        ]
    )
