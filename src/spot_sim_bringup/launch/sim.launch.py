"""Start the Spot simulator: a Gazebo server whose world serves the Spot API, plus /clock for ROS."""

import os
import re
import string
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def render_robot(context: LaunchContext, model_name: str) -> str:
    """Spot as an SDF <model> element: xacro -> URDF -> SDF (gz sdf -p)."""
    xacro_file = os.path.join(get_package_share_directory("spot_sim_description"), "urdf", "spot_sim.urdf.xacro")
    urdf = xacro.process_file(
        xacro_file, mappings={"base_plugin": LaunchConfiguration("base_plugin").perform(context)}
    ).toxml()
    # Gazebo resolves meshes through GZ_SIM_RESOURCE_PATH (set below).
    urdf = urdf.replace("package://", "model://")
    with tempfile.NamedTemporaryFile("w", prefix="spot_sim_", suffix=".urdf", delete=False) as f:
        f.write(urdf)
        urdf_path = f.name
    sdf = subprocess.run(["gz", "sdf", "-p", urdf_path], check=True, capture_output=True, text=True).stdout

    match = re.search(r"<model\b.*</model>", sdf, re.DOTALL)
    if match is None:
        raise RuntimeError(f"no <model> in SDF converted from {urdf_path}")
    model = match.group(0)
    model = re.sub(r"<model name=['\"][^'\"]*['\"]>", f"<model name='{model_name}'>", model, count=1)
    # Spawn sitting; the base plugin sets the exact height on the first step.
    pose = LaunchConfiguration("spawn_pose").perform(context)
    return model.replace(f"<model name='{model_name}'>", f"<model name='{model_name}'><pose>{pose}</pose>", 1)


def render_world(context: LaunchContext) -> str:
    """Fill the world template with launch arguments and the robot; return the rendered file's path."""
    world = LaunchConfiguration("world").perform(context)
    template_path = world if os.path.isfile(world) else os.path.join(
        get_package_share_directory("spot_sim_bringup"), "worlds", f"{world}.sdf"
    )
    with open(template_path, encoding="utf-8") as f:
        template = string.Template(f.read())
    model_name = LaunchConfiguration("model_name").perform(context)
    rendered = template.substitute(
        rtf=LaunchConfiguration("rtf").perform(context),
        step_size=LaunchConfiguration("step_size").perform(context),
        port=LaunchConfiguration("port").perform(context),
        username=os.environ.get("BOSDYN_CLIENT_USERNAME", "user"),
        password=os.environ.get("BOSDYN_CLIENT_PASSWORD", "password"),
        model_name=model_name,
        robot=render_robot(context, model_name),
    )
    with tempfile.NamedTemporaryFile("w", prefix="spot_sim_", suffix=".sdf", delete=False) as f:
        f.write(rendered)
        return f.name


def launch_setup(context: LaunchContext):
    world_file = render_world(context)
    gz_args = ["gz", "sim", "-r", "-v", LaunchConfiguration("verbosity").perform(context)]
    if LaunchConfiguration("headless").perform(context).lower() in ("true", "1"):
        gz_args.append("-s")
    gz_args.append(world_file)

    # model://spot_description/... resolves against the parent of its share directory.
    resource_paths = [os.path.dirname(get_package_share_directory("spot_description"))]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(resource_paths)),
        # emulate_tty: line-buffered output, so plugin messages show up promptly.
        ExecuteProcess(
            cmd=gz_args, output="screen", emulate_tty=True, on_exit=Shutdown(reason="gz sim exited")
        ),
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
            DeclareLaunchArgument("model_name", default_value="spot", description="Name of the robot model"),
            DeclareLaunchArgument("spawn_pose", default_value="0 0 0.2 0 0 0", description="Initial pose x y z roll pitch yaw"),
            DeclareLaunchArgument(
                "base_plugin", default_value="KinematicBaseSystem", description="Locomotion base plugin (spot_sim_gz)"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
