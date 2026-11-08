"""RViz for the simulated Spot: spot_driver's RViz setup, on simulation time."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("spot_name", default_value="spot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Driver output is stamped with sim time; RViz must use the same clock.
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("spot_driver"), "launch", "rviz.launch.py"])
                ),
                launch_arguments={"spot_name": LaunchConfiguration("spot_name")}.items(),
            ),
        ]
    )
