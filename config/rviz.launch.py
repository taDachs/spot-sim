"""RViz for the simulated Spot, on simulation time.

Uses spot_driver's RViz template, adapted to the driver's namespaced frames:
the fixed frame is the driver's TF root (config/driver.yaml: tf_root) and the
camera follows the robot's body. With point_clouds:=true it also shows the
body cameras' point clouds (/<spot_name>/depth_registered/<cam>/points).
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The hand camera's display stays empty on a robot without an arm.
CAMERAS = ["frontleft", "frontright", "left", "right", "back", "hand"]


def point_cloud_display(topic: str, name: str) -> dict:
    return {
        "Class": "rviz_default_plugins/PointCloud2",
        "Name": name,
        "Enabled": True,
        "Value": True,
        "Topic": {
            "Value": topic,
            "Depth": 1,
            "Durability Policy": "Volatile",
            "History Policy": "Keep Last",
            # Clouds are large; dropping a stale one beats waiting for it.
            "Reliability Policy": "Best Effort",
        },
        "Style": "Points",
        "Size (Pixels)": 2,
        "Color Transformer": "RGB8",
        "Decay Time": 0,
        "Selectable": False,
    }


def rviz_config(spot_name: str, fixed_frame: str, follow_frame: str, point_clouds: bool) -> str:
    template_path = os.path.join(get_package_share_directory("spot_driver"), "rviz", "spot_template.yaml")
    with open(template_path, encoding="utf-8") as f:
        config = yaml.safe_load(f)

    manager = config["Visualization Manager"]
    manager["Global Options"]["Fixed Frame"] = fixed_frame
    manager["Views"]["Current"]["Target Frame"] = follow_frame
    if spot_name:
        for display in manager["Displays"]:
            if "RobotModel" in display["Class"]:
                display["Description Topic"]["Value"] = f"/{spot_name}/robot_description"
            if "Image" in display["Class"]:
                display["Topic"]["Value"] = f"/{spot_name}{display['Topic']['Value']}"
    if point_clouds:
        prefix = f"/{spot_name}" if spot_name else ""
        manager["Displays"].append(
            {
                "Class": "rviz_common/Group",
                "Name": "Point clouds",
                "Enabled": True,
                "Value": True,
                "Displays": [
                    point_cloud_display(f"{prefix}/depth_registered/{camera}/points", camera) for camera in CAMERAS
                ],
            }
        )

    with tempfile.NamedTemporaryFile("w", prefix="spot_sim_", suffix=".rviz", delete=False) as f:
        yaml.safe_dump(config, f)
        return f.name


def launch_setup(context: LaunchContext):
    spot_name = LaunchConfiguration("spot_name").perform(context)
    prefix = f"{spot_name}/" if spot_name else ""
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context) or f"{prefix}odom"
    point_clouds = LaunchConfiguration("point_clouds").perform(context).lower() in ("true", "1")
    config = rviz_config(spot_name, fixed_frame, follow_frame=f"{prefix}body", point_clouds=point_clouds)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", config],
            # Driver output is stamped with sim time; RViz must use the same clock.
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"}],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("spot_name", default_value="spot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "point_clouds", default_value="false", description="Show the body cameras' point clouds"
            ),
            DeclareLaunchArgument(
                "fixed_frame", default_value="", description="RViz fixed frame (default: <spot_name>/odom, the TF root)"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
