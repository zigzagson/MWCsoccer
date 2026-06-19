from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("mwc_soccer_control")
    default_params = os.path.join(
        package_share,
        "config",
        "soccer_brain.yaml",
    )
    default_kick_model_path = os.path.join(package_share, "config", "kick3_v0_50000.onnx")
    default_kick_trajectory_path = os.path.join(package_share, "config", "kick3.trajbin")

    params_file = LaunchConfiguration("params_file")
    kick_model_path = LaunchConfiguration("kick_model_path")
    kick_trajectory_path = LaunchConfiguration("kick_trajectory_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="ROS2 parameter file for soccer_brain_node",
        ),
        DeclareLaunchArgument(
            "kick_model_path",
            default_value=default_kick_model_path,
            description="Default kick model path under package share/config",
        ),
        DeclareLaunchArgument(
            "kick_trajectory_path",
            default_value=default_kick_trajectory_path,
            description="Default kick trajectory path under package share/config",
        ),
        Node(
            package="mwc_soccer_control",
            executable="soccer_brain_node",
            name="soccer_brain_node",
            output="screen",
            parameters=[
                # Launch args first as fallback defaults, so the params file
                # (loaded afterwards) can override them. This lets the YAML
                # kick_model_path / kick_trajectory_path take priority.
                {
                    "kick_model_path": kick_model_path,
                    "kick_trajectory_path": kick_trajectory_path,
                },
                params_file,
            ],
        ),
    ])
