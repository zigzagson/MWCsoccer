from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    default_script = os.path.join(
        package_share,
        "scripts",
        "pre_action_flow_test.py",
    )
    default_kick_model_path = os.path.join(package_share, "config", "kick3_v0_50000.onnx")
    default_kick_trajectory_path = os.path.join(package_share, "config", "kick3.trajbin")

    params_file = LaunchConfiguration("params_file")
    test_script = LaunchConfiguration("test_script")
    sim_align_ball_x_m = LaunchConfiguration("sim_align_ball_x_m")
    sim_align_ball_y_m = LaunchConfiguration("sim_align_ball_y_m")
    kick_model_path = LaunchConfiguration("kick_model_path")
    kick_trajectory_path = LaunchConfiguration("kick_trajectory_path")

    controller = Node(
        package="mwc_soccer_control",
        executable="soccer_brain_node",
        name="soccer_brain_node",
        output="screen",
        parameters=[
            params_file,
            {
                "start_mode": "PENALTY_ATTACK",
                "remote_mode_control_enabled": False,
                "enable_kick_action": False,
                "align_require_standing_for_sample": False,
                "kick_model_path": kick_model_path,
                "kick_trajectory_path": kick_trajectory_path,
            },
        ],
    )

    tester = ExecuteProcess(
        cmd=[
            "python3",
            test_script,
            "--sim-align-ball-x-m",
            sim_align_ball_x_m,
            "--sim-align-ball-y-m",
            sim_align_ball_y_m,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="ROS2 parameter file for soccer_brain_node",
        ),
        DeclareLaunchArgument(
            "test_script",
            default_value=default_script,
            description="Path to the pre-action flow test script",
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
        DeclareLaunchArgument(
            "sim_align_ball_x_m",
            default_value="0.32",
            description="Simulated ball x during ALIGN test phase",
        ),
        DeclareLaunchArgument(
            "sim_align_ball_y_m",
            default_value="-0.15",
            description="Simulated ball y during ALIGN test phase",
        ),
        controller,
        tester,
    ])
