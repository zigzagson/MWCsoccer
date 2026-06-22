from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="18080",
            description="HTTP port for the soccer visualization dashboard",
        ),
        DeclareLaunchArgument(
            "bind_address",
            default_value="0.0.0.0",
            description="HTTP bind address; use 127.0.0.1 for local-only access",
        ),
        DeclareLaunchArgument(
            "align_target_ball_x_m",
            default_value="0.8",
            description="Penalty alignment target in the robot forward axis",
        ),
        DeclareLaunchArgument(
            "align_target_ball_y_m",
            default_value="-0.3",
            description="Penalty alignment target in the robot lateral axis",
        ),
        DeclareLaunchArgument(
            "align_x_tolerance_m",
            default_value="0.1",
            description="Displayed penalty forward tolerance",
        ),
        DeclareLaunchArgument(
            "align_y_tolerance_m",
            default_value="0.07",
            description="Displayed penalty lateral tolerance",
        ),
        DeclareLaunchArgument(
            "goalkeeper_center_deadband_x",
            default_value="0.05",
            description="Displayed goalkeeper image-center deadband",
        ),
        Node(
            package="mwc_soccer_control",
            executable="soccer_visualizer.py",
            name="soccer_visualizer",
            output="screen",
            parameters=[{
                "port": ParameterValue(
                    LaunchConfiguration("port"),
                    value_type=int,
                ),
                "bind_address": LaunchConfiguration("bind_address"),
                "align_target_ball_x_m": ParameterValue(
                    LaunchConfiguration("align_target_ball_x_m"),
                    value_type=float,
                ),
                "align_target_ball_y_m": ParameterValue(
                    LaunchConfiguration("align_target_ball_y_m"),
                    value_type=float,
                ),
                "align_x_tolerance_m": ParameterValue(
                    LaunchConfiguration("align_x_tolerance_m"),
                    value_type=float,
                ),
                "align_y_tolerance_m": ParameterValue(
                    LaunchConfiguration("align_y_tolerance_m"),
                    value_type=float,
                ),
                "goalkeeper_center_deadband_x": ParameterValue(
                    LaunchConfiguration("goalkeeper_center_deadband_x"),
                    value_type=float,
                ),
            }],
        ),
    ])
