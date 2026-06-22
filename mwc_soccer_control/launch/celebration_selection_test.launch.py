from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interval_s = LaunchConfiguration("interval_s")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "interval_s",
            default_value="2.0",
            description="Seconds between simulated celebration selections",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="18080",
            description="Visualization dashboard HTTP port",
        ),
        Node(
            package="mwc_soccer_control",
            executable="soccer_visualizer.py",
            name="soccer_visualizer",
            output="screen",
            parameters=[{
                "bind_address": "0.0.0.0",
                "port": ParameterValue(port, value_type=int),
            }],
        ),
        Node(
            package="mwc_soccer_control",
            executable="celebration_selection_test.py",
            name="celebration_selection_test",
            output="screen",
            parameters=[{
                "interval_s": ParameterValue(interval_s, value_type=float),
            }],
        ),
    ])
