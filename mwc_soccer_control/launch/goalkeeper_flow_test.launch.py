from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("mwc_soccer_control")
    params_file = os.path.join(package_share, "config", "soccer_brain.yaml")
    test_script = os.path.join(
        package_share, "scripts", "goalkeeper_flow_test.py"
    )

    controller = Node(
            package="mwc_soccer_control",
            executable="soccer_brain_node",
            name="soccer_brain_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "start_mode": "GOALKEEPER",
                    "remote_mode_control_enabled": False,
                    "goalkeeper_enable_motion": False,
                },
            ],
        )
    tester = ExecuteProcess(
        cmd=["python3", test_script],
        output="screen",
    )

    return LaunchDescription([
        controller,
        tester,
        RegisterEventHandler(
            OnProcessExit(
                target_action=tester,
                on_exit=[Shutdown(reason="goalkeeper flow test completed")],
            )
        ),
        TimerAction(
            period=8.0,
            actions=[Shutdown(reason="goalkeeper flow test timeout")],
        ),
    ])
