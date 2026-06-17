"""Bring up the ORCP simulator + driver together for hardware-free testing.

    ros2 launch orcp_ros2 sim_bringup.launch.py

Starts:
  - orcp-sim          a virtual ORCP controller on a PTY at /tmp/orcp
  - orcp_driver       our ROS 2 driver, connected to that PTY

Then, in another terminal, drive it with the keyboard:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

To target real hardware instead, skip this and run the driver directly:
    ros2 run orcp_ros2 orcp_driver --ros-args -p port:=/dev/ttyACM0
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # The simulator is a plain (non-ROS) process, so launch runs it directly.
    sim = ExecuteProcess(
        cmd=["orcp-sim", "--link", "/tmp/orcp"],
        output="screen",
    )

    # Our ROS 2 driver, pointed at the simulator's PTY.
    driver = Node(
        package="orcp_ros2",
        executable="orcp_driver",
        name="orcp_driver",
        output="screen",
        parameters=[{"port": "/tmp/orcp", "preset": "SLOW"}],
    )

    return LaunchDescription([
        sim,
        # Give the simulator a moment to create the PTY before connecting.
        TimerAction(period=2.5, actions=[driver]),
    ])
