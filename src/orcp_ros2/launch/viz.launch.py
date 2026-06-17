"""Bring up the simulator + driver + visualization (Foxglove).

    ros2 launch orcp_ros2 viz.launch.py

Starts:
  - orcp-sim                a virtual ORCP controller (/tmp/orcp)
  - orcp_driver             our ROS 2 driver
  - robot_state_publisher   publishes the robot body shape (URDF) + wheel frames
  - foxglove_bridge         a websocket server on port 8765

Then open Foxglove (https://app.foxglove.dev or the desktop app), connect to
  ws://localhost:8765
add a 3D panel, set its frame to "odom", and drive with:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("orcp_ros2")
    with open(os.path.join(pkg, "urdf", "orcp_bot.urdf")) as f:
        robot_description = f.read()

    sim = ExecuteProcess(cmd=["orcp-sim", "--link", "/tmp/orcp"], output="screen")

    driver = Node(
        package="orcp_ros2", executable="orcp_driver", name="orcp_driver",
        output="screen",
        parameters=[{"port": "/tmp/orcp", "preset": "SLOW"}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    foxglove = Node(
        package="foxglove_bridge", executable="foxglove_bridge",
        output="screen",
    )

    return LaunchDescription([
        sim,
        robot_state_publisher,
        foxglove,
        # Let the simulator create its PTY before the driver connects.
        TimerAction(period=2.5, actions=[driver]),
    ])
