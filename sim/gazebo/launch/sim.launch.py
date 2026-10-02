import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world = os.path.join(base, "worlds", "sim.sdf")
    bridge_cfg = os.path.join(base, "config", "bridge.yaml")

    return LaunchDescription([
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(base, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(base, "worlds")),

        ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "3", world], output="screen"),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_cfg}],
            output="screen",
        ),
    ])
