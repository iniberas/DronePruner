import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bridge_cfg = os.path.join(base, "config", "bridge.yaml")

    world_arg = DeclareLaunchArgument(
        'world_file', 
        default_value='sim.sdf',
    )
    world_name = LaunchConfiguration('world_file')
    world_path = PathJoinSubstitution([base, "worlds", world_name])

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': 'udp://:14550@'}.items()
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"autorepeat_rate": 20.0}],
        output="screen",
    )

    return LaunchDescription([
        world_arg,

        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(base, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(base, "worlds")),

        ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "3", world_path], output="screen"),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_cfg}],
            output="screen",
        ),

        mavros_launch,
        joy_node,
    ])