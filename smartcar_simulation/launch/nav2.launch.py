from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    smartcar_sim_path = FindPackageShare('smartcar_simulation')

    default_map_config_path = PathJoinSubstitution([smartcar_sim_path, 'map', 'smalltown_world.yaml'])

    ld.add_action(DeclareLaunchArgument(name="map_config", default_value=default_map_config_path,
                                        description='Path to map config'))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]
        ),
        launch_arguments={'map': LaunchConfiguration('map_config')}.items(),
    ))

    return ld
