from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    smartcar_sim_path = FindPackageShare('smartcar_simulation')

    default_model_path = PathJoinSubstitution([smartcar_sim_path, 'urdf', 'smartcar.urdf'])
    default_rviz_config_path = PathJoinSubstitution([smartcar_sim_path, 'rviz', 'urdf_config.rviz'])

    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument(name='smartcar_model', default_value=default_model_path,
                                        description='Path to smartcar urdf file relative to urdf_tutorial package'))


    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        arguments=[LaunchConfiguration('smartcar_model')]
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))

    return ld
