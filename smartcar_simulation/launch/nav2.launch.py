from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    smartcar_sim_path = FindPackageShare('smartcar_simulation')
    nav2_beingup_path = FindPackageShare('nav2_bringup')

    default_rviz_config_path = PathJoinSubstitution([nav2_beingup_path, 'rviz', 'nav2_default_view.rviz'])

    default_model_path = PathJoinSubstitution([smartcar_sim_path, 'urdf', 'smartcar.urdf'])
    default_world_path = PathJoinSubstitution([smartcar_sim_path, 'world', 'smalltown.world'])

    default_ekf_config_path = PathJoinSubstitution([smartcar_sim_path, 'config', 'ekf.yaml'])
    default_map_config_path = PathJoinSubstitution([smartcar_sim_path, 'map', 'smalltown_world.yaml'])

    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(DeclareLaunchArgument(name='smartcar_model', default_value=default_model_path,
                                        description='Path to smartcar urdf file relative to urdf_tutorial package'))

    ld.add_action(DeclareLaunchArgument(name='world', default_value=default_world_path,
                                        description='Path to the Gazebo world file'))
    
    ld.add_action(DeclareLaunchArgument(name="ekf_config", default_value=default_ekf_config_path,
                                        description='Path to extended kalman filter config'))
    
    ld.add_action(DeclareLaunchArgument(name="map_config", default_value=default_map_config_path,
                                        description='Path to map config'))

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_smartcar',
        output='screen',
        arguments=[LaunchConfiguration('smartcar_model')]
    ))
        
    ld.add_action(Node(
        package='smartcar_simulation',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='smartcar_simulation',
        executable='wheel_odometry',
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), {'use_sim_time': True}]
    ))

    # Call this launch file before the next otherwise the map config doesn't get passed to the launch file????
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]
        ),
        launch_arguments={'map': LaunchConfiguration('map_config')}.items()
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    ))

    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'smartcar', '-file', LaunchConfiguration('smartcar_model')],
        output='screen'
    ))

    return ld
