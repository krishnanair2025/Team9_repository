from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Scan filtering node to omit any detections within the robot's radius
    scan_filtering_node = Node(
        package='scan_filtering_package',
        executable='scan_filtering_node',
        name='scan_filtering_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # SLAM toolbox 
    slam_params_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/mapper_params_online_async.yaml'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': slam_params_path,
            'use_sim_time': 'false'
        }.items()
    )

    # Nav2
    nav2_params_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/nav2_params.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
            'params_file': nav2_params_path
        }.items()
    )

    # Frontier exploration
    frontier_exploration = Node(
        package='frontier_exploration_package',
        executable='frontier_exploration_node',
        output='screen'
    )

    # Vision node (replace with actual vision node !!!!!)
    obj_detect = Node(
        package='colour_detect_pkg',
        executable='colour_detect_node',
        output='screen'
    )

    # Task manager node
    task_manager = Node(
        package='task_manager_pkg',
        executable='task_manager',
        output='screen'
    )

    # Approach object node
    approach_object = Node(
        package='approach_obj_pkg',
        executable='approach_obj_node',
        output='screen'
    )

    # Move to spawn node
    move_to_spawn = Node(
        package='move_to_spawn_pkg',
        executable='move_to_spawn_node',
        output='screen'
    )

    # Backup node
    backup_node = Node(
        package='backup_pkg',
        executable='back_up_node',
        output='screen'
    )

    # Temporary manipulator node
    temp_manipulator_node = Node(
        package='temp_manipulator_pkg',
        executable='temp_manipulator_node',
        output='screen'
    )
    
    # Lidar starter
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }]
    )
    
    # Lidar TF publisher
    lidar_tf_publisher = Node(
        package = 'lidar_tf_publisher_node',
        executable = 'lidar_tf_publisher_node',
        output='screen')
        
    
    
    # --- RViz ---
    rviz_config_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/leo_nav_mapping.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}]
    )

    # --- Launch ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        scan_filtering_node,
        slam_launch,
        nav2_launch,
        frontier_exploration,
        obj_detect,
        task_manager,
        approach_object,
        move_to_spawn,
        backup_node,
        temp_manipulator_node,
        rplidar_node,
        lidar_tf_publisher,
        rviz_node,
    ])
