from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare common launch argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Gazebo Simulation ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('leo_gz_bringup'),
                'launch',
                'leo_search_world.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

        # --- Scan Filtering Node ---
    scan_filtering_node = Node(
        package='scan_filtering_package',
        executable='scan_filtering_node',
        name='scan_filtering_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    slam_params_path = os.path.expanduser(
        '~/Team9_repository/Packages/nav_ws/src/config/mapper_params_online_async.yaml'
    )

    # Include SLAM Toolbox online async launch file
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
            'use_sim_time': 'true'   # optional, set to 'false' if not simulating
        }.items()
    )

    # --- Nav2 Navigation ---
# Path to your custom Nav2 parameters
    nav2_params_path = os.path.expanduser(
        '~/Team9_repository/Packages/nav_ws/src/config/nav2_params.yaml'
    )

    # --- Nav2 Navigation ---
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

    # Custom frontier exploration node
    frontier_exploration = Node(
        package = 'frontier_exploration_package',
        executable = 'frontier_exploration_node',
        output = 'screen'
    )

    # --- RViz2 with your custom config ---
    rviz_config_path = os.path.expanduser('~/Team9_repository/Packages/nav_ws/src/config/leo_nav_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Build the launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        scan_filtering_node,
        slam_launch,
        nav2_launch,
        frontier_exploration,
        rviz_node
    ])
