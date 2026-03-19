from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Scan Filtering Node ---
    scan_filtering = Node(
        package='scan_filtering_package',
        executable='scan_filtering_node',
        name='scan_filtering_node',
        output='screen'
    )

    slam_params_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/mapper_params_online_async.yaml'
    )

    # --- SLAM Toolbox ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': slam_params_path
        }.items()
    )

    # --- Nav2 Navigation ---
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
            'slam': 'True',
            'params_file': nav2_params_path
        }.items()
    )

    # --- Frontier Exploration ---
    frontier_exploration = Node(
        package='frontier_exploration_package',
        executable='frontier_exploration_node',
        output='screen'
    )

    # --- RViz2 ---
    rviz_config_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/leo_nav_mapping.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        scan_filtering,
        slam_launch,
        nav2_launch,
        frontier_exploration,
        rviz_node
    ])
