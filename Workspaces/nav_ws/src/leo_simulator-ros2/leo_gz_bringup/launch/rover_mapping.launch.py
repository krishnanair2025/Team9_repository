from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, \
    FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Real robot → do not use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- SLAM Toolbox (async mode) ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- RViz2 (default configuration) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Leo Rover teleop (XML launch file) ---
    teleop_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('leo_teleop'),
                'launch',
                'key_teleop.launch.xml'
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam_launch,
        rviz_node,
        teleop_launch
    ])
