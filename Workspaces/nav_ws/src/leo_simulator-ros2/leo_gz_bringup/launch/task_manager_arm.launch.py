from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

# Package name (where your ros2_control config lives)
packageName = 'demo_control'
ros2controlRelativePath = 'config/robot_controller.yaml'


def generate_launch_description():
    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Package paths ---
    pkgPath = FindPackageShare(packageName).find(packageName)
    ros2ControlPath = os.path.join(pkgPath, ros2controlRelativePath)

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

    # --- Sensor Processing Nodes ---
    scan_filtering_node = Node(
        package='scan_filtering_package',
        executable='scan_filtering_node',
        name='scan_filtering_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    point_cloud_node = Node(
        package='point_cloud_gen_pkg',
        executable='point_cloud_gen_node',
        name='point_cloud_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- SLAM Toolbox ---
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
            'use_sim_time': 'true'
        }.items()
    )

    # --- Nav2 ---
    nav2_params_path = os.path.expanduser(
        '~/Team9_repository/Workspaces/nav_ws/src/config/nav2_params_pointcloud.yaml'
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

    # --- Custom Nodes ---
    frontier_exploration = Node(
        package='frontier_exploration_package',
        executable='frontier_exploration_node',
        output='screen'
    )

    #obj_detect = Node(
    #    package='colour_detect_pkg',
    #    executable='colour_detect_node',
    #    output='screen'
    #)
    
    vision_node = Node(
        package = 'cylinder_detector_pkg',
        executable = 'yolo_realsense_node',
        output = 'screen')

    task_manager = Node(
        package='task_manager_pkg',
        executable='task_manager',
        output='screen'
    )

    approach_object = Node(
        package='approach_obj_pkg',
        executable='approach_obj_node',
        output='screen'
    )

    move_to_spawn = Node(
        package='move_to_spawn_pkg',
        executable='move_to_spawn_node',
        output='screen'
    )

    backup_node = Node(
        package='backup_pkg',
        executable='back_up_node',
        output='screen'
    )

    #temp_manipulator_node = Node(
    #    package='temp_manipulator_pkg',
    #    executable='temp_manipulator_node',
    #    output='screen'
    #)

    simplified_manipulator_node = Node(
        package='simplified_manipulator_pkg',
        executable='simplified_manipulator_node',
        output='screen'
    )

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
        parameters=[{'use_sim_time': True}]
    )

    # --- ros2_control ---
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2ControlPath],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", ros2ControlPath],
        output="screen"
    )

    # Delay controller spawning to ensure Gazebo + robot are ready
    delayed_controller_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            robot_controller_spawner
        ]
    )

    # --- Launch সব ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        scan_filtering_node,
        point_cloud_node,
        slam_launch,
        nav2_launch,
        frontier_exploration,
        vision_node,
        #obj_detect,
        task_manager,
        approach_object,
        move_to_spawn,
        backup_node,
        simplified_manipulator_node,
        rviz_node,
        control_node,
        delayed_controller_spawners
    ])
