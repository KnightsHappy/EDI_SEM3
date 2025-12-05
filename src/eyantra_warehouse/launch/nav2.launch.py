#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    eyantra_warehouse_dir = get_package_share_directory('eyantra_warehouse')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Define paths to files
    nav2_params_file = os.path.join(eyantra_warehouse_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(eyantra_warehouse_dir, 'maps', 'campus_map.yaml')
    launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # Declare launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # RViz config file
    rviz_config_file = os.path.join(eyantra_warehouse_dir, 'config', 'nav2.rviz')

    # Include the Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)

    return ld
