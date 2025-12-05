#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:       task2_nav.launch.py
*  Description:    Launch Ignition Gazebo with Nav2 for autonomous navigation
*  Created:        2025-12-01
*****************************************************************************************
'''

import os
import xacro
from os.path import join
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node

def generate_launch_description():
    eyantra_warehouse = get_package_share_directory("eyantra_warehouse")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ebot_description = get_package_share_directory('ebot_description')
    ebot_prefix = get_package_prefix('ebot_description')

    # World file
    world_file = LaunchConfiguration(
        "world_file", 
        default=join(eyantra_warehouse, "worlds", "eyantra_warehouse_task2.world")
    )

    # Nav2 configuration files
    nav2_params_file = os.path.join(eyantra_warehouse, 'config', 'nav2_params.yaml')
    map_file = os.path.join(eyantra_warehouse, 'maps', 'campus_map.yaml')
    rviz_config_file = os.path.join(eyantra_warehouse, 'config', 'nav2.rviz')

    # Process robot description from XACRO
    xacro_file = os.path.join(ebot_description, 'models', 'ebot', 'ebot_description.xacro')
    robot_desc = xacro.process_file(xacro_file, mappings={'prefix': 'ebot_'}).toxml()

    # Gazebo simulation
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ebot_bridge',
        parameters=[{
            'config_file': os.path.join(eyantra_warehouse, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True
        }],
        output='screen'
    )

    # Robot State Publisher (publishes robot transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ebot_state_publisher',
        namespace='ebot',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }]
    )

    # Spawn robot in Gazebo (delayed to allow simulation to start)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_ebot',
                output='screen',
                arguments=[
                    '-name', 'ebot',
                    '-topic', '/ebot/robot_description',
                    '-x', '-1.5339',
                    '-y', '-6.6156',
                    '-z', '0.0550',
                    '-Y', '1.57'
                ],
            )
        ]
    )

    # Nav2 bringup (delayed to allow simulation and robot to start)
    nav2_bringup_launch = TimerAction(
        period=8.0,  # Wait 8 seconds for simulation clock to sync properly
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'True',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    # RViz (delayed to allow Nav2 to start)
    rviz_node = TimerAction(
        period=10.0,  # Wait 10 seconds for Nav2 to initialize
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Armed0 script (if needed)
    armed0 = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="eyantra_warehouse",
                executable="armed0.py",
                name="_armed0",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Initial pose publisher (sets AMCL pose automatically)
    initial_pose_publisher = TimerAction(
        period=9.0,  # Publish after Nav2 starts but before RViz
        actions=[
            Node(
                package="eyantra_warehouse",
                executable="publish_initial_pose.py",
                name="initial_pose_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    return LaunchDescription([
        # Set resource paths for Gazebo
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(eyantra_warehouse, "worlds")
        ),
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(eyantra_warehouse, "models")
        ),
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(ebot_description, "models")
        ),
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_PLUGIN_PATH",
            value=join(ebot_prefix, "lib")
        ),
        # Declare launch arguments
        DeclareLaunchArgument("world_file", default_value=world_file),
        # Launch components in order
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_robot,
        armed0,
        nav2_bringup_launch,
        initial_pose_publisher,
        rviz_node
    ])
