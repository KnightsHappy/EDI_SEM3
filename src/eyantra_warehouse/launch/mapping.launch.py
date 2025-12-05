# mapping.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Set to your package name
    pkg_share = get_package_share_directory('eyantra_warehouse')

    # Path to the slam_toolbox parameters file
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # Set use_sim_time to True because you are using a simulation
    use_sim_time = True 

    # SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        start_async_slam_toolbox_node,
        rviz_node
    ])