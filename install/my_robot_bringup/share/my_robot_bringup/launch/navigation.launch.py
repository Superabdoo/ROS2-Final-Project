import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Package Directories
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 2. Files
    # Path to YOUR Gazebo launch file
    gazebo_launch_file = os.path.join(pkg_my_robot_bringup, 'launch', 'gazebo.launch.py')
    
    # Path to Standard Nav2 launch file
    nav2_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    
    # Path to Map and Params
    map_file = os.path.join(pkg_my_robot_bringup, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_my_robot_bringup, 'config', 'my_nav2_params.yaml')

    # Path to Standard RViz Config (So you see the Map and Arrows immediately)
    rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # 3. Actions
    
    # A: Launch Gazebo (The Robot & World)
    # This must run so the robot exists!
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    # B: Launch Navigation (Nav2, AMCL, Map Server, Controller)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # C: Launch RViz (Explicitly!)
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_cmd,  # 1. World
        nav2_cmd,    # 2. Brain
        rviz_cmd     # 3. Eyes
    ])
