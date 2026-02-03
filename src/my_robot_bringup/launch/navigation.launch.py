import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package Directories
    pkg_my_robot_bringup = get_package_share_directory('my_robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Files
    nav2_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    map_file = os.path.join(pkg_my_robot_bringup, 'maps', 'my_map.yaml')

    # Launch Navigation
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': os.path.join(pkg_my_robot_bringup, 'config', 'my_nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        nav2
    ])
