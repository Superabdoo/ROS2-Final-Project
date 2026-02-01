import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Path to the standard slam_toolbox launch file
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    # Launch it
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    return LaunchDescription([
        slam_node
    ])

