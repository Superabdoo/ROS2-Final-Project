# ============================================================================
# Name        : Launch File for Gazebo Simulation
# Author      : Edges For Training
# Description : Launches the robot in Gazebo along with required ROS 2 nodes
# ============================================================================

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# ------------------- MAIN FUNCTION -------------------
def generate_launch_description():

    ld = LaunchDescription()
    
    # Path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('edges_description'),
        'urdf',
        'edges.urdf.xacro'
    ])

    #Path to Rviz Configuration file
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('edges_description'),
        'rviz',
        'edges_config.rviz'
    ])

    # Path to custom Gazebo world
    world_file = PathJoinSubstitution([
        FindPackageShare('edges_description'),
        'worlds',
        'Test.world'
    ])


    # Robot description using xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ------------------- robot_state_publisher -------------------
    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ------------------- Start Gazebo Server -------------------
    start_gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # ------------------- Start Gazebo Client (GUI) -------------------
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # ------------------- Spawn Robot in Gazebo -------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'edges_robot'],
        output='screen'
    )

     # ----------------------RVIz ---------------------------------
    start_rviz = Node(
        package="rviz2",
        executable="rviz2", 
        arguments=['-d', rviz_config_path]
    )

    # ------------------- ADD NODES TO LAUNCH -------------------
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_gzserver)
    ld.add_action(
        TimerAction(period=5.0, actions=[start_gzclient])
    )  # Wait 5 seconds, then open GUI
    ld.add_action(spawn_entity)
    ld.add_action(start_rviz)

    # ------------------- RETURN LAUNCH DESCRIPTION -------------------
    return ld
