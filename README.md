# ROS 2 Final Project: Navigation & Data Replay

## Project Overview
This project implements a complete ROS 2 workflow integrating robot simulation, autonomous navigation, data logging, and cross-simulator replay. A custom differential drive robot navigates a Gazebo world, records its trajectory to a CSV file, and effectively "teleports" that path to a TurtleSim actor for visualization.

## Project Architecture
1. **Simulation:** A custom URDF robot spawns in Gazebo.
2. **Navigation:** The robot is driven via teleoperation (or Nav2).
3. **Data Recording:** The `path_recorder` node subscribes to Odometry and logs pose data to `path.csv`.
4. **Data Replay:** The `turtlesim_replay` node reads the CSV and reproduces the exact path in Turtlesim.

## Folder Structure
```text
~/robot_ws/src/
├── my_robot_description    # URDF, Meshes, Materials
├── my_robot_bringup        # Launch files, World files
├── path_recorder           # Node to record Odom -> CSV
└── turtlesim_replay        # Node to replay CSV -> Turtlesim
