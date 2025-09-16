# ROS2 Framework for Mark II Robotic Arm

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/index.html)
[![MoveIt](https://img.shields.io/badge/MoveIt-2-orange)](https://moveit.ros.org/)
[![Isaac ROS](https://img.shields.io/badge/Isaac%20ROS-NVIDIA-green)](https://nvidia.github.io/isaac_ros/index.html)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## Project Overview

This repository contains a complete ROS2-based framework for controlling the Mark II custom robotic arm. The arm is designed for affordable, no-expert robotics with a focus on AI experimentation, such as visual SLAM, pose estimation, and grasping tasks. It integrates ros2_control for hardware interfacing, MoveIt for motion planning and execution, and is set up in an NVIDIA Isaac ROS Docker container for GPU-accelerated AI capabilities.

The codebase includes custom hardware drivers, control plugins, MoveIt configurations, and launch files. It supports both real hardware (Feetech STS3215 servos) and simulation modes, with RViz for visualization and interactive control.

### Key Features
- **Hardware Control**: Custom C++ driver and ros2_control plugin for servo communication and joint state publishing.
- **Motion Planning**: MoveIt with OMPL planner (robust for timestamps/velocities) and KDL kinematics.
- **Gripper Support**: Prismatic joint with gear-rack mechanism for opposite-moving clamps.
- **AI Readiness**: Built on Isaac ROS for integration with GEMs (e.g., usb_cam + visual_slam for pose, foundationpose for grasping).
- **Simulation/Testing**: Toggle real/sim with launch arg; manual torque mode for calibration.
- **Reliability**: Persistent Docker setup, Git backups, real-time kernel recommendations.

## Goals

### Main Goal
Create a stable, low-latency robotic arm system for AI-driven tasks, enabling sim-to-real transfer in robotics research. Focus on usability (RViz dragging for execution) and affordability.

### Sub-Goals
- Achieve end-to-end control: Planning, execution, and visualization in RViz/Foxglove.
- Fix MoveIt issues: Kinematics loading, interactive markers, trajectory validation (tolerances, timestamps).
- Ensure Docker persistence: Auto-restart on reboots, no auto-deletion.
- Backup codebase: GitHub syncing for recovery.
- Prepare for AI: Integrate cameras/SLAM for pose estimation, RL fine-tuning (e.g., grasping in Isaac Lab).

## System Requirements

- **Host OS**: Ubuntu 22.04 LTS (NVIDIA-compatible for Isaac ROS).
- **Hardware**: NVIDIA GPU (for Isaac GEMs); serial port (/dev/ttyUSB0) for arm servos.
- **ROS Version**: Humble (source /opt/ros/humble/setup.bash; container base may be Jazzy—manual fixes for compatibility).
- **Dependencies**: rclcpp, hardware_interface, pluginlib, controller_manager, moveit_ros_move_group, rviz2, foxglove_bridge, colcon, rosdep.
- **Container**: NVIDIA Isaac ROS Docker (built via run_dev.sh; mounts host workspace).
- **Arm Hardware**: Mark II arm with 7 DOF (6 revolute + 1 prismatic gripper), Feetech STS3215 servos (1Mbps baud, IDs 1-13).

No real-time kernel by default (add for stability: `apt install linux-lowlatency-hwe-22.04`).
