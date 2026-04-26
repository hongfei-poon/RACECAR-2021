# RACECAR-2021

This repository contains the solution for the **16th National Smart Car Competition (全国大学生智能汽车竞赛)**, which won **1st Prize - 4th Place** from South China University of Technology (SCUT).

> **Note:** This is not the final competition version but an earlier development version of our autonomous racing car system.

## Overview

This project implements a complete autonomous navigation stack for intelligent racing vehicles, featuring robust localization, path planning, and control algorithms.

## Key Features

### Localization & State Estimation

- **Extended Kalman Filter (EKF):** Fuses wheel odometry (ODOM) data for improved state estimation
- **Adaptive Monte Carlo Localization (AMCL):** Integrates ODOM and LiDAR measurements using the **likelihood-field model** for robust pose estimation against a known map

### Path Planning

- **Global Planning:** A* algorithm for optimal path generation in static environments
- **Local Planning:** TEB (Timed Elastic Band) local trajectory planner for dynamic obstacle avoidance and smooth trajectory following

### Vehicle Control

- **Sliding-Mode Controller:** Incorporates vehicle dynamics for precise trajectory tracking and robust performance under varying conditions

### Alternative Odometry

- **RF2O (Robust Fast Lidar Odometry):** When motor encoders are inaccurate or unreliable, the system can compute odometry directly from LiDAR scan data using the RF2O algorithm, ensuring continuous localization capability

## System Architecture

```
LiDAR ──┬──→ AMCL (Localization)
        │       ↑
ODOM ───┴───────┘
        ↓
    EKF Fusion
        ↓
   A* Global Planner
        ↓
   TEB Local Planner
        ↓
   Sliding-Mode Controller
        ↓
   Vehicle Actuators
```

## Hardware Platform

- **Platform:** ROS-based autonomous racing vehicle
- **Sensors:** 2D LiDAR, wheel encoders, IMU
- **Compute:** Embedded computing platform (Jetson/Intel NUC)

## Dependencies

- ROS (Robot Operating System)
- `robot_localization` (EKF package)
- `amcl` (Adaptive Monte Carlo Localization)
- `global_planner` / `navfn` (A* implementation)
- `teb_local_planner` (TEB planner)
- `rf2o_laser_odometry` (RF2O algorithm)

## Quick Start

```bash
# Build the workspace
catkin_make

# Source the environment
source devel/setup.bash

# Launch the navigation stack
roslaunch racecar_navigation racecar.launch
```

## Team

- **Institution:** South China University of Technology (华南理工大学)
- **Competition:** 16th National Smart Car Competition
- **Achievement:** 1st Prize - 4th Place

## License

This project is for educational and research purposes.
