# ROS2 Automation Project: Autonomous Driving

## Project Overview
This semester project implements a **ROS2-based robotic system** using a Raspberry Pi, demonstrating key capabilities in robot control, mapping, and autonomous navigation. The project showcases the integration of various ROS2 nodes for keyboard control, environment mapping, and path planning.

---

## Features
- **Keyboard-controlled robot movement** using ROS2 Twist messages
- **Real-time environment mapping** with laser scan data and occupancy grid
- **Autonomous path planning** based on initial pose, goal pose, and map data
- Modular ROS2 node architecture for each project component
- Integration with Raspberry Pi for embedded robotics applications

---

## Table of Contents
1. [Introduction](#introduction)
2. [Project Components](#project-components)
3. [Technical Stack](#technical-stack)
4. [How It Works](#how-it-works)
5. [Future Improvements](#future-improvements)
6. [Acknowledgments](#acknowledgments)

---

## Introduction
Autonomous robot navigation requires a combination of manual control, environment perception, and intelligent decision-making. This project demonstrates these capabilities using ROS2, implementing nodes for keyboard control, laser-based mapping, and path planning algorithms.

---

## Project Components

### Task 1: Keyboard Control
- **`keyboard_control_node`**: Translates keyboard inputs into robot movement commands
- Publishes `Twist` messages for robot motion
- Supports directional and combined movements

### Task 2: Environment Mapping
- **`mapping_node`**: Processes laser scan data to create an occupancy grid map
- Utilizes Bresenham's algorithm for obstacle mapping
- Handles coordinate frame transformations

### Task 3: Path Planning
- **`path_planning_node`**: Generates optimal paths based on start and goal poses
- Integrates with the occupancy grid map for obstacle avoidance
- Publishes planned paths for robot navigation

---

## Technical Stack
- **ROS2 (Robot Operating System)**
- **Raspberry Pi** for embedded computing
- **Python** for node implementation
- **Laser scanning sensors** for environment perception
- **Twist message communication** for robot control

---

## How It Works

### Keyboard Control:
- Listens for keyboard inputs
- Translates inputs to `geometry_msgs/Twist` messages
- Publishes messages to `/cmd_vel` topic for robot movement

### Mapping:
- Subscribes to `/scan` topic for laser data
- Processes scans using Bresenham's line algorithm
- Updates and publishes occupancy grid to `/map` topic

### Path Planning:
- Subscribes to `/initialpose`, `/goal_pose`, and `/map` topics
- Implements path planning algorithm (e.g., A*, RRT)
- Publishes planned path to `/planned_path` topic

---

## Future Improvements
- Implement SLAM (Simultaneous Localization and Mapping) for improved mapping
- Integrate computer vision for object recognition
- Develop multi-robot coordination capabilities
- Enhance path planning with machine learning algorithms

---

## Acknowledgments

Special thanks to the professor and my team member for guiding and following up for this project.
