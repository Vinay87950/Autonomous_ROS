# ROS2 Automation Project: Autonomous ROS2 Exploration 

## Project Overview

This semester project focuses on developing a ROS2-based robotic system using a Raspberry Pi, demonstrating key capabilities in robot control, mapping, and autonomous navigation.

## Project Stages

### Task 1: Keyboard Control

#### Objective
Implement a ROS2 node for manual robot control using keyboard inputs.

#### Key Features
- Real-time robot movement control
- Directional commands (forward, backward, left, right)
- Smooth and intuitive keyboard interface

#### Keyboard Controls
- `w`: Move forward
- `s`: Move backward
- `a`: Turn left
- `d`: Turn right
- `q`: Forward-left diagonal movement
- `e`: Forward-right diagonal movement
- `z`: Program termination

### Task 2: Environment Mapping

#### Objective
Create a dynamic occupancy grid map using laser scan data.

#### Key Components
- Laser scan data processing
- Occupancy grid map generation
- Obstacle detection and mapping
- Coordinate transformation handling

#### Mapping Techniques
- Bresenham's line algorithm
- Laser scan data interpretation
- Real-time map updates

### Task 3: Autonomous Path Planning

#### Objective
Develop an intelligent path planning algorithm for autonomous navigation.

#### Key Features
- Dynamic path generation
- Obstacle avoidance
- Efficient route calculation

#### Navigation Components
- Initial pose subscription
- Goal pose identification
- Occupancy grid map integration
- Path optimization algorithms

## Technical Stack

- ROS2 (Robot Operating System)
- Raspberry Pi
- Python
- Laser scanning sensors
- Twist message communication

## Project Challenges

1. Implementing real-time robot control
2. Accurate environment mapping
3. Efficient path planning algorithms
4. Sensor data integration
5. Coordinate frame transformations

## Future Improvements

- Advanced machine learning path planning
- Enhanced obstacle detection
- Multi-robot coordination
- Improved mapping resolution

## Acknowledgments

Special thanks to the professor and course instructors for guiding this project.
