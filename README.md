# Autonomous Robot Control System

This repository contains the embedded system logic for an autonomous robot designed to navigate through a course autonomously. It includes routines for IR signal following, edge detection, obstacle avoidance using sonar, and motor control for steering and manoeuvring.

## Features

- **Speed Regulation**: Controls the robot's speed to ensure stable navigation.
- **IR Navigation**: Follows an IR beacon for direction.
- **Edge Detection**: Detects edges to prevent the robot from veering off the course.
- **Sonar Obstacle Avoidance**: Uses sonar to detect and avoid obstacles.
- **Steering Control**: Manages the robot's steering mechanism for smooth transitions and turns using PID.
- **Zipline Interaction**: Engages with a zipline for course completion (and style).

## Hardware Requirements

- Arduino-compatible board (e.g., STM32 Bluepill)
- Servo motors
- IR sensors
- Sonar modules
- Reflective sensors for edge detection
- Servo-controlled mechanisms for additional interactions (e.g., zipline)

## Software Setup

1. Ensure you have the Arduino IDE or compatible software installed.
2. Install any necessary libraries for servo control and sensor input.
3. Configure pin definitions in the code to match your hardware setup.

## Installation

To install the code on your robot's microcontroller:

1. Clone the repository or download the source code.
2. Open the source code in your Arduino IDE or compatible software.
3. Connect your microcontroller to your computer.
4. Upload the code to the microcontroller via the IDE / STM32.

## Usage

Once powered, the robot will start in stage 1, looking for the IR beacon. As it navigates through the course, it will 
automatically transition through various stages, engaging with the environment using pre-defined behaviours like following 
edges, avoiding obstacles, and interacting with a zipline.

## Configuration

- **Speed Limit**: Adjust the `divisor` variable to change the robot's speed.
- **Sensor Thresholds**: Calibrate `IRThreshold`, `EdgeFollowThreshold`, and sonar thresholds to match environmental conditions.

## Acknowledgments

- Thank you to Michael Khoo, Steven Xu and Jake Lee for their contributions
- Special thanks to Rudransh Kumar (Instructor)

## Contact

For questions please contact amjadclnyaghi@gmail.com
