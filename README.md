# Rover

This repository contains the ROS 2 software stack for the Rover, a mobile robot platform.

## Overview

The Rover project is a comprehensive robotics software suite built on ROS 2. It provides functionalities for robot control, perception, navigation, and high-level applications. The system is designed to be extensible, allowing developers to easily add new capabilities and applications.

## Features

- **Mobile Platform Control:** Core drivers and kinematic calculations for robot movement.
- **Perception:** Integrates a variety of sensors, including:
    - 2D LiDAR for SLAM and navigation.
    - Depth Camera for perception and vision-based applications.
- **SLAM and Navigation:** Includes packages for simultaneous localization and mapping (SLAM) and autonomous navigation.
- **Teleoperation:** Control the robot using a joystick.
- **Web Interface:**
    - Streams live video from the camera to a web browser.
    - Provides a WebSocket connection (`rosbridge`) for remote monitoring and control.
- **Application Framework:** A system for running and managing high-level applications, such as:
    - **Augmented Reality (AR):** Overlays 3D models onto the real world using AprilTag markers.
    - **Line Following:** A classic robotics application for following a line on the ground.
    - **Object Tracking:** Tracks objects detected by the camera.

## System Architecture

The software is organized into several ROS 2 packages:

- `bringup`: Contains the main launch files to start the robot system.
- `driver`: Low-level drivers for controlling the robot's motors and hardware.
- `peripherals`: Drivers and nodes for external sensors like LiDAR, depth camera, and joystick.
- `navigation`: Implements the ROS 2 navigation stack (Nav2) for autonomous path planning and obstacle avoidance.
- `slam`: Contains launch and configuration files for SLAM algorithms.
- `app`: Houses various high-level applications that run on the robot.
- `interfaces`: Defines custom ROS 2 messages and services for communication between packages.

## Prerequisites

Before you begin, ensure you have the following installed:

- **ROS 2:** This project is built on ROS 2.
- **Colcon:** The standard ROS 2 build tool.
- **Python Dependencies:** The project requires several Python packages. You may need to install them via pip.
  ```bash
  pip install apriltag opencv-python scipy
  ```
  It is recommended to check the `package.xml` files within each package for a full list of dependencies.

## Build Instructions

1.  **Clone the repository** into your ROS 2 workspace's `src` directory.
2.  **Navigate to the root of your workspace.**
3.  **Build the project** using `colcon`:

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

## Running the Robot

1.  **Source the workspace:**
    After a successful build, source your workspace's setup file from the root of the workspace:
    ```bash
    source install/setup.bash
    ```

2.  **Launch the main bringup file:**
    This will start all the core components of the robot.
    ```bash
    ros2 launch bringup bringup.launch.py
    ```

    **Important Note:** The `bringup.launch.py` file may contain hardcoded paths (e.g., `/home/ubuntu/ros2_ws/...`) and rely on an environment variable `need_compile`. This can cause issues if your setup is different. It is highly recommended to modify this launch file to use the `get_package_share_directory` function consistently for locating package resources.

## Using Applications

The applications in the `app` package are typically started and stopped via ROS 2 services. You can discover them using `ros2 service list`.

For example, to control the Augmented Reality application:

- **Start the AR App:**
  ```bash
  ros2 service call /ar_app/enter std_srvs/srv/Trigger
  ```

- **Set the AR Model:**
  The following models are available: `bicycle`, `fox`, `chair`, `cow`, `wolf`, `rectangle`.
  ```bash
  ros2 service call /ar_app/set_model interfaces/srv/SetString "{data: 'bicycle'}"
  ```

- **Stop the AR App:**
  ```bash
  ros2 service call /ar_app/exit std_srvs/srv/Trigger
  ```
