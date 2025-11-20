# Swift Pro Robot Arm Control with ROS 2 & MoveIt!

![ROS Version](https://img.shields.io/badge/ROS-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Linux-green)

This project provides a complete ROS 2 (Humble) framework for controlling the uFactory Swift Pro robotic arm using the MoveIt! 2 motion planning platform.

## Overview

This workspace is designed to provide everything needed to get started with the Swift Pro arm in ROS 2, from basic hardware communication to advanced motion planning. It allows for both simulated and real-world control of the robot.

## Workspace Structure

The project is organized into three main ROS 2 packages:

-   `swiftpro`: The core hardware driver package. It contains the nodes for communicating with the robot's controller, publishing joint states, and handling custom message types.
-   `pro_moveit_config`: A MoveIt! 2 configuration package for the "Pro" version of the arm. It includes all necessary configuration files for motion planning, kinematics, and visualization.
-   `swift_moveit_config`: A similar MoveIt! 2 configuration package, but tailored for the standard "Swift" version of the arm.

## System Requirements

-   Ubuntu 22.04
-   ROS 2 Humble Hawksbill
-   MoveIt! 2 for ROS 2 Humble
-   Colcon (ROS 2 build tool)

## Installation and Build

1.  **Install ROS 2 and Dependencies:**
    First, ensure you have a working installation of ROS 2 Humble. Then, install the required MoveIt! and other packages.

    ```bash
    sudo apt update && sudo apt install -y \
      ros-humble-desktop \
      ros-humble-moveit \
      ros-humble-joint-state-publisher-gui \
      ros-humble-robot-state-publisher \
      ros-humble-xacro
    ```

2.  **Create a Colcon Workspace:**
    Create a new workspace directory to house the project.

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

3.  **Clone the Repository:**
    Clone this repository into your workspace's `src` directory.

    ```bash
    # Replace <YOUR_REPOSITORY_URL> with the actual URL of your GitHub repository
    git clone <YOUR_REPOSITORY_URL> src/swiftpro_ros2
    ```

4.  **Build the Workspace:**
    From the root of your workspace (`~/ros2_ws`), build all the packages using `colcon`.

    ```bash
    colcon build
    ```

5.  **Source the Workspace:**
    Before running any nodes, you need to source the workspace's setup file in every new terminal.

    ```bash
    source install/setup.bash
    ```
    *(Tip: Add this command to your `~/.bashrc` file to source it automatically.)*

## Usage

### 1. Grant Serial Port Permission

To allow ROS to communicate with the robot via its USB connection, you may need to grant permissions to the serial port.

```bash
# Find your device port, it's usually /dev/ttyACM0
ls /dev/tty*

# Grant read/write permissions (example for /dev/ttyACM0)
sudo chmod 666 /dev/ttyACM0
```

### 2. Launch the Demo (Simulation)

You can run a full simulation with MoveIt! and RViz to test motion planning without a physical robot connected.

-   **For the Swift Pro arm:**
    ```bash
    ros2 launch pro_moveit_config demo.launch.py
    ```
-   **For the standard Swift arm:**
    ```bash
    ros2 launch swift_moveit_config demo.launch.py
    ```

Inside RViz, you can drag the interactive marker at the end of the arm to set a goal and then click "Plan and Execute".

## Changelog (recent)

### 2025-11-21 — Local ROS2 port and simulation additions
- Renamed message (.msg) files to PascalCase and fixed `package.xml` to be ROS2-compliant.
- Ported original ROS1 nodes to ROS2: read, write, moveit and rviz nodes (C++).
- Added a simple Python `sim_publisher.py` that publishes `SwiftproState` for visualization/testing.
- Created `launch/sim.launch.py` to start the sim publisher, `swiftpro_rviz_node_ros2`, and `robot_state_publisher` + RViz config.
- Added `rviz/swiftpro_default.rviz` (default RViz view to show robot model and joint_states).
- Temporarily added a compile-time `include/serial/serial.h` stub to allow building without the external serial library.

Notes:
- This commit aims to make the package build and run in a ROS2 Humble environment for simulation and visualization. Replace the serial stub with a proper serial library (e.g. wjwwood/serial) before connecting real hardware.
