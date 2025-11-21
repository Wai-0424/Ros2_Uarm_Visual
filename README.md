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

### 2025-11-21（補充紀錄，中文）

- 新增 / 修改（我在本地執行並驗證）：
    - 新增 `scripts/start_sim.sh` 與 `scripts/start_write.sh`，用來在正確的 ROS2 環境下背景啟動模擬與寫入節點，並將日誌與 PID 寫至 `/tmp` 以方便檢查。
    - 修正 `scripts` 在 `set -u`（嚴格模式）下 sourcing `/opt/ros/humble/setup.bash` 會失敗的問題（暫時關閉 nounset 再恢復），避免啟動時出現未綁定變數導致的 exit code 1。
    - 修改 `swiftpro/src/swiftpro_write_node_ros2.cpp`：當參數 `enable_writes` 為 `false`（模擬模式）時，不再嘗試開啟 serial port，避免在沒有實體機時拋出例外導致程式終止；此外，write node 會把「命令狀態」publish 到 `SwiftproCommand`，供模擬器 mirror 使用。
    - 新增一個簡易模擬器 `sim_publisher.py`（已安裝到 `install/.../sim_publisher.py`），會訂閱 `SwiftproCommand` 並把模擬狀態發布到 `SwiftproState_topic`，可以在 RViz 中看到與寫入節點對應的模擬結果。
    - 在本地完成一次端到端軟體驗證（未連實體）：啟動 sim 與 write（`enable_writes:=false`），發一筆 `/position_write_topic`，確認 write node publish `/SwiftproCommand`，sim 收到後將狀態更新到 `/SwiftproState_topic`（x=120,y=0,z=100 範例）。相關日誌會寫到 `/tmp/swiftpro_sim.out` 與 `/tmp/swiftpro_write_node.log`。

- 使用與注意事項（中文）：
    - 若要切換到實機測試，請先確保機械手臂的序列埠（例如 `/dev/ttyACM0`）存在且權限正確（可用 `sudo usermod -a -G dialout $USER` 或建立 udev 規則）。
    - 預設啟動腳本會把 `enable_writes` 設為 `false`（安全），要寫入實機請以 `--ros-args -p enable_writes:=true` 明確啟動寫入節點。建議初次連實機時先下小幅度指令並準備急停。
    - 若需要，我可以幫你：自動化 udev 規則、把 `start_write.sh` 改為可接參數、或將變更 commit 並推到 GitHub（下方已執行）。

以上修改已於本地 build 並驗證。如需我把變更推送到遠端 GitHub repo，請確認你授權本機有權限推送（或告訴我 remote URL 與是否要我立即推）。
