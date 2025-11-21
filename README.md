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
    # Swift Pro 機械手臂（ROS 2 + MoveIt!）

    本專案為 uFactory Swift Pro 機械手臂在 ROS 2（Humble）環境下的整合範例，包含驅動、模擬、MoveIt! 設定與 RViz 視覺化，方便開發者在模擬或實機上進行運動規劃與控制。

    ## 主要內容

    - `swiftpro`：核心驅動套件，包含與機械手臂通訊的 node、custom msg 與相關工具。
    - `pro_moveit_config` / `swift_moveit_config`：MoveIt! 的設定包，用於不同型號的機械手臂（Pro 與 Swift）。

    ## 系統需求

    - Ubuntu 22.04
    - ROS 2 Humble Hawksbill
    - MoveIt! 2 (Humble 版本)
    - colcon（用於建立工作區）

    ## 快速安裝與建立

    1. 安裝 ROS 2 與必要套件（範例）：

    ```bash
    sudo apt update && sudo apt install -y \
      ros-humble-desktop \
      ros-humble-moveit \
      ros-humble-joint-state-publisher-gui \
      ros-humble-robot-state-publisher \
      ros-humble-xacro
    ```

    2. 建立 colcon 工作區並 clone 本 repo：

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    git clone https://github.com/Wai-0424/Ros2_Uarm_Visual.git src/Ros2_Uarm_Visual
    cd ..
    colcon build
    ```

    3. 每次打開新的 terminal 時請 source 工作區環境：

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```

    ## 使用說明（模擬與實機）

    1) 模擬（建議初學者先從模擬開始）

    - 已內建一個簡單模擬節點 `sim_publisher.py`，在本地能根據 `SwiftproCommand` 更新並發布模擬狀態到 `SwiftproState_topic`，可以在 RViz 中觀察。
    - 範例：啟動模擬（或使用 repo 中的 `launch`）後，在另一個 terminal 發送測試位置：

    ```bash
    # 發一個測試位置
    ros2 topic pub --once /position_write_topic swiftpro/msg/Position "{x: 120.0, y: 0.0, z: 100.0}"

    # 檢查寫入節點是否 publish 出 SwiftproCommand
    ros2 topic echo /SwiftproCommand swiftpro/msg/SwiftproState --once

    # 檢查模擬器是否把狀態更新到 SwiftproState_topic
    ros2 topic echo /SwiftproState_topic swiftpro/msg/SwiftproState --once
    ```

    2) 實機（上機前請務必注意安全）

    - 確認序列埠：通常是 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。使用 `ls /dev/ttyACM* /dev/ttyUSB*` 找出裝置。
    - 權限問題：若使用者沒有權限，請加入 `dialout` 群組或建立 udev 規則：

    ```bash
    sudo usermod -a -G dialout $USER
    # 登出/登入或重新啟動後生效，或使用 udev 規則避免每次手動 chmod
    ```

    - 啟動寫入節點時請帶上參數 `enable_writes:=true` 才會真正開 serial 並下達 G-code 到硬體；預設 `enable_writes` 為 `false`（安全模式），不會觸發硬體動作。

    範例（安全地啟動並測試小幅移動）：

    ```bash
    # 先 source
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    # 啟動寫入節點（實機）
    nohup ./install/swiftpro/lib/swiftpro/swiftpro_write_node_ros2 --ros-args -p enable_writes:=true > /tmp/swiftpro_write_node.log 2>&1 &
    echo $! > /tmp/swiftpro_write_node.pid

    # 只下小幅度動作做驗證
    ros2 topic pub --once /position_write_topic swiftpro/msg/Position "{x: 5.0, y: 0.0, z: 100.0}"
    ```

    請務必在第一次通電或第一次啟動實機時準備好硬體急停（E-stop）與觀察點，並從小幅度運動開始逐步驗證。

    ## 今天的修改與驗證（2025-11-21，中文紀錄）

    - 新增並驗證：`scripts/start_sim.sh`、`scripts/start_write.sh`（用於在正確的 ROS 環境下背景啟動模擬與寫入節點，並把日誌/PID 寫到 `/tmp`）。
    - 修正：當 shell 使用 `set -u` 時，sourcing `/opt/ros/humble/setup.bash` 會因為未綁定變數導致失敗；現已暫時關閉 nounset（`set +u`）再 source，然後恢復 `set -u`，避免啟動錯誤。
    - 修改：`swiftpro/src/swiftpro_write_node_ros2.cpp`，當 `enable_writes=false`（模擬模式）時跳過 serial open，且 write node 會把「命令狀態」發布到 `/SwiftproCommand` 以供模擬器 mirror。
    - 新增：`sim_publisher.py`（安裝至 `install/.../sim_publisher.py`），會訂閱 `/SwiftproCommand` 並發佈 `/SwiftproState_topic`，可在 RViz 中觀察鏡像結果。
    - 驗證：本地在沒有實體硬體下完成 end-to-end 測試：啟動 sim 與 write（`enable_writes=false`），由 `/position_write_topic` 發送命令，確認 `SwiftproCommand` 與 `SwiftproState_topic` 都反映相同命令值。

    ## 其他注意事項與後續建議

    - serial library：目前專案內含一份簡易的 `serial.h` stub 以方便本地 build；若要在實機上穩定運作，建議使用成熟的 serial library（例如 wjwwood/serial），並在 CMake 中正確連結。
    - 若需要，我可以幫你：
      - 將 `start_write.sh` 支援參數化（可選 `--enable-writes true/false`）並提交到 repo；
      - 產生範例 udev 規則以免每次手動 chmod；
      - 或在你插上手臂後協助遠端執行小幅度測試。

    ---

    如果你同意，我現在可以把這個中文 README commit 並推到 `main`（或你指定的分支）。
