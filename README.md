# Swift Pro Robot Arm Control with ROS 2 & MoveIt!

![ROS Version](https://img.shields.io/badge/ROS-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Linux-green)

This project provides a complete ROS 2 (Humble) framework for controlling the uFactory Swift Pro robotic arm using the MoveIt! 2 motion planning platform.

## 🚨 當前狀態與問題

### 已解決的問題
- ✅ 添加了碰撞幾何到URDF文件 (`swiftpro/urdf/pro_model.xacro`)
- ✅ 修改了RViz配置以使用基本RobotModel顯示
- ✅ 設置了靜態變換發布器從world到Base坐標系
- ✅ 系統可以啟動但有各種警告

### 當前問題
- ❌ **主要問題**: RViz只顯示機器人軸線而不顯示完整的3D模型
- ❌ **joint_states主題**: 無法確認joint_states主題是否正確發布
- ❌ **共享庫錯誤**: `libswiftpro__rosidl_typesupport_cpp.so` 無法加載
- ❌ **模塊導入錯誤**: `ModuleNotFoundError: No module named 'swiftpro'`
- ❌ **重複節點**: combined.launch.py啟動重複的節點導致衝突

### 我的推測
1. **URDF網格路徑問題**: STL文件路徑可能不正確或文件不存在
2. **MoveIt插件兼容性**: ROS2的MoveIt RViz插件可能與當前版本不兼容
3. **Launch文件設計問題**: combined.launch.py同時啟動真實機器人和模擬節點導致衝突
4. **包安裝問題**: colcon build可能沒有正確安裝共享庫或Python模塊
5. **TF樹問題**: 變換樹可能沒有正確建立

## 詳細修改記錄

### 1. URDF碰撞幾何添加 (2025-11-22)
**文件**: `swiftpro/urdf/pro_model.xacro`

為所有10個鏈接添加了碰撞幾何：
- Base: 盒子 (0.1x0.1x0.08)
- Link1: 圓柱體 (半徑0.02, 長度0.08)
- Link2: 圓柱體 (半徑0.015, 長度0.15)
- Link3: 圓柱體 (半徑0.02, 長度0.2)
- Link4-Link7/Link9: 盒子 (0.05x0.05x0.05 / 0.04x0.04x0.04 / 0.03x0.03x0.03)
- Link8/Gripper: 盒子 (0.05x0.03x0.05)

**原因**: MoveIt需要碰撞幾何進行運動規劃

### 2. RViz配置修改 (2025-11-22)
**文件**: `swiftpro/rviz/swiftpro_default.rviz`

從MoveIt MotionPlanning插件改為基本RobotModel顯示：
- 顯示類型: `rviz_default_plugins/RobotModel`
- 機器人描述主題: `/robot_description`
- 固定坐標系: `world`
- TF前綴: 空

**原因**: MoveIt插件在ROS2中導致段錯誤

### 3. Launch文件修改 (2025-11-22)
**文件**: `swiftpro/launch/real_robot.launch.py`

添加了靜態變換發布器：
```xml
<node pkg="tf2_ros" exec="static_transform_publisher" name="static_transform_publisher" args="0 0 0 0 0 0 world Base"/>
```

**原因**: 建立world到Base的坐標變換

### 4. 系統修復與清理 (2025-11-22)
**文件**: `swiftpro/launch/sim.launch.py`, `README.md`

- **修復**: 移除了 `sim.launch.py` 中的硬編碼路徑，改為使用 `FindPackageShare` 動態查找。
- **修復**: 修正了模型加載邏輯，預設使用 `pro_model.xacro`。
- **清理**: 移除了未使用的 `swift_moveit_config` 資料夾，簡化專案結構。

**原因**: 解決無法在其他電腦上運行模擬的問題，並保持專案整潔。

## 系統架構

### 數據流
```
sim_publisher.py → SwiftproState_topic → swiftpro_rviz_node → joint_states → robot_state_publisher → TF樹
```

### 關鍵組件
- **sim_publisher.py**: 發布模擬的SwiftproState消息
- **swiftpro_rviz_node**: 將位置轉換為關節角度並發布joint_states
- **robot_state_publisher**: 從joint_states和URDF生成TF變換
- **RViz**: 可視化機器人狀態

## 安裝和構建

### 系統需求
- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- MoveIt! 2
- Colcon

### 安裝步驟

1. **安裝ROS 2和依賴**:
```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

2. **創建工作區**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

3. **克隆倉庫**:
```bash
git clone https://github.com/Wai-0424/Ros2_Uarm_Visual.git src
```

4. **構建**:
```bash
colcon build --merge-install
```

## 使用方法

### 啟動模擬
```bash
source install/setup.bash
ros2 launch swiftpro sim.launch.py
```

### 啟動真實機器人 (需要硬件)
```bash
source install/setup.bash
ros2 launch swiftpro real_robot.launch.py
```

### 啟動組合模式 (當前有問題)
```bash
source install/setup.bash
ros2 launch swiftpro combined.launch.py
```

## 故障排除

### 常見問題

1. **RViz只顯示軸線**
   - 檢查joint_states主題: `ros2 topic echo /joint_states`
   - 檢查TF樹: `ros2 run tf2_tools view_frames.py`
   - 確認URDF網格文件存在

2. **共享庫錯誤**
   - 重新構建: `colcon build --packages-select swiftpro --merge-install`
   - 檢查AMENT_PREFIX_PATH設置

3. **模塊導入錯誤**
   - 確保Python路徑正確設置
   - 檢查包安裝是否完整

### 調試命令

```bash
# 檢查主題
ros2 topic list
ros2 topic echo /joint_states --once

# 檢查節點
ros2 node list

# 檢查TF
ros2 run tf2_tools view_frames.py

# 檢查包
ros2 pkg list | grep swiftpro
```

## 開發計劃

### 短期目標
1. 修復joint_states主題發布問題
2. 解決共享庫加載問題
3. 修復模塊導入錯誤
4. 實現完整的機器人3D可視化

### 長期目標
1. 優化launch文件避免重複節點
2. 添加真實硬件通信支持
3. 實現運動規劃和控制
4. 添加測試和文檔

## 貢獻

歡迎提交問題和拉取請求！

## 許可證

MIT License

---

**最後更新**: 2025-11-22
**ROS版本**: Humble Hawksbill
**MoveIt版本**: 2.5.5

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
    - `pro_moveit_config`：MoveIt! 的設定包，用於 Swift Pro 機械手臂。

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
