# Swift Pro 機械手臂控制專案 (ROS 2 Humble)

![ROS Version](https://img.shields.io/badge/ROS-Humble-blue)
![Platform](https://img.shields.io/badge/Platform-Linux-green)

本專案提供了一個完整的 ROS 2 (Humble) 框架，用於控制 uFactory Swift Pro 機械手臂，並整合了 MoveIt! 2 運動規劃平台與 RViz 視覺化。

## 🚀 專案功能

- **硬體驅動**：透過 USB 序列埠與 Swift Pro 機械手臂通訊（G-code）。
- **運動規劃**：整合 MoveIt! 2，支援路徑規劃與避障。
- **視覺化**：在 RViz 中即時顯示機械手臂的 3D 模型與運動狀態。
- **模擬模式**：無需實體手臂即可進行開發與測試。

## 🛠️ 系統需求

- **作業系統**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS 2 Humble Hawksbill
- **核心套件**: MoveIt! 2, Colcon

## 📦 安裝步驟

### 1. 安裝 ROS 2 與相依套件

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### 2. 建立工作空間並下載專案

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Wai-0424/Ros2_Uarm_Visual.git
```

### 3. 編譯專案

```bash
cd ~/ros2_ws
colcon build --packages-select swiftpro pro_moveit_config
```

### 4. 設定環境變數

每次開啟新的終端機時，請執行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

*(建議將上述指令加入 `~/.bashrc` 以自動執行)*

## 🎮 使用說明

### 1. 實體機器人控制 (Real Robot)

這是本專案的主要模式，用於控制真實的 Swift Pro 機械手臂。

**前置準備：**
1. 將機械手臂透過 USB 連接至電腦。
2. 確認序列埠權限（通常是 `/dev/ttyACM0`）：
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

**啟動指令：**
```bash
ros2 launch swiftpro real_robot.launch.py
```

**功能：**
- 啟動 `swiftpro_write_node`：負責發送控制指令。
- 啟動 `swiftpro_read_node`：負責讀取手臂即時位置。
- 啟動 `swiftpro_rviz_node`：將位置轉換為關節角度並發布 TF。
- 啟動 MoveIt! 與 RViz：進行路徑規劃與視覺化。

### 2. 模擬模式 (Simulation)

若沒有實體手臂，可使用此模式進行測試。

```bash
ros2 launch swiftpro sim.launch.py
```

## 📝 最近更新與修復紀錄 (2025-11-24)

本次更新主要解決了 RViz 模型與實體手臂不同步的問題，並修復了多個啟動錯誤。

### ✅ 已解決的問題

1.  **RViz 同步問題**：
    - **症狀**：實體手臂移動，但 RViz 中的模型靜止不動。
    - **原因**：`real_robot.launch.py` 中缺少了負責讀取手臂狀態的 `swiftpro_read_node`。
    - **修復**：將 `swiftpro_read_node_ros2` 加入啟動流程，確保 `/SwiftproState_topic` 能接收到真實數據。

2.  **啟動檔錯誤 (Package not found)**：
    - **症狀**：執行 `ros2 launch` 時報錯 `Package 'swiftpro' not found`。
    - **原因**：環境變數 `AMENT_PREFIX_PATH` 未正確更新，且 `install/setup.bash` 被 `.gitignore` 阻擋導致無法確認內容。
    - **修復**：清理了 `build/` 與 `install/` 目錄並重新編譯，確認環境變數設定正確。

3.  **序列埠衝突 (Serial Port Conflict)**：
    - **症狀**：`read_node` 與 `write_node` 同時嘗試開啟 `/dev/ttyACM0` 導致崩潰。
    - **修復**：確認了此硬體限制。目前的解決方案是確保節點正確處理序列埠資源，或在測試時單獨運行。

4.  **RViz 顯示問題**：
    - **修復**：更新了 RViz 設定檔 (`moveit.rviz`)，將舊版 ROS 1 類別名稱 (`rviz/Displays`) 更新為 ROS 2 格式 (`rviz_common/Displays`)，解決了載入錯誤。

### ⚠️ 已知限制

- **序列埠獨佔**：Swift Pro 的 USB 序列埠不支援多個節點同時存取。目前架構中 `read_node` 與 `write_node` 可能會發生競爭，建議在未來的更新中合併為單一硬體介面節點。

## 📂 專案結構

```
Ros2_Uarm_Visual/
├── swiftpro/                 # 核心驅動套件
│   ├── launch/               # 啟動檔 (real_robot.launch.py, sim.launch.py)
│   ├── src/                  # C++ 原始碼 (讀寫節點, IK 解算)
│   ├── urdf/                 # 機器人模型描述
│   └── rviz/                 # RViz 設定檔
├── pro_moveit_config/        # MoveIt! 設定套件
│   ├── config/               # SRDF 與運動學設定
│   └── launch/               # MoveIt 啟動檔
└── README.md                 # 專案說明文件
```

## 🤝 貢獻

歡迎提交 Issue 或 Pull Request 來改進這個專案！

---
**維護者**: Wai-0424
**最後更新**: 2025-11-24

