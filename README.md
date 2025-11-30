## 📦 專案轉移與相依性（給另一台電腦的快速上手指南）

以下步驟與套件可讓你將整個工作區安全地複製到另一台 Ubuntu 22.04 機器並繼續開發：
- 必要軟體與套件

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    ros-humble-desktop \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui
```
- 把專案 clone 到新機器並編譯

```bash
# 假設放在 ~/ros2_ws/src
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Wai-0424/Ros2_Uarm_Visual.git
cd ~/ros2_ws
colcon build --packages-select swiftpro pro_moveit_config
```
- 啟動前環境設定（每個新 shell 都要 source）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
- 啟動 demo（使用吸盤）

```bash
ros2 launch pro_moveit_config demo.launch.py end_effector:=suction
```
注意事項：
- 若出現 `Package 'pro_moveit_config' not found`，請確認你已對該機器執行 `source ~/ros2_ws/install/local_setup.bash` 並且 `AMENT_PREFIX_PATH` 包含新工作區的 `install` 位置。
- 若在兩台機器間切換時遇到二進位不相容（不同 glibc / CPU 架構），請在新機器上重新執行 `colcon build`。

## 🔁 當前修改與進度（狀態快照）

- 分支 (建議推送分支)：`convert/ros1-to-ros2`
- 主要已完成項目：
    - `swiftpro/urdf/swift_model.xacro`：已清理並加入 `<xacro:arg name="end_effector" default="suction"/>`，`Link8` 支援 `Suction.STL` 與 `Gripper.STL`，預設為 `suction`，並微調 origin 為 `-0.16201 0 -0.23651`。
    - `pro_moveit_config`：已調整 `move_group` 參數格式以支援 `--params-file`，並更新 demo launch 參數化 `end_effector`。
    - 成功 `colcon build` 並在本機測試 MoveIt + RViz 能啟動（但請注意啟動時需正確 source 工作區的 `install`）。

- 待完成項目（建議優先順序）：
    1. 如欲減少 MoveIt 警告，補上或微調各 Link 的 collision geometry（可先用 visual mesh 快速填充）。
    2. 若要正式上線真實控制，整合序列埠存取（避免 read/write 衝突）。
    3. 若需 CI/CD 或在其他機器自動化建置，請新增 `rosdep` 與 `vcstool` 的安裝腳本（我可以協助）。

---

如要我直接幫你把變更 commit 並推到 GitHub，請回覆「請推」；若你要我先只產生 commit 而不推，回覆「先不推」。我會回報 git push 的結果或提供替代指令（例如產生 patch 或給你完整 git 命令）。
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



## 🔧 2025-11-28 — 轉換與進度備註

今天已完成的工作（快速紀錄）:

- 把多數原本的 ROS1 XML launch 檔轉為 ROS2 Python launch（或新增 ROS2 wrapper），並把原始 XML 移到  做備份。
- 修正 ，避免因為系統上缺少  而直接失敗；以  與專案內的 joint topic 驅動模型顯示。
- 新增或修正了 、、以及相關的 ROS2 版 launch 檔，並在本機測試 move_group + rviz 能啟動（可看到機械臂模型）。

目前仍需處理（尚未轉換的 legacy 檔案）:

1.  -> 轉成 （或已有一份 wrapper，需檢查並統一）
2.  -> 轉成 ROS2 launch.py（controller manager 的參數與 plugin 需確認）
3.  -> 轉成 ROS2 launch.py（sensor plugins/3D 感測設定）
4.  -> 轉成 ROS2 launch.py（若你使用 ROS 參數化的 warehouse/DB 設定）

其他待辦（建議優先順序）:

- 把 、、 轉為 ROS2 參數格式（以  包裹，或由對應的 launch 以  傳入），讓  可以正確載入 OMPL 與 kinematics 設定。
- 把 controller 設定（controllers, controller_names）整合到 ROS2 參數並確認  或 swiftpro controller plugin 能正確讀取。
- 清理 （可移到  或直接刪除，視你是否要保留備份）。

分支與提交（本地）:

- 已在本地建立分支建議名稱：（若你允許，我會把目前變更 commit 到這個分支並嘗試推送到遠端）。

如果你明天要我繼續，我可以從上述第一項（ 與 controller/sensor manager 的轉換）開始進行，並每完成一個檔案就跑一次 launch 驗證。


---

### 🔎 2025-11-28 — 今日詳細紀錄（由自動化助理紀錄）

今天我在專案中完成並驗證了下列改動：

- 參數化 xacro 結構以支援 end_effector（新增或修正 `<xacro:arg name="end_effector" ...>`）
    - 在已安裝的 xacro 裡頭，將末端執行器條件改為可選 suction/gripper，並確認 `Suction.STL` 可被渲染。
- 修正並統一 MoveIt 的參數檔 `move_group_params.yaml` 的格式，避免 rcl 在啟動時解析 `--params-file` 發生錯誤：
    - 將參數包在 node key 下（`move_group:` -> `ros__parameters:`）以符合 `--params-file` 的預期格式。
- 調整 `pro_moveit_config` 的 demo launch（Python launch）：
    - 確保 `end_effector` 的 launch argument 在產生 xacro 命令前宣告，並以 `end_effector:=suction` 渲染 robot_description（目前在啟動時可看到吸盤）。
- 建置並啟動測試：
    - 使用已建置的 install（請 source 對應的 `install/local_setup.bash`）啟動 `ros2 launch pro_moveit_config demo.launch.py end_effector:=suction`，已驗證 `move_group` 啟動並印出 "You can start planning now!"，RViz 也能顯示機械臂模型與吸盤。

當前觀察與問題：

- 吸盤（Suction.STL）已成功載入，但位置有「輕微不對位」（在 RViz 中末端吸盤相對於手臂末節點的 pose 需要微調）。
    - 原因推測：xacro 中該 link/visual 的 origin (pose/xyz/rpy) 需要調整，或 SRDF/關節固定狀態造成視覺差異。目標修正方式是在 xacro 對該視覺元素微調平移或旋轉參數。
- 其他非致命警告（可接受於測試階段）：
    - 多個 link 只有 visual 沒有 collision（MoveIt 會警告）。
    - rviz 印出 Qt Wayland plugin 與 class_loader namespace collision 的警告（通常不影響基本功能，但可觀察是否影響 GPU/渲染行為）。

短期建議（下一步可由我代為執行）:

1. 微調吸盤位置（我可以直接在 `swiftpro/urdf/swift_model.xacro` 或 `pro_model.xacro` 的 Link8/相應節點調整 `<origin xyz="..." rpy="..."/>` 數值，然後重建並重新啟動驗證）。
2. 將已修好的已安裝 xacro 變更回寫到 source，以避免 source 與 install 版本不一致（我可以把安裝版內容同步到 `Ros2_Uarm_Visual-main/swiftpro/urdf/`，但會在修改前先列出會變更的檔案供你確認）。
3. 暫時把 visual mesh 複製到 collision 欄位，快速消除 MoveIt 的大量 collision warnings（此為快速且粗糙的臨時處理）。

如何重現（簡短命令）：

```bash
source /opt/ros/humble/setup.bash
source /media/wai4424/Data/Ros2_Humble/ros2_ws_moveit/install/local_setup.bash
ros2 launch pro_moveit_config demo.launch.py end_effector:=suction
```

要我做的選項（請回覆 A / B / C 或其他指示）：

- A：我將把 `swift_model.xacro` & `pro_model.xacro` 的 suction origin 微調（嘗試小幅 xyz/rpy 變動）並 rebuild，直到對位良好。
- B：我將把已安裝的 xacro 變更回寫到 source，確保 source 與 install 一致（包含 end_effector 參數化），並提交這些變更。
- C：我把 README 的變更 commit 並推上 GitHub（我將嘗試推到 `convert/ros1-to-ros2` 分支；若推送需要憑證或遠端未設定會回報錯誤訊息）。

我已紀錄本次變更，並建議若要我直接修正吸盤對位請回覆 A；若只要把紀錄上傳 GitHub 請回覆 C（或同時選 A+C）。

---

**備註**：今天的操作包含直接替換與修復已安裝檔案（install 下的 xacro 與 launch），以快速排查 runtime 問題。為長期維護，建議把那些已修好的改動同步回 source 並以 commit 的方式管理。

## 🔧 2025-11-30 — 吸盤對位修復與 MoveIt 設定更新

今天已完成的工作（詳細紀錄）:

1.  **吸盤對位修復 (Suction Alignment Fix)**:
    - 發現吸盤模型 (\Suction.STL\) 與末端連桿 (\Link9.STL\) 的座標原點不一致，導致視覺上有縫隙。
    - 修改 \swiftpro/urdf/pro_model.xacro\，將吸盤的 \<origin>\ 調整為與 \Link9\ 一致 (\-0.19941 0 -0.27471\)，成功消除縫隙。

2.  **專案清理 (Project Cleanup)**:
    - 移除標準版 Swift (非 Pro) 的相關檔案，避免混淆：
        - \swiftpro/urdf/swift_model.xacro        - \swiftpro/urdf/swift_links/        - \swiftpro/launch/swift_control.launch        - \swiftpro/launch/swift_display.launch
3.  **MoveIt 設定更新 (MoveIt Configuration)**:
    - 修改 \pro_moveit_config/launch/demo.launch.py\：
        - 將模型路徑從 \swift_model.xacro\ 改為正確的 \pro_model.xacro\。
        - 加入 \static_transform_publisher\ 發布 \world\ -> \Base\ 的座標轉換，解決 RViz 中的 \�
