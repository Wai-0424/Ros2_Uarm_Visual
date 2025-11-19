# SwiftAndProForROS2 (Humble)

## 安裝
```bash
sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-xacro
```
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/uArm-Developer/RosForSwiftAndSwiftPro.git swiftpro_ros1_orig
# 將轉換後 swiftpro / swift_moveit_config / pro_moveit_config 放入此處
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 啟動示例
```bash
ros2 launch swift_moveit_config demo.launch.py
```

## 訊息 (沿用)
```bash
SwiftproState.msg
position.msg
angle4th.msg
status.msg
```

## 串列埠權限
```bash
sudo chmod 666 /dev/ttyACM0
```
