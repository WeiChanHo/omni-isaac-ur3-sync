# 啟動 UR3 實體手臂與 RViz

本文件說明如何啟動 UR3 實體手臂、ROS 2 Driver 與 RViz，並透過 MoveIt 規劃及執行動作。

> **安全提醒：** 首次測試前請確認工作區淨空，並先以低速及 Mock Hardware 驗證流程。`Cancel Goal` 不是緊急停止機制；操作實體手臂時，請隨時準備使用實體急停按鈕。

## 1. 啟動 UR3 控制器

在 UR3 Teach Pendant（觸控面板）上依序操作：

1. 開啟電源。
2. 若 Shell 顯示啟動提示，輸入 `exit` 跳過 `startup.nsh`。
3. 進入初始化畫面，依序選擇：
   - **Power On**
   - **Start**
   - **Normal Robot**

## 2. Teach Pendant 基本操作

進入 **Run Program > Move** 後，可使用下列功能：

1. **TCP Position**：調整工具中心點位置。
2. **TCP Orientation**：調整工具中心點方向。
3. **Home > Auto > OK**：讓手臂回到 Home 位置。
4. **Free Drive**：按住按鈕後，可手動調整手臂姿態。

## 3. 設定 External Control

Real-to-Sim 或 Sim-to-Real 操作皆需先在 Teach Pendant 建立 External Control 程式：

1. 選擇 **Program Robot > Empty Program > Empty**。
2. 進入 **Structure > URCaps > External Control**。
3. 加入 External Control 節點後，按下 **Play**。

## 4. 啟動 UR ROS 2 Driver

開啟第一個終端機並執行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  launch_rviz:=false
```

## 5. 啟動 MoveIt 與 RViz

開啟第二個終端機並執行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 \
  launch_rviz:=true
```

## 6. 在 RViz 規劃與執行動作

1. 在 UR3 Teach Pendant 上確認 External Control 程式正在執行（已按下 **Play**）。
2. 在 RViz 中按 **Plan**，先檢查規劃出的模擬軌跡。
3. 確認軌跡安全後，再按 **Execute** 讓實體手臂執行。
