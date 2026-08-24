# Isaac Sim UR3 Digital Twin 初始設定

本文件說明 UR3 Digital Twin 的初始設定，以及 Real-to-Sim 與 Sim-to-Real 的基本流程。以下操作以 Isaac Sim 6.0.1 為例。

> **安全提醒：** 請先使用 UR Mock Hardware 驗證 Sim-to-Real 流程。連接實體手臂前，務必確認工作區淨空、速度限制正確，並準備使用實體急停按鈕。

## 1. 啟動 Isaac Sim

開啟終端機，離開 Conda 環境後啟動 Isaac Sim：

```bash
conda deactivate
cd ~/isaacsim
./isaac-sim.sh
```

## 2. 建立 UR3 場景

1. 在 Isaac Sim 的 **Content** 面板中找到 UR3 Asset：

   ```text
   https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur3/
   ```

2. 將 UR3 Prim 拖曳到 `/World`。
3. 選擇 **Create > Physics > Ground Plane** 建立地面。

## 3. 重設 Articulation Root

1. 使用 Filter 找到並刪除原本的 Articulation Root。
2. 在 Stage 中對最上層的 `/World/ur3` Prim 按右鍵。
3. 選擇 **Add > Physics > Articulation Root**。

## 4. 設定 Real-to-Sim Action Graph

1. 選擇 **Tools > Robotics > ROS 2 OmniGraph > Joint State**。
2. 將 Articulation Root 設為 `/World/ur3`。
3. 僅勾選 Subscriber。
4. 將 Subscriber Topic 設為 `/joint_states`。
5. 按 **OK** 建立 Action Graph。

完成後，Isaac Sim 會訂閱實體或 Mock Driver 發布的 Joint State，並同步 UR3 姿態。

## 5. 啟動 UR Driver

先離開 Conda 環境並載入 ROS 2 Jazzy：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
```

### Mock Hardware（建議先使用）

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true
```

### 實體手臂

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101
```

## 6. 啟動 MoveIt 與 RViz

另開終端機並執行：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 \
  launch_rviz:=true
```

## 7. Sim-to-Real 設定

1. 先完成上述 Real-to-Sim 設定。
2. 中斷 Action Graph 的 Subscriber 節點，避免即時 Joint State 覆寫模擬端姿態。
3. 開啟 **Window > Script Editor**。
4. 貼上並執行以下測試程式。

```python
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rclpy.init()
    node = rclpy.create_node("isaacsim_ur3_test")
    client = ActionClient(
        node,
        FollowJointTrajectory,
        "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    )

    print("Waiting for UR driver...")
    if not client.wait_for_server(timeout_sec=5):
        print("Action server not found")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("Connected!")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.2, 1.2, -1.5, -1.57, 0.0]
    point.time_from_start.sec = 3
    goal.trajectory.points.append(point)

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    print("Goal sent")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

此程式會將一組六關節目標送到：

```text
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

> 這是基本連線測試，尚未包含 Goal 接受結果、執行結果與錯誤處理。操作實體手臂前，應先以 Mock Hardware 驗證目標姿態與軌跡。

## 8. Robot Poser 與 UI 參考

若要調整或擷取姿態，可開啟：

**Tools > Robotics > Robot Poser**

並將 Articulation Root 指向最上層 UR3 Prim。若需自行製作操作介面，可加入 **Load** 與 **Start** 按鈕，分別負責載入目標與開始執行。

可參考 Isaac Sim 內建範例與說明：

- **Help > Robotics Examples > Franka Cortex Examples**
- **Python Scripting and Tutorials — Isaac Sim Documentation**

## 驗證清單

- [ ] UR3 Prim 位於 `/World/ur3`。
- [ ] Articulation Root 設定在最上層 UR3 Prim。
- [ ] Real-to-Sim 使用的 Topic 為 `/joint_states`。
- [ ] Mock Hardware 可正常啟動。
- [ ] Isaac Sim 可接收 Joint State 並同步姿態。
- [ ] Sim-to-Real Action Server 可連線。
- [ ] 實體測試前已確認工作區、速度限制與急停操作。
