# UR3 Robot Poser 執行器

這是一個 Isaac Sim Extension，用來驗證 Robot Poser 已儲存的逆向運動學
（Inverse Kinematics，IK）解，並透過 ROS 2 將其傳送至實體 Universal Robots
UR3。

本 Extension 採用需要操作員監督的謹慎流程：

1. 選擇一個 Robot Poser 命名姿勢（Named Pose）。
2. 載入並驗證其 IK 結果。
3. 檢查六個目標關節位置。
4. 確認實機回授用 UR3 持續同步 `/joint_states`。
5. 確認運動內容。
6. 將一個 `FollowJointTrajectory` 目標傳送至機器人控制器。

本 Extension **不會**將 Isaac Sim 的關節位置持續鏡像或串流至實體機器人。

> [!WARNING]
> 本 Extension 會控制實體硬體。測試時請降低速度、清空機器人的完整工作空間，
> 並確保可立即操作緊急停止按鈕。實機執行期間，介面中的
> **Cancel Goal** 按鈕只會提出 ROS 2 Action 取消要求；它不是
> 通過安全認證的停止裝置，也不能取代機器人的實體緊急停止按鈕。

## 功能

- 尋找 Isaac Sim Robot Poser Extension 儲存的命名姿勢。
- 在姿勢選擇器下方直接顯示目前選擇的命名姿勢。
- 拒絕遺失、失敗、不完整、NaN 或無限值的 IK 結果。
- 啟用實機執行前，要求從 `/joint_states` 收到 UR 六個關節的有效回授。
- 傳送指令前顯示目標關節位置，供操作員檢查。
- 每一次實體運動都必須經過確認。
- Timeline 狀態不會阻止實機執行；若要讓模擬 UR3 同步顯示實機回授，
  請在執行前將 Timeline 設為 **Play**。
- 提供 `0.05-3.14 rad/s` 的關節速度滑桿（預設 `0.5 rad/s`），並根據最大
  關節位移自動計算軌跡時間。
- 執行期間監控實體關節回授。如果尚未到達目標便停止移動，會顯示警告
  對話框並自動要求取消軌跡。
- 軌跡控制器中止或執行失敗時，也會顯示相同的警告。
- 顯示目標接受、完成、取消及控制器錯誤狀態。

## Timeline 操作規則

兩支模擬手臂分工，因此 Timeline 在整個工作流程中保持 **Play**：

| 階段 | Isaac Sim Timeline | 原因 |
| --- | --- | --- |
| 編輯 Robot Poser 目標 | **Play** | Robot Poser 只操作規劃用 UR3，不會與實機回授用 UR3 衝突。 |
| 在實體 UR3 上執行 | **Play** | Action Graph 將實機的 `/joint_states` 套用至實機回授用 UR3。 |

Extension 內部的 ROS 2 subscriber 會持續接收實機 `/joint_states`，用於
計算軌跡時間，以及執行停滯和最終位置檢查。

> [!IMPORTANT]
> 請讓 Timeline 全程保持 **Play**，並確認 Robot Poser 與
> 實機回授 Action Graph 操作不同的 UR3。實機執行期間，請同時
> 比較模擬手臂、實體手臂、目標關節值及 Status 結果。畫面一致是有用的操作
> 依據，但不能取代控制器結果或實體安全檢查。

## 需求

- NVIDIA Isaac Sim，並可使用下列 Extension：
  - `isaacsim.robot.poser`
  - `isaacsim.ros2.bridge`
  - `isaacsim.core.utils`
- 與目前 Isaac Sim 版本相容的 ROS 2 環境。
- 提供下列套件的 ROS 2 message：
  - `action_msgs`
  - `control_msgs`
  - `sensor_msgs`
  - `trajectory_msgs`
- 正在執行的 UR ROS 2 driver 與 controller，且必須：
  - 在 `/joint_states` 發布 `sensor_msgs/msg/JointState`；
  - 將 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 公開為
    `control_msgs/action/FollowJointTrajectory` Action。
- 目前 USD Stage 中的 `/World/ur3` 必須是一個 UR3 articulation。
- Robot Poser 必須已為該 articulation 儲存至少一個有效命名姿勢。

預期的 UR 關節名稱如下：

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

## 替代方案：使用 Mock Hardware 測試

使用 UR driver 的 Mock Hardware 模式，可以在不連接或移動實體 UR3 的情況下
測試本 Extension。Mock 模式會測試 ROS 2 關節回授、軌跡控制器、Robot Poser
驗證、確認、執行及取消功能，但不會驗證機器人網路、Teach Pendant 的
External Control、實體運動或真實環境安全。

> [!IMPORTANT]
> 請勿同時執行 Mock driver 與實體 UR driver。開始實機測試前，必須先在
> Mock driver 終端機中按下 `Ctrl+C` 停止它。

### 1. 啟動 Mock UR3 driver

在**終端機 1** 中載入 ROS 2，並設定與 Isaac Sim 相同的 `ROS_DOMAIN_ID`。
如果實驗室使用其他 Domain ID，請替換下列 `0`。

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  launch_rviz:=true \
  use_mock_hardware:=true
```

讓此終端機保持執行。Mock 模式不需要開啟 UR3 電源，也不需要啟動 Teach
Pendant 的 External Control 程式。

### 2. 驗證 Mock ROS 2 介面

在**終端機 2** 使用相同的 ROS Domain：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

確認可以取得 Mock 關節回授：

```bash
ros2 topic echo /joint_states --once
```

確認 `scaled_joint_trajectory_controller` 為 `active`：

```bash
ros2 service call \
  /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers \
  "{}"
```

確認預期的軌跡 Action 有對應的 Action server：

```bash
ros2 action info \
  /scaled_joint_trajectory_controller/follow_joint_trajectory
```

### 3. 啟動 Isaac Sim 並啟用 Extension

在**終端機 3**，使用相同 ROS Domain 啟動 Isaac Sim：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
cd /home/spatiallabs/isaacsim
./isaac-sim.sh
```

接著依照[在 Isaac Sim 中安裝並啟用 Extension](#7-在-isaac-sim-中安裝並啟用-extension)
及[開啟並檢查 UR3 場景](#8-開啟並檢查-ur3-場景)操作。

### 4. 建立並執行 Mock 目標

1. 按下 Timeline **Play**，並讓 Timeline 在整個測試期間持續播放。
2. 確認 Mock `/joint_states` 只驅動實機回授用 UR3。
3. 依照[使用 Robot Poser 建立小幅度命名姿勢](#10-使用-robot-poser-建立小幅度命名姿勢)操作。
4. 在 **UR3 Robot Poser Execution** 中按下 **Refresh**。
5. 選擇命名姿勢，然後按下 **Load and Validate IK Solution**。
6. 檢查六個目標關節值，並選擇合適的關節速度上限。
7. 確認實機回授用 UR3 在執行期間跟隨 Mock `/joint_states`。
8. 按下 **Execute on Physical UR3**。Mock 模式中的按鈕仍會使用此名稱，
   但目標只會傳送至 Mock controller，不會移動實體機器人。
9. 檢查確認對話框，然後按下 **Execute**。
10. 確認 UI 先顯示目標已接受，接著顯示成功完成。

### 5. 驗證 Mock 結果並停止 driver

在終端機 2 中讀取更新後的 Mock 關節位置：

```bash
ros2 topic echo /joint_states --once
```

最終數值應接近 Extension 顯示的目標值。也可以使用時間較長的目標，測試
**Cancel Goal**。測試完成後，關閉 Isaac Sim，並在終端機 1
中按下 `Ctrl+C` 停止 Mock driver。

## 逐步操作：搭配實體 UR3 執行

本流程依照目前工作站的設定撰寫：

| 項目 | 值 |
| --- | --- |
| ROS 2 | Jazzy |
| Isaac Sim 啟動程式 | `/home/spatiallabs/isaacsim/isaac-sim.sh` |
| Extension repository | `/home/spatiallabs/Desktop/omni.isaac.ur3_sync` |
| UR3 場景 | `/home/spatiallabs/Desktop/ur3_arm_ik_solver/ur3_arm_handoff/sim/Collected_real2sim_ur3/real2sim_ur3.usd` |
| UR3 IP 位址 | `192.168.56.101` |
| Robot prim | `/World/ur3` |

操作過程中需使用三個終端機：

| 終端機 | 用途 |
| --- | --- |
| 1 | 執行實體 UR driver |
| 2 | 檢查 ROS 2 topic、controller 及 Action |
| 3 | 在 ROS 2 環境中啟動 Isaac Sim |

### 1. 完成實體安全檢查

開啟手臂電源或傳送指令前：

- 清除機器人完整工作空間內的人員、工具、電纜及其他障礙物。
- 在 Teach Pendant 上確認 payload 及 TCP（Tool Center Point）設定。
- 確保可立即操作 Teach Pendant 與實體緊急停止按鈕。
- 根據實驗室 SOP 設定 Teach Pendant 速度滑桿。第一次有人監督的測試請從
  `5-10%` 開始。
- 停止所有 Mock driver，以及其他所有可能控制 UR3 的程式。
- 第一個目標只能使用小幅度位移，並遠離關節限制、桌面、人員及障礙物。

> [!CAUTION]
> 本 Extension 不會執行碰撞檢查、完整關節限制檢查或無碰撞路徑規劃，
> 也不提供通過安全認證的停止功能。執行本實機流程前，必須先完成 Mock Hardware
> 測試。

### 2. 所有終端機使用相同的 ROS Domain

Isaac Sim、UR driver 及所有診斷終端機都必須使用相同的 `ROS_DOMAIN_ID`。
每次開啟新終端機時，執行下列設定。如果實驗室使用不同 Domain ID，請替換
`0`。

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
```

如果不同終端機顯示的 Domain ID 不同，請勿繼續。

### 3. 開啟 UR3 電源並完成準備

在 UR3 Teach Pendant 上：

1. 根據實驗室程序開啟控制器及機器人電源。
2. 解除煞車，並確認機器人顯示正常運作狀態。
3. 確認設定的 payload、TCP 及低速滑桿設定。
4. 載入 URCap **External Control** 程式，但請先依照下一個步驟啟動 ROS 2
   driver，再執行該程式。

如果機器人顯示 Protective Stop 或其他安全錯誤，請先依照 UR 及實驗室程序
解決問題，再繼續操作。

### 4. 啟動實體 UR ROS 2 driver

在**終端機 1** 中，先確認工作站能夠連線至機器人：

```bash
ping -c 3 192.168.56.101
```

接著啟動 driver：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  launch_rviz:=false
```

讓此終端機保持執行。Driver 準備完成後，從 Teach Pendant 執行
**External Control** 程式。請勿同時執行 Mock driver。

### 5. 驗證機器人的 ROS 2 介面

在**終端機 2** 載入相同的 ROS 環境：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

讀取一則實體關節狀態訊息：

```bash
ros2 topic echo /joint_states --once
```

訊息必須包含全部六個預期 UR3 關節的有限位置值。接著檢查 controller：

```bash
ros2 service call \
  /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers \
  "{}"
```

確認 `scaled_joint_trajectory_controller` 為 `active`。最後檢查軌跡 Action：

```bash
ros2 action info \
  /scaled_joint_trajectory_controller/follow_joint_trajectory
```

輸出內容必須顯示存在一個 Action server。若任一檢查失敗，請在此停止，並修正
driver、controller、網路、External Control 程式或 ROS Domain。

### 6. 從 ROS 2 環境啟動 Isaac Sim

在**終端機 3** 使用相同的 Domain ID 啟動 Isaac Sim：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
cd /home/spatiallabs/isaacsim
./isaac-sim.sh
```

必須從此終端機啟動 Isaac Sim，因為 Extension 會匯入 `rclpy`，而且必須能在
ROS 2 graph 中找到 UR driver。

### 7. 在 Isaac Sim 中安裝並啟用 Extension

通常只需要執行一次此設定：

1. 開啟 **Window -> Extensions**。
2. 開啟 Extension Manager 設定。
3. 將 `/home/spatiallabs/Desktop` 加入 **Extension Search Paths**。這是
   `omni.isaac.ur3_sync` repository 的上層目錄。
4. 重新整理 Extension 清單。
5. 啟用 `isaacsim.ros2.bridge`。
6. 如果 `isaacsim.robot.poser` 與 `isaacsim.robot.poser.ui` 分開顯示，請將
   兩者都啟用。
7. 找到 **UR3 Robot Poser Executor**（`omni.isaac.ur3_sync`）並啟用。

啟用 Extension 後，會開啟 **UR3 Robot Poser Execution** 視窗，並啟動
ROS 2 node `isaacsim_ur3_sync_extension`。如果清單中沒有顯示此自訂
Extension，請確認下列檔案存在：

```text
/home/spatiallabs/Desktop/omni.isaac.ur3_sync/config/extension.toml
```

### 8. 開啟並檢查 UR3 場景

在 Isaac Sim 中：

1. 選擇 **File -> Open**。
2. 開啟
   `/home/spatiallabs/Desktop/ur3_arm_ik_solver/ur3_arm_handoff/sim/Collected_real2sim_ur3/real2sim_ur3.usd`。
3. 確認 Stage 包含 articulation `/World/ur3`。
4. 確認 Extension 視窗顯示 `Robot: /World/ur3`。
5. 確認場景中的實機至模擬器 Action Graph 訂閱 `/joint_states`，且目標為
   `/World/ur3`。

如果不希望將姿勢變更寫入收集完成的 handoff 場景，請使用 **File -> Save As**
建立工作副本。

### 9. 將目前實體姿勢同步至 Isaac Sim

1. 按下 Isaac Sim Timeline 的 **Play**。
2. 確認模擬 UR3 移動至與實體 UR3 相同的六軸姿勢，而且沒有控制實體機器人。
3. 檢查關節方向及大致位置是否一致。
4. 讓 Timeline 保持 **Play**。

編輯 Robot Poser 目標時，Timeline 保持 **Play**。Action Graph 必須只將
`/joint_states` 寫入實機回授用 UR3，Robot Poser 則只操作規劃用 UR3。
如果規劃用 UR3 被拉回實機位置，請修正 Action Graph
的 articulation 目標，不要停止 Timeline。

### 10. 使用 Robot Poser 建立小幅度命名姿勢

Timeline 保持 **Play**：

1. 開啟 **Robot Poser**。
2. 將 **Active Robot** 設為 `/World/ur3`。
3. 將 **Start Site** 設為 `/World/ur3/base_link`。
4. 將 **End Site** 設為 `/World/ur3/wrist_3_link/flange`。
5. 從同步完成的目前姿勢建立一個命名姿勢，例如
   `physical_verify_small_move`。
6. 啟用 target/tracking，第一次測試時只將 End Effector Target 移動幾毫米，
   並避免大幅改變方向。
7. 確認 Robot Poser 顯示 IK 求解成功。
8. 停用 target/tracking，然後儲存或更新命名姿勢。

拖曳 Robot Poser target 只會改變模擬目標。實體 UR3 不會跟隨滑鼠移動；只有在
Extension 顯示確認對話框，且使用者確認執行之後，實體機器人才應該移動。

### 11. 載入並檢查 IK 解

在 **UR3 Robot Poser Execution** 視窗中：

1. 按下 **Refresh**。
2. 從 **Named Pose** 選擇 `physical_verify_small_move`。
3. 按下 **Load and Validate IK Solution**。
4. 確認 UI 顯示六個以弧度為單位的有限關節值。
5. 第一次實機測試時，將 **Joint speed limit** 設為最小值 `0.05 rad/s`。
6. 確認 Timeline 仍為 **Play**，且實機回授用 UR3 仍從實體
   `/joint_states` 持續更新。
7. 按下 **Execute on Physical UR3** 開啟確認對話框，但先不要確認。
8. 檢查目標關節、實機目前位置至目標的最大位移、方向及軌跡時間。

第一次測試時，最大關節位移不可超過 `0.05 rad`（約 `2.9 度`）。如果超過，
請按下 **Cancel** 並建立較小的目標。這是保守的功能測試數值，不是經認證的
安全限制。

Extension 會比較最近一次收到的實體關節位置與目標，並計算軌跡時間。使用者
不需輸入時間：

```text
時間 = max(0.1 s, 最大關節位移 / 所選關節速度)
```

滑桿上限為 `180 度/s`（`pi rad/s`），這是 UR3 datasheet 所列兩種額定值中
較低的一個：肩部／肘部關節額定速度為 `180 度/s`，腕部關節則為
`360 度/s`。使用這個共同上限，可確保單一滑桿值對每一個關節都有效。Teach
Pendant 速度滑桿與 UR scaled trajectory controller 可能進一步降低實際速度。

Timeline 未開始播放時，Extension 仍允許開啟確認對話框並送出軌跡。
此時 Extension 仍會直接接收 ROS 2 `/joint_states` 用於執行監控，但模擬
UR3 不會透過 Action Graph 同步顯示實機姿態。

### 12. 確認並執行實體運動

在確認對話框中按下 **Execute** 前，請確認下列所有項目：

- 完整工作空間內沒有任何障礙物。
- 目標非常接近機器人目前姿勢。
- 六個目標角度、最大位移、時間及預期方向都正確。
- 可立即操作 Teach Pendant 及緊急停止按鈕。
- 沒有其他 ROS 2 node 正在傳送機器人指令。
- 若需要模擬畫面同步顯示實機回授，Isaac Sim Timeline 為 **Play**。

只有每項檢查都通過時，才可按下 **Execute**。預期的狀態順序如下：

```text
Goal accepted. Executing pose 'physical_verify_small_move'...
Pose 'physical_verify_small_move' completed successfully.
```

請在整個運動期間持續觀察實體手臂。如果方向、速度、聲音或姿勢不如預期，
請立即使用 Teach Pendant 的安全停止功能或實體緊急停止按鈕。
**Cancel Goal** 在實機執行期間只會提出 ROS 2 Action 取消要求；
它不是通過安全認證的停止功能，也不能取代緊急停止按鈕。

### 13. 驗證結果

成功完成後：

1. 讓 Timeline 保持 **Play**，使實體最終姿勢持續同步至 Isaac Sim。
2. 確認模擬 UR3 與實體 UR3 的姿勢一致。
3. 在終端機 2 擷取最終關節位置：

   ```bash
   ros2 topic echo /joint_states --once
   ```

4. 確認最終數值接近 Extension 顯示的六個目標角度。
5. 確認沒有發生 Protective Stop 或 controller 錯誤。
6. 編輯下一個 Robot Poser 目標時，讓 Timeline 繼續保持 **Play**。

### 14. 安全關閉

測試完成後：

1. 不要開始下一個運動。
2. 停止或停用自訂 Extension，然後關閉 Isaac Sim。
3. 在終端機 1 中按下 `Ctrl+C`，停止 UR driver。
4. 停止 Teach Pendant 上的 External Control 程式。
5. 如果不再需要機器人，請依實驗室程序關閉電源。

## 固定設定

目前實作在 `exts/omni/isaac/ur3_sync/extension.py` 中定義機器人及 ROS 2
介面：

| 設定 | 預設值 |
| --- | --- |
| Robot prim | `/World/ur3` |
| 關節狀態 topic | `/joint_states` |
| 軌跡 Action | `/scaled_joint_trajectory_controller/follow_joint_trajectory` |
| 使用者速度範圍 | `0.05-pi rad/s` |
| 預設使用者速度 | `0.5 rad/s` |
| 內部最短軌跡時間 | `0.1 s` |
| 停滯偵測啟動寬限時間 | `1.0 s` |
| 停滯逾時 | `3.0 s` |
| 有效移動門檻 | `0.002 rad` |
| 停滯監控的目標容許誤差 | `0.01 rad` |

這些值是 `Ur3SyncExtension` 的 class constants。如果 Stage、ROS namespace
或 controller 使用不同路徑，請在啟動 Isaac Sim 前更新對應 constants：

```python
ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/joint_states"
ROBOT_PRIM_PATH = "/World/ur3"
MIN_COMMAND_SPEED = 0.05
MAX_COMMAND_SPEED = math.radians(180.0)
DEFAULT_COMMAND_SPEED = 0.5
MIN_TRAJECTORY_DURATION = 0.1
STALL_STARTUP_GRACE = 1.0
STALL_TIMEOUT = 3.0
STALL_MOVEMENT_THRESHOLD = 0.002
STALL_GOAL_TOLERANCE = 0.01
```

## 驗證方式

姿勢可以執行前，Extension 會驗證：

- 存在使用中的 USD Stage；
- `/World/ur3` 是有效 prim；
- 所選命名姿勢存在；
- Robot Poser 將其 IK 結果標示為成功；
- 儲存結果包含全部六個預期的 UR 關節名稱；
- 每個目標關節值都是有限值；
- 已收到完整的實體 `/joint_states` 訊息；
- 軌跡 Action server 已準備就緒。

變更所選姿勢或重新整理姿勢清單，都會使已載入的解失效，因此必須重新載入並
再次檢查。

### 動作完成判定

最終執行結果以 `FollowJointTrajectory` Action server 回傳為準。只有 ROS Goal
狀態為 `STATUS_SUCCEEDED`，且 controller 結果為 `SUCCESSFUL`，Extension 才會
顯示動作成功。Controller 會依自身設定的 goal tolerance 判斷手臂是否到站。

`/joint_states` 與 Action result 是兩條非同步訊息來源，因此 Action 完成當下的
最新一筆 `/joint_states` 快取可能仍是到站前的樣本。Extension 不再用這筆快取
推翻 controller 的成功結果；它仍用於執行期間的停滯偵測，以及 Action 失敗時
顯示剩餘關節誤差。

## 疑難排解

### 沒有列出任何命名姿勢

- 確認目前 Stage 包含 `/World/ur3`。
- 在 Robot Poser 中建立、求解並儲存一個命名姿勢。
- 按下 **Refresh**。

### `Robot prim not found: /World/ur3`

Articulation 位於不同的 USD 路徑。請將它移動或 reference 至 `/World/ur3`，
或變更 `extension.py` 中的 `ROBOT_PRIM_PATH`。

### `IK result is missing UR joints`

已儲存的 Robot Poser 結果並未包含全部六個預期關節名稱。請確認 Robot Poser
的目標是 UR3 articulation，且其 joint prim 名稱符合[需求](#需求)中列出的名稱。

### 未收到有效的 `/joint_states`

- 確認 UR driver 正在執行並發布 `/joint_states`。
- 確認訊息包含全部六個預期關節。
- 確認 Isaac Sim 與 driver 使用相容的 ROS 2 middleware 設定及相同的
  `ROS_DOMAIN_ID`。

### Action server 尚未準備就緒

檢查 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 是否存在，
以及 scaled joint trajectory controller 是否為 active。如果 driver 使用
namespace 或其他 controller 名稱，請更新 `ACTION_NAME`。

### 目標被拒絕或執行失敗

查看 Extension 顯示的狀態，並檢查 UR driver/controller log。常見原因包括
controller 未啟用、超出關節限制、Protective Stop，或實體機器人無法安全到達
目標。

### 偵測到可能的碰撞或運動停滯

Extension 推斷手臂在距離目標仍超過 `0.01 rad` 時，已連續 `3.0 s` 沒有產生
至少 `0.002 rad` 的關節進度。它會要求取消軌跡並顯示警告對話框。這是保守的
停滯偵測器，不是碰撞感測器：Teach Pendant 速度滑桿暫停、過度激進的外部
速度縮放、遺失 `/joint_states` 或 controller 故障，都可能顯示相同警告。
重設或傳送下一個目標前，請先檢查 UR controller 及實體工作空間。

## 專案結構

```text
.
├── config/
│   └── extension.toml
└── exts/
    └── omni/isaac/ur3_sync/
        ├── __init__.py
        └── extension.py
```

Extension 版本定義於 `config/extension.toml`。
