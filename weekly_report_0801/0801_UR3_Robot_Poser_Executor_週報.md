# 08/01 週報：UR3 Robot Poser Executor

## 一、專案資訊

| 項目 | 內容 |
| --- | --- |
| 專案名稱 | UR3 Robot Poser Executor |
| Extension ID | `omni.isaac.ur3_sync` |
| 報告日期 | 2026/08/01 |
| 機器人 | Universal Robots UR3 |
| 模擬平台 | NVIDIA Isaac Sim |
| 通訊方式 | ROS 2 |
| 主要目的 | 讀取並驗證 Robot Poser 儲存的 IK 解，再將六軸關節目標傳送至實體 UR3 |

## 二、本週工作摘要

本 extension 建立了 Isaac Sim Robot Poser 與實體 UR3 之間的受控執行流程。使用者可以在 Robot Poser 中設定目標姿態、完成逆向運動學（Inverse Kinematics，IK）求解並儲存為 Named Pose；extension 再讀取該 Named Pose 中的六軸關節角，完成資料驗證、安全速度計算與人工確認後，透過 ROS 2 `FollowJointTrajectory` Action 將目標傳給實體 UR3。

本 extension 的定位是「IK 結果讀取、驗證與執行器」，不是 IK 求解器，也不會持續同步 Isaac Sim 與實體機器人的關節位置。每次操作只會送出一筆軌跡目標。

## 三、Extension 功能介紹

主要功能如下：

1. 從目前開啟的 USD Stage 尋找 `/World/ur3`。
2. 讀取 Robot Poser 儲存於 UR3 上的 Named Pose 清單。
3. 取得指定 Named Pose 的 IK 求解結果。
4. 驗證 IK 是否成功、六個 UR3 關節是否完整，以及數值是否合法。
5. 以固定順序顯示六個目標關節角，單位為弧度（rad）。
6. 從 `/joint_states` 取得實體 UR3 的目前關節位置。
7. 根據目前位置與目標位置的最大角度差，自動調整軌跡時間，使規劃平均速度不超過 `0.5 rad/s`。
8. 顯示實機動作確認視窗。
9. 透過 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 傳送單點關節軌跡。
10. 顯示目標接受、執行成功、取消或控制器錯誤等狀態。

### 固定設定

| 設定 | 目前值 | 用途 |
| --- | --- | --- |
| Robot Prim | `/World/ur3` | 在 USD Stage 中尋找 UR3 |
| Joint State Topic | `/joint_states` | 接收實體 UR3 的目前關節角 |
| Trajectory Action | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | 傳送實體機器人的關節軌跡 |
| 最大規劃關節速度 | `0.5 rad/s` | 自動延長軌跡時間 |
| 最短軌跡時間 | `1.5 s` | 避免設定過短的執行時間 |

固定的六個關節順序為：

```text
1. shoulder_pan_joint
2. shoulder_lift_joint
3. elbow_joint
4. wrist_1_joint
5. wrist_2_joint
6. wrist_3_joint
```

## 四、IK 解的取得與處理方式

### 4.1 IK 解由哪一個元件計算

IK 解是由 Isaac Sim 的 Robot Poser extension 計算。本 extension 沒有實作 Jacobian、Lula、解析式或數值迭代 IK，也不會根據 Base 與 Flange 的 Transform 自行重新求解。

完整流程如下：

```text
在 Robot Poser 設定末端執行器目標姿態
                    ↓
          Robot Poser 執行 IK 求解
                    ↓
       將成功結果儲存為 Named Pose
                    ↓
   UR3 Robot Poser Executor 讀取 Named Pose
                    ↓
       驗證並整理成六個 UR3 關節角
                    ↓
      計算安全時間並送出 ROS 2 軌跡
```

因此，如果 Robot Poser 尚未成功求解或尚未儲存 Named Pose，本 extension 就沒有可載入的 IK 解。

### 4.2 尋找 Named Pose

按下 `Refresh` 時，程式先取得目前的 USD Stage，再確認 `/World/ur3` 是否為有效 Prim。確認成功後呼叫：

```python
list_named_poses(stage, robot_prim)
```

取得該 UR3 所屬的 Named Pose 清單，排序後放入 `Named Pose` 下拉選單。

### 4.3 讀取指定的 IK 解

使用者選擇 Named Pose 並按下 `Load and Validate IK Solution` 後，程式呼叫：

```python
pose = get_named_pose(stage, robot_prim, pose_name)
```

回傳物件中主要使用：

| 欄位 | 意義 |
| --- | --- |
| `pose.success` | Robot Poser 是否將這筆 IK 結果標記為成功 |
| `pose.joints` | 關節 USD Prim 路徑與關節角數值的對應表 |

`pose.joints` 的資料概念如下：

```python
{
    "/World/ur3/.../shoulder_pan_joint": 0.25,
    "/World/ur3/.../shoulder_lift_joint": -1.10,
    "/World/ur3/.../elbow_joint": 1.42,
    "/World/ur3/.../wrist_1_joint": -0.65,
    "/World/ur3/.../wrist_2_joint": 1.20,
    "/World/ur3/.../wrist_3_joint": 0.10,
}
```

關節角的單位為弧度。

### 4.4 將 USD 路徑轉換成關節名稱

由於 `pose.joints` 使用完整 USD Prim 路徑，程式會逐一呼叫 `stage.GetPrimAtPath(joint_path)`，再使用 `joint_prim.GetName()` 取出關節名稱。

例如：

```text
/World/ur3/.../shoulder_pan_joint
```

會轉換為：

```text
shoulder_pan_joint
```

接著以關節名稱建立 `positions_by_name`，避免因 USD 階層路徑長度不同而影響排序。

### 4.5 IK 解驗證

載入後依序執行以下檢查：

1. 目前是否存在有效 USD Stage。
2. `/World/ur3` 是否存在且為有效 Prim。
3. 選取的 Named Pose 是否仍然存在。
4. `pose.success` 是否為 `True`。
5. 六個 UR3 關節是否全部存在。
6. 每個關節角是否為有限數值。
7. 關節角是否包含 `NaN` 或正負無限大。

通過後，程式依固定的 UR3 關節順序整理成：

```text
[q1, q2, q3, q4, q5, q6]
```

並儲存於：

```python
self._pending_pose_name
self._pending_positions
```

同時在介面顯示 Pose 名稱與六個目標關節角，並啟用 `Execute on Physical UR3`。

切換 Named Pose 或按下 `Refresh` 都會清除已載入的結果並停用執行按鈕，使用者必須重新載入與檢查，避免誤送舊資料。

### 4.6 實體位置與軌跡時間

extension 訂閱實體 UR3 的 `/joint_states`，依關節名稱重新排列目前位置。這些實體位置不會參與 IK 求解，只用來計算目前位置到目標位置的最大關節角差：

```text
max_delta = max(|target[i] - current[i]|)
```

所需軌跡時間為：

```text
required_duration = max_delta / 0.5 rad/s

final_duration = max(
    使用者要求的時間,
    1.5 秒,
    required_duration
)
```

如果使用者要求的時間太短，介面會自動將時間增加到符合速度限制的數值。

### 4.7 傳送至實體 UR3

人工確認後，程式建立 `FollowJointTrajectory.Goal`：

```python
goal.trajectory.joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

point.positions = [q1, q2, q3, q4, q5, q6]
point.time_from_start = final_duration
```

目前軌跡只有一個目標點，由 UR ROS 2 trajectory controller 負責從實體機器人的目前狀態移動到該目標。

## 五、介面元件與按鈕功能

### 5.1 `Named Pose` 下拉選單

- 顯示目前 `/World/ur3` 可使用的 Robot Poser Named Poses。
- 切換選項時，會清除已載入的 IK 解、關閉尚未完成的確認視窗，並停用實機執行按鈕。
- 選擇新項目後必須再次按下 `Load and Validate IK Solution`。

### 5.2 `Refresh`

- 重新取得目前 USD Stage。
- 確認 `/World/ur3` 是否存在。
- 重新呼叫 Robot Poser API 取得 Named Pose 清單。
- 清除先前載入的 IK 解。
- 適用於新增、修改或刪除 Robot Poser Named Pose 之後。

### 5.3 `Load and Validate IK Solution`

- 讀取目前選取的 Named Pose。
- 檢查 Robot Poser 是否將 IK 標記為成功。
- 讀取 `pose.joints`。
- 將關節 USD 路徑轉換成關節名稱。
- 確認六個 UR3 關節完整且數值有限。
- 依固定順序建立六軸關節角陣列。
- 在畫面顯示關節角。
- 驗證成功後啟用 `Execute on Physical UR3`。
- 軌跡執行期間不可載入另一個 Pose。

### 5.4 `Requested duration`

此元件不是按鈕，而是軌跡時間輸入欄位。

- 預設值為 `3.0 s`。
- 可設定範圍為 `1.5–60.0 s`。
- 每次調整步進為 `0.5 s`。
- 若設定值會使任一關節的規劃平均速度超過 `0.5 rad/s`，extension 會自動延長時間。

### 5.5 `Execute on Physical UR3`

按下後會：

1. 確認目前沒有其他軌跡正在執行。
2. 確認已載入有效 IK 解。
3. 確認已收到完整且合法的實體 `/joint_states`。
4. 確認 ROS 2 Action Server 已就緒。
5. 計算目前到目標的最大關節角差。
6. 根據速度政策計算最終軌跡時間。
7. 顯示實機動作確認視窗。

此按鈕不會立即讓機器人移動，必須在確認視窗中再次按下 `Execute`。

### 5.6 確認視窗的 `Execute`

- 確認工作空間安全後，將單點 `FollowJointTrajectory` Goal 傳給 UR controller。
- Goal 傳出後，主畫面的執行按鈕會停用。
- 控制器接受 Goal 後，`Stop / Cancel Goal` 才會啟用。

### 5.7 確認視窗的 `Cancel`

- 關閉確認視窗。
- 不會送出 ROS 2 軌跡。
- 實體 UR3 不會因這次操作而移動。

### 5.8 `Stop / Cancel Goal`

- 對已被控制器接受的 ROS 2 Goal 發出非同步取消要求。
- 按鈕只會在 Goal 已被接受後啟用。
- 取消要求不等於安全級急停，也不能保證控制器立即停止。
- 發生危險或機器人未停止時，必須使用實體 Emergency Stop。

## 六、錯誤與狀態訊息說明

### 6.1 USD Stage、Robot Prim 與 Named Pose

| 顯示訊息 | 狀態 | 代表情況 | 建議處理 |
| --- | --- | --- | --- |
| `No active USD stage.` | 錯誤 | Isaac Sim 目前沒有開啟有效的 USD Stage。 | 開啟包含 UR3 的 USD 場景後再按 `Refresh`。 |
| `Robot prim not found: /World/ur3` | 錯誤 | Stage 中沒有 `/World/ur3`，或該路徑不是有效 Prim。 | 確認 UR3 的 Prim 路徑；若路徑不同，修改 `ROBOT_PRIM_PATH`。 |
| `No Robot Poser named poses found. Create and save a pose, then click Refresh.` | 警告 | `/World/ur3` 存在，但 Robot Poser 沒有為它儲存任何 Named Pose。 | 在 Robot Poser 完成 IK 求解並儲存 Named Pose，再按 `Refresh`。 |
| `No Robot Poser named pose is selected.` | 錯誤 | 清單是空的，或下拉選單索引無效。 | 先新增 Named Pose、按 `Refresh`，再選擇項目。 |
| `Named pose not found: <pose_name>` | 錯誤 | 畫面選取的 Pose 已被刪除、重新命名，或與目前 Stage 不一致。 | 按 `Refresh` 更新清單後重新選取。 |
| `Pose selection changed. Load and validate the IK solution.` | 資訊 | 使用者切換了 Named Pose，舊的待執行 IK 解已被清除。 | 按 `Load and Validate IK Solution` 載入新選項。 |
| `Found <N> Robot Poser named pose(s). Select one and load its IK solution.` | 成功 | 已找到可用 Named Pose，但尚未載入 IK 解。 | 選擇 Pose 並載入。 |

### 6.2 IK 解載入與驗證

| 顯示訊息 | 狀態 | 代表情況 | 建議處理 |
| --- | --- | --- | --- |
| `Named pose has an invalid IK result: <pose_name>` | 錯誤 | `pose.success` 為 `False`，Robot Poser 沒有成功求得此 Pose。 | 在 Robot Poser 調整目標姿態或初始姿態後重新求解並儲存。 |
| `IK result is missing UR joints: [...]` | 錯誤 | Named Pose 的 `pose.joints` 缺少一個或多個預期的 UR3 關節。 | 確認 Robot Poser 的目標是 `/World/ur3`，並檢查 USD 關節名稱是否與六個固定名稱一致。 |
| `IK result contains NaN or infinite values` | 錯誤 | 至少一個關節角為 `NaN`、`+∞` 或 `-∞`，不能作為軌跡目標。 | 重新求解 IK；同時檢查機器人模型、目標姿態及求解器狀態。 |
| `Cannot load another pose while a trajectory is executing.` | 警告 | 實體軌跡尚未結束，系統禁止替換待執行目標。 | 等待軌跡完成或取消後再載入。 |
| `Load and validate an IK solution first.` | 錯誤 | 尚未成功載入 IK 解，或切換／刷新 Pose 後舊解已失效。 | 按 `Load and Validate IK Solution`。 |
| `IK solution loaded and validated. Review the joint values before execution.` | 成功 | IK 解已通過目前程式的完整性與有限值檢查。 | 人工確認六個關節角後再執行。 |

### 6.3 ROS 2 與實體關節狀態

| 顯示或紀錄訊息 | 狀態 | 代表情況 | 建議處理 |
| --- | --- | --- | --- |
| `[UR3 Sync] ROS 2 initialization failed: <exception>` | Log 錯誤 | `rclpy` 初始化、ROS 2 Node、Action Client 或 Subscriber 建立失敗。 | 確認 Isaac Sim 的 ROS 2 Bridge、ROS 2 環境、訊息套件與 `ROS_DOMAIN_ID`。 |
| `[UR3 Sync] ROS 2 spin failed: <exception>` | Log 錯誤 | 非阻塞 `rclpy.spin_once()` 執行失敗，ROS callback 可能無法繼續更新。 | 查看 Console 例外內容，確認 Node／ROS context 尚未被其他程式關閉。 |
| `No valid /joint_states received from the physical UR3. Execution is blocked.` | 錯誤 | 尚未收到同時包含六個指定關節且數值有限的 JointState。 | 確認 UR driver 正在發布 `/joint_states`、名稱完整、ROS Domain 與 middleware 相容。 |
| `Action server is not ready: /scaled_joint_trajectory_controller/follow_joint_trajectory` | 錯誤 | Action Server 不存在、controller 未啟動、命名空間不同或 ROS 2 無法連線。 | 確認 scaled joint trajectory controller 為 active，並核對 Action 名稱。 |

特別注意：如果 `/joint_states` 訊息缺少任一預期關節，或其中有非有限值，callback 會直接忽略該訊息；介面最後會顯示「No valid `/joint_states` received」。

### 6.4 軌跡傳送與執行

| 顯示訊息 | 狀態 | 代表情況 | 建議處理 |
| --- | --- | --- | --- |
| `A trajectory is already executing.` | 警告 | 已有一筆軌跡正在執行，系統禁止重複送出。 | 等待完成或先取消目前 Goal。 |
| `Physical execution cancelled by the user.` | 資訊 | 使用者在確認視窗按下 `Cancel`，軌跡未送出。 | 不需處理；如要執行可重新按主畫面的執行按鈕。 |
| `Sending pose '<pose_name>' to the trajectory controller...` | 資訊 | 已開始呼叫 ROS 2 Action Server，正在等待是否接受 Goal。 | 等待控制器回覆。 |
| `Failed to send trajectory goal: <exception>` | 錯誤 | 呼叫 `send_goal_async()` 時立即發生例外，Goal 未正常送出。 | 查看例外內容，檢查 ROS context、Action Client 與 controller 連線。 |
| `Trajectory goal request failed: <exception>` | 錯誤 | 非同步 Goal request 已送出，但取得回覆時 Future 發生例外。 | 檢查 ROS 通訊、controller 是否中斷或重新啟動。 |
| `Trajectory goal was rejected by the UR controller.` | 錯誤 | Controller 有收到 Goal，但明確拒絕接受。 | 檢查 controller 狀態、機器人模式、關節名稱、關節限制、Protective Stop 與 driver log。 |
| `Goal accepted. Executing pose '<pose_name>'...` | 成功 | Controller 已接受 Goal，實體機器人可能正在移動。 | 監看機器人並保持 Emergency Stop 可立即操作。 |
| `Failed to receive trajectory result: <exception>` | 錯誤 | Goal 已接受，但取得最終執行結果時發生例外。 | 檢查 ROS 通訊與 controller 是否在執行期間中斷。機器人狀態不明時應先確保現場安全。 |
| `Pose '<pose_name>' completed successfully.` | 成功 | ROS Goal 狀態為 `SUCCEEDED`，且 `FollowJointTrajectory` 回報 `SUCCESSFUL`。 | 本次目標完成。 |
| `Pose '<pose_name>' was cancelled.` | 警告 | Controller 最終回報 Goal 已取消。 | 確認機器人已停止並檢查目前姿態。 |
| `Trajectory failed: status=<status>, error_code=<code>, message=<text>` | 錯誤 | Goal 未成功完成；訊息會保留 ROS Goal 狀態、trajectory error code 與 controller 說明。 | 依 `status`、`error_code` 和 UR driver log 進一步判斷。 |

`FollowJointTrajectory` 常見 `error_code`：

| error_code | 名稱 | 一般意義 |
| --- | --- | --- |
| `0` | `SUCCESSFUL` | 軌跡成功完成。 |
| `-1` | `INVALID_GOAL` | Goal 格式、時間或內容不合法。 |
| `-2` | `INVALID_JOINTS` | 關節名稱或關節集合不符合 controller 設定。 |
| `-3` | `OLD_HEADER_TIMESTAMP` | 軌跡時間戳已過期。 |
| `-4` | `PATH_TOLERANCE_VIOLATED` | 執行過程中偏離規劃路徑超過容許值。 |
| `-5` | `GOAL_TOLERANCE_VIOLATED` | 到達終點後仍未進入目標容許範圍。 |

常見 ROS Goal `status`：

| status | 名稱 | 一般意義 |
| --- | --- | --- |
| `0` | `UNKNOWN` | Goal 狀態未知。 |
| `1` | `ACCEPTED` | Goal 已接受但尚未開始執行。 |
| `2` | `EXECUTING` | Goal 執行中。 |
| `3` | `CANCELING` | 正在取消。 |
| `4` | `SUCCEEDED` | Goal 成功。 |
| `5` | `CANCELED` | Goal 已取消。 |
| `6` | `ABORTED` | Goal 被控制器中止。 |

### 6.5 Goal 取消

| 顯示或紀錄訊息 | 狀態 | 代表情況 | 建議處理 |
| --- | --- | --- | --- |
| `There is no accepted trajectory goal to cancel.` | 警告 | 目前沒有已被 controller 接受且可取消的 Goal。 | 不需取消；確認目前是否真的有實機動作。 |
| `Requesting trajectory cancellation. Use the emergency stop if the robot does not stop.` | 警告 | extension 已送出取消要求，但尚未取得 controller 確認。 | 持續監看；若未停止，立即使用實體 Emergency Stop。 |
| `Failed to request goal cancellation: <exception>` | 錯誤 | 呼叫 `cancel_goal_async()` 時立即失敗。 | 視為機器人可能仍在移動，必要時使用 Emergency Stop，並檢查 ROS 連線。 |
| `Cancel request failed: <exception>` | 錯誤 | 取消 request 已建立，但讀取非同步回覆時失敗。 | 視為取消結果不明；先確保現場安全，再檢查 controller。 |
| `Controller acknowledged the cancel request. Waiting for the final cancelled result.` | 警告 | Controller 接受取消要求，但尚未回報最終 `CANCELED` 狀態。 | 等待最終結果並目視確認機器人停止。 |
| `Controller did not accept the cancel request. The robot may still be moving; use the emergency stop if necessary.` | 錯誤 | 回覆中沒有任何 Goal 進入 canceling，取消要求未被接受。 | 將機器人視為仍可能移動；必要時立即按 Emergency Stop。 |
| `[UR3 Sync] Extension closed with an active goal; requesting cancellation` | Log 警告 | 關閉 extension 時仍有 Goal 執行中，程式嘗試送出取消要求。 | 不應依賴關閉 extension 來停止機器人；必須確認實體狀態。 |

## 七、目前限制與風險

1. 本 extension 不會自行重新計算 IK，也不會比較或選擇其他 IK 分支。
2. `BASE_PRIM_PATH` 與 `FLANGE_PRIM_PATH` 雖已定義，但目前沒有參與 IK 求解或驗證。
3. IK 驗證主要依賴 Robot Poser 的 `pose.success`、關節完整性與有限值檢查。
4. extension 沒有自行執行碰撞檢查、奇異點分析或完整的實體關節限制檢查。
5. 目前只傳送一個目標點，不包含多段路徑規劃或中間避障點。
6. `0.5 rad/s` 限制的是「目前位置到目標位置的規劃平均速度」，不是即時速度監控器。
7. `Stop / Cancel Goal` 是 ROS 2 Action 取消要求，不是安全級急停。
8. 模擬 UR3 的目前關節位置不會直接作為實體命令；實際命令來源是 Named Pose 中儲存的 IK 解。

## 八、建議操作流程

1. 啟動 UR ROS 2 driver，確認實體機器人可正常控制。
2. 確認 `/joint_states` 正常發布且包含六個 UR3 關節。
3. 確認 scaled joint trajectory controller 為 active。
4. 在 Isaac Sim 開啟包含 `/World/ur3` 的 USD Stage。
5. 在 Robot Poser 設定目標姿態並成功求解 IK。
6. 將 IK 結果儲存為 Named Pose。
7. 在 UR3 Robot Poser Executor 按下 `Refresh`。
8. 選擇 Named Pose。
9. 按下 `Load and Validate IK Solution`。
10. 人工檢查畫面上的六個關節角。
11. 設定 `Requested duration`。
12. 確認工作區域無人、無障礙物，Emergency Stop 在可立即操作的位置。
13. 按下 `Execute on Physical UR3`。
14. 在確認視窗再次核對目標關節角、最大角度差與執行時間。
15. 確認安全後按下確認視窗的 `Execute`。
16. 全程監看實體機器人的動作與 extension 狀態。

### 結論

本週完成的 UR3 Robot Poser Executor 已具備從 Robot Poser 取得 Named Pose IK 解、驗證六軸關節資料、依實體回授調整軌跡時間、人工二次確認及透過 ROS 2 執行實體 UR3 的完整流程。

其核心設計為：

> Robot Poser 負責「求解 IK」，UR3 Robot Poser Executor 負責「讀取、驗證、顯示與執行 IK 結果」。

此設計避免直接連續同步模擬關節位置到實體機器人，並透過明確的載入、檢查、確認與單次執行流程降低誤操作風險。不過，碰撞、奇異點、實體環境障礙及安全停止仍須由操作人員、Robot Poser、UR controller 與現場安全設備共同管理。


# Next Step

建議是把 **Robot Poser 當作「末端執行器操作與 IK 求解介面」**，而你的 UR3 extension 永遠只讀取模擬 articulation 的六軸關節角。這樣 `Current Pose` 和 `Live Stream` 能共用同一條資料路徑，也不用讓 extension 綁死某個 Named Pose。

```text
拖曳 Robot Poser 目標 Gizmo
            ↓
Robot Poser 求解 IK
            ↓
模擬 UR3 articulation 運動
            ↓
UR3 extension 讀取實際六軸 q_sim
        ↙                 ↘
Current Pose          Live Stream
單次確認執行           持續安全串流
```

## 第一階段：Current Simulation Pose

這部分建議優先實作，風險低很多，也能驗證 Robot Poser、模擬 articulation 和實體 UR3 的關節映射是否正確。

建議操作流程：

1. Isaac Sim Timeline 設為 `Play`。
2. 在 Robot Poser 建立一個可重複使用的，例如 `scratch_target`。
3. 開啟 Robot Poser 的 Target Tracking。
4. 拖曳 Robot Poser 的末端目標 Gizmo。
5. Robot Poser 求解 IK，模擬 UR3 移動到新位置。
6. 在你的 extension 選擇 `Current Simulation Pose`。
7. 按下 `Capture Current Pose`。
8. Extension 讀取 articulation 的實際六軸關節位置。
9. 顯示模擬目標、實體目前位置、最大角度差及執行時間。
10. 人工確認後，以 `FollowJointTrajectory` 執行一次。

你不需要在 UR3 extension 裡選擇 `scratch_target` 或其他 Named Pose。Robot Poser 裡的 Named Pose 只是提供拖曳目標與 IK 功能。

### UI 建議

不要再把所有功能都放進 `Named Pose` 下拉選單。建議增加明確的模式選擇：

```text
Control Mode:
  ○ Saved Named Pose
  ● Current Simulation Pose
  ○ Live Simulation Follow
```

Current Pose 模式提供：

```text
[Capture Current Pose]
[Execute on Physical UR3]

Simulation: PLAYING
IK/Articulation: VALID
Hardware Joint State: VALID
```

按下 `Capture` 後要保存一份不可變的關節快照。即使使用者之後繼續拖曳 Robot Poser，尚未確認的執行目標也不應暗中改變；必須重新按 `Capture`。

### 資料來源

應讀取：

```python
q_sim = articulation.get_dof_positions()
```

不要直接使用：

- Named Pose 儲存值；
- Robot Poser 的 IK 結果物件；
- articulation drive target；
- Gizmo 的 Cartesian transform。

原因是 `get_dof_positions()` 表示模擬機器人實際到達的位置，會保留模擬 drive 的速度和動態效果。

## 第二階段：Live Simulation Follow

Current Pose 完成並測試穩定後，再加入 Live Stream。

建議的使用流程：

1. 啟動 External Control。
2. 收到完整且持續更新的 `/joint_states`。
3. 將模擬 UR3 同步到實體 UR3 的目前姿態。
4. 確認：

```text
max(|q_sim - q_hardware|) <= 0.10 rad
```

5. 將 controller 從：

```text
scaled_joint_trajectory_controller
```

切換到：

```text
forward_position_controller
```

6. 使用者持續按住 `Hold to Enable Live Motion`。
7. 使用者拖曳 Robot Poser 的目標 Gizmo。
8. Robot Poser 求解 IK並驅動模擬 UR3。
9. Extension 擷取模擬 UR3 的實際關節運動。
10. 經過限速與安全檢查後發布到：

```text
/forward_position_controller/commands
```

### Live Mode UI 建議

```text
[Arm Live Mode]
[ HOLD TO ENABLE LIVE MOTION ]

Controller: FORWARD POSITION ACTIVE
Simulation/Hardware difference: 0.024 rad
Joint state age: 12 ms

LIVE CONTROL ACTIVE
```

我不建議使用一般的 On/Off 按鈕直接保持 Live Control。應使用 Deadman 操作：

- 按住才跟隨；
- 放開立刻停止接收新目標並保持當前位置；
- 視窗失去焦點時自動釋放；
- Timeline 停止時退出；
- IK 失敗時保持上一個安全命令；
- `/joint_states` 過期時退出；
- tracking error 過大時退出。

## Live Stream 的程式架構

不要在 Isaac Sim UI callback 裡直接高速發布 ROS command。建議分成兩條路徑。

### Isaac Sim／Physics 路徑

每個 Physics Update：

```text
讀取 q_sim
→ 依關節名稱排序
→ 檢查 finite values
→ 寫入 thread-safe latest-sample buffer
```

只允許 Isaac Sim thread 存取 articulation、USD 和 Physics API。

### ROS Publisher 路徑

獨立 ROS timer：

```text
讀取最新 q_sim
→ angle unwrap
→ 每週期位移限制
→ 速度限制
→ 加速度限制
→ tracking error 檢查
→ 發布 Float64MultiArray
```

第一版建議先用較保守的發布頻率測試，例如 `30–50 Hz`，確認穩定後再提高；CB3 的上限雖然是 `125 Hz`，但不代表第一版一定要直接使用最高頻率。

## Robot Poser 的限制

這個設計可行，但需要注意：

- 必須拖曳 Robot Poser 的目標 Gizmo，不是直接移動 `wrist_3_link` 或 `flange` prim。
- Robot Poser Tracking 必須啟用。
- Timeline 必須為 `Play`，IK 結果才會透過 DOF target 驅動模擬 articulation。
- 不可達位置會造成 IK 失敗；此時實體 UR3 不得繼續追逐無效目標。
- 奇異點附近可能出現大幅關節變化或 IK branch 切換，Live Mode 必須拒絕關節跳變。
- Robot Poser 可以保留一個 `scratch_target`，但 extension 不需要知道或選擇它的名稱。

## 最終建議順序

1. 先重構 extension，讓「目標關節位置」不再一定來自 Named Pose。
2. 實作 `Current Simulation Pose`。
3. 驗證六軸名稱、方向、零位及 `±2π` 表示。
4. 用 Robot Poser 拖曳末端目標，再執行 Current Pose。
5. 小幅度、逐關節測試實體 UR3。
6. 加入 Live Mode 狀態機與 controller 切換。
7. 加入 Deadman、watchdog、速度／加速度／跳變限制。
8. 最後才允許 Robot Poser end-effector dragging 控制實體 UR3。

總結來說：**Current Pose 可以做到「拖完後確認並執行」；Live Stream 可以做到「按住 Deadman 時，拖曳 Robot Poser 目標，實體手臂平順跟隨」。** 這是我最推薦的整體方式。