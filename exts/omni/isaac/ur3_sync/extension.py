"""Bridge Isaac Sim joint targets to a physical UR3 controller.

The operator loads a Robot Poser named pose or captures the planning
articulation's current positions, then sends one FollowJointTrajectory goal.
Joint positions are never continuously streamed.
"""

import math
import time

import carb
import carb.eventdispatcher
import omni.ext
import omni.kit.app
import omni.kit.window.popup_dialog
import omni.timeline
import omni.ui as ui
import omni.usd

import rclpy

from pxr import Usd

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from isaacsim.core.experimental.prims import Articulation
from isaacsim.robot.poser import (
    get_named_pose,
    list_named_poses,
    validate_robot_schema,
)

from .joint_targets import normalize_joint_positions
from .robot_selection import (
    DEFAULT_ROBOT_PRIM_PATH,
    format_selected_robot_label,
    resolve_robot_selection,
)


class Ur3SyncExtension(omni.ext.IExt):
    """載入模擬 UR3 關節目標，並將其傳送至實體 UR3。"""

    ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    JOINT_STATE_TOPIC = "/joint_states"

    TARGET_SOURCE_NAMED_POSE = "named_pose"
    TARGET_SOURCE_CURRENT_SIMULATION = "current_simulation"

    # UR3 data-sheet limits (feedback/ur3_us.pdf): the three arm joints are
    # rated for 180 deg/s and the three wrist joints for 360 deg/s.  The UI
    # exposes one speed cap for all joints, so its maximum must not exceed the
    # slowest joint's physical limit.
    UR3_RATED_JOINT_SPEEDS = (
        math.radians(180.0),
        math.radians(180.0),
        math.radians(180.0),
        math.radians(360.0),
        math.radians(360.0),
        math.radians(360.0),
    )
    MIN_COMMAND_SPEED = 0.05
    MAX_COMMAND_SPEED = min(UR3_RATED_JOINT_SPEEDS)
    DEFAULT_COMMAND_SPEED = 0.5
    MIN_TRAJECTORY_DURATION = 0.1

    # Feedback-based stall detection. This identifies a lack of progress; the
    # UR controller remains authoritative about collisions/Protective Stops.
    STALL_STARTUP_GRACE = 1.0
    STALL_TIMEOUT = 3.0
    STALL_MOVEMENT_THRESHOLD = 0.002
    STALL_GOAL_TOLERANCE = 0.01

    STATUS_INFO = 0xFFDDDDDD
    STATUS_OK = 0xFF66DD88
    STATUS_WARN = 0xFF44CCFF
    STATUS_ERROR = 0xFF5555FF

    def on_startup(self, ext_id):
        """初始化狀態、連接 ROS 2，並建立使用者介面。"""
        carb.log_info("[UR3 Sync] Extension starting")

        self.node = None
        self.client = None
        self._joint_state_sub = None
        self._app_update_sub = None
        self._stage_event_sub_opened = None
        self._stage_event_sub_assets_loaded = None
        self._window = None
        self._warning_dialog = None

        self._robot_paths = []
        self._selected_robot_path = None
        self._updating_robot_combo = False
        self._prefer_default_robot = True
        self._robot_refresh_pending = False
        self._pose_names = []
        self._updating_pose_combo = False
        self._pending_target_source = None
        self._pending_target_label = None
        self._pending_positions = None
        self._hardware_positions = None

        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._goal_handle = None
        self._is_executing = False
        self._active_target_label = None
        self._active_target_positions = None
        self._execution_watchdog_active = False
        self._execution_started_time = None
        self._last_motion_time = None
        self._last_motion_positions = None
        self._stall_detected = False
        self._stall_details = None
        self._cancel_reason = None

        self.ur_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        if not self._initialize_ros():
            return

        # Process ROS callbacks on each app update without blocking the UI.
        self._app_update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._on_app_update,
                name="ur3_sync_update",
            )
        )

        self._build_ui()
        self._subscribe_stage_events()
        self._refresh_robots_and_poses()

    # ------------------------------------------------------------------
    # ROS 2
    # ------------------------------------------------------------------

    def _initialize_ros(self):
        """建立 ROS 節點、Action 用戶端及關節狀態訂閱者。"""
        try:
            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node("isaacsim_ur3_sync_extension")
            self.client = ActionClient(
                self.node,
                FollowJointTrajectory,
                self.ACTION_NAME,
            )
            self._joint_state_sub = self.node.create_subscription(
                JointState,
                self.JOINT_STATE_TOPIC,
                self._on_joint_state,
                10,
            )
            return True
        except Exception as exc:
            carb.log_error(f"[UR3 Sync] ROS 2 initialization failed: {exc}")
            return False

    def _on_joint_state(self, msg):
        """依控制器順序快取完整且為有限值的 UR3 關節狀態樣本。"""
        # JointState ordering is not guaranteed; normalize to controller order.
        positions_by_name = dict(zip(msg.name, msg.position))
        if not all(name in positions_by_name for name in self.ur_joint_names):
            return

        positions = [
            float(positions_by_name[name])
            for name in self.ur_joint_names
        ]
        if all(math.isfinite(value) for value in positions):
            self._hardware_positions = positions
            self._monitor_execution_stall(positions)

    def _on_app_update(self, event):
        """每次 Isaac Sim 更新時推進一次 ROS 回呼。"""
        del event
        if self.node is None or not rclpy.ok():
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            carb.log_error(f"[UR3 Sync] ROS 2 spin failed: {exc}")

        if self._hardware_positions is not None:
            self._monitor_execution_stall(self._hardware_positions)

    # ------------------------------------------------------------------
    # USD Stage lifecycle
    # ------------------------------------------------------------------

    def _subscribe_stage_events(self):
        """在 Stage 開啟及資產載入完成時重新掃描可用機器人。"""
        usd_context = omni.usd.get_context()
        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._stage_event_sub_opened = dispatcher.observe_event(
            event_name=usd_context.stage_event_name(
                omni.usd.StageEventType.OPENED
            ),
            on_event=self._on_stage_changed,
            observer_name="omni.isaac.ur3_sync.stage_opened",
        )
        self._stage_event_sub_assets_loaded = dispatcher.observe_event(
            event_name=usd_context.stage_event_name(
                omni.usd.StageEventType.ASSETS_LOADED
            ),
            on_event=self._on_stage_changed,
            observer_name="omni.isaac.ur3_sync.assets_loaded",
        )

    def _on_stage_changed(self, event):
        """Stage 變更後更新 robots 與目前 robot 的 Named Poses。"""
        del event
        if self._is_executing:
            self._robot_refresh_pending = True
            return
        self._refresh_robots_and_poses()

    # ------------------------------------------------------------------
    # 實機執行監控
    # ------------------------------------------------------------------

    def _target_error(self, positions):
        """傳回目前回授到作用中目標的最大關節誤差。"""
        if self._active_target_positions is None:
            return None
        return max(
            abs(target - current)
            for target, current in zip(
                self._active_target_positions,
                positions,
            )
        )

    def _start_execution_watchdog(self):
        """在 Action goal 被接受後啟動關節進度監控。"""
        now = time.perf_counter()
        self._execution_watchdog_active = True
        self._execution_started_time = now
        self._last_motion_time = now
        self._last_motion_positions = (
            list(self._hardware_positions)
            if self._hardware_positions is not None
            else None
        )
        self._stall_detected = False
        self._stall_details = None
        self._cancel_reason = None

    def _reset_execution_watchdog(self):
        """清除實機關節進度監控狀態。"""
        self._execution_watchdog_active = False
        self._execution_started_time = None
        self._last_motion_time = None
        self._last_motion_positions = None
        self._active_target_positions = None
        self._stall_detected = False
        self._stall_details = None
        self._cancel_reason = None

    def _monitor_execution_stall(self, positions):
        """以關節回授偵測尚未到站但長時間沒有進展的執行。"""
        if (
            not self._execution_watchdog_active
            or not self._is_executing
            or self._goal_handle is None
            or self._stall_detected
        ):
            return

        target_error = self._target_error(positions)
        if target_error is None:
            return
        if target_error <= self.STALL_GOAL_TOLERANCE:
            self._execution_watchdog_active = False
            return

        now = time.perf_counter()
        if self._last_motion_positions is None:
            self._last_motion_positions = list(positions)
            self._last_motion_time = now
            return

        movement = max(
            abs(current - previous)
            for current, previous in zip(
                positions,
                self._last_motion_positions,
            )
        )
        if movement >= self.STALL_MOVEMENT_THRESHOLD:
            self._last_motion_positions = list(positions)
            self._last_motion_time = now
            return

        if now - self._execution_started_time < self.STALL_STARTUP_GRACE:
            return
        stalled_for = now - self._last_motion_time
        if stalled_for < self.STALL_TIMEOUT:
            return

        self._handle_suspected_stall(target_error, stalled_for)

    def _handle_suspected_stall(self, target_error, stalled_for):
        """警告使用者並取消疑似卡住的實機軌跡。"""
        self._stall_detected = True
        self._execution_watchdog_active = False
        self._cancel_reason = "stall"
        self._stall_details = (
            f"No meaningful joint motion was detected for "
            f"{stalled_for:.1f} s while the maximum remaining error was "
            f"{target_error:.3f} rad."
        )

        warning = (
            "The physical UR3 appears to have stopped before reaching the "
            "goal. A collision, Protective Stop, paused speed slider, or "
            "controller fault may have occurred.\n\n"
            f"{self._stall_details}\n\n"
            "A trajectory cancellation has been requested. Keep the "
            "workspace clear and use the emergency stop if the robot is "
            "still moving. Inspect and reset the UR controller before "
            "trying again."
        )
        self._set_status(
            "Possible collision or motion stall detected. Cancelling the "
            "trajectory goal...",
            self.STATUS_ERROR,
        )
        self._show_motion_warning(warning)

        self.stop_btn.enabled = False
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(
                self._on_cancel_response
            )
        except Exception as exc:
            self._set_status(
                "Motion stall detected, but goal cancellation failed: "
                f"{exc}. Use the emergency stop if necessary.",
                self.STATUS_ERROR,
            )

    # ------------------------------------------------------------------
    # 使用者介面
    # ------------------------------------------------------------------

    def _build_ui(self):
        """建立姿勢選擇與受保護執行流程的視窗。"""
        self._window = ui.Window(
            "UR3 Robot Poser Execution",
            width=470,
            height=635,
        )

        with self._window.frame:
            with ui.VStack(spacing=10, padding=12):
                ui.Label(
                    "Simulation Target => Physical UR3",
                    style={"font_size": 16, "color": 0xFFFFAA00},
                )

                with ui.HStack(height=28, spacing=8):
                    ui.Label("Active Robot:", width=90)
                    self.robot_combo = ui.ComboBox(
                        0,
                        "None",
                        width=0,
                    )
                    self.robot_combo.model.add_item_changed_fn(
                        self._on_robot_selection_changed
                    )
                    self.refresh_btn = ui.Button(
                        "Refresh",
                        width=80,
                        clicked_fn=self._refresh_robots_and_poses,
                    )

                self.selected_robot_label = ui.Label(
                    format_selected_robot_label(None),
                    word_wrap=True,
                    height=22,
                    style={"font_size": 12, "color": 0xFFFFCC66},
                )

                with ui.HStack(height=28, spacing=8):
                    ui.Label("Named Pose:", width=90)
                    self.pose_combo = ui.ComboBox(
                        0,
                        "<no named poses>",
                        width=0,
                    )
                    self.pose_combo.model.add_item_changed_fn(
                        self._on_pose_selection_changed
                    )

                self.selected_pose_label = ui.Label(
                    "Selected pose: <none>",
                    word_wrap=True,
                    height=22,
                    style={"font_size": 12, "color": 0xFFFFCC66},
                )

                with ui.HStack(height=36, spacing=10):
                    self.load_btn = ui.Button(
                        "Load and Validate IK Solution",
                        height=36,
                        clicked_fn=self._on_load_clicked,
                    )

                    self.get_current_btn = ui.Button(
                        "Get Current Simulation Pose",
                        height=36,
                        clicked_fn=self._on_get_current_clicked,
                        tooltip=(
                            "Capture the actual PhysX joint positions of "
                            "the selected Active Robot. The Timeline must be "
                            "playing."
                        ),
                    )

                with ui.HStack(height=28, spacing=8):
                    ui.Label("Joint speed limit:", width=125)
                    self.speed_slider = ui.FloatSlider(
                        min=self.MIN_COMMAND_SPEED,
                        max=self.MAX_COMMAND_SPEED,
                        step=0.01,
                        width=ui.Fraction(1),
                        tooltip=(
                            "0.05-pi rad/s. The upper bound is the UR3 "
                            "arm-joint rating of 180 deg/s."
                        ),
                    )
                    self.speed_slider.model.set_value(
                        self.DEFAULT_COMMAND_SPEED
                    )
                    self.speed_value_label = ui.Label(
                        self._format_speed(self.DEFAULT_COMMAND_SPEED),
                        width=100,
                        alignment=ui.Alignment.RIGHT_CENTER,
                    )
                    self.speed_slider.model.add_value_changed_fn(
                        self._on_speed_changed
                    )

                ui.Label(
                    "Motion time is calculated automatically from the "
                    "largest joint delta and selected speed.",
                    word_wrap=True,
                    style={"font_size": 11, "color": 0xFFAAAAAA},
                )

                self.solution_label = ui.Label(
                    "No target loaded.",
                    word_wrap=True,
                    height=82,
                    style={"font_size": 12, "color": 0xFFBBBBBB},
                )

                with ui.HStack(height=40, spacing=10):
                    self.execute_btn = ui.Button(
                        "Execute on Physical UR3",
                        clicked_fn=self._on_execute_clicked,
                    )
                    self.execute_btn.enabled = False

                    self.stop_btn = ui.Button(
                        "Cancel Goal",
                        clicked_fn=self._on_stop_clicked,
                    )
                    self.stop_btn.enabled = False

                ui.Separator()
                ui.Label("Status")
                self.status_label = ui.Label(
                    "Load a named pose or capture the current simulation "
                    "pose.",
                    word_wrap=True,
                    height=65,
                    style={"font_size": 12, "color": self.STATUS_INFO},
                )

    def _set_status(self, message, color=None):
        """記錄狀態訊息，並同步顯示於擴充功能視窗。"""
        if color is None:
            color = self.STATUS_INFO

        carb.log_info(f"[UR3 Sync] {message}")
        if getattr(self, "status_label", None) is not None:
            self.status_label.text = message
            self.status_label.style = {
                "font_size": 12,
                "color": color,
            }

    @staticmethod
    def _format_speed(speed):
        """將速度格式化為 UI 顯示字串。"""
        return f"{speed:.2f} rad/s"

    def _on_speed_changed(self, model):
        """拖曳速度滑桿時更新數值標籤。"""
        speed = float(model.get_value_as_float())
        if getattr(self, "speed_value_label", None) is not None:
            self.speed_value_label.text = self._format_speed(speed)

    def _get_command_speed(self):
        """讀取並防禦性限制使用者所選的關節速度。"""
        speed = float(self.speed_slider.model.get_value_as_float())
        if not math.isfinite(speed):
            speed = self.DEFAULT_COMMAND_SPEED

        clamped_speed = min(
            self.MAX_COMMAND_SPEED,
            max(self.MIN_COMMAND_SPEED, speed),
        )
        if clamped_speed != speed:
            self.speed_slider.model.set_value(clamped_speed)
        return clamped_speed

    @classmethod
    def _calculate_motion_duration(
        cls,
        current_positions,
        target_positions,
        speed,
    ):
        """依最大關節位移與速度上限計算單點軌跡時間。"""
        if not math.isfinite(speed) or not (
            cls.MIN_COMMAND_SPEED <= speed <= cls.MAX_COMMAND_SPEED
        ):
            raise ValueError("Command speed is outside the supported range")
        if len(current_positions) != len(target_positions):
            raise ValueError("Current and target joint counts do not match")

        max_delta = max(
            (
                abs(target - current)
                for target, current in zip(
                    target_positions,
                    current_positions,
                )
            ),
            default=0.0,
        )
        duration = max(
            cls.MIN_TRAJECTORY_DURATION,
            max_delta / speed,
        )
        return max_delta, duration

    def _discover_robot_paths(self, stage):
        """傳回目前 Stage 中非 prototype 的 IsaacRobotAPI prim paths。"""
        return [
            str(prim.GetPath())
            for prim in Usd.PrimRange(stage.GetPseudoRoot())
            if validate_robot_schema(prim) and not prim.IsInPrototype()
        ]

    def _replace_robot_combo_items(self, paths, selected_path):
        """重建 Active Robot 選單並同步所選完整 prim path。"""
        self._updating_robot_combo = True
        try:
            model = self.robot_combo.model
            for child in list(model.get_item_children()):
                model.remove_item(child)

            for path in ["None", *paths]:
                model.append_child_item(None, ui.SimpleStringModel(path))

            selected_index = (
                paths.index(selected_path) + 1
                if selected_path in paths
                else 0
            )
            model.get_item_value_model().set_value(selected_index)
        finally:
            self._updating_robot_combo = False
        self._update_selected_robot_label()

    def _update_selected_robot_label(self):
        """在下拉選單之外清楚顯示目前 Active Robot。"""
        if getattr(self, "selected_robot_label", None) is not None:
            self.selected_robot_label.text = format_selected_robot_label(
                self._selected_robot_path
            )

    def _refresh_robots_and_poses(self):
        """重新掃描 robots，保留有效舊選擇，並更新其 Named Poses。"""
        if self._is_executing:
            self._robot_refresh_pending = True
            return

        self._robot_refresh_pending = False
        self._invalidate_pending_target()
        stage = omni.usd.get_context().get_stage()
        try:
            discovered_paths = (
                self._discover_robot_paths(stage)
                if stage is not None
                else []
            )
        except Exception as exc:
            self._robot_paths = []
            self._selected_robot_path = None
            self._pose_names = []
            self._replace_robot_combo_items([], None)
            self._replace_pose_combo_items([])
            self._set_status(
                f"Failed to scan Active Robots: {exc}",
                self.STATUS_ERROR,
            )
            return
        self._robot_paths, selected_path = resolve_robot_selection(
            discovered_paths,
            self._selected_robot_path,
            prefer_default=self._prefer_default_robot,
            default_path=DEFAULT_ROBOT_PRIM_PATH,
        )
        self._prefer_default_robot = False
        self._selected_robot_path = selected_path
        self._replace_robot_combo_items(
            self._robot_paths,
            self._selected_robot_path,
        )
        self._refresh_pose_names()

    def _on_robot_selection_changed(self, model, item):
        """切換 Active Robot 後使舊 target 失效並載入新 poses。"""
        del model, item
        if self._updating_robot_combo:
            return

        index = (
            self.robot_combo.model
            .get_item_value_model()
            .get_value_as_int()
        )
        selected_path = (
            self._robot_paths[index - 1]
            if 1 <= index <= len(self._robot_paths)
            else None
        )
        if selected_path == self._selected_robot_path:
            return

        self._selected_robot_path = selected_path
        self._update_selected_robot_label()
        self._invalidate_pending_target()
        self._refresh_pose_names()

    def _get_selected_robot_prim(self, stage=None):
        """傳回所選 robot prim；Stage 或選擇無效時丟出可操作錯誤。"""
        if stage is None:
            stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No active USD stage")
        if self._selected_robot_path is None:
            raise RuntimeError("No Active Robot is selected")

        robot_prim = stage.GetPrimAtPath(self._selected_robot_path)
        if not robot_prim.IsValid():
            raise RuntimeError(
                f"Robot prim not found: {self._selected_robot_path}"
            )
        if (
            not validate_robot_schema(robot_prim)
            or robot_prim.IsInPrototype()
        ):
            raise RuntimeError(
                "Selected robot is no longer a valid non-prototype "
                "IsaacRobotAPI prim. Refresh and select an Active Robot "
                "again"
            )
        return robot_prim

    def _replace_pose_combo_items(self, names):
        """取代所有姿勢選單項目，且不觸發選取邏輯。"""
        # Suppress selection callbacks while rebuilding the model.
        self._updating_pose_combo = True
        try:
            model = self.pose_combo.model
            for child in list(model.get_item_children()):
                model.remove_item(child)

            display_names = names if names else ["<no named poses>"]
            for name in display_names:
                model.append_child_item(None, ui.SimpleStringModel(name))

            model.get_item_value_model().set_value(0)
        finally:
            self._updating_pose_combo = False
        self._update_selected_pose_label()

    def _update_selected_pose_label(self):
        """在下拉選單之外清楚顯示目前選取的 Named Pose。"""
        pose_name = self._get_selected_pose_name()
        display_name = pose_name if pose_name is not None else "<none>"
        if getattr(self, "selected_pose_label", None) is not None:
            self.selected_pose_label.text = (
                f"Selected pose: {display_name}"
            )

    def _refresh_pose_names(self):
        """重新掃描目前 Active Robot 的 Robot Poser 命名解。"""
        self._invalidate_pending_target()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._pose_names = []
            self._replace_pose_combo_items([])
            self._set_status("No active USD stage.", self.STATUS_ERROR)
            return

        if self._selected_robot_path is None:
            self._pose_names = []
            self._replace_pose_combo_items([])
            self._set_status(
                "No Active Robot selected. Select a robot before loading "
                "or capturing a target.",
                self.STATUS_WARN,
            )
            return

        try:
            robot_prim = self._get_selected_robot_prim(stage)
        except RuntimeError as exc:
            self._pose_names = []
            self._replace_pose_combo_items([])
            self._set_status(str(exc), self.STATUS_ERROR)
            return

        try:
            self._pose_names = sorted(
                list_named_poses(stage, robot_prim)
            )
        except Exception as exc:
            self._pose_names = []
            self._replace_pose_combo_items([])
            self._set_status(
                f"Failed to scan Robot Poser named poses for "
                f"{self._selected_robot_path}: {exc}",
                self.STATUS_ERROR,
            )
            return
        self._replace_pose_combo_items(self._pose_names)

        if self._pose_names:
            self._set_status(
                f"Found {len(self._pose_names)} Robot Poser named pose(s) "
                f"for {self._selected_robot_path}. "
                "Load one, or capture the current simulation pose.",
                self.STATUS_OK,
            )
        else:
            self._set_status(
                f"No Robot Poser named poses found for "
                f"{self._selected_robot_path}. You can still capture "
                "the current simulation pose while the Timeline is playing.",
                self.STATUS_WARN,
            )

    def _on_pose_selection_changed(self, model, item):
        """使用者選擇其他姿勢後，捨棄已載入的目標。"""
        del model, item
        if self._updating_pose_combo:
            return
        self._update_selected_pose_label()
        self._invalidate_pending_target()
        self._set_status(
            "Pose selection changed. Load and validate the IK solution.",
            self.STATUS_INFO,
        )

    def _get_selected_pose_name(self):
        """傳回所選的已儲存姿勢名稱；若無有效項目則傳回 None。"""
        if not self._pose_names:
            return None

        index = (
            self.pose_combo.model
            .get_item_value_model()
            .get_value_as_int()
        )
        if not 0 <= index < len(self._pose_names):
            return None
        return self._pose_names[index]

    def _invalidate_pending_target(self):
        """清除已驗證或已擷取的目標，並停用實體執行。"""
        self._pending_target_source = None
        self._pending_target_label = None
        self._pending_positions = None

        if getattr(self, "execute_btn", None) is not None:
            self.execute_btn.enabled = False
        if getattr(self, "solution_label", None) is not None:
            self.solution_label.text = "No target loaded."

    def _format_joint_positions(self, positions):
        """依 controller 順序格式化六個關節值。"""
        return "  ".join(
            f"{name.replace('_joint', '')}={value:+.3f}"
            for name, value in zip(self.ur_joint_names, positions)
        )

    def _set_pending_target(self, source, label, positions):
        """儲存一份不可由後續模擬變更影響的關節目標快照。"""
        self._pending_target_source = source
        self._pending_target_label = label
        self._pending_positions = list(positions)
        self.execute_btn.enabled = True

    # ------------------------------------------------------------------
    # Robot Poser 結果處理
    # ------------------------------------------------------------------

    def _load_named_pose_positions(self, pose_name):
        """載入一個成功求解的姿勢，並傳回依序排列的六個關節位置。"""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No active USD stage")

        robot_prim = self._get_selected_robot_prim(stage)

        pose = get_named_pose(stage, robot_prim, pose_name)
        if pose is None:
            raise RuntimeError(f"Named pose not found: {pose_name}")
        if not pose.success:
            raise RuntimeError(
                f"Named pose has an invalid IK result: {pose_name}"
            )

        # Pose joints are stored by prim path; the controller expects names.
        positions_by_name = {}
        for joint_path, value in pose.joints.items():
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                joint_name = joint_prim.GetName()
                positions_by_name[joint_name] = float(value)

        missing = [
            name
            for name in self.ur_joint_names
            if name not in positions_by_name
        ]
        # Never send an incomplete IK solution to the physical robot.
        if missing:
            raise RuntimeError(
                f"IK result is missing UR joints: {missing}"
            )

        positions = [
            positions_by_name[name]
            for name in self.ur_joint_names
        ]
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("IK result contains NaN or infinite values")

        return positions

    def _on_load_clicked(self):
        """驗證所選 IK 姿勢，並使其可供執行。"""
        if self._is_executing:
            self._set_status(
                "Cannot load another pose while a trajectory is executing.",
                self.STATUS_WARN,
            )
            return

        pose_name = self._get_selected_pose_name()
        if pose_name is None:
            self._set_status(
                "No Robot Poser named pose is selected.",
                self.STATUS_ERROR,
            )
            return

        try:
            positions = self._load_named_pose_positions(pose_name)
        except Exception as exc:
            self._invalidate_pending_target()
            self._set_status(str(exc), self.STATUS_ERROR)
            return

        self._set_pending_target(
            self.TARGET_SOURCE_NAMED_POSE,
            pose_name,
            positions,
        )
        self.solution_label.text = (
            "Source: Robot Poser Named Pose\n"
            f"Target: {pose_name}\n"
            "Joint positions (rad): "
            f"{self._format_joint_positions(positions)}"
        )
        self._set_status(
            "IK solution loaded and validated. Review the target joints "
            "before physical execution.",
            self.STATUS_OK,
        )

    # ------------------------------------------------------------------
    # 目前模擬姿勢擷取
    # ------------------------------------------------------------------

    def _read_current_simulation_positions(self):
        """讀取規劃用 articulation 當下真正到達的六軸關節位置。"""
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No active USD stage")

        self._get_selected_robot_prim(stage)

        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            raise RuntimeError(
                "Start the Isaac Sim Timeline before capturing the current "
                "simulation pose"
            )

        articulation = Articulation(self._selected_robot_path)
        if not articulation.is_physics_tensor_entity_valid():
            raise RuntimeError(
                "The planning articulation is not ready. Keep the "
                "Timeline playing, wait one simulation frame, and try "
                "Get Current again"
            )

        position_data = articulation.get_dof_positions()
        position_rows = position_data.to("cpu").numpy()
        return normalize_joint_positions(
            articulation.dof_names,
            position_rows,
            self.ur_joint_names,
        )

    def _on_get_current_clicked(self):
        """擷取規劃用模擬 UR3 的實際姿勢，作為下一個實機目標。"""
        if self._is_executing:
            self._set_status(
                "Cannot capture a simulation pose while a trajectory is "
                "executing.",
                self.STATUS_WARN,
            )
            return

        # A failed capture must not leave an older target executable.
        self._invalidate_pending_target()
        try:
            positions = self._read_current_simulation_positions()
        except Exception as exc:
            self._set_status(str(exc), self.STATUS_ERROR)
            return

        target_label = "Current Simulation Snapshot"
        self._set_pending_target(
            self.TARGET_SOURCE_CURRENT_SIMULATION,
            target_label,
            positions,
        )
        self.solution_label.text = (
            "Source: Current Simulation Pose\n"
            f"Target: {target_label}\n"
            "Joint positions (rad): "
            f"{self._format_joint_positions(positions)}"
        )
        self._set_status(
            "Current simulation pose captured. Review the six-joint "
            "snapshot before physical execution.",
            self.STATUS_OK,
        )

    # ------------------------------------------------------------------
    # 軌跡執行
    # ------------------------------------------------------------------

    def _on_execute_clicked(self):
        """檢查執行前提並送出目標。"""
        if self._is_executing:
            self._set_status(
                "A trajectory is already executing.",
                self.STATUS_WARN,
            )
            return

        if (
            self._pending_target_source is None
            or self._pending_target_label is None
            or self._pending_positions is None
        ):
            self._set_status(
                "Load a named pose or capture the current simulation pose "
                "first.",
                self.STATUS_ERROR,
            )
            return

        if self._hardware_positions is None:
            self._set_status(
                "No valid /joint_states received from the physical UR3. "
                "Execution is blocked.",
                self.STATUS_ERROR,
            )
            return

        if self.client is None or not self.client.server_is_ready():
            self._set_status(
                f"Action server is not ready: {self.ACTION_NAME}",
                self.STATUS_ERROR,
            )
            return

        speed = self._get_command_speed()
        _, duration = self._calculate_motion_duration(
            self._hardware_positions,
            self._pending_positions,
            speed,
        )

        # Capture an immutable snapshot so later UI changes cannot alter it.
        target_label = self._pending_target_label
        target_positions = list(self._pending_positions)
        self._send_trajectory_goal(
            target_positions,
            duration,
            target_label,
        )

    def _show_motion_warning(self, message):
        """顯示實機未正常完成運動的模態警告。"""
        self._dismiss_warning_dialog()

        def _handle_acknowledge(dialog):
            dialog.hide()
            self._warning_dialog = None

        self._warning_dialog = (
            omni.kit.window.popup_dialog.MessageDialog(
                title="Physical UR3 Motion Warning",
                message=message,
                warning_message=(
                    "The trajectory did not complete normally."
                ),
                ok_label="Acknowledge",
                disable_cancel_button=True,
                ok_handler=_handle_acknowledge,
            )
        )
        self._warning_dialog.show()

    def _dismiss_warning_dialog(self):
        """隱藏並釋放實機運動警告（若存在）。"""
        dialog = self._warning_dialog
        if dialog is None:
            return

        self._warning_dialog = None
        try:
            dialog.hide()
        except Exception:
            pass

    def _send_trajectory_goal(
        self,
        target_positions,
        duration_sec,
        target_label,
    ):
        """建立並非同步傳送一個單點軌跡目標。"""
        # Recheck defensively in case server state changed after validation.
        if self.client is None or not self.client.server_is_ready():
            self._set_status(
                f"Action server is not ready: {self.ACTION_NAME}",
                self.STATUS_ERROR,
            )
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.ur_joint_names)

        point = JointTrajectoryPoint()
        point.positions = [
            float(value)
            for value in target_positions
        ]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )
        goal.trajectory.points.append(point)

        self._active_target_positions = list(target_positions)
        self._active_target_label = target_label
        self._is_executing = True
        self.execute_btn.enabled = False
        self.load_btn.enabled = False
        self.get_current_btn.enabled = False
        self.robot_combo.enabled = False
        self.refresh_btn.enabled = False
        self.speed_slider.enabled = False
        self.stop_btn.enabled = False

        self._set_status(
            f"Sending target '{target_label}' to the trajectory controller...",
            self.STATUS_INFO,
        )

        try:
            self._send_future = self.client.send_goal_async(goal)
            self._send_future.add_done_callback(
                self._on_goal_response
            )
        except Exception as exc:
            self._finish_execution_state()
            self._set_status(
                f"Failed to send trajectory goal: {exc}",
                self.STATUS_ERROR,
            )

    def _on_goal_response(self, future):
        """處理軌跡目標被接受或拒絕的結果。"""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_execution_state()
            self._set_status(
                f"Trajectory goal request failed: {exc}",
                self.STATUS_ERROR,
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finish_execution_state()
            self._set_status(
                "Trajectory goal was rejected by the UR controller.",
                self.STATUS_ERROR,
            )
            return

        self._goal_handle = goal_handle
        self._start_execution_watchdog()
        self.stop_btn.enabled = True
        self._set_status(
            "Goal accepted. Executing target "
            f"'{self._active_target_label}'...",
            self.STATUS_OK,
        )

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """回報最終 Action 狀態與控制器結果。"""
        try:
            response = future.result()
            result = response.result
            status = response.status
        except Exception as exc:
            self._finish_execution_state()
            message = f"Failed to receive trajectory result: {exc}"
            self._set_status(message, self.STATUS_ERROR)
            self._show_motion_warning(
                f"{message}\n\nThe physical robot state is uncertain. "
                "Keep the workspace clear, inspect the UR controller, and "
                "use the emergency stop if the robot is still moving."
            )
            return

        target_label = self._active_target_label or "<unknown>"
        stall_detected = self._stall_detected
        stall_details = self._stall_details
        cancel_reason = self._cancel_reason
        target_error = (
            self._target_error(self._hardware_positions)
            if self._hardware_positions is not None
            else None
        )
        self._finish_execution_state()

        # The trajectory controller is authoritative for final completion: it
        # evaluates its configured goal tolerances before returning SUCCESSFUL.
        # Do not override that result with the latest /joint_states sample.
        # JointState and action callbacks are asynchronous, so the cached sample
        # can still be from just before the robot entered the goal tolerance and
        # would incorrectly turn a completed motion into an error.
        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self._set_status(
                f"Target '{target_label}' completed successfully.",
                self.STATUS_OK,
            )
        elif status == GoalStatus.STATUS_CANCELED:
            if stall_detected or cancel_reason == "stall":
                details = stall_details or (
                    "The motion stopped before reaching the goal."
                )
                self._set_status(
                    f"Target '{target_label}' was cancelled after a suspected "
                    f"motion stall. {details}",
                    self.STATUS_ERROR,
                )
            else:
                self._set_status(
                    f"Target '{target_label}' was cancelled.",
                    self.STATUS_WARN,
                )
        else:
            message = (
                f"Trajectory failed: status={status}, "
                f"error_code={result.error_code}, "
                f"message={result.error_string}"
            )
            if target_error is not None:
                message += (
                    ". Maximum remaining joint error: "
                    f"{target_error:.3f} rad"
                )
            self._set_status(message, self.STATUS_ERROR)
            self._show_motion_warning(
                "The physical trajectory was aborted or failed before a "
                "normal completion. A collision, Protective Stop, or "
                "controller fault may have occurred.\n\n"
                f"{message}\n\n"
                "Keep the workspace clear, inspect the UR controller, and "
                "do not execute another motion until the cause is resolved."
            )

    def _on_stop_clicked(self):
        """要求取消目前已接受的軌跡目標。"""
        if self._goal_handle is None:
            self._set_status(
                "There is no accepted trajectory goal to cancel.",
                self.STATUS_WARN,
            )
            return

        self.stop_btn.enabled = False
        self._execution_watchdog_active = False
        self._cancel_reason = "user"
        # Software cancellation is not an emergency stop.
        self._set_status(
            "Requesting trajectory cancellation. Use the emergency stop "
            "if the robot does not stop.",
            self.STATUS_WARN,
        )

        try:
            self._cancel_future = (
                self._goal_handle.cancel_goal_async()
            )
            self._cancel_future.add_done_callback(
                self._on_cancel_response
            )
        except Exception as exc:
            self.stop_btn.enabled = True
            self._set_status(
                f"Failed to request goal cancellation: {exc}",
                self.STATUS_ERROR,
            )

    def _on_cancel_response(self, future):
        """回報控制器是否接受取消要求。"""
        # A late cancel response must not overwrite an already final result.
        if not self._is_executing:
            return

        try:
            response = future.result()
            cancel_count = len(response.goals_canceling)
        except Exception as exc:
            self.stop_btn.enabled = self._cancel_reason != "stall"
            self._set_status(
                f"Cancel request failed: {exc}",
                self.STATUS_ERROR,
            )
            return

        if cancel_count:
            if self._cancel_reason == "stall":
                self._set_status(
                    "Controller acknowledged the automatic cancellation "
                    "after a suspected stall. Waiting for the final "
                    "cancelled result.",
                    self.STATUS_ERROR,
                )
            else:
                self._set_status(
                    "Controller acknowledged the cancel request. Waiting "
                    "for the final cancelled result.",
                    self.STATUS_WARN,
                )
        else:
            self.stop_btn.enabled = self._cancel_reason != "stall"
            self._set_status(
                "Controller did not accept the cancel request. The robot may "
                "still be moving; use the emergency stop if necessary.",
                self.STATUS_ERROR,
            )

    def _finish_execution_state(self):
        """取得最終結果後清除 Action 狀態並還原控制項。"""
        self._is_executing = False
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._active_target_label = None
        self._reset_execution_watchdog()

        if getattr(self, "execute_btn", None) is not None:
            self.execute_btn.enabled = self._pending_positions is not None
        if getattr(self, "load_btn", None) is not None:
            self.load_btn.enabled = True
        if getattr(self, "get_current_btn", None) is not None:
            self.get_current_btn.enabled = True
        if getattr(self, "robot_combo", None) is not None:
            self.robot_combo.enabled = True
        if getattr(self, "refresh_btn", None) is not None:
            self.refresh_btn.enabled = True
        if getattr(self, "speed_slider", None) is not None:
            self.speed_slider.enabled = True
        if getattr(self, "stop_btn", None) is not None:
            self.stop_btn.enabled = False

        if self._robot_refresh_pending:
            self._refresh_robots_and_poses()

    # ------------------------------------------------------------------
    # 關閉
    # ------------------------------------------------------------------

    def on_shutdown(self):
        """取消作用中的工作，並釋放介面與 ROS 資源。"""
        carb.log_info("[UR3 Sync] Extension shutting down")
        self._dismiss_warning_dialog()

        # Best-effort cancellation reduces the risk of unmanaged motion.
        if self._goal_handle is not None:
            carb.log_warn(
                "[UR3 Sync] Extension closed with an active goal; "
                "requesting cancellation"
            )
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass

        self._app_update_sub = None
        self._stage_event_sub_opened = None
        self._stage_event_sub_assets_loaded = None

        if self._window is not None:
            self._window.destroy()
            self._window = None

        if self.client is not None:
            try:
                self.client.destroy()
            except Exception:
                pass
            self.client = None

        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
