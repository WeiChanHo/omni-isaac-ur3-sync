"""Bridge Isaac Sim joint targets to a physical UR3 controller.

The operator loads a Robot Poser named pose or captures the planning
articulation's current positions, then sends one FollowJointTrajectory goal.
Joint positions are never continuously streamed.
"""

import math

import carb
import carb.eventdispatcher
import omni.ext
import omni.kit.app
import omni.usd

from .target_workflow import _TargetWorkflowMixin
from .trajectory_workflow import _TrajectoryWorkflowMixin
from .ui_workflow import _UiWorkflowMixin


class Ur3SyncExtension(
    _UiWorkflowMixin,
    _TargetWorkflowMixin,
    _TrajectoryWorkflowMixin,
    omni.ext.IExt,
):
    """載入模擬 UR3 關節目標，並將其傳送至實體 UR3。"""

    ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    JOINT_STATE_TOPIC = "/joint_states"

    POSE_METHOD_NAMED_POSE = "Robot Poser Named Pose"
    POSE_METHOD_CURRENT_SIMULATION = "Current Simulation Pose"

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
        self._pending_pose_method = None
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
