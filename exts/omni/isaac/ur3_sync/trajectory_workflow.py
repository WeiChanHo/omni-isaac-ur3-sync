"""ROS 2 trajectory execution and motion watchdog workflow."""

import math
import time

import carb
import rclpy

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class _TrajectoryWorkflowMixin:
    """Manage ROS callbacks, trajectory actions, and stall detection."""

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
