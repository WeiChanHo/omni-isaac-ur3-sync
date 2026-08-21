"""Short-trajectory topic streaming workflow for UR3 Live Follow."""

import time

import carb
import omni.timeline

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .live_follow import (
    RateGate,
    SettleMonitor,
    TrackingErrorMonitor,
    calculate_waypoint_delta,
    is_fresh,
    limit_waypoint,
    make_hold_target,
    normalize_controller_state,
    should_publish,
    validate_initial_alignment,
)


class _LiveFollowWorkflowMixin:
    """Manage fire-and-forget Live Follow without changing controllers."""

    def _on_controller_state(self, msg):
        try:
            state = normalize_controller_state(
                msg.joint_names,
                msg.reference.positions,
                msg.feedback.positions,
                msg.error.positions,
                self.ur_joint_names,
            )
        except (AttributeError, ValueError) as exc:
            carb.log_warn(
                f"[UR3 Sync] Ignoring invalid controller state: {exc}"
            )
            return
        self._controller_state = state
        self._controller_state_received_at = time.perf_counter()

    def _set_live_follow_controls(self, active):
        for name in (
            "execute_btn", "load_btn", "get_current_btn", "robot_combo",
            "refresh_btn", "speed_slider",
        ):
            control = getattr(self, name, None)
            if control is not None:
                control.enabled = not active
        if not active and getattr(self, "execute_btn", None) is not None:
            self.execute_btn.enabled = self._pending_positions is not None

    def _set_live_follow_mode_model(self, enabled):
        toggle = getattr(self, "live_follow_mode_toggle", None)
        if toggle is None:
            return
        self._updating_live_follow_mode = True
        try:
            toggle.model.set_value(bool(enabled))
        finally:
            self._updating_live_follow_mode = False

    def _on_live_follow_mode_changed(self, model):
        if self._updating_live_follow_mode:
            return
        if model.get_value_as_bool():
            if not self._start_live_follow():
                self._set_live_follow_mode_model(False)
        else:
            self._stop_live_follow(
                "Live Streaming Mode OFF. A hardware-feedback hold was "
                "requested. Turning the mode OFF is not an emergency stop.",
                self.STATUS_WARN,
            )

    def _start_live_follow(self):
        if self._is_executing or self._goal_handle is not None:
            self._set_status(
                "Cannot enable Live Streaming Mode while a trajectory Action "
                "is active.",
                self.STATUS_ERROR,
            )
            return False
        if self._live_follow_active:
            return True
        now = time.perf_counter()
        try:
            simulation = self._read_current_simulation_positions()
            if not is_fresh(
                self._joint_state_received_at,
                now,
                self.LIVE_FOLLOW_FEEDBACK_TIMEOUT,
            ):
                raise RuntimeError("Fresh /joint_states feedback is required")
            if not is_fresh(
                self._controller_state_received_at,
                now,
                self.LIVE_FOLLOW_FEEDBACK_TIMEOUT,
            ):
                raise RuntimeError("Fresh controller_state feedback is required")
            if self._controller_state is None:
                raise RuntimeError("No valid controller_state is available")
            if (
                self._live_follow_publisher is None
                or self._live_follow_publisher.get_subscription_count() < 1
            ):
                raise RuntimeError("Live Follow command topic has no subscriber")
            validate_initial_alignment(
                simulation, self._hardware_positions,
                self.LIVE_FOLLOW_INITIAL_ERROR_LIMIT,
            )
        except Exception as exc:
            self._set_status(
                f"Live Streaming Mode start blocked: {exc}. Use Execute to "
                "align "
                "first if needed.",
                self.STATUS_ERROR,
            )
            return False

        self._live_follow_active = True
        self._live_follow_robot_path = self._selected_robot_path
        self._live_follow_rate_gate = RateGate(self.LIVE_FOLLOW_MAX_RATE_HZ)
        self._live_follow_error_monitor = TrackingErrorMonitor(
            self.LIVE_FOLLOW_TRACKING_ERROR_LIMIT,
            self.LIVE_FOLLOW_TRACKING_ERROR_DURATION,
        )
        self._live_follow_settle_monitor = SettleMonitor(
            self.LIVE_FOLLOW_DEADBAND,
            self.LIVE_FOLLOW_ARRIVAL_TOLERANCE,
            self.LIVE_FOLLOW_SETTLE_DURATION,
        )
        self._live_follow_last_published = None
        self._live_follow_idle = False
        self._live_follow_phase = "ACTIVE"
        self._set_live_follow_controls(True)
        self._set_status(
            "Live Streaming Mode ACTIVE: fixed 0.50 rad/s, maximum 30 Hz. "
            "Turning the mode OFF is not an emergency stop.",
            self.STATUS_OK,
        )
        return True

    def _publish_live_follow_trajectory(self, positions):
        positions = make_hold_target(positions, len(self.ur_joint_names))
        message = JointTrajectory()
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.joint_names = list(self.ur_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        duration = self.LIVE_FOLLOW_WAYPOINT_DURATION
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9
        )
        message.points.append(point)
        self._live_follow_publisher.publish(message)
        self._live_follow_last_published = positions

    def _stop_live_follow(self, message, color, publish_hold=True):
        was_active = self._live_follow_active
        self._live_follow_active = False
        self._live_follow_idle = False
        self._live_follow_phase = "OFF"
        self._set_live_follow_controls(False)
        self._set_live_follow_mode_model(False)
        if not was_active:
            return
        now = time.perf_counter()
        if publish_hold and is_fresh(
            self._joint_state_received_at, now,
            self.LIVE_FOLLOW_FEEDBACK_TIMEOUT,
        ):
            try:
                self._publish_live_follow_trajectory(self._hardware_positions)
            except Exception as exc:
                message = (
                    f"{message} Hold publish failed: {exc}. "
                    "Robot state uncertain."
                )
                color = self.STATUS_ERROR
        elif publish_hold:
            message = (
                f"{message} /joint_states is stale; robot state uncertain "
                "and no hold was sent."
            )
            color = self.STATUS_ERROR
        if color == self.STATUS_ERROR:
            self._live_follow_phase = "ERROR"
        self._set_status(message, color)

    def _update_live_follow(self):
        if not self._live_follow_active:
            return
        now = time.perf_counter()
        try:
            if self._selected_robot_path != self._live_follow_robot_path:
                raise RuntimeError("Active Robot changed")
            if not omni.timeline.get_timeline_interface().is_playing():
                raise RuntimeError("Timeline stopped")
            if not is_fresh(
                self._joint_state_received_at,
                now,
                self.LIVE_FOLLOW_FEEDBACK_TIMEOUT,
            ):
                raise RuntimeError("/joint_states feedback became stale")
            if not is_fresh(
                self._controller_state_received_at,
                now,
                self.LIVE_FOLLOW_FEEDBACK_TIMEOUT,
            ):
                raise RuntimeError("controller_state feedback became stale")
            if self._live_follow_error_monitor.update(
                self._controller_state.error,
                now,
            ):
                self._stop_live_follow(
                    "Live Streaming Mode fault: controller tracking error "
                    "exceeded 0.15 rad for 0.5 s.",
                    self.STATUS_ERROR,
                )
                return
            if not self._live_follow_rate_gate.ready(now):
                return
            simulation = self._read_current_simulation_positions()
            settle = self._live_follow_settle_monitor.update(
                simulation,
                self._hardware_positions,
                now,
            )
            if settle.enter_idle:
                self._publish_live_follow_trajectory(
                    self._hardware_positions
                )
                self._live_follow_idle = True
                self._live_follow_phase = "IDLE"
                self._set_status(
                    "Live Streaming Mode IDLE: target reached and unchanged "
                    "for 3.0 s; one hold was sent and trajectory publishing "
                    "is paused.",
                    self.STATUS_OK,
                )
                return
            if settle.resume_streaming:
                self._live_follow_idle = False
                self._live_follow_phase = "ACTIVE"
                self._live_follow_last_published = None
                self._set_status(
                    "Live Streaming Mode ACTIVE: target changed or the robot "
                    "left the arrival tolerance; publishing resumed.",
                    self.STATUS_OK,
                )
            elif settle.idle:
                return
            elif settle.settling and self._live_follow_phase != "SETTLING":
                self._live_follow_phase = "SETTLING"
                self._set_status(
                    "Live Streaming Mode SETTLING: target is within 0.01 rad; "
                    "waiting for 3.0 s without a 0.001 rad target change.",
                    self.STATUS_INFO,
                )
            elif not settle.settling:
                if self._live_follow_phase != "ACTIVE":
                    self._set_status(
                        "Live Streaming Mode ACTIVE: tracking the simulation "
                        "target at fixed 0.50 rad/s.",
                        self.STATUS_OK,
                    )
                self._live_follow_phase = "ACTIVE"
            waypoint = limit_waypoint(
                self._controller_state.reference,
                simulation,
                calculate_waypoint_delta(
                    self.LIVE_FOLLOW_SPEED,
                    self.LIVE_FOLLOW_WAYPOINT_DURATION,
                ),
            )
            if should_publish(
                simulation,
                waypoint,
                self._live_follow_last_published,
                self.LIVE_FOLLOW_DEADBAND,
            ):
                self._publish_live_follow_trajectory(waypoint)
        except Exception as exc:
            self._stop_live_follow(
                f"Live Streaming Mode stopped automatically: {exc}.",
                self.STATUS_ERROR,
            )
