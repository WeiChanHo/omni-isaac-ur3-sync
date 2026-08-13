"""Isaac Sim UI workflow for the UR3 synchronization extension."""

import math

import carb
import omni.kit.window.popup_dialog
import omni.ui as ui

from .robot_selection import format_selected_robot_label


class _UiWorkflowMixin:
    """Build and update the extension window and warning dialogs."""

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
