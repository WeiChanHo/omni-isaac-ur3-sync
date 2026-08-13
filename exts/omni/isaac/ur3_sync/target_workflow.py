"""Robot selection and target acquisition workflow for UR3 sync."""

import math

import omni.timeline
import omni.ui as ui
import omni.usd

from pxr import Usd

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


class _TargetWorkflowMixin:
    """Discover robots and capture immutable physical-motion targets."""

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
