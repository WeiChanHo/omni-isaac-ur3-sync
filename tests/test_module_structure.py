"""Source-level regression tests for extension responsibility boundaries."""

import ast
import unittest
from pathlib import Path


MODULE_DIR = (
    Path(__file__).resolve().parents[1]
    / "exts"
    / "omni"
    / "isaac"
    / "ur3_sync"
)


def _class_methods(path, class_name):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name
        for node in class_node.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def _base_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return f"{_base_name(node.value)}.{node.attr}"
    raise TypeError(f"Unsupported base node: {ast.dump(node)}")


class ModuleStructureTests(unittest.TestCase):
    def test_extension_composes_workflows_and_kit_entry_point(self):
        tree = ast.parse(
            (MODULE_DIR / "extension.py").read_text(encoding="utf-8")
        )
        extension_class = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "Ur3SyncExtension"
        )

        self.assertEqual(
            [_base_name(base) for base in extension_class.bases],
            [
                "_UiWorkflowMixin",
                "_TargetWorkflowMixin",
                "_TrajectoryWorkflowMixin",
                "_LiveFollowWorkflowMixin",
                "omni.ext.IExt",
            ],
        )

    def test_live_follow_workflow_owns_streaming_lifecycle(self):
        expected_methods = {
            "_on_controller_state",
            "_on_live_follow_mode_changed",
            "_set_live_follow_mode_model",
            "_update_live_follow",
            "_stop_live_follow",
            "_publish_live_follow_trajectory",
            "_set_live_follow_controls",
        }
        methods = _class_methods(
            MODULE_DIR / "live_follow_workflow.py",
            "_LiveFollowWorkflowMixin",
        )
        self.assertTrue(expected_methods <= methods)

    def test_ui_workflow_owns_window_status_and_warning_methods(self):
        expected_methods = {
            "_build_ui",
            "_set_status",
            "_format_speed",
            "_on_speed_changed",
            "_get_command_speed",
            "_show_motion_warning",
            "_dismiss_warning_dialog",
        }

        ui_methods = _class_methods(
            MODULE_DIR / "ui_workflow.py",
            "_UiWorkflowMixin",
        )
        extension_methods = _class_methods(
            MODULE_DIR / "extension.py",
            "Ur3SyncExtension",
        )

        self.assertTrue(expected_methods <= ui_methods)
        self.assertTrue(expected_methods.isdisjoint(extension_methods))

    def test_target_workflow_owns_robot_pose_and_snapshot_methods(self):
        expected_methods = {
            "_discover_robot_paths",
            "_replace_robot_combo_items",
            "_update_selected_robot_label",
            "_refresh_robots_and_poses",
            "_on_robot_selection_changed",
            "_get_selected_robot_prim",
            "_replace_pose_combo_items",
            "_update_selected_pose_label",
            "_refresh_pose_names",
            "_on_pose_selection_changed",
            "_get_selected_pose_name",
            "_invalidate_pending_target",
            "_format_joint_positions",
            "_set_pending_target",
            "_load_named_pose_positions",
            "_on_load_clicked",
            "_read_current_simulation_positions",
            "_on_get_current_clicked",
        }

        target_methods = _class_methods(
            MODULE_DIR / "target_workflow.py",
            "_TargetWorkflowMixin",
        )
        extension_methods = _class_methods(
            MODULE_DIR / "extension.py",
            "Ur3SyncExtension",
        )

        self.assertTrue(expected_methods <= target_methods)
        self.assertTrue(expected_methods.isdisjoint(extension_methods))

    def test_trajectory_workflow_owns_ros_action_and_watchdog_methods(self):
        expected_methods = {
            "_initialize_ros",
            "_on_joint_state",
            "_on_app_update",
            "_target_error",
            "_start_execution_watchdog",
            "_reset_execution_watchdog",
            "_monitor_execution_stall",
            "_handle_suspected_stall",
            "_calculate_motion_duration",
            "_on_execute_clicked",
            "_send_trajectory_goal",
            "_on_goal_response",
            "_on_goal_result",
            "_on_stop_clicked",
            "_on_cancel_response",
            "_finish_execution_state",
        }

        trajectory_methods = _class_methods(
            MODULE_DIR / "trajectory_workflow.py",
            "_TrajectoryWorkflowMixin",
        )
        extension_methods = _class_methods(
            MODULE_DIR / "extension.py",
            "Ur3SyncExtension",
        )

        self.assertTrue(expected_methods <= trajectory_methods)
        self.assertTrue(expected_methods.isdisjoint(extension_methods))


if __name__ == "__main__":
    unittest.main()
