"""Regression tests for the README target-acquisition flowchart."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class ReadmeDiagramTests(unittest.TestCase):
    """Keep the documented operator ordering aligned with the UI workflow."""

    def test_active_robot_selection_precedes_both_target_methods(self):
        readme = (REPOSITORY_ROOT / "README.md").read_text(
            encoding="utf-8"
        )

        self.assertIn(
            'Resolve --> SelectRobot["Select Active Robot',
            readme,
        )
        self.assertIn("SelectRobot --> Poses", readme)
        self.assertIn("SelectRobot --> CurrentClick", readme)

    def test_pending_target_edge_distinguishes_motion_data_from_status_text(
        self,
    ):
        readme = (REPOSITORY_ROOT / "README.md").read_text(
            encoding="utf-8"
        )

        self.assertIn(
            "_pending_positions: trajectory target",
            readme,
        )
        self.assertIn(
            "_pending_target_label: status text only",
            readme,
        )

    def test_live_follow_topics_and_safety_limits_are_documented(self):
        readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn(
            "/scaled_joint_trajectory_controller/joint_trajectory", readme
        )
        self.assertIn(
            "/scaled_joint_trajectory_controller/controller_state", readme
        )
        self.assertIn("maximum 30 Hz", readme)
        self.assertIn("0.050 rad", readme)
        self.assertIn("Live Streaming Mode", readme)
        self.assertIn("not an emergency stop", readme)

    def test_live_follow_is_part_of_architecture_diagram(self):
        readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn('LiveMode["Live Streaming Mode toggle', readme)
        self.assertIn("LiveWaypoint --> CommandTopic", readme)
        self.assertIn(
            'ControllerState["/scaled_joint_trajectory_controller/controller_state',
            readme,
        )
        self.assertIn('reference, feedback, error"] --> LiveMonitor', readme)
        self.assertIn("LiveSettle --> LiveIdle", readme)
        self.assertIn("LiveIdle -->|target moves| LiveWaypoint", readme)

    def test_live_streaming_disconnect_conditions_are_documented(self):
        readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")
        for condition in (
            "Timeline stops",
            "Stage changes",
            "Active Robot changes",
            "stale `/joint_states`",
            "stale controller state",
            "tracking error",
            "read or publish exception",
            "extension shutdown",
        ):
            with self.subTest(condition=condition):
                self.assertIn(condition, readme)


if __name__ == "__main__":
    unittest.main()
