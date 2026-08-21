"""Tests for simulation joint-target normalization."""

import math
import sys
import unittest
from pathlib import Path


MODULE_DIR = (
    Path(__file__).resolve().parents[1]
    / "exts"
    / "omni"
    / "isaac"
    / "ur3_sync"
)
sys.path.insert(0, str(MODULE_DIR))

from joint_targets import (  # noqa: E402
    format_target_summary,
    normalize_joint_positions,
)


EXPECTED_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class FormatTargetSummaryTests(unittest.TestCase):
    def test_identifies_pose_acquisition_method(self):
        cases = (
            (
                "Robot Poser Named Pose",
                "ready",
                "Method: Robot Poser Named Pose\n"
                "Target: ready\n"
                "Joint positions (rad): joints",
            ),
            (
                "Current Simulation Pose",
                "Current Simulation Snapshot",
                "Method: Current Simulation Pose\n"
                "Target: Current Simulation Snapshot\n"
                "Joint positions (rad): joints",
            ),
        )

        for method, label, expected in cases:
            with self.subTest(method=method):
                self.assertEqual(
                    format_target_summary(method, label, "joints"),
                    expected,
                )


class NormalizeJointPositionsTests(unittest.TestCase):
    def test_reorders_articulation_positions_to_controller_order(self):
        dof_names = [
            "wrist_2_joint",
            "elbow_joint",
            "shoulder_pan_joint",
            "wrist_3_joint",
            "shoulder_lift_joint",
            "wrist_1_joint",
        ]
        position_rows = [[5.0, 3.0, 1.0, 6.0, 2.0, 4.0]]

        result = normalize_joint_positions(
            dof_names,
            position_rows,
            EXPECTED_JOINTS,
        )

        self.assertEqual(result, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_returns_snapshot_independent_of_source_row(self):
        source_row = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        result = normalize_joint_positions(
            EXPECTED_JOINTS,
            [source_row],
            EXPECTED_JOINTS,
        )

        source_row[0] = 99.0

        self.assertEqual(result, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_rejects_missing_joint(self):
        with self.assertRaisesRegex(RuntimeError, "missing UR joints"):
            normalize_joint_positions(
                EXPECTED_JOINTS[:-1],
                [[0.0] * 5],
                EXPECTED_JOINTS,
            )

    def test_rejects_multiple_articulation_rows(self):
        with self.assertRaisesRegex(RuntimeError, "row count"):
            normalize_joint_positions(
                EXPECTED_JOINTS,
                [[0.0] * 6, [0.0] * 6],
                EXPECTED_JOINTS,
            )

    def test_rejects_name_position_count_mismatch(self):
        with self.assertRaisesRegex(RuntimeError, "mismatched DOF"):
            normalize_joint_positions(
                EXPECTED_JOINTS,
                [[0.0] * 5],
                EXPECTED_JOINTS,
            )

    def test_rejects_duplicate_dof_names(self):
        duplicate_names = list(EXPECTED_JOINTS)
        duplicate_names[-1] = duplicate_names[0]

        with self.assertRaisesRegex(RuntimeError, "duplicate DOF"):
            normalize_joint_positions(
                duplicate_names,
                [[0.0] * 6],
                EXPECTED_JOINTS,
            )

    def test_rejects_non_finite_position(self):
        for invalid_value in (math.nan, math.inf, -math.inf):
            with self.subTest(invalid_value=invalid_value):
                positions = [0.0] * 6
                positions[2] = invalid_value
                with self.assertRaisesRegex(
                    RuntimeError,
                    "NaN or infinite",
                ):
                    normalize_joint_positions(
                        EXPECTED_JOINTS,
                        [positions],
                        EXPECTED_JOINTS,
                    )


if __name__ == "__main__":
    unittest.main()
