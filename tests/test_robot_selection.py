"""Tests for Isaac-independent Active Robot selection policy."""

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

from robot_selection import (  # noqa: E402
    format_selected_robot_label,
    resolve_robot_selection,
)


class RobotSelectionTests(unittest.TestCase):
    def test_sorts_paths_and_prefers_ur3_on_initial_scan(self):
        paths, selected = resolve_robot_selection(
            ["/World/z_robot", "/World/ur3", "/World/a_robot"],
            previous_path=None,
            prefer_default=True,
        )

        self.assertEqual(
            paths,
            ["/World/a_robot", "/World/ur3", "/World/z_robot"],
        )
        self.assertEqual(selected, "/World/ur3")

    def test_formats_selected_robot_full_path(self):
        self.assertEqual(
            format_selected_robot_label("/World/ur3"),
            "Selected robot: /World/ur3",
        )

    def test_formats_none_when_no_robot_is_selected(self):
        self.assertEqual(
            format_selected_robot_label(None),
            "Selected robot: None",
        )

    def test_preserves_previous_path_instead_of_switching_to_default(self):
        paths, selected = resolve_robot_selection(
            ["/World/ur3", "/World/selected"],
            previous_path="/World/selected",
            prefer_default=False,
        )

        self.assertEqual(paths, ["/World/selected", "/World/ur3"])
        self.assertEqual(selected, "/World/selected")

    def test_missing_previous_path_falls_back_to_none(self):
        paths, selected = resolve_robot_selection(
            ["/World/another", "/World/ur3"],
            previous_path="/World/removed",
            prefer_default=False,
        )

        self.assertEqual(paths, ["/World/another", "/World/ur3"])
        self.assertIsNone(selected)

    def test_initial_scan_without_default_falls_back_to_none(self):
        paths, selected = resolve_robot_selection(
            ["/World/z_robot", "/World/a_robot"],
            previous_path=None,
            prefer_default=True,
        )

        self.assertEqual(paths, ["/World/a_robot", "/World/z_robot"])
        self.assertIsNone(selected)

    def test_removes_duplicate_discovery_paths(self):
        paths, selected = resolve_robot_selection(
            ["/World/ur3", "/World/other", "/World/ur3"],
            previous_path="/World/ur3",
            prefer_default=False,
        )

        self.assertEqual(paths, ["/World/other", "/World/ur3"])
        self.assertEqual(selected, "/World/ur3")


if __name__ == "__main__":
    unittest.main()
