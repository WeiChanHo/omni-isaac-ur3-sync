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


if __name__ == "__main__":
    unittest.main()
