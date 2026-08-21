"""Unit tests for Isaac-independent Live Follow safety logic."""

import importlib.util
import math
from pathlib import Path
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "exts/omni/isaac/ur3_sync/live_follow.py"
)
SPEC = importlib.util.spec_from_file_location("live_follow", MODULE_PATH)
live_follow = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(live_follow)


JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


class NormalizeControllerStateTests(unittest.TestCase):
    def test_reorders_each_state_vector(self):
        result = live_follow.normalize_controller_state(
            list(reversed(JOINTS)),
            [6, 5, 4, 3, 2, 1],
            [12, 10, 8, 6, 4, 2],
            [0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
            JOINTS,
        )
        self.assertEqual(result.reference, (1, 2, 3, 4, 5, 6))
        self.assertEqual(result.feedback, (2, 4, 6, 8, 10, 12))
        self.assertEqual(result.error, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

    def test_rejects_malformed_state(self):
        bad_cases = [
            (JOINTS[:-1], [0] * 5, [0] * 5, [0] * 5),
            (JOINTS[:-1] + ["j5"], [0] * 6, [0] * 6, [0] * 6),
            (JOINTS, [0] * 5, [0] * 6, [0] * 6),
            (JOINTS, [0] * 6, [0] * 5, [0] * 6),
            (JOINTS, [0] * 6, [0] * 6, [0] * 5),
            (JOINTS, [0, 0, 0, 0, 0, math.nan], [0] * 6, [0] * 6),
        ]
        for names, reference, feedback, error in bad_cases:
            with self.subTest(names=names, reference=reference):
                with self.assertRaises(ValueError):
                    live_follow.normalize_controller_state(
                        names, reference, feedback, error, JOINTS
                    )


class LiveFollowLimiterTests(unittest.TestCase):
    def test_waypoint_delta_matches_fixed_speed_and_duration(self):
        self.assertEqual(
            live_follow.calculate_waypoint_delta(0.5, 0.1),
            0.05,
        )

    def test_accepts_initial_alignment_boundary(self):
        live_follow.validate_initial_alignment([0] * 6, [0.1] * 6, 0.1)

    def test_rejects_initial_alignment_beyond_boundary(self):
        with self.assertRaises(ValueError):
            live_follow.validate_initial_alignment([0] * 6, [0.10001] * 6, 0.1)

    def test_limits_each_joint_relative_to_controller_reference(self):
        result = live_follow.limit_waypoint(
            [0, 0, 0, 0, 0, 0],
            [1, -1, 0.01, -0.01, 0.02, -0.02],
            0.02,
        )
        self.assertEqual(result, (0.02, -0.02, 0.01, -0.01, 0.02, -0.02))

    def test_rejects_invalid_limiter_input(self):
        for reference, target in [([0], [0, 1]), ([math.inf], [0])]:
            with self.assertRaises(ValueError):
                live_follow.limit_waypoint(reference, target, 0.02)

    def test_deadband_suppresses_a_stationary_reachable_target(self):
        self.assertFalse(
            live_follow.should_publish([1.0], [1.0], [1.0009], 0.001)
        )

    def test_limiter_gap_keeps_publishing_as_reference_advances(self):
        self.assertTrue(
            live_follow.should_publish([1.0], [0.02], [0.02], 0.001)
        )

    def test_first_waypoint_is_published(self):
        self.assertTrue(
            live_follow.should_publish([1.0], [1.0], None, 0.001)
        )


class RateGateTests(unittest.TestCase):
    def test_rate_gate_does_not_catch_up_with_a_burst(self):
        gate = live_follow.RateGate(30.0)
        self.assertTrue(gate.ready(10.0))
        self.assertFalse(gate.ready(10.01))
        self.assertTrue(gate.ready(20.0))
        self.assertFalse(gate.ready(20.001))


class TrackingErrorMonitorTests(unittest.TestCase):
    def test_requires_continuous_error_for_half_second(self):
        monitor = live_follow.TrackingErrorMonitor(0.15, 0.5)
        self.assertFalse(monitor.update([0.16], 1.0))
        self.assertFalse(monitor.update([0.16], 1.49))
        self.assertTrue(monitor.update([0.16], 1.5))

    def test_normal_error_resets_timer(self):
        monitor = live_follow.TrackingErrorMonitor(0.15, 0.5)
        monitor.update([0.2], 1.0)
        self.assertFalse(monitor.update([0.1], 1.4))
        self.assertFalse(monitor.update([0.2], 1.6))


class SettleMonitorTests(unittest.TestCase):
    def test_does_not_idle_before_robot_arrives(self):
        monitor = live_follow.SettleMonitor(0.001, 0.01, 3.0)
        monitor.update([1.0], [0.0], 0.0)
        result = monitor.update([1.0], [0.98], 10.0)
        self.assertFalse(result.idle)
        self.assertFalse(result.enter_idle)

    def test_enters_idle_after_arrival_is_stable_for_three_seconds(self):
        monitor = live_follow.SettleMonitor(0.001, 0.01, 3.0)
        monitor.update([1.0], [0.995], 2.0)
        before = monitor.update([1.0005], [0.996], 4.99)
        at_boundary = monitor.update([1.0005], [0.996], 5.0)
        self.assertFalse(before.idle)
        self.assertTrue(at_boundary.idle)
        self.assertTrue(at_boundary.enter_idle)

    def test_leaving_arrival_tolerance_resets_timer(self):
        monitor = live_follow.SettleMonitor(0.001, 0.01, 3.0)
        monitor.update([1.0], [0.995], 0.0)
        monitor.update([1.0], [0.98], 2.0)
        result = monitor.update([1.0], [0.995], 3.1)
        self.assertFalse(result.idle)

    def test_target_change_wakes_idle_mode(self):
        monitor = live_follow.SettleMonitor(0.001, 0.01, 3.0)
        monitor.update([1.0], [1.0], 0.0)
        monitor.update([1.0], [1.0], 3.0)
        result = monitor.update([1.001], [1.0], 3.1)
        self.assertFalse(result.idle)
        self.assertTrue(result.resume_streaming)

    def test_idle_entry_is_reported_only_once(self):
        monitor = live_follow.SettleMonitor(0.001, 0.01, 3.0)
        monitor.update([1.0], [1.0], 0.0)
        first = monitor.update([1.0], [1.0], 3.0)
        later = monitor.update([1.0], [1.0], 4.0)
        self.assertTrue(first.enter_idle)
        self.assertFalse(later.enter_idle)
        self.assertTrue(later.idle)


class FeedbackSafetyTests(unittest.TestCase):
    def test_freshness_includes_boundary(self):
        self.assertTrue(live_follow.is_fresh(4.5, 5.0, 0.5))
        self.assertFalse(live_follow.is_fresh(4.49, 5.0, 0.5))

    def test_hold_target_is_a_valid_snapshot(self):
        source = [1, 2, 3, 4, 5, 6]
        hold = live_follow.make_hold_target(source, 6)
        source[0] = 99
        self.assertEqual(hold, (1, 2, 3, 4, 5, 6))
        with self.assertRaises(ValueError):
            live_follow.make_hold_target([0] * 5, 6)


if __name__ == "__main__":
    unittest.main()
