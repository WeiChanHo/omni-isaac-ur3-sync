"""Pure-Python validation and safety primitives for UR3 Live Follow."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControllerState:
    reference: tuple
    feedback: tuple
    error: tuple


@dataclass(frozen=True)
class SettleResult:
    idle: bool
    enter_idle: bool = False
    resume_streaming: bool = False
    settling: bool = False


def _finite_vector(values, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric values") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or infinite values")
    return result


def normalize_controller_state(
    names, reference, feedback, error, controller_names
):
    """Validate Jazzy JTC state vectors and reorder them for the controller."""
    names = tuple(names)
    controller_names = tuple(controller_names)
    if len(names) != len(set(names)):
        raise ValueError("Controller state contains duplicate joint names")
    if set(names) != set(controller_names):
        raise ValueError("Controller state joint names are incomplete")

    vectors = [
        _finite_vector(reference, "Controller reference"),
        _finite_vector(feedback, "Controller feedback"),
        _finite_vector(error, "Controller error"),
    ]
    if any(len(vector) != len(names) for vector in vectors):
        raise ValueError("Controller state name and position counts differ")

    indices = {name: index for index, name in enumerate(names)}
    reordered = [
        tuple(vector[indices[name]] for name in controller_names)
        for vector in vectors
    ]
    return ControllerState(*reordered)


def validate_initial_alignment(simulation, hardware, maximum_error):
    simulation = _finite_vector(simulation, "Simulation target")
    hardware = _finite_vector(hardware, "Hardware feedback")
    if len(simulation) != len(hardware) or not simulation:
        raise ValueError("Simulation and hardware joint counts differ")
    error = max(abs(a - b) for a, b in zip(simulation, hardware))
    if error > maximum_error:
        raise ValueError(f"Initial alignment error is {error:.3f} rad")
    return error


def calculate_waypoint_delta(speed, duration):
    """Return the per-command displacement implied by speed and duration."""
    if (
        not math.isfinite(speed)
        or not math.isfinite(duration)
        or speed <= 0
        or duration <= 0
    ):
        raise ValueError("Speed and waypoint duration must be positive")
    return speed * duration


def limit_waypoint(reference, target, maximum_delta):
    reference = _finite_vector(reference, "Controller reference")
    target = _finite_vector(target, "Simulation target")
    if len(reference) != len(target) or not reference:
        raise ValueError("Reference and target joint counts differ")
    if not math.isfinite(maximum_delta) or maximum_delta <= 0:
        raise ValueError("Maximum waypoint delta must be positive")
    return tuple(
        current + max(-maximum_delta, min(maximum_delta, desired - current))
        for current, desired in zip(reference, target)
    )


def should_publish(target, publishable_target, last_published, deadband):
    target = _finite_vector(target, "Simulation target")
    publishable_target = _finite_vector(publishable_target, "Waypoint")
    if len(target) != len(publishable_target):
        raise ValueError("Target and waypoint joint counts differ")
    if last_published is None:
        return True
    last_published = _finite_vector(last_published, "Last waypoint")
    if len(last_published) != len(target):
        raise ValueError("Last waypoint joint count differs")

    def outside_deadband(a, b):
        delta = abs(a - b)
        return delta >= deadband or math.isclose(delta, deadband, rel_tol=1e-12)

    limiter_still_active = any(
        outside_deadband(desired, waypoint)
        for desired, waypoint in zip(target, publishable_target)
    )
    waypoint_changed = any(
        outside_deadband(waypoint, previous)
        for waypoint, previous in zip(publishable_target, last_published)
    )
    return limiter_still_active or waypoint_changed


def is_fresh(received_at, now, timeout):
    return (
        received_at is not None
        and math.isfinite(received_at)
        and math.isfinite(now)
        and 0.0 <= now - received_at <= timeout
    )


def make_hold_target(hardware_positions, joint_count):
    positions = _finite_vector(hardware_positions, "Hardware feedback")
    if len(positions) != joint_count:
        raise ValueError("Hardware feedback joint count is invalid")
    return positions


class RateGate:
    """Monotonic maximum-rate gate that never accumulates missed ticks."""

    def __init__(self, frequency_hz):
        if not math.isfinite(frequency_hz) or frequency_hz <= 0:
            raise ValueError("Frequency must be positive")
        self._period = 1.0 / frequency_hz
        self._last_time = None

    def ready(self, now):
        if not math.isfinite(now):
            raise ValueError("Gate time must be finite")
        if self._last_time is None or now - self._last_time >= self._period:
            self._last_time = now
            return True
        return False


class TrackingErrorMonitor:
    """Latch only after tracking error stays above threshold continuously."""

    def __init__(self, threshold, duration):
        self._threshold = threshold
        self._duration = duration
        self._exceeded_since = None

    def update(self, errors, now):
        errors = _finite_vector(errors, "Controller tracking error")
        if not errors or max(abs(value) for value in errors) <= self._threshold:
            self._exceeded_since = None
            return False
        if self._exceeded_since is None:
            self._exceeded_since = now
            return False
        return now - self._exceeded_since >= self._duration


class SettleMonitor:
    """Detect an arrived, stable target and wake when it moves or drifts."""

    def __init__(self, target_deadband, arrival_tolerance, settle_duration):
        limits = (target_deadband, arrival_tolerance, settle_duration)
        if any(not math.isfinite(value) or value <= 0 for value in limits):
            raise ValueError("Settle monitor limits must be positive")
        self._target_deadband = target_deadband
        self._arrival_tolerance = arrival_tolerance
        self._settle_duration = settle_duration
        self._target_anchor = None
        self._stable_since = None
        self._idle = False

    def update(self, target, feedback, now):
        target = _finite_vector(target, "Simulation target")
        feedback = _finite_vector(feedback, "Hardware feedback")
        if len(target) != len(feedback) or not target:
            raise ValueError("Target and feedback joint counts differ")
        if not math.isfinite(now):
            raise ValueError("Settle time must be finite")

        target_moved = (
            self._target_anchor is None
            or any(
                abs(current - anchor) >= self._target_deadband
                or math.isclose(
                    abs(current - anchor),
                    self._target_deadband,
                    rel_tol=1e-12,
                )
                for current, anchor in zip(target, self._target_anchor)
            )
        )
        arrived = max(
            abs(desired - actual)
            for desired, actual in zip(target, feedback)
        ) <= self._arrival_tolerance

        resume_streaming = False
        if target_moved:
            resume_streaming = self._idle
            self._target_anchor = target
            self._stable_since = None
            self._idle = False
        elif self._idle and not arrived:
            resume_streaming = True
            self._stable_since = None
            self._idle = False

        if not arrived:
            self._stable_since = None
            return SettleResult(
                idle=False,
                resume_streaming=resume_streaming,
            )

        if self._stable_since is None:
            self._stable_since = now
        if (
            not self._idle
            and now - self._stable_since >= self._settle_duration
        ):
            self._idle = True
            return SettleResult(idle=True, enter_idle=True)
        return SettleResult(
            idle=self._idle,
            resume_streaming=resume_streaming,
            settling=not self._idle,
        )
