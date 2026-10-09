"""Stateful low-speed command stabilizer for bidirectional NeuPAN output."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CommandStabilizerConfig:
    angular_deadband_radps: float = 0.05
    linear_deadband_mps: float = 0.015
    angular_flip_threshold_radps: float = 0.12
    linear_flip_threshold_mps: float = 0.04
    flip_confirm_cycles: int = 2
    low_speed_linear_mps: float = 0.03
    decisive_turn_radps: float = 0.10

    def __post_init__(self) -> None:
        positive = (
            self.angular_deadband_radps,
            self.linear_deadband_mps,
            self.angular_flip_threshold_radps,
            self.linear_flip_threshold_mps,
            self.low_speed_linear_mps,
            self.decisive_turn_radps,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("command stabilizer thresholds must be finite and positive")
        if self.flip_confirm_cycles < 1:
            raise ValueError("flip_confirm_cycles must be positive")


@dataclass(frozen=True)
class StabilizedCommand:
    linear_x: float
    angular_z: float
    state: str
    reason: str


class BidirectionalCommandStabilizer:
    """Reject low-amplitude sign chatter without removing reverse maneuvers."""

    def __init__(self, config: CommandStabilizerConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._linear_sign = 0
        self._angular_sign = 0
        self._pending_linear_sign = 0
        self._pending_linear_cycles = 0
        self._pending_angular_sign = 0
        self._pending_angular_cycles = 0

    @staticmethod
    def _sign(value: float, deadband: float) -> int:
        if abs(value) < deadband:
            return 0
        return 1 if value > 0.0 else -1

    def _stabilize_axis(
        self,
        value: float,
        *,
        deadband: float,
        flip_threshold: float,
        axis: str,
    ) -> tuple[float, str | None]:
        if not math.isfinite(value):
            raise ValueError("NeuPAN command must be finite")
        sign = self._sign(value, deadband)
        sign_attr = f"_{axis}_sign"
        pending_sign_attr = f"_pending_{axis}_sign"
        pending_cycles_attr = f"_pending_{axis}_cycles"
        previous_sign = getattr(self, sign_attr)
        if sign == 0:
            setattr(self, pending_sign_attr, 0)
            setattr(self, pending_cycles_attr, 0)
            return 0.0, f"{axis}_deadband"
        if previous_sign == 0 or sign == previous_sign:
            setattr(self, sign_attr, sign)
            setattr(self, pending_sign_attr, 0)
            setattr(self, pending_cycles_attr, 0)
            return value, None
        # Weak translation reversals still require deliberate gear intent.
        # Steering corrections above the deadband need only bounded temporal
        # confirmation; suppressing them indefinitely causes yaw overshoot.
        if axis == "linear" and abs(value) < flip_threshold:
            setattr(self, pending_sign_attr, 0)
            setattr(self, pending_cycles_attr, 0)
            return 0.0, f"{axis}_flip_below_threshold"
        pending_sign = getattr(self, pending_sign_attr)
        pending_cycles = getattr(self, pending_cycles_attr)
        if pending_sign == sign:
            pending_cycles += 1
        else:
            pending_sign = sign
            pending_cycles = 1
        setattr(self, pending_sign_attr, pending_sign)
        setattr(self, pending_cycles_attr, pending_cycles)
        if pending_cycles < self.config.flip_confirm_cycles:
            return 0.0, f"{axis}_flip_confirming"
        setattr(self, sign_attr, sign)
        setattr(self, pending_sign_attr, 0)
        setattr(self, pending_cycles_attr, 0)
        return value, None

    def stabilize(self, raw_linear_x: float, raw_angular_z: float) -> StabilizedCommand:
        linear_x, linear_reason = self._stabilize_axis(
            float(raw_linear_x), deadband=self.config.linear_deadband_mps,
            flip_threshold=self.config.linear_flip_threshold_mps, axis="linear")
        angular_z, angular_reason = self._stabilize_axis(
            float(raw_angular_z), deadband=self.config.angular_deadband_radps,
            flip_threshold=self.config.angular_flip_threshold_radps, axis="angular")
        reasons = [reason for reason in (linear_reason, angular_reason) if reason]
        # A command that has cleared the linear deadband is deliberate enough
        # to track. Holding every command below ``low_speed_linear_mps`` made
        # normal low-speed approach motion alternate between TRACK and HOLD.
        # Preserve an explicit strong turn-in-place request, while sign changes
        # remain handled above and cannot admit a brief reverse sample.
        if (abs(linear_x) < self.config.low_speed_linear_mps and
                abs(angular_z) >= self.config.decisive_turn_radps):
            linear_x = 0.0
            reasons.append("low_speed_turn_in_place")
            return StabilizedCommand(linear_x, angular_z, "TURN_IN_PLACE", ";".join(reasons))
        if abs(linear_x) < self.config.linear_deadband_mps:
            reasons.append("low_speed_indecisive_hold")
            return StabilizedCommand(0.0, 0.0, "HOLD", ";".join(reasons))
        return StabilizedCommand(
            linear_x, angular_z, "TRACK",
            ";".join(reasons) if reasons else "pass_through")


@dataclass(frozen=True)
class GearTransitionConfig:
    mismatch_deadband_mps: float = 0.025
    confirm_cycles: int = 5
    stopped_speed_mps: float = 0.015
    stop_timeout_sec: float = 2.0
    # ``confirm_cycles`` is retained for old launch files.  A HW1 profile can
    # explicitly require a longer, route-independent reverse intent before a
    # gear rewrite is even considered.
    opposite_direction_commit_cycles: int | None = None

    def __post_init__(self) -> None:
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                    self.mismatch_deadband_mps, self.stopped_speed_mps,
                    self.stop_timeout_sec)) or self.confirm_cycles < 1):
            raise ValueError("gear transition thresholds must be positive")
        if (self.opposite_direction_commit_cycles is not None and
                self.opposite_direction_commit_cycles < 1):
            raise ValueError("opposite direction commit cycles must be positive")

    @property
    def direction_commit_cycles(self) -> int:
        return (self.opposite_direction_commit_cycles
                if self.opposite_direction_commit_cycles is not None
                else self.confirm_cycles)


@dataclass(frozen=True)
class GearTransitionDecision:
    linear_x: float
    angular_z: float
    state: str
    reason: str
    request_gear: int | None = None


class GearTransitionGate:
    """Serialize automatic direction reversals through a confirmed stop."""

    def __init__(self, config: GearTransitionConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._candidate_gear: int | None = None
        self._candidate_cycles = 0
        self._target_gear: int | None = None
        self._stop_started = 0.0

    @staticmethod
    def _sign(value: float, deadband: float) -> int:
        if abs(value) < deadband:
            return 0
        return 1 if value > 0.0 else -1

    def decide(
        self,
        *,
        linear_x: float,
        angular_z: float,
        active_gear: int,
        observed_linear_x: float,
        odom_fresh: bool,
        now: float,
    ) -> GearTransitionDecision:
        if active_gear not in (-1, 1):
            raise ValueError("active gear must be -1 or +1")
        if not all(math.isfinite(value) for value in (
                linear_x, angular_z, observed_linear_x, now)):
            raise ValueError("gear gate inputs must be finite")
        requested_gear = self._sign(linear_x, self.config.mismatch_deadband_mps)
        if requested_gear == 0:
            if self._candidate_gear is not None or self._target_gear is not None:
                self.reset()
            low_speed_sign = self._sign(linear_x, 0.0)
            if low_speed_sign == 0:
                return GearTransitionDecision(0.0, angular_z, "DRIVE", "linear_deadband")
            if low_speed_sign == active_gear:
                return GearTransitionDecision(
                    linear_x, angular_z, "DRIVE", "aligned_below_transition_deadband")
            return GearTransitionDecision(
                0.0, angular_z, "DRIVE", "opposite_below_transition_deadband")
        if requested_gear == active_gear:
            if self._candidate_gear is not None or self._target_gear is not None:
                self.reset()
            return GearTransitionDecision(
                linear_x, angular_z, "DRIVE", "gear_aligned")

        if self._target_gear is None:
            if self._candidate_gear == requested_gear:
                self._candidate_cycles += 1
            else:
                self._candidate_gear = requested_gear
                self._candidate_cycles = 1
            if self._candidate_cycles < self.config.direction_commit_cycles:
                return GearTransitionDecision(
                    0.0, 0.0, "CONFIRM", "opposite_direction_commit_pending")
            self._target_gear = requested_gear
            self._stop_started = now

        if self._target_gear != requested_gear:
            self.reset()
            return GearTransitionDecision(0.0, 0.0, "CONFIRM", "opposite_gear_changed")
        if not odom_fresh:
            return GearTransitionDecision(0.0, 0.0, "STOPPING", "odom_stale_before_gear_change")
        if abs(observed_linear_x) > self.config.stopped_speed_mps:
            return GearTransitionDecision(0.0, 0.0, "STOPPING", "waiting_for_zero_speed")
        if now - self._stop_started > self.config.stop_timeout_sec:
            return GearTransitionDecision(0.0, 0.0, "STOPPING", "gear_change_stop_timeout")
        target = self._target_gear
        self.reset()
        return GearTransitionDecision(
            0.0, 0.0, "SWITCH", "zero_speed_confirmed", request_gear=target)
