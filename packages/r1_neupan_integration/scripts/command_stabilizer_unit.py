#!/usr/bin/env python3
"""Deterministic offline checks for bidirectional NeuPAN command stabilization."""

from __future__ import annotations

from r1_neupan_integration.command_stabilizer import (
    BidirectionalCommandStabilizer,
    CommandStabilizerConfig,
    GearTransitionConfig,
    GearTransitionGate,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    stabilizer = BidirectionalCommandStabilizer(CommandStabilizerConfig())
    stable = stabilizer.stabilize(0.08, 0.14)
    check(stable.state == "TRACK" and stable.linear_x > 0.0 and stable.angular_z > 0.0,
          "clear forward command must pass through")
    low_speed_sequence = [
        stabilizer.stabilize(0.02, 0.04),
        stabilizer.stabilize(0.02, -0.04),
        stabilizer.stabilize(-0.02, 0.04),
    ]
    check(all(item.linear_x > 0.0 for item in low_speed_sequence[:2]),
          "aligned low-speed forward commands must keep tracking")
    check(low_speed_sequence[-1].linear_x == 0.0,
          "brief low-speed opposite chatter must not command reverse")
    first_reverse = stabilizer.stabilize(-0.08, -0.16)
    confirmed_reverse = stabilizer.stabilize(-0.08, -0.16)
    check(first_reverse.linear_x == 0.0 and confirmed_reverse.linear_x < 0.0,
          "reverse must pass after bounded direction confirmation")
    stabilizer.reset()
    turn = stabilizer.stabilize(0.02, 0.16)
    check(turn.state == "TURN_IN_PLACE" and turn.linear_x == 0.0 and turn.angular_z > 0.0,
          "decisive low-speed turn must rotate in place")


    # A sustained small yaw correction must not wait for a large overshoot.
    yaw = BidirectionalCommandStabilizer(CommandStabilizerConfig())
    yaw.stabilize(0.08, 0.14)
    check(yaw.stabilize(0.08, -0.08).angular_z == 0.0,
          "one opposite yaw sample must still be confirmed")
    check(yaw.stabilize(0.08, -0.08).angular_z == -0.08,
          "sustained small yaw correction must pass after bounded confirmation")
    yaw.reset()
    yaw.stabilize(0.08, 0.14)
    yaw.stabilize(0.08, -0.08)
    check(yaw.stabilize(0.08, 0.08).angular_z == 0.08,
          "isolated opposite yaw chatter must not change the accepted sign")
    for _ in range(5):
        check(yaw.stabilize(-0.02, 0.08).linear_x == 0.0,
              "weak reverse commands must remain blocked")
    gentle = BidirectionalCommandStabilizer(
        CommandStabilizerConfig(angular_deadband_radps=0.02))
    gentle.stabilize(0.08, 0.03)
    gentle.stabilize(0.08, -0.03)
    check(gentle.stabilize(0.08, -0.03).angular_z == -0.03,
          "HW1 gentle steering must permit confirmed corrections")

    gear_gate = GearTransitionGate(GearTransitionConfig(confirm_cycles=3))
    for now in (0.0, 0.1):
        waiting = gear_gate.decide(
            linear_x=-0.10, angular_z=0.0, active_gear=1,
            observed_linear_x=0.10, odom_fresh=True, now=now)
        check(waiting.linear_x == 0.0 and waiting.request_gear is None,
              "opposite gear must be held at zero before confirmation")
    stopping = gear_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.10, odom_fresh=True, now=0.2)
    check(stopping.state == "STOPPING" and stopping.linear_x == 0.0,
          "confirmed reverse must still wait for odometry to stop")
    switching = gear_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=0.3)
    check(switching.request_gear == -1 and switching.linear_x == 0.0,
          "only a stopped base may request the reverse gear")

    stale_odom_gate = GearTransitionGate(GearTransitionConfig(confirm_cycles=1))
    stale_odom = stale_odom_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=False, now=0.0)
    check(stale_odom.state == "STOPPING" and stale_odom.request_gear is None,
          "stale odometry must prevent automatic gear changes")
    timeout_gate = GearTransitionGate(GearTransitionConfig(
        confirm_cycles=1, stop_timeout_sec=0.5))
    timeout_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.10, odom_fresh=True, now=0.0)
    timed_out = timeout_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=0.6)
    check(timed_out.state == "STOPPING" and timed_out.request_gear is None,
          "a delayed stop confirmation must not trigger a gear change")

    intent_gate = GearTransitionGate(GearTransitionConfig(
        confirm_cycles=1, opposite_direction_commit_cycles=4))
    for now, velocity in enumerate((-0.10, 0.10, -0.10, -0.10)):
        decision = intent_gate.decide(
            linear_x=velocity, angular_z=0.0, active_gear=1,
            observed_linear_x=0.0, odom_fresh=True, now=float(now))
        check(decision.request_gear is None,
              "brief opposite chatter must not request a gear transition")
    commit = intent_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=4.0)
    check(commit.request_gear is None,
          "reverse intent must remain pending until all commit cycles arrive")
    switch = intent_gate.decide(
        linear_x=-0.10, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=5.0)
    check(switch.request_gear == -1,
          "sustained opposite intent must still permit a real reverse")

    low_speed_gate = GearTransitionGate(GearTransitionConfig())
    low_speed_forward = low_speed_gate.decide(
        linear_x=0.02, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=0.0)
    check(low_speed_forward.linear_x > 0.0,
          "aligned low speed must not be erased before stabilization")
    low_speed_opposite = low_speed_gate.decide(
        linear_x=-0.02, angular_z=0.0, active_gear=1,
        observed_linear_x=0.0, odom_fresh=True, now=0.1)
    check(low_speed_opposite.linear_x == 0.0,
          "sub-threshold opposite noise must never command reverse")
    print("command_stabilizer_unit PASS")


if __name__ == "__main__":
    main()
