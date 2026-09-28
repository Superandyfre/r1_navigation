#!/usr/bin/env python3
"""Focused pure checks for the N2.4 recovery design."""

from __future__ import annotations

from r1_neupan_integration.n2_4_recovery import (
    ForwardOnlyRecoveryFSM,
    RecoveryConfig,
    RecoveryObservation,
    choose_heading,
    compute_reanchor,
    evaluate_trigger,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def observation(cycle: int, v: float, *, margin: float = 1.0, yaw: float = 0.0, stop: bool = False) -> RecoveryObservation:
    path = ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))
    return RecoveryObservation(
        cycle_id=cycle,
        monotonic=cycle * 0.1,
        raw_linear=v,
        raw_angular=0.0,
        pose=(0.4, 0.2, yaw),
        path=path,
        active_path_index=0,
        path_deviation_m=0.2,
        goal_distance_m=1.6,
        stop=stop,
        corridor_margin_m=margin,
        obstacle_clearance_m=0.5,
        scan_topology={"min_front": 0.5, "min_left": 0.6, "min_right": 0.6},
        reference_points=((0.0, 0.0), (1.0, 0.0)),
        optimized_points=((0.4, 0.2), (0.5, 0.2)),
    )


def main() -> int:
    config = RecoveryConfig()
    first = observation(1, -0.03)
    check(not evaluate_trigger([], first, config)["trigger"], "one negative sample must not trigger")
    second = observation(2, -0.02)
    third = observation(3, -0.04)
    decision = evaluate_trigger([first, second], third, config)
    check(decision["trigger"], "persistent negative intent must trigger")
    check(decision["negative_run_length"] == 3, "negative run length")

    c3 = evaluate_trigger(
        [observation(1, -0.03, margin=0.024), observation(2, -0.04, margin=0.024)],
        observation(3, -0.05, margin=0.024),
        config,
    )
    check(not c3["trigger"] and c3["geometry_rejected"], "C3 must be rejected as a geometry boundary")
    stopped = evaluate_trigger([first, second], observation(3, -0.04, stop=True), config)
    check(not stopped["trigger"], "planner stop closure must not trigger recovery")

    reanchor = compute_reanchor(third, last_reanchor_index=-1)
    check(reanchor is not None, "forward reanchor must exist")
    check(reanchor["segment_index"] >= third.active_path_index, "path index cannot move backward")
    check(reanchor["avoids_same_failure_point"], "reanchor must advance beyond projection")
    heading = choose_heading(third, reanchor, config)
    check(heading["selected"]["feasible"], "selected heading must be forward feasible")

    fsm = ForwardOnlyRecoveryFSM(config)
    history = []
    for row in (first, second, third):
        command = fsm.update(row, history)
        check(command["linear_x"] >= 0.0, "FSM must never emit reverse")
        history.append(row)
    check(fsm.state == "RECOVERY_STOP", "trigger must enter stop")
    for _ in range(config.stop_settle_samples):
        command = fsm.update(third, history, shadow_stop_settled=True)
        check(command["linear_x"] == 0.0, "stop interlock emits zero")
    check(fsm.state == "RECOVERY_ROTATE", "settled stop must precede rotate")
    target = fsm.heading["target_heading"]
    rotated = observation(4, 0.0, yaw=target)
    command = fsm.update(rotated, history)
    check(command["linear_x"] == 0.0, "rotate transition cannot emit linear motion")
    check(fsm.state == "RECOVERY_REANCHOR", "heading tolerance enters reanchor")
    for cycle in (5, 6, 7):
        command = fsm.update(observation(cycle, 0.08, yaw=target), history)
        check(command["linear_x"] >= 0.0, "reentry must remain forward-only")
    check(fsm.state == "NORMAL", "fresh nonnegative reentry returns to normal")

    fail_config = RecoveryConfig(max_recovery_attempts=1, reentry_negative_samples_before_retry=1)
    failed = ForwardOnlyRecoveryFSM(fail_config)
    failed.state = "RECOVERY_REANCHOR"
    failed.attempt = 1
    failed.reanchor = reanchor
    failed._state_start_t = 0.0
    failed.heading = heading
    command = failed.update(observation(10, -0.1), history)
    check(failed.state == "RECOVERY_FAILED" and command["linear_x"] == 0.0, "retry exhaustion must latch zero")
    check(all(command["linear_x"] >= 0.0 for command in (command,)), "failed output remains safe")
    print("N2.4 recovery unit PASS: trigger, C3 boundary, stop interlock, heading, reanchor, retry/latch, no-reverse")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
