#!/usr/bin/env python3
"""Small deterministic admission tests for the R1 safety state machine."""

import math

from r1_neupan_integration.n2_pre_safety_gate import SafetyGateCore


def healthy(now):
    return dict(command_seen=now, scan_seen=now, odom_seen=now,
                tf_seen=now, timing_seen=now)


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    now = 10.0
    gate = SafetyGateCore()
    d = gate.step(now, 5.0, -5.0, **healthy(now))
    check(d.zeroed and d.reasons == ("emergency_stop",), "default e-stop must zero")
    gate.release()
    d = gate.step(now, 5.0, -5.0, **healthy(now))
    check(d.clamped and abs(d.linear) <= 0.15 and abs(d.angular) <= 0.4, "velocity clamp")
    d = gate.step(now + 0.02, 0.1, 0.1, **healthy(now + 0.02))
    check(abs(d.linear) <= 0.15 and abs(d.angular) <= 0.4, "acceleration clamp")
    d = gate.step(now + 0.04, float("nan"), 0.0, **healthy(now + 0.04))
    check(d.zeroed and "nonfinite_command" in d.reasons, "NaN must zero")
    stale = healthy(now + 0.06)
    stale["scan_seen"] = now - 1.0
    d = gate.step(now + 0.06, 0.1, 0.0, **stale)
    check(d.zeroed and "scan_watchdog" in d.reasons, "stale scan must zero")
    stale_command = healthy(now + 0.07)
    stale_command["command_seen"] = now - 1.0
    d = gate.step(now + 0.07, 0.1, 0.0, **stale_command)
    check(d.zeroed and "command_watchdog" in d.reasons, "stale command must zero")
    d = gate.step(now + 0.08, 0.1, 0.0, path_version_mismatch=True,
                  **healthy(now + 0.08))
    check(d.zeroed and "path_version_mismatch" in d.reasons, "path mismatch must zero")
    gate.clear_failure()
    d = gate.step(now + 0.10, 0.0, 0.0, **healthy(now + 0.10))
    check(not d.zeroed and d.linear == 0.0 and d.angular == 0.0, "explicit stop")
    gate.latch_failure("worker_timeout")
    d = gate.step(now + 0.12, 0.1, 0.1, **healthy(now + 0.12))
    check(d.zeroed and d.linear == 0.0 and d.angular == 0.0, "failure latch")
    gate.clear_failure()
    gate.engage("operator_emergency_stop")
    d = gate.step(now + 0.14, float("inf"), 0.1, **healthy(now + 0.14))
    check(d.zeroed and math.isfinite(d.linear) and math.isfinite(d.angular), "Inf and e-stop")
    gate.release()
    d = gate.step(now + 0.16, -0.05, 0.0, **healthy(now + 0.16))
    check(d.negative_input and d.negative_clamped and d.linear == 0.0,
          "negative linear command must be clamped to exact zero")
    rotation_gate = SafetyGateCore()
    rotation_gate.release()
    d = rotation_gate.step(now + 0.18, -0.15, 0.2, **healthy(now + 0.18))
    check(d.negative_input and d.negative_clamped and d.linear == 0.0
          and d.angular > 0.0, "negative linear must preserve rotation")
    rotation_gate = SafetyGateCore()
    rotation_gate.release()
    d = rotation_gate.step(now + 0.20, -0.15, -0.2, **healthy(now + 0.20))
    check(d.negative_input and d.negative_clamped and d.linear == 0.0
          and d.angular < 0.0, "negative linear must preserve reverse-direction rotation")
    print("N2-PRE safety unit PASS: clamp, no-reverse, acceleration, freshness, mismatch, failure, NaN/Inf, e-stop")


if __name__ == "__main__":
    main()
