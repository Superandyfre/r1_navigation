# CogNav A+ protected candidate

This is an isolated snapshot of the ROS-side bridge from
`/home/rpp/cognav_ws/objnav_live_round3/robot_live`. `baseline.json` records the
original hashes. The running production tree is never imported by this candidate
and is not overwritten. The CogNav Python 3.8 driver is outside this snapshot.

Changes: observe fresh Worker/Controller/Guard status, latch first fatal/boot or
worker-generation change, cancel unresolved goals on veto, refuse new spool plans
on veto, guard `_send`, expose veto in bridge status, and report the actual ROS
FollowPath UUID. A new bridge session cannot reset Controller/Worker/Guard faults;
all three observers must be present and current. Observation never grants motion.

New CLI inputs (production defaults): `--worker-health-topic /neupan_worker_health`
and `--controller-status-topic /neupan_controller_status`. The Controller candidate
publishes `r1_neupan_execution_status_v1` periodically on its diagnostic topic,
including after Action termination. Legacy diagnostic messages on that topic are
ignored. Missing/old schemas fail admission; mixed versions are not supported.

Do not deploy this snapshot by replacing the production directory wholesale.
Audit baseline drift and apply the source diff as part of the full compatible
release, including the Controller/Worker/Guard candidate. No deployment or real
motion acceptance has been performed.

Unit tests: `PYTHONPATH=integrations/cognav /usr/bin/python3 -m unittest discover -s integrations/cognav/tests -v`.
Isolated ROS test: `packages/navigation_neupan_bridge/test/failstop_cognav_ros.py`
uses ROS Domain192 with explicit opt-in, synthetic sensors, real bridge, real
FollowPath server/controller/Guard/smoother, and optional real spawn solver.
No driver or production `/cmd_vel` is connected.
