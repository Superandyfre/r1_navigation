# NeuPAN native obstacle avoidance (HW1)

The HW1 profile enables `pan.candidate_avoidance_enabled`. NeuPAN now selects
obstacle points by predicted proximity to the nominal horizon while retaining
scan coverage. DUNE projects its learned nonnegative dual features onto the
norm bound used by its training problem. These changes remain inside NeuPAN.

For a threatened reference path NeuPAN optimizes several initial hypotheses,
including detours and, when necessary, retreat or turning hypotheses. It scores
nonlinear rollouts with the same DUNE model and configured NRMP terms, prefers
feasible progress, retains the chosen side and searches further alternatives
across cycles. Returned trajectories are dynamics rollouts, not unvalidated
linearized states. The previous optimized control horizon is shifted each
cycle. Candidate state survives same-goal path and gear revisions and worker
snapshot/restore, but resets with a new goal. If the starting configuration
already has negative DUNE distances, a candidate may still be selected when it
introduces no new overlap, its aggregate existing overlap never increases,
no single point worsens more than the net improvement, and the aggregate
overlap is reduced by the end of the horizon. This permits a modeled escape from
arm-projected overlap without treating the initial pose as proof that all
future motions are impossible. If no evaluated clear or improving candidate
exists, NeuPAN reports `candidate_infeasible`. No independent external
obstacle-speed or braking-distance policy is added.

The existing scan adapter now preserves the actual sensor returns along with
arm-projected points. This is needed because a lateral arm projection is not a
replacement for the measured points when the chassis rotates. It does not make
an independent motion decision. `planner_input_points` records the resulting
input size. Candidate scores, selected side, seed mode and linearization error
are carried in NeuPAN semantic diagnostics.

Run the source-tree unit and synthetic closed-loop tests in the robot's NeuPAN
venv with `third_party/NeuPAN`, `packages/r1_neupan_integration` and
`packages/neupan_ros2` on `PYTHONPATH`. The test module is
`packages/r1_neupan_integration/test/test_neupan_avoidance.py`.

`tools/diagnostics/replay_neupan_scan.py` is a read-only offline static-scan
replay. It expects an NPZ with `retained` and `inflated` 2xN arrays and does not
connect to ROS or send commands. Its model includes the gear state machine but
assumes ideal odometry, a fixed point cloud, a straight reference and no
smoother/actuator dynamics. It checks the chassis rectangle at discrete time
samples only; it does not check arm collisions or prove real-world safety.

The local chair scan at
`/home/rpp/neupan_failstop_ws/robustness/chair-static-20261109.npz` is kept
outside Git. A 140-cycle replay recorded at
`/home/rpp/neupan_failstop_ws/robustness/chair-replay-neupan-20261109.json`
reached x=3.83 m on a 4 m synthetic reference, with 0.68 m peak lateral offset,
two simulated gear transitions and no sampled chassis overlap. The recorded
p95 and max solve durations were 282 ms and 331 ms on that run. These are
specific offline observations, not real-robot acceptance or guarantees.
