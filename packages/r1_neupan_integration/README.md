# R1 NeuPAN N1 integration

This package is the R1-specific N1 interface test layer. It composes the
already-validated `r1_gazebo` and `r1_lidar_preprocess` packages with the
ROS 2 wrapper `neupan_ros2` at upstream commit `4ffb7ec2dc45ff7ee9024f64083813237906af98`.

The upstream wrapper documents Humble, but its ROS 2 Python node builds and
starts with Jazzy after the N1 environment is selected. Its original default
topics are overridden here:

- input: `/scan_neupan` (`sensor_msgs/msg/LaserScan`, `frame_id=base_link`)
- fixed path input: `/r1_test_path` (`nav_msgs/msg/Path`, actual odom frame)
- output: `/neupan_cmd_vel` (`geometry_msgs/msg/Twist`)
- planned path: `/neupan_plan`
- diagnostic timing: `/neupan_timing`

There is intentionally no remap or relay from `/neupan_cmd_vel` to `/cmd_vel`.
The R1 remains stationary throughout N1.

The N1.5 scheduling correction is enabled by default in this integration
launch: `latest_scan_only=true`. The wrapper keeps only the newest scan and a
MutuallyExclusive timer performs at most one planner forward at a time. A
planner cycle is skipped when no newer scan has arrived, so the same scan is
never planned twice. Set `latest_scan_only:=false` only when reproducing the
N1 baseline.

The wrapper source has a minimal N1.5 instrumentation and scheduling patch in
this workspace: it publishes latest-scan counter, scan arrival time, planner
start/end, stage timings, and output publication time on `/neupan_timing`. It
does not change planner inputs, algorithm code, DUNE, NRMP, robot description,
Gazebo physics, or scan preprocessing. The patch is the diff between
`src/neupan_ros2` and the clean clone at `/home/pinggu/neupan_n1/neupan_ros2`.

The planner configuration uses the upstream small differential-drive proxy
model and checkpoint. It is suitable only for interface and timing evidence;
it is not an R1 geometry model and must not be interpreted as R1 navigation
or a safe velocity configuration.

## Run

Use Python 3.12 and source Jazzy before building. The N1 venv uses an explicit
read-only path to the frozen N0 NeuPAN dependencies; see the generated N1
environment report for exact paths and versions.

```bash
source /opt/ros/jazzy/setup.bash
source /home/pinggu/venvs/neupan_ros_jazzy/bin/activate
cd /home/pinggu/r1_sim_ws
colcon build --symlink-install --packages-select neupan_ros2 r1_neupan_integration
source install/setup.bash
ros2 launch r1_neupan_integration n1_validation.launch.py \
  scenario:=free_space duration_sec:=30 \
  metrics_file:=n1_results/free_space.json
```

Use `scenario:=obstacle` and a different output file for the obstacle case.
Both cases use `headless:=true` internally and do not publish a chassis command.
The free-space world is kept in this package because the existing R1 empty
world intentionally omits the Gazebo Sensors system; the obstacle case uses
the already-validated R1 navigation test world.
