# R1 Navigation

Version-controlled R1 high-sampling navigation stack: NeuPAN, dual lidar fusion,
dynamic dual-arm horizontal footprint, Nav2 integration, and final collision guard.

## Included packages

- `packages/navigation_2d`: map, localization, high-sampling navigation launches, and RViz.
- `packages/navigation_neupan_bridge`: Nav2 bridge, dynamic footprint, costmap plugins, and the read-only arm-state adapter.
- `packages/r1_lidar_preprocess`: Livox and rear 2D scan fusion.
- `packages/r1_neupan_integration` and `packages/neupan_ros2`: R1 NeuPAN runtime integration.
- `packages/r1_collision_guard`: final command and collision validation.
- `third_party/NeuPAN`: versioned NeuPAN core source.

## Runtime contract

The dual-lidar motion launch uses `/scan_livox` and `/scan` to produce
`/scan_neupan`. It reads `/left/joint_states` and `/right/joint_states` through
read-only adapters; it never starts an arm driver or publishes an arm command.
The dynamic footprint contains the base and both arm horizontal projections.
`inflation_radius` is only an external safety margin (0.15 m).

No launch in this repository publishes a navigation goal by itself. A valid
2D Pose Estimate is required after a localization restart before real navigation.

## Build

Build this repository as an isolated colcon base path first. Do not place it in
the active workspace alongside the legacy packages until the migration switch is
complete, because ROS package names must be unique.

```bash
colcon build --base-paths /home/rpp/r1_navigation/packages \
  --build-base /home/rpp/r1_navigation/build \
  --install-base /home/rpp/r1_navigation/install \
  --log-base /home/rpp/r1_navigation/log
```

See `docs/MIGRATION.md` for the reversible workspace cutover procedure.
