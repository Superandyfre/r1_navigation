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
The HW1 global costmap inflation radius is 0.30 m; its local costmap radius is
0.55 m. The traversable global soft-clearance field extends to 0.50 m.

No launch in this repository publishes a navigation goal by itself. A valid
2D Pose Estimate is required after a localization restart before real navigation.

## Build

Build this repository as an isolated colcon overlay. UI7 sources its install
after the robot workspace and checks that all navigation packages resolve here.
The older workspace sources remain available for rollback and must not be
edited for UI7 changes.

```bash
/home/rpp/r1_navigation/tools/build_ui7.sh
```

See `docs/MIGRATION.md` for the reversible workspace cutover procedure.

## Previous UI 7 source snapshot

See [navigation7 synchronization record](docs/NAVIGATION7_SYNC_20261009.md) for the active robot source, entry scripts, validation, and research reuse limits.
