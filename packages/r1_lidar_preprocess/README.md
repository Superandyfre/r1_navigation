# r1_lidar_preprocess

This package provides the reusable R1 navigation preprocessing layer:

```text
PointCloud2 -> TF to base_link -> height filter -> 2D projection -> LaserScan
```

It uses the ROS 2 Jazzy `pointcloud_to_laserscan` package and has no Gazebo
dependency.  The same launch file can consume the simulated `/lidar/points`
topic or a future hardware PointCloud2 topic through `input_topic`.

## Profiles

`lidar_smoke` matches the current 64 x 8, 360 degree, 10 Hz simulation sensor.
Its output increment is 2*pi/64 (5.625 degrees), so the conversion does not
claim angular detail that the input does not contain.

`lidar_navigation` uses the proposed 720 x 16, 360 degree, 10 Hz simulation
sensor and a 0.5 degree LaserScan output increment.  Both sensor profiles are
temporary simulation values because the exported R1 description does not
contain the hardware LiDAR model, driver, or scan specification.

The Jazzy node includes both angle endpoints.  With `[-pi, pi]` and a 0.5
degree increment, the measured output therefore has 721 range entries and
720 angular intervals; this is consistent with the configured angular span.

## Height band

The current launch spawns the robot with `base_link` at z=0.09 m.  The wheel
and caster collision bottoms are approximately z=-0.083 m in `base_link`, so
the ground plane is approximately z=-0.09 m.  The navigation experiment uses
`min_height=-0.04 m`, retaining returns at least about 0.05 m above ground,
and `max_height=0.55 m`, retaining the mobile-base obstacle band while
discarding overhead structure above about 0.64 m world height.  This band is
intended to detect walls, boxes, legs, and low obstacles; it is not a claim
about the real robot's final navigation calibration.

## Launch

```bash
ros2 launch r1_lidar_preprocess pointcloud_to_scan.launch.py \
  profile:=lidar_navigation \
  input_topic:=/lidar/points \
  output_topic:=/scan_neupan \
  use_sim_time:=true
```

Keep a subscriber such as `ros2 topic hz /scan_neupan` active during tests.
The upstream node may use lazy subscription behavior when no scan subscriber
exists.
## Dual LiDAR scan fusion

`scan_fusion_node` transforms two `sensor_msgs/msg/LaserScan` inputs into
`base_link`, removes chassis returns, and publishes their closest valid return
per 0.5 degree bin. It is intended for the R1 NeuPAN chain: Livox supplies the
front scan and BlueSea supplies rear coverage.

The node publishes only while both inputs are fresh and synchronized. This is
deliberate: a missing rear scan must make the downstream collision guard stop
the chassis instead of creating an unobserved reversing region.

The high-sampling NeuPAN launcher accepts `scan_source:=livox_only` (default)
or `scan_source:=dual`. The Starterbutton wrapper forwards
`RPP_NEUPAN_SCAN_SOURCE`; set it to `dual` only after the stationary and
low-speed rear-obstacle checks have passed.
