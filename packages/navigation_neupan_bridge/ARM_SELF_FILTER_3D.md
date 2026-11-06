# Arm self-return filtering

`mesh_filter_mode:=3d` selects the time-aligned filter in the motion launch.
The UI7 starter defaults to `3d` and accepts `RPP_NEUPAN_MESH_FILTER_MODE=2d`
for the previous scan path. This source change does not alter an already-running
UI7 deployment. Disabling mesh filtering bypasses either selection.

The 3D path filters Livox points before point-cloud-to-scan conversion and the
rear laser beams before scan fusion. It uses each point/beam timestamp and TF
history in 5 ms bins, preserving source frames and timestamps. A point near a
mesh surface is removed within 0.02 m. For a mesh whose triangle topology is
verified closed, its interior is also removed. The dynamic robot footprint and
mesh components are still produced separately for the planner.

The filter does not publish velocity or authorize motion. It queues up to four
frames per sensor and waits at most 0.12 s for historical TF. During a short
wait, it can republish a previously filtered frame only with its original
stamp and only while that stamp is at most 0.20 s old. After 0.08 s, a recent
*earlier* TF may be used only when its last 50 ms shows limited motion; this is
reported as degraded. Missing or malformed data is not given a new timestamp.
These bounds mitigate brief gaps; they cannot guarantee uninterrupted motion
when input, transforms, or other navigation components remain unavailable.

The `status_topic` is `/r1_arm_self_filter_3d/status` on the 3D motion path.
It includes per-sensor state, reason, input and output timestamps, processing
latency, dropped frames, maximum pose-time error, and `open_mesh_frames`.
The footprint status topic `/r1_arm_footprint/status` separately exposes
left/right joint message and source-stamp ages, the mesh geometry version,
age since validation, and the latest maximum mesh and left/right gripper TF
ages. A value of `-1` means that age has not been observed. During a TF gap,
ages increase from their last verified values; geometry is not declared new.

Open meshes have surface filtering but no reliable interior classification.
The bundled robot models include open meshes, including a large opening in an
arm link; do not claim complete interior removal without corrected, validated
models. Occluded real obstacles and unknown attachments likewise cannot be
reconstructed by this filter.

## Optional attachments

Set `RPP_ARM_EXTRA_MESH_CONFIG` to the absolute path of a YAML file, for example:

```yaml
schema: r1_arm_extra_meshes_v1
meshes:
  - frame: held_object
    uri: package://my_robot_description/meshes/object.stl
    scale: 1.0
```

Every entry must have a valid TF frame, loadable mesh URI and finite positive
scale in metres. Model each mounted tool or carried object explicitly; the
same list is used for sensor filtering and the robot footprint. Configuration
is read at launch and must be updated and the owned runtime restarted after a
model changes. A missing attachment transform is reported rather than silently
presented as a verified complete footprint.
