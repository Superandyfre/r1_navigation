# R1 NeuPAN HW0 Configuration

These files are restricted to non-motion HW0 shadow validation. The dimensions
and LiDAR filters are candidate values carried from the migration package and
the existing R1 navigation envelope; they are not mechanical measurements.

HW1 is blocked until a recorded measurement replaces the candidate footprint,
wheel geometry, LiDAR `lidar_link -> base_link` transform, point-cloud rate,
height band, and usable range. The safety gate remains the final speed limiter
and permanently clamps reverse commands to zero.
