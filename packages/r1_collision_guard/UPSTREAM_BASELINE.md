# Upstream Baseline

The collision geometry and scan-to-predicted-footprint approach are derived
from Nav2 Collision Monitor (Apache-2.0), using the installed ROS 2 Humble
`nav2_collision_monitor` 1.1.17 as the API baseline.

`/home/rpp/navigation2-main.zip` was reviewed as an upstream reference only:

```text
SHA256 29fa0c9652e0672ca8a6ccae2779fee3506a25faf3284c1452446344a4e62932
nav2_collision_monitor package version 1.5.0
```

This package deliberately remains a Humble-local package. It implements the
mainline fail-closed intent for stale inputs and adds the R1 braking rollout;
it does not replace Nav2 packages in the workspace.
