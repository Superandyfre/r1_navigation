# Isolated Fast DDS service-response observer

This diagnostic interposes the locally installed Humble rmw_fastrtps shared
implementation. It forwards each request/response exactly once. It does not retry,
change QoS, alter the request identity, or extend any runtime safety deadline.
Before/after queries take the listener mutex and logging changes timing, so this
is evidence about the observed execution, not a transparent timing measurement.

Build manually outside the source/install tree, using the locally installed
headers. The observer is deliberately absent from CMake/install/release payloads:

```python
from pathlib import Path
import subprocess
source = Path('/home/rpp/neupan_failstop_ws/src/r1_navigation/packages/navigation_neupan_bridge/test/rmw_response_observer.cpp')
output = Path('/absolute/new-evidence-directory/rmw_response_observer.so')
subprocess.run([
    'g++', '-shared', '-fPIC', '-std=c++17', str(source), '-o', str(output),
    '-ldl', '-L/opt/ros/humble/lib', '-lrmw_fastrtps_shared_cpp', '-lfastrtps',
    *['-I' + str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
], check=True)
```

Run only an isolated test process with `ROS_DOMAIN_ID=192`,
`ROS_LOCALHOST_ONLY=1`, `NEUPAN_ISOLATED_ROS_TEST=1` and a per-command
`LD_PRELOAD=/absolute/new-evidence-directory/rmw_response_observer.so`.
Never export the preload globally or apply it to the production robot.
Both wrappers log only in the isolated test domain. Original functions are
resolved from the already loaded implementation library (the RMW loader uses
local symbol scope, so RTLD_NEXT alone is insufficient). The first failed
RTLD_NEXT diagnostic attempt was preserved in evidence; it is not a navigation
regression result.

`RMW_RESPONSE_OBSERVE` records the response target GUID, local listener match
before/after, original return value and original call duration. On failure,
`RMW_WRITER_MATCHES` additionally queries actual writer matched subscriptions;
the API result must be checked before interpreting that list. Client request
logs include cached/actual response-reader GUID and local match counts. These
are observations, not a fix or proof of the global discovery root cause.
Request records also include match counts immediately before the original send,
and both records include the same-host steady-clock timestamp, so the two
directions can be correlated by reader GUID, service and sequence. The response
timestamp is the call start; `seconds` includes the observer's initial listener
query and the original call. A client-side match is not proof that the server's
reverse discovery state is already complete.

The test harness also accepts `NEUPAN_LIFECYCLE_CLIENT_MODE=precreated` for a
bounded A/B experiment: create persistent ChangeState clients before launching
servers. Default remains `ephemeral`; neither mode sleeps to settle discovery or
retries state transitions. A pass in one mode must not erase failures in another.
