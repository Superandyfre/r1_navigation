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

`NEUPAN_RMW_OBSERVER_FAILURE_ONLY=1` reduces instrumentation: successful responses
return without listener queries/logging, and only Navigator change-state requests
are logged. A failed response is queried only after the original call returns;
`matched_before=-1` explicitly means not measured. It still interposes calls,
reads the steady clock and logs configure requests, so it is not transparent.
On failure `RMW_WRITER_MATCH_STATUS` queries the writer publication-match status
after the failed send. Check its return code before interpreting counts/GUID.
The installed Fast DDS matched-subscriptions list API returns UNSUPPORTED(2),
so an empty list from that API does not prove no reader was matched. Match-status
counts are aggregate; even a supported count alone cannot identify every reader.

Do not infer UDP-only operation from a participant XML profile alone under
Humble rmw_fastrtps6.2.10 with `ROS_LOCALHOST_ONLY=1`: its participant creation
code appends a localhost UDP transport and an SHM transport after loading XML.
The diagnostic XML A/B/A retained SHM maps in every role and did not isolate
SHM from UDP. Existing user transports are also retained; any future diagnostic
XML must explicitly whitelist loopback on its own UDP descriptor. No transport
profile from this diagnostic is a production recommendation. Inspect effective
participant transport QoS and maps before claiming a transport was disabled.

`dds_participant_transport_probe.cpp` is a separate, manually built diagnostic
interposer, never CMake-built or installed. Compile outside source/build/install
with the local include directories and `-shared -fPIC -std=c++17 -ldl
-L/opt/ros/humble/lib -lfastrtps`. Preload it only for an isolated command.
`NEUPAN_DDS_PARTICIPANT_DIAGNOSTIC=observe` records input, outgoing and actual
post-creation participant QoS without changing the input transports.
`udp-loopback` is an **active test intervention**: it removes SHM, disables
builtin transports, preserves copies of existing UDPv4 descriptors and explicitly
limits them to 127.0.0.1. It rejects unknown transports or no existing UDP and
requires domain192, ROS_LOCALHOST_ONLY=1 and NEUPAN_ISOLATED_ROS_TEST=1.
The original create-participant function is called exactly once. Unknown mode
or non-isolated use fails rather than changing a production participant.
Actual post-creation QoS plus process maps must both validate the intervention.
Do not describe `udp-loopback` as read-only or a production middleware fix.

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

`failstop_navigator_configure_probe.py` is a separate configure-only diagnostic:
source the selected bundle environment, then pass `--bundle ABSOLUTE_BUNDLE
--navigator ABSOLUTE_NAVIGATOR --report NEW_DIRECTORY --rounds 12` under the same
domain192/localhost/opt-in flags. It alternates mode order, creates a fresh actual
Navigator process for each call, and compares clients created before the server
with clients created after node discovery. It sends configure exactly once,
queries state without retrying configure, then stops its own child. It never
activates Navigator, sends a goal, starts a driver, or claims solver provenance.
Saved process maps must contain the actual selected BT library. This reduced
experiment excludes the Controller/Worker/Guard/smoother coexistence of the full
acceptance harness; a pass is not a fix or full startup acceptance.
Add `--with-chain` for a second diagnostic with one persistent, actual isolated
Controller/Guard/smoother/Worker-node fixture sending synthetic inputs in a
separate process. It sends no goals, so the lazy solver is not expected to start.
Only Navigator remains configure-only; background components are activated by
the reused chain fixture. This changes coexistence but still does not recreate
the full suite's repeated chain teardown/startup or prior motion traffic. Keep
the two reports separate and do not infer a cause from passes alone.

`accept_candidate_ros.py --suite navigator-configure-churn` is a separate
12-case diagnostic with fresh full peer/chain fixtures and original configure
client timing. It omits only Navigator activation and all goals. It is excluded
from standard all43; no solver provenance is claimed because no solver should
start. All bundle/install hashes and actual BT loading checks still run. A
failed configure response remains a test failure; a subsequent GetState query
only records the state, without retrying configure or changing its deadline.
