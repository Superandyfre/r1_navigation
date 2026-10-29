# A+ release preflight (candidate)

`external_dependencies.py` creates a separate, read-only inventory of external
files observed in saved scene maps, executable records and nested solver runtime
summaries. Supply `--bundle ABSOLUTE_BUNDLE --reports ABSOLUTE_SCENE ... --output
NEW_JSON`; the output must not already exist. `--check JSON` checks current file
hashes and resolved symlink targets and refuses unresolved observations. This
standalone tool is not a deployment preflight gate. It hashes files **after** the
test and cannot prove which bytes were loaded during that test. Non-executable
`/dev/shm/fastrtps_*` maps are retained separately as transient shared-memory data;
executable maps and executable/summary observations never get that exception.
Missing libraries remain unresolved. The inventory is not full dependency
closure: Python imports, lazy modules, drivers, sensors, effective parameters and
runtime arm identity still need independent evidence. `review_complete` remains
false; a successful consistency check never authorizes motion or deployment.

`preflight.py` is a read-only, same-host path/hash check against an independently
reviewed manifest (`r1_neupan_release_v1`). It never generates approval. Keep the
manifest protected and independent of the environment being checked: a freshly
self-generated manifest does not prove compatibility. The test-generated
`wp7-diagnostic-pins-20261029.json` pins only the tested development snapshot;
it has no deployment approval and is not a release artifact.

After sourcing the intended overlay with the same Python interpreter/environment
that will launch the Worker:

```
/usr/bin/python3 tools/release/preflight.py --manifest /absolute/reviewed-manifest.json --compatibility-only
```

Deployment mode (omit `--compatibility-only`) also requires reviewed acceptance
gates, `deployment_approved: true` and `build_testing: false`. The current build
is intentionally denied. Exit 2 means refusal. An optional command following
`--` executes only after deployment checks pass; diagnostic mode cannot execute
a command. This wrapper is not yet wired into the UI7 startup entry.

Mandatory artifacts are defined in `ROLES`. Each has an absolute pinned path and
SHA256. ROS package paths follow first AMENT index selection; libraries follow
LD_LIBRARY_PATH; Python modules follow first regular r1_neupan_integration
package on sys.path. Missing modules in a selected old package are refused.
This initial checker covers the core fail-stop chain and two BT XMLs, not the
entire release inventory (CogNav, all dependencies, model/planner/sensors/driver
and configuration overrides still need the final release bundle).

It does not model arbitrary Python import hooks/zip packages, LD_PRELOAD, RPATH,
RUNPATH, loader cache, dependencies or already loaded libraries. It does not
prove runtime maps, effective ROS parameters, fault semantics, physical stop,
or safe deployment. Verify process maps and effective configuration separately
before arming; do not interpret a diagnostic pass as approval. No robot is
started by the checker itself. Production source and current UI7 are unchanged.

Tests use actual preserved local old/new artifacts and a sentinel command to
verify refusal never dispatches. Run from the development overlay:

```
/usr/bin/python3 tools/release/test_preflight.py -v
```

## Loaded C++ artifacts (diagnostic only)

`runtime_maps.py --manifest PIN.json --bindings PROCESSES.json` checks the
ControllerServer executable, loaded core and NeuPAN plugin, and Guard executable
against four corresponding artifact pins. Bindings contain `controller` and
`guard`, each with the explicitly selected PID and Linux `/proc/PID/stat`
`start_ticks` (field22), so a recycled PID is refused. It verifies live process
identity, mapped canonical paths/inodes, mountinfo device identity and SHA256.
Deleted/replaced mapped objects are refused even if replacement bytes match.
Btrfs maps use mount device identity rather than the stat subvolume device;
foreign mount namespaces are refused. It never arms, launches or approves.

This is a point-in-time C++ artifact check. It does not verify in-memory code
pages, every dependency, Python imports, BT runtime loading, effective ROS
parameters, or continuously prevent post-check changes. Keep release files
immutable and verify the remaining release/runtime inventory separately. It is
not yet integrated with the UI7 startup/arm workflow. Tests cover actual isolated
ROS process maps (candidate accepted and preserved old core refused), wrong
start_ticks, temporary mapped-file replacement and terminated process refusal.

## Development UI7 entry

The candidate `tools/starterbutton/start_navigation_high_sampling_neupan.sh`
sources `release_gate.sh` after sourcing its overlay and before spawning ROS.
The gate requires `${navigation_repo}/releases/a-plus/manifest.json`, or the
explicit `RPP_NEUPAN_RELEASE_MANIFEST` path, and runs deployment checks. There is
no diagnostic-mode bypass in that entry. No approved manifest is shipped; the
current development tree is intentionally refused. Tests run the real helper
against missing, corrupt and unapproved manifests, verify no sentinel command
runs, and check placement before process creation. They do not execute the full
startup script or replace the production button registration.

The production `/home/rpp/r1_navigation` UI7 entry remains unchanged. This is a
candidate integration only. Its eventual release still needs a complete reviewed
inventory, Worker interpreter/import checks, effective configuration and runtime
map checks before arm, plus release/rollback and operator-approved hardware
acceptance. Static preflight alone does not establish these facts.

## Candidate build

`bash tools/release/build_candidate.sh /absolute/new-directory` builds all eight
repository ROS packages in an isolated prefix with `BUILD_TESTING=OFF`, no
symlink-install, and two parallel package jobs. It refuses an existing output
directory or dirty source tree and verifies the Git snapshot did not change
while building. It sources preserved rpp_ws for external dependencies; this is
not a fully hermetic toolchain. Build logs and source SHA are retained. The
script never starts ROS nodes or approves deployment. Acceptance still needs to
use the new artifacts, and a candidate install alone does not contain a complete
UI7 release layout, third-party solver snapshot, runtime/driver inventory or
rollback bundle.

## Candidate no-motion acceptance

After sourcing the candidate's setup.bash, set the exact isolated flags
`ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1 NEUPAN_ISOLATED_ROS_TEST=1`
and `NEUPAN_CHAIN_WORKER=actual-solver`, then run:

```
/usr/bin/python3 tools/release/accept_candidate_ros.py --candidate /absolute/candidate --suite navigator
```

Suites: chain, collisionoff, navigator, restart, startup-tf, revoke-loss,
handshake-loss, source-recovery, owner-loss, owner-protocol,
navigator-owner, navigator-freeze, pending-halt, ack-delay, or all (42 cases,
sequential). CogNav is excluded by user instruction; its historical fixtures
and evidence remain archived. New bundles omit integrations/cognav. Never run multiple
suites concurrently in domain192. The runner verifies the entire saved install
inventory, testing-off caches recorded in it, and absence of the test adapter;
it selects candidate C++ executables/libraries, installed Python, planner/model
and BT XMLs. Existing synthetic sensors/global planner/test configurations and
preserved external Nav2/smoother/solver environment remain dependencies.
These are bounded-scope acceptance tests, not production startup or hardware
acceptance; running them never approves deployment. Preserve raw logs and traces.

`--suite effective-parameters` is a separate, one-scene check of the installed
Controller/Guard/smoother/actual Worker parameter services before the fixture's
arm request. It checks pinned fixture launch values, rejects a wrong shadow-mode
pin, then verifies normal solver output and cancel. It does not certify the
production UI7 overrides or runtime remaps and is not in the default suite.

`effective_parameters.py --pins /absolute/reviewed-pins.json --output
/absolute/new-report.json` reads ListParameters/GetParameters without setting
parameters, arming, or launching nodes. Pin format:

```json
{"schema":"r1_neupan_parameter_pins_v1","nodes":{"/controller_server":{
  "parameters":{"FollowPath.shadow_mode":{"type":1,"value":false}}
}}}
```

ROS parameter types are bool=1, integer=2, double=3, string=4, byte array=5,
bool/integer/double/string arrays=6/7/8/9. Types and values must both match;
non-finite values, unavailable services, absent parameters and duplicate node
names reject. Optional `closed_prefixes:["FollowPath"]` additionally rejects
unreviewed names in that namespace; all names there must be pinned. The example
above is a format example only, not a complete approved production contract.
Pins must come from independently reviewed effective launch inputs, not a live
snapshot being checked against itself. The tool proves the named values only
during a bounded interval, not PID ownership, topic remaps, code identity or
physical readiness. It is not yet connected to the production arm gate.

`launch_parameter_contract.py --bundle /absolute/bundle --calibration
/absolute/calibration.json --output /absolute/new-draft.json` evaluates the
installed HW1 motion and Nav2 control builders without executing any ROS action.
It checks the bundle inventory and selected package roots, merges YAML plus
Launch overrides, preserves exact ROS parameter types, and records intended
resolved command remappings. Default draft covers dual scan, mesh enabled,
collision checking enabled and smoothed commands; `--profile direct` selects
the other command path. The returned `pins` object is input to the live
parameter checker API; the enclosing draft also includes an expected `topology`.
Drafts have `review_complete=false` and authorize no motion. This does not
execute or prove the complete upstream UI7/arm-stream launch path, and it does
not include parameter defaults absent from launch inputs.

`runtime_topology.py --contract /absolute/new-draft.json --output
/absolute/new-graph-report.json` reads actual resolved publisher/subscriber
edges and checks exclusive publishers for final/raw/smoothed commands and
Worker source/health. It rejects wrong topics/types, duplicate node names,
missing nodes or extra publishers. Graph discovery is sampled within a bounded
window; a matching graph does not prove transport delivery or associate a ROS
node with a specific PID. Combine it with process/maps and parameter checks;
these tools do not grant arm permission or establish continuous authorization.

The explicit `effective-parameters` suite additionally checks resolved endpoints
before its test-only arm, rejects a wrong raw topic and a temporary second final
output publisher (no messages published), then verifies normal solver/cancel.

`runtime_admission.py --manifest /absolute/manifest.json --bindings
/absolute/current-pids.json --contract /absolute/reviewed-contract.json --output
/absolute/new-audit.json` composes compatibility/inventory, C++ loaded identity,
live typed parameters and resolved graph checks. It checks file inputs and
mapped identities again after sampling; changed inputs, stale PID start ticks
or any failed stage reject. Static identity failures stop before ROS queries.
The result is a bounded diagnostic (`motion_authorized=false`,
`coverage_complete=false`), never an arm permit. Worker live import/PID binding,
full dependencies, physical readiness and continuous/production arm integration
remain separate. `effective-parameters` now requires `--bundle` for these checks.

Launch drafts preserve `robot_type`/`robot_description` under
`informational_launch_inputs`: N116 does not declare/read these historical YAML
metadata entries. They are excluded from effective parameter pins, rather than
mistakenly treated as runtime configuration. Default pins cover221 explicit
runtime inputs (including82 Worker parameters); missing safety defaults still
require independent review.

`ui7_control_contract.py --bundle /absolute/bundle --calibration
/absolute/calibration.json --environment /absolute/options.json --output
/absolute/new-ui7-draft.json` traces installed selector and intermediate launch
argument forwarding before deriving control pins. Options JSON accepts only
`RPP_NEUPAN_SCAN_SOURCE`, `RPP_NEUPAN_USE_MESH_SELF_FILTER`,
`RPP_NEUPAN_COLLISION_CHECK_ENABLED` and `RPP_NEUPAN_COLLISION_CHECK_DEFAULT`;
empty values follow the button's Bash fallback semantics. Unknown options
reject. Registered `Additional_environment_variables` are loaded first; supplied
JSON is an explicitly reviewed effective override after external startup config,
not a claim about the UI application's environment merge order. The current
registration sets `RPP_NEUPAN_COLLISION_CHECK_DEFAULT=false`; an empty override
JSON therefore produces collision checking disabled, unlike a bare script run.
Its `bash_url` still points to the preserved production tree, not the bundle;
the draft records this and does not change registration. The arm-gate success
callback is evaluated with a synthetic success
event solely to construct control include descriptions; no arm state probe,
Node/LaunchService or hardware driver is executed. The tool does not source
`.rpprc` or certify hardware admission. Review the selected environment and
external shell configuration separately; a draft never grants motion.

User-directed HW1 profile revision: the duplicate final obstacle checker is
removed from the NeuPAN navigation path. HW1 launches accept only
`collision_check_enabled=false`; the button defaults false and rejects an
enable request. With that profile the final runtime node does not construct
GuardLogic or subscribe to scan/mesh/footprint, and their absence cannot block
motion. NeuPAN's own obstacle inputs remain required. Authorization, Worker
health/source freshness, command freshness/non-finite checks and the existing
odom/localization checks remain active. The profile cannot change live.
Historical bundles before this revision do not contain this behavior.

The separate `--suite navigator-tick-loss` is not part of standard `all`: it
requires a deliberately test-only BT decorator, never installed in a candidate.
Compile `packages/navigation_neupan_bridge/test/pause_tick_bt_node.cpp` outside
the source/build/install/release trees using `g++ -shared -fPIC -std=c++17
-DBT_PLUGIN_EXPORT -I/opt/ros/humble/include SOURCE -o
/absolute/new-directory/libtest_neupan_pause_tick_bt_node.so
-L/opt/ros/humble/lib -lbehaviortree_cpp_v3`. Set
`NEUPAN_TEST_TICK_PLUGIN_PATH` to that absolute library path for this one test.
The fixture wraps actual bundle `NeuPANFollowPath` in a test decorator that skips
only its ticks after a marker appears. Navigator lifecycle services and the
outer BT keep running. It checks owner expiry/zero output with a live Worker,
then removes the marker and verifies failure/new-goal refusal. It is evidence
for BT tick loss, not an unmodified production tree or hardware test.

To test a relocated review bundle, source its `install/local_setup.bash` after
the required external ROS overlay and add `--bundle /absolute/bundle` alongside
`--candidate /absolute/original-candidate`. The runner verifies the bundle's
complete payload inventory, build SHA and installed file hashes against the
candidate inventory. C++ components, installed Worker/config/model/BT, CogNav
integration and child solver PYTHONPATH then select the bundle. Test adapters
and synthetic sensor/planner helpers still come from the development test tree.
Retain maps, imported CogNav source paths and Worker summary
`neupan_source_root` records to verify actual loading. This is an isolated test
entry; it does not invoke or validate the full production UI7 startup script.

A pending-ACK halt can deny execution before lazy solver startup. Its successful
result may explicitly record `solver_not_expected`; the checker additionally
requires an independent trace with no ACTIVE context, Source or COMMIT, and
nonempty all-zero output. This proves pre-execution denial only, not solver
provenance. Failed setup or a test without that result still fails missing-source
checks. When a solver starts, every observed source must resolve inside the bundle.

The ACK-delay proxy writes `ack-peer/ready.json` only after its actual downstream
send-goal client reports the service ready. Tests wait for this preparation before
launching Navigator; synthetic inputs keep running. This separates discovery
preparation from the injected two-second response hold and leaves runtime
deadlines unchanged. Each event records `service_ready_on_receive`. Readiness is
local discovery evidence, not proof of server reverse matching; an actual RMW
response timeout remains a failure. Proxy JSON records are replaced atomically.

The Chinese [deployment and rollback runbook](DEPLOYMENT_RUNBOOK.md) describes
the proposed complete-version switch, required operator evidence, refusal
conditions, and rollback sequence. It is a draft awaiting rehearsal. There is
currently no accepted prior A+ release to use as a safe rollback target.

## Review bundle (unapproved)

`package_candidate.py --candidate /absolute/candidate --destination /absolute/new-bundle`
verifies the saved install inventory and copies it into `bundle/install`. It
archives committed startup/release tools, third_party/NeuPAN and the protected
CogNav integration from the clean tooling tree into the same root. The build SHA
and tooling SHA are separate facts. The generated `releases/a-plus/manifest.json`
pins all payload files and core roles with relocated paths; all deployment gates
and approval remain false. Refuse existing destinations; never overwrite a
previous bundle. Run packaging before collecting additional test scripts in the
source tree, or commit those changes first.

`preflight.py` verifies a supplied full inventory in both modes and requires one
in deployment mode. The manifest itself must be independently reviewed/protected;
it cannot establish its own trust. This bundle still depends on preserved ROS,
venv, calibration, drivers and other external artifacts, and is not a complete
validated release/rollback kit. The relocated environment/import check and
inhibited full startup control-flow test do not constitute production startup.

When re-running preflight tests on another candidate, set
`NEUPAN_PREFLIGHT_REPORT_DIR` to a fresh evidence directory to preserve earlier
snapshot pins and analysis. No production path or button registration is changed
by packaging or these tests.
