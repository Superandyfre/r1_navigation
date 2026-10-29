# A+ release preflight (candidate)

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

Suites: chain, navigator, cognav, restart, startup-tf, revoke-loss,
handshake-loss, source-recovery, owner-loss, ack-delay, or all (30 cases, sequential). Never run multiple
suites concurrently in domain192. The runner verifies the entire saved install
inventory, testing-off caches recorded in it, and absence of the test adapter;
it selects candidate C++ executables/libraries, installed Python, planner/model
and BT XMLs. Existing synthetic sensors/global planner/test configurations and
preserved external Nav2/smoother/solver environment remain dependencies. CogNav
uses the protected source integration, not a full packaged driver/vision stack.
These are bounded-scope acceptance tests, not production startup or hardware
acceptance; running them never approves deployment. Preserve raw logs and traces.

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
