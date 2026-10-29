# Confirmed execution ownership (development protocol)

Protocol set: `A+context1-source2-grant1-health1-owner1`. Development contract;
release/hardware acceptance remains required. Worker generation replacement
and automatic recovery stay disabled.

ControllerServer binds each execution to its actual FollowPath goal UUID, server
boot and plugin boot. UUID text is exactly 32 lowercase hex digits, two per byte.
Humble's human-readable `rclcpp_action::to_string()` does not guarantee that
encoding and must not be used as this protocol's identity encoder.

The client must receive an accepted goal handle before publishing
`/neupan_execution_owner` (`std_msgs/String`, JSON):

```json
{
  "schema": "r1_neupan_execution_owner_v1",
  "authority_boot_id": "current server boot",
  "controller_boot_id": "current plugin boot",
  "navigation_attempt_id": "32 lowercase hex digits from accepted goal UUID",
  "owner_id": "unique client instance",
  "sequence": 1,
  "monotonic": 123.0
}
```

This is a same-host steady-clock protocol. Controller sends ACTIVE context
periodically even before ownership confirmation so discovery or a missed first
context message cannot create a circular handshake dependency. Context alone
never grants movement. The exact execution must receive its first confirmation
within 1 second of begin; no PREPARE/COMMIT or nonzero command is permitted before
confirmation. Once confirmed, source timestamps must remain within 0.5 seconds.
Receipt time never refreshes the lease. Missing confirmation or expiration
irreversibly faults the run; late confirmation, a new goal or component health
cannot clear that fault.

Sequence is positive and increasing; exact replay cannot renew. A conflicting
payload with the same sequence or a changed owner identity faults the execution.
New sequence with non-increasing time does not renew. Wrong execution/boot and
future or old timestamps cannot create permission. This is a correctness and
liveness protocol between trusted components, not a network authentication scheme.

BT renews only after accepted handle reception, no pending ACK, and while its
FollowPath tick returns RUNNING. Halt/stopped ticks stop renewal. CogNav requires
an accepted executing goal, no cancellation, healthy current component records,
a valid parent driver lease and fresh robot inputs on every renewal. Neither
client runs an unconditional authority heartbeat. The existing deny-only revoke
RPC remains a fast path; ownership expiration also works if that RPC is absent.
A blocked Controller cannot renew its independent Guard grant indefinitely.

Ordinary route changes retain the actual execution identity; new FollowPath
goals require fresh ownership. Ownership is only one necessary permission:
explicit run arm, Worker health/Source, matching Guard handshake, command barrier,
fresh physical-safety inputs and final permission checks still apply.

UI7's `NeuPANFollowPath` now revises routes through `/update_follow_path`
(`r1_neupan_runtime/srv/UpdatePath`) while holding the acknowledged action handle.
The request supplies the exact `authority_boot_id`, `navigation_attempt_id` and
`nav_msgs/Path`. The server rejects inactive/retired/wrong identities, invalid
geometry and changed terminals. Acceptance means queued, not motion authorized;
only the action loop applies the newest queued route. Route PREPARE/COMMIT,
Worker/source version checks, ownership expiry and cancellation remain effective.
BT waits at most 0.5 seconds for the service response and revokes on uncertainty.
Material revisions are repeated until the published path version advances,
allowing the existing route debounce to collect consistent observations.
Timestamp-only route changes are ignored. Goal checker state survives revisions.

UI7 supplies the accepted parent pose/poses with the corresponding path; a
changed parent, terminal or controller/checker creates a fresh FollowPath UUID.
An identical parent request with identical pose headers is not distinguishable
from continuation through these BT ports. Clients requiring distinct executions
for repeated identical destinations must cancel and await termination before
resubmission, or supply a fresh stamped parent request. CogNav is unchanged and
need not use the optional route service; normal FollowPath preemption retains
the original fresh-UUID semantics.

Action outcome, final zero Twist and actual robot stopping are distinct evidence.
Tests use isolated domain192, synthetic inputs and no driver. Physical stopping,
full startup reliability, complete races and release/rollback acceptance remain
required; local unit or scoped chain passes do not establish deployability.
