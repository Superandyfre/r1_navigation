#!/bin/bash
# Read-only subscription recording. Stop with Ctrl-C; never publishes velocity.
set -euo pipefail
if [[ $# -ne 1 || "$1" != /* || -e "$1" ]]; then
  echo "Usage: $0 /absolute/new-output-directory" >&2
  exit 2
fi
output="$1"
mkdir -p "$output"
/usr/bin/python3 - "$output/recording.json" <<PY
import json,os,sys,datetime
with open(sys.argv[1],"x") as f:
    json.dump(dict(schema="ui7_observation_v1", read_only=True,
      started_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
      ros_domain_id=os.environ.get("ROS_DOMAIN_ID","0"),
      ros_localhost_only=os.environ.get("ROS_LOCALHOST_ONLY","0")),f,indent=2)
PY
exec ros2 bag record --storage sqlite3 --output "$output/bag" \
 /odom /tf /tf_static /scan_neupan /scan_neupan_mesh_shadow \
 /left_arm/joint_states /right_arm/joint_states /plan /neupan_path /neupan_goal \
 /neupan_semantics /neupan_timing /neupan_planner_result /neupan_execution_context \
 /neupan_execution_owner /neupan_motion_grant /neupan_worker_health /neupan_controller_status \
 /r1_collision_guard/status /r1_localization_health /indoor_location/status \
 /r1_arm_footprint /r1_arm_footprint_planner /r1_arm_footprint/status \
 /r1_arm_mesh_filter/status /r1_dynamic_footprint_sync/status /r1_passage_clearance/status \
 /neupan_nav2_raw_cmd_vel /neupan_nav2_smoothed_cmd_vel /cmd_vel \
 /navigate_to_pose/_action/status /follow_path/_action/status /rosout
