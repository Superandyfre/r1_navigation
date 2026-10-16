# UI7 Navigation Source

`/home/rpp/r1_navigation` is the source of truth for the UI7 navigation stack.
The UI7 button in starterbutton points to
`tools/starterbutton/start_navigation_high_sampling_neupan.sh`. That script
sources the robot workspace for hardware dependencies, then this repository's
install overlay, and pins the NeuPAN Python import to `third_party/NeuPAN`.
It checks that the five navigation ROS packages resolve to this repository.

The old starterbutton script path is a compatibility symlink for a running UI
process that cached its button configuration before this change.

The old source directories in `/home/rpp/rpp_ws/src/application/navigation_2d`
and `/home/rpp/rpp_ws/src/application/neupan_migration` are retained as a
rollback snapshot. Do not edit them for new UI7 navigation changes. Upstream
Nav2, robot description, hardware drivers, localization data, and calibration
remain external dependencies.

Build from this repository after changing navigation code:

```bash
/home/rpp/r1_navigation/tools/build_ui7.sh
```

The current navigation process keeps its loaded code until restarted. Before
using the updated UI7 entry, stop navigation with no active goal and verify
the chassis is stationary. Start UI7 without a goal, set a fresh 2D Pose
Estimate, and check package paths, LiDAR data, localization, and zero command
output before sending a target.

To roll back, stop navigation, restore UI7's `bash_url` to
`~/rpp_tools/starterbutton/.bin/start_navigation_high_sampling_neupan.sh`,
replace that compatibility symlink with the saved script at
`/home/rpp/r1_navigation_presync_20261016/active_starterbutton_script.sh`,
then start UI7 without a goal. The legacy workspace install and source
directories are left untouched.
