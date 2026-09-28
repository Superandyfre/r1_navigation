# Migration

The active navigation stack remains in `/home/rpp/rpp_ws/src` until this
repository has passed the static checks below. Do not keep both source trees in
one colcon discovery path: they contain identical package names.

1. Stop navigation and verify that no goal is active and `/cmd_vel` has no
   non-zero output.
2. Build this repository in its isolated build, install, and log directories.
3. Run the no-motion checks: dual scan rates, `r1_arm_footprint` state `READY`,
   0.15 m local/global inflation, localization health, and no autonomous goal.
4. Move the legacy navigation package directories out of `rpp_ws/src` into a
   dated backup directory. Do not delete them.
5. Add only this repository's `packages/` directory to the workspace source
   discovery path, rebuild, then restart navigation with no goal.
6. Set a fresh 2D Pose Estimate and repeat the no-motion checks before any
   real navigation target is sent.

Rollback consists of stopping navigation, restoring the backed-up source
directories, rebuilding the legacy workspace, and starting without a goal.
