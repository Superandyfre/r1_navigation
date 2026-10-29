# Baseline capture

Captured and checksum-verified; not navigation validated.

- No normal-navigation rosbag collected; motion not authorized for this snapshot.
- No independent restore/build/navigation acceptance yet.
- System package lock has no complete offline .deb/wheel cache or OS image; bare-host offline recovery unproven.
- Absolute paths, external symlinks and native/CUDA system dependencies need restore-host verification.
- Some credential/cache files excluded; inspect environment/excluded.json.
- Runtime parameters: 44 nodes queried, 16 failed or timed out; see raw per-node outcomes.
- 513 symlinks have missing original targets; references recorded, recovery needs resolution.
- File archives observed no in-copy changes; sequential multi-process capture is not an atomic machine image.

Verified: all 29 git bundles pass git bundle verify and include the recorded commit; 4 restore/filter checks pass; isolated calibration extraction matches the original SHA256. These are bounded artifact checks, not full environment recovery or navigation acceptance.
