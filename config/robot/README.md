# Robot model sources

Canonical robot descriptions remain in the recorded `rpp_ws/src/robot` repositories;
this reorganization does not move files used by running launches. Their exact Git
commits, dirty changes, URDF/Xacro/meshes and unversioned sources are included in the
snapshot source archives. See `deployment/dependencies.repos` and the snapshot
profile. Do not copy only a URDF while omitting its package mesh dependencies.
