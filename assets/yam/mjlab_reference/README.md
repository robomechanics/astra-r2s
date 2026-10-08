# Native mjlab YAM reference

Unmodified model/meshes from [mjlab](https://github.com/mujocolab/mjlab), commit `033ae22a2c7a30a25a6fa77b16c113ed88dd1b55`. Licensed under Apache 2.0, see LICENSE. The SHA256 manifest is in PROVENANCE.json.

This older `yam_v0` model has separate black/white visual meshes, crank gripper geometry, capsule/sphere collisions and a D405 wrist render camera. It is a good visual match to the video. Its joint-frame convention differs from the newer manufacturer YAM v1 models in the parent directory; use its own kinematics consistently. The native mjlab configuration is `mjlab.asset_zoo.robots.i2rt_yam.yam_constants.get_yam_robot_cfg()`.
