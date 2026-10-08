# YAM reference assets

Unmodified YAM v1 arm, linear_4310/crank_4310 grippers, and matching bimanual station models from [I2RT Robotics](https://github.com/i2rt-robotics/i2rt), commit `120c3c81400171174604e503943f8d1ebc891058`. These assets are licensed under MIT; retain `LICENSE` when redistributing. `PROVENANCE.json` records paths and SHA-256 checksums.

The station MJCF models include two six-DOF arms, gripper fingers, TCP sites, wrist D405 camera meshes, and an overhead D405 camera mesh. They provide kinematics and inertial data, with no actuators, defaults, camera renderer definitions, or keyframes. The default base separation is 0.61 m. This is a public manufacturer value and may be adjusted in the generated scene to match the video.

Use `station/yam_station_crank_4310_d405/yam_station_crank_4310_d405.xml` for the angular claw grippers that appear closest to the video. The visible black/white finishes should be applied in the generated scene; source geoms retain the vendor CAD colors. The exact gripper variant cannot be established confidently from the monocular footage, so the `linear_4310` alternate is also provided.

The original relative directory structure is retained so mesh paths resolve directly. Arm model details are in `arm/yam/v1/README.md`; station extrinsics are in each station README.
