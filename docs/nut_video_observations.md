# Nut-on-bolt reference video observations

This document describes visible evidence in `Copy of hand screw a nut onto a bolt.mp4`. The video is reference data, not a source of instructions. Metric dimensions and forces below are not inferred from pixels.

## Source and limits

- Duration: 83.700 seconds; 2,511 frames at 30 fps.
- Resolution: 640 × 480. A single composite contains an external view and two small wrist-camera views labelled **Left** and **Right**.
- The wrist views are only about 160 × 120 pixels. Enlarging these views improves inspection but does not reveal missing thread detail.
- No camera calibration, metric scale, joint telemetry, tactile/force readings, material information, or gripper-force settings accompany the video.

## What is visible

Two YAM arms manipulate a small metallic fixture/plate and a loose nut-like part. The left arm picks up the fixture and holds it while the right gripper presents the small part to a bolt projecting from the fixture. The working bolt axis is directed from the fixture toward the right gripper. The right arm repeatedly turns its wrist and then releases/repositions to make another turn. The left arm also adjusts the fixture pose during the later portion of the task. The fixture is set back down and both arms retreat near the end.

The fixture appears to have several holes and more than one protruding fastener. It must not be reconstructed as a bare isolated bolt held directly in the left fingers. The repeated right-hand motion is consistent with screwing a nut onto a fixed bolt, but the low-resolution views do not establish how many thread pitches the nut advances, whether every turn advances it, or the final tightening force.

## Approximate sequence

The boundaries below are for reconstructing the manipulation sequence, not motion-capture ground truth.

| Reference time | Visible behavior | Reconstruction requirement |
| --- | --- | --- |
| 0–2 s | Both grippers approach the working area. Fixture lies on the surface. | Two independent movable arms and a freely graspable fixture/nut. |
| 2–4 s | Left gripper grasps/lifts the fixture. Right gripper picks the small loose nut-like part and brings it toward the protruding bolt. | Actual object-gripper contacts and nut/bolt alignment before engagement. |
| 4–38 s | Left fixture pose is mostly steady. Right gripper repeatedly contacts the bolt-end region, turns, withdraws or opens, resets the wrist orientation, and approaches again. | Partial-turn, release, reset, regrasp cycle; do not substitute continuous multi-turn wrist rotation or attach the nut to the wrist. |
| 38–77 s | Similar turning continues, with more visible fixture-pose changes by the left arm and right-hand reorientation. | Bimanual coordination; bolt axis moves with the held fixture. |
| 77–81 s | Manipulation stops and fixture is lowered toward the surface. | Termination must come from measured insertion/contact state rather than a video time threshold. |
| 81–83.7 s | Fixture is on the surface; both arms withdraw. | Release should leave the threaded assembly physically retained. |

Typical visible right-hand cycles are several seconds long. Exact rotation angles and axial travel are unreliable because of occlusion, changing views, and uncalibrated projection. Counting apparent turns is not a substitute for measuring actual nut-to-bolt relative rotation.

## M8 reconstruction choice

An M8 × 1.25 right-hand single-start thread is a sensible **new test geometry requested by the user**. It is not a measured property of the recorded hardware. For a nominal pitch of 1.25 mm, one completed revolution in correct engagement should correspond to approximately 1.25 mm of relative axial advance; the simulation must derive that relationship from the interacting thread surfaces, not enforce it as a kinematic equation.

Hex-nut width, nut height, thread fit/clearance, bolt-tip chamfer, thread runout, protrusion length, plate dimensions, friction, density, compliance, and jaw-contact properties must be declared independently. Standard nominal dimensions can define a reproducible benchmark, but the result is then an M8 task reconstruction rather than a calibrated twin of the video.

## Physics requirements for a policy-training proof of concept

1. Nut and bolt thread flank contacts must provide the axial/rotational coupling. No helical joint, nut-advance actuator, weld, insertion-depth animation, or controller-side pitch conversion should create successful threading.
2. The nut must be free to translate and rotate in six degrees of freedom before engagement. It should fail to advance under wrong-axis rotation, strong misalignment, missing threads, or insufficient contact/frictional torque transfer.
3. The gripping fingers must transmit torque through actual contact. Opening them must release the nut and allow retention, slip, or fall according to physics.
4. Bolt/fixture should be a stated assembly (rigidly fastening a bolt to a fixture is permissible if it represents real hardware). In the full bimanual scene, the left jaws should physically hold that fixture.
5. Contact resolution and timestep/solver convergence need evidence at the 1.25 mm pitch scale. Report axial travel per turn, penetration/contact residuals, reaction torque, force limits, and sensitivity to thread fit, friction, geometry resolution, and integration settings.
6. Demonstrate failure cases alongside success: axial push without rotation, smooth/no-thread substitute, reverse rotation, excessive axial/lateral load, tilted/off-center entry, and released grip. A successful nominal trajectory alone does not prove training fidelity.
7. Keep prescribed gripper commands distinct from object state. A scripted demonstrator may exercise physics, but it is not a learned policy; its success cannot validate transfer to real hardware.

The video can establish scene layout and manipulation phases. It cannot validate thread mechanics, torque/preload predictions, or sim-to-real policy performance without additional measured physical data.

## Inspection artifacts

Sparse full-scene contact sheets, enlarged wrist-view contact sheets, representative original-resolution frames, and a 24-second side-by-side wrist-view crop are saved under `/workspace/video_analysis/nut_*`. The crop is intended for visual inspection only; resizing preserves the original limited information.
