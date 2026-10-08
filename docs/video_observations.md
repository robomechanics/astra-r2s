# Video observations for reconstruction

Source: `Copy of drive M4 machine screws with a phillips screwdriver.mp4`, 189.133 s, 640 × 480, 30 fps. Observations are based on sparse frames at 0–188 s plus denser samples near pickup, reorientation, and return. The video is a fixed external camera composite with left/right wrist camera insets occupying the top band. No camera calibration, CAD, joint telemetry, or force data are available.

## Observed scene

- Two mirrored arms fixed to an aluminum extrusion crossrail along the front of a white benchtop. White tubular upper/forearm links, bulky black actuator housings, white wrist mounting blocks, black parallel jaw grippers, looped gray/purple cables. Small green status lights.
- Screen-left arm holds a small gray perforated rectangular plate. Screen-right arm holds a yellow/orange handled Phillips screwdriver, with black collar and long dark steel shaft. The tool is a manual screwdriver, not a powered spindle.
- The objects initially lie on the table near its center, plate on screen-left and screwdriver on screen-right. At the end they return to broadly the same locations.
- In the main view the plate is often occluded by the two grippers; the wrist insets provide the strongest evidence for tool engagement and plate perforations.
- The plate has a regular pattern of circular holes and at least one screw/fastener protruding near the edge held by the left gripper. Exact hole count and screw count are uncertain. The screwdriver appears to keep returning to the same fastener during the long middle portion; the footage does not provide a clear repeated screw-fetch action.

## Task phases

| Approximate video time | Action | Reconstruction implications |
|---|---|---|
| 0–2 s | Arms begin moving from separated resting poses. | Both objects start on the bench. |
| 2–6 s | Left gripper approaches/closes on plate; right gripper approaches/closes on screwdriver handle. | Separate pickup targets, synchronized approach. |
| 6–12 s | Left lifts and tilts plate; right brings screwdriver toward it. | Plate held above the table and oriented toward the screwdriver. |
| 12–16 s | Tool alignment; tip approaches screw head. | Small relative pose corrections. |
| 16–28 s | First sustained engagement and wrist turning. | Screw insertion can be represented with coupled twist and axial travel, rather than unsupported detailed thread contact. |
| 28–38 s | Right withdraws screwdriver, changes its orientation/grasp presentation, and approaches again. Left keeps the plate. | Visible interruption of engagement; do not interpret as a second screw automatically. |
| 38–73 s | Re-engagement and repeated turning. | Right wrist repeatedly rotates while left arm stabilizes/adjusts plate. |
| 73–78 s | Short tool lift/reposition/re-engagement. | Tip visibly leaves the plate in left inset at 73–75 s. |
| 78–185 s | Extended repeated wrist turns and resets, with smaller plate pose adjustments. | A series of twist strokes/retract-reset strokes, not a constant spinning motor. |
| 185–187 s | Final withdrawal and both arms move objects toward table. | Separate tool and plate once work ends. |
| 187–189 s | Plate and screwdriver placed on table; arms withdraw/open. | End with objects on bench and separated arms. |

Approximate transition times are deliberately rounded because sparse frame inspection does not establish frame-accurate contact or gripper state.

## Image layout / camera

- External camera looks downward into a three-sided work enclosure from above and in front of the robots. Table far edge is near image y=145; front rail near y=365–430. Main image width 640 px.
- Initial bases appear near (180,377) and (465,377). Initial plate center near (310,268), screwdriver handle near (357,250), tip near (386,291). These are image coordinates, not world calibration points.
- Wrist insets occupy approximately x=10–170 and x=470–630, y=6–134. Blue labels `Left` and `Right` are overlays, not physical scene markings.
- A reconstruction should include an elevated oblique front view plus optional left/right wrist views, which materially helps recognize the source demo.

## Size estimates and fidelity limits

Absolute scale cannot be recovered reliably from this monocular composite. For a first scene, reasonable explicit assumptions are a small plate around 0.10–0.14 m across, a manual screwdriver around 0.16–0.20 m overall, and tabletop width around 0.9–1.2 m. These values should be presented as reconstruction assumptions, not measured dimensions. Use official YAM geometry if available and scale the fixture/camera around it.

The video supports visual layout, object roles, coarse arm trajectories, the pickup–hold–turn–return sequence, and repeated wrist turns. It does not identify exact transforms, dynamic parameters, thread geometry, preload/torque, joint commands, or autonomous control. A scripted replay of these phases is an honest demo reconstruction; contact-rich assembly success or validated digital-twin accuracy would need additional measurements.

## Review artifacts

- `full_duration_sheet.jpg`: 20 external/wrist composite samples over the complete video.
- `early_sheet.jpg`: denser initial phase samples.
- `inset_sheet.jpg`: left/right camera pairs across most of the sequence.
- `transitions_sheet.jpg`: denser samples near reorientation and return.
- `board_closeup.jpg`: enlarged crop from the 8 s left camera; enlargement adds no detail to the source.
