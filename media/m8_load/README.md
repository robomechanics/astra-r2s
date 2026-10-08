# M8 load validation evidence

Complete: yes. All conditions accepted: False. Nominal μ=.15 conditions accepted: False.

All running torques are inside the predeclared geometry envelope: True. All scalar mean-pitch comparisons pass: False. All finite zero-torque hold/backdrive diagnostics pass: True.

The fixture has independent passive axial translation and yaw. A finite yaw-speed servo supplies only torque. There is no axial servo, helical joint, equality coupling, or coordinate overwrite. Static tests remove the yaw controller and apply zero yaw torque. These are mechanical diagnostics, separate from the free-body and gripper task.

| μ | dt, µs | Mode | Measured torque, mNm | Predicted, mNm | Lead, mm/rev | Penetration, µm | Accepted |
|---:|---:|:---|---:|---:|---:|---:|:---|
| 0 | 25 | raise | 0.198986 | 0.198944 | 1.250190 | 2.2504 | True |
| 0 | 25 | lower | 0.198901 | 0.198944 | 1.250207 | 2.2504 | True |
| 0.05 | 25 | raise | 0.412281 | 0.407631 | 1.246742 | 2.2111 | True |
| 0.05 | 25 | lower | -0.014693 | -0.008414 | 1.231335 | 2.2111 | False |
| 0.08 | 25 | raise | 0.540994 | 0.533488 | 1.246291 | 2.1880 | True |
| 0.08 | 25 | lower | -0.141907 | -0.132195 | 1.224033 | 2.1880 | False |
| 0.15 | 25 | raise | 0.847671 | 0.829049 | 1.250253 | 2.1221 | True |
| 0.15 | 25 | lower | -0.442028 | -0.419188 | 1.251183 | 2.1221 | False |
| 0.25 | 25 | raise | 1.298161 | 1.255945 | 1.247163 | 2.0203 | False |
| 0.25 | 25 | lower | -0.875716 | -0.824790 | 1.248063 | 2.0203 | False |
| 0.15 | 12.5 | raise | 0.852550 | 0.829049 | 1.250252 | 2.0861 | True |
| 0.15 | 12.5 | lower | -0.446587 | -0.419188 | 1.250402 | 2.0861 | False |

| μ | dt, µs | Duration, s | Late drift, µm | Late mean yaw speed, rad/s | Hold | Backdrive | Accepted |
|---:|---:|---:|---:|---:|:---|:---|:---|
| 0 | 25 | 0.08 | 362.264 | -70.0205 | False | True | True |
| 0.05 | 25 | 0.08 | 0.00121315 | -0.000227534 | True | False | True |
| 0.08 | 25 | 0.08 | 0.00109466 | -0.000136889 | True | False | True |
| 0.15 | 25 | 0.08 | 1.46634e-05 | -0.000128978 | True | False | True |
| 0.25 | 25 | 0.08 | 0.000643935 | -0.000112842 | True | False | True |
| 0.15 | 12.5 | 0.08 | 0.000274059 | -0.000128329 | True | False | True |
| 0.15 | 25 | 0.5 | 0.00251408 | -0.000134672 | True | False | True |

Raw NPZ traces accompany each JSON report. `summary.json` retains every failed gate and separate refinement metrics. Mean torque, lead and penetration refinements are finite-condition checks, not a claim that every dynamic or work statistic has converged.
