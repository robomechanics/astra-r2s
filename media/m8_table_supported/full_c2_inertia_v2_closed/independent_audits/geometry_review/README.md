Closed reset reachability review

This diagnostic preserves the original failed fresh native run from producer da69a9cd44a8312cc7b97365faf5e09c27a646e2. It compiles the exact archived XML/assets with the already-built explicit replay plugin, then calls only mj_kinematics for original bounded ArmIK/FK. No plugin build, mj_forward, collision query, contact/force solve or mj_step runs. The replay binary SHA is supplied separately; the original run recorded the plugin SOURCE SHA, not that binary SHA.

Three 65-point release-to-minus-pi sweeps use the actual saved release/reset-start six-joint state as a warm seed, holding the release, reset-start, or abort saved hole frame fixed in turn. These are sampled geometric branches with verified residuals, not a continuous all-clock or collision/dynamic feasibility proof. The exact frozen release calibration comes from the original physical event; no head feedback or groove phase is added.

The complete planned orbit has worst position residual about 13 nm, orientation residual 0.269 microrad, and minimum joint margin 0.185655 rad. The full remaining goal also solves from the actual failed arm state. The failed saved post-state instead has 6.82718 mm / 0.0395043 rad target error; the original command observer has 6.82512 mm / 0.0395124 rad at retained geometry time t-2dt. Original forces belong to t-dt, saved qpos/qvel to t. These clocks are kept separate. The analytic pad-axis projections use post-state geometry only: lack of a separating axis is inconclusive and is never called a native penetration or force.

Files in source_archive preserve the exact used controller, kinematics, target helper, compiler helper and archived-asset auditor source. reset_reachability_report.json binds all original 74 tested source hashes, exact trace/report/AFTER/model/assets/runtime/core and supplied plugin binary identities. Original inputs were SHA-checked unchanged before/after. The execution record is a geometry diagnostic, not a new source software proof or native success.

Reproduce only into a fresh scratch location by copying this script there and running from the same source-bound repository/root/runtime:

    PYTHONDONTWRITEBYTECODE=1 LP_NUM_THREADS=1 /workspace/.venvs/m8-contact/bin/python /fresh/scratch/review_reset_reachability.py > /fresh/scratch/review_reset_reachability.log 2>&1

ROOT and original run depth are explicit in the archived script; any relocation requires a separately reviewed derivative with new identity. Do not write into the original native run or replace this frozen report.
