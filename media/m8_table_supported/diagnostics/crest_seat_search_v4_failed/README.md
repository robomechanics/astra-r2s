# Closed cold crest-search V4: radial failure during smooth braking

This is a **separate cold failed diagnostic** from the original `full_canonical_v1` postintegration checkpoint at **13.194899999975918 s**, row2651, local time reset to zero. It copies exact qpos/qvel/ctrl and original closed grasp/reference calibration, with **no solver warm-start, old force samples or prior force-window reuse**. It starts again from the original full-attempt checkpoint, not the V3 abort. Its footage is never spliced into another trajectory.

After a fresh100ms local support/quiet window, the branch reverses. At **1.6191500000012047 s**, the unchanged measured-crest observer requests CLOSED deceleration after an actual50.0351879µm return. V4 continues the robot's independent yaw target with a **150ms C2 command profile**, initialized from the preceding scheduled theta **−1.84364899rad**, omega **−1.84271420rad/s** and alpha **1.215387rad/s²**. The robot target continues smoothly; no bolt/body pose, screw pitch or groove phase is prescribed by this brake helper.

The branch fails at **1.7091000000013945 s**, **89.95ms into the planned150ms brake**, on the unchanged150µm radial guard: original solved radial offset **150.0112715µm**. The planned brake never finishes. Original native wall time is **260.938879s**, exit **1**; validation remains **passed=false, partial=true**. Unlike V3's abrupt stop, observed boundary Cartesian command peaks are **4.827487N** and **1.071605N·m**, below the unchanged8N/2N·m caps. This improves command continuity while still failing physical alignment; it does not establish a successful stop or thread result.

No quiet-direction confirmation, forward scan, formed/interior capture, opening, reset or full fresh trajectory ran. The final invalid radial sample clears the original observer's epoch/current return flags. Earlier live crest-request and C2-brake metadata remain preserved. Neither the earlier request nor a crest-referenced diagnostic curve retrospectively confirms the failed trial.

Cold startup preserves the unavailable rolling table windows and original failed acceptance labels. It delays only the undefined rolling mean until enough NEW local samples exist; instantaneous pads/table/collision/depth, tip/support, grasp, drive/joint/cap guards remain active. Complete local windows then use the unchanged table90% / positive-left-up10% /99%duty rule. The controller uses privileged perfect-state robot/object feedback and finite robot controls; it is a diagnostic, not learned-policy qualification.

## Inspect actual native media

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Stop-request detail](render/stop_request_detail.png), [actual mid-brake detail](render/mid_brake_detail.png), [radial abort detail](render/abort_detail.png), [full-arm failure view](render/demo.png).
- [Original dense six-panel brake-boundary plot](render/scientific_plot/native_stop_boundary.png).
- Original [validation](native_v4/insertion_validation.json), [cold declaration](native_v4/diagnostic_declaration.json), corrected independent [audit binding](independent_audits_v2/independent_audit_binding.json) and [original execution closure](execution_provenance/crest_seat_search_v4_execution_after.json).

Video/GIF show **21 exact original saved states at12fps**, normal1×, encoded **1.750s** versus original native **1.7091s**. The exact endpoint is the last frame; finite display timestamp quantization adds no interpolated state. The inherited canonical “1× slow motion” footer means factor1 normal playback. The mid-brake still is the first original saved row after the planned midpoint: **1.694200000001363s**, actual **75.05ms** after the request; its exact time/index/state hashes remain in the manifest, not a synthesized75ms pose. All seven stills and21 original frame indices/times are independently checked in `render/media_identity_review.json`.

Rendering uses original qpos/qvel with ONLY **`mj_kinematics`, `mj_comPos`, `mj_camlight`**. There is no `mj_forward`, collision discovery, force solve, integration, interpolation, workpiece posing or extra body hiding. Canonical cameras/filter are unchanged, and clear detail images use real cameras/default visible geometry. Captions and the plot use ORIGINAL preintegration force solves/retained geometry at row **t−50µs**; qpos/qvel and actual aperture are postintegration **t**. Executed command calibration uses the prior retained solve **t−100µs**, except the initialized first command. No force is inferred from sparse qpos or recomputed during replay.

The plot labels signed independent-clock command omega separately from actual angular-speed magnitudes, shows original commanded alpha, radial offset, finite Cartesian wrench norms and native support. Its axial-return curve uses the retained prior REQUEST crest reference for diagnosis only; final cleared current flags remain false. All observed data stop at the actual guard failure, before the planned150ms endpoint. No hypothetical completed brake is shown as measured motion.

## Original raw bytes and separate proof scopes

All original native arrays, JSON metadata, control/phase/event declarations, XML/assets and source snapshots are preserved byte-for-byte: **34,182 ticks /50 feedback data columns**. The three V4 additions are `robot_yaw_brake_active`, commanded angular acceleration and commanded angular jerk. Every original table/left-pad/contact record is retained; no rows, columns, dtypes or binary archive bytes are dropped or reencoded. Whole/stored SHA and the45MB lossless cap are enforced; these originals are below that limit, so no chunks are needed.

The parent producer remains **`6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1`**, original **402 software tests /65 unchanged files**, all archived under `producer_sources/` and `original402_software_proof/`. The new V4 harness **aee1c3d5…** and C2 helper **4bc45250…** have exact original before/after identities outside that canonical proof. `source_proof/reverse_brake_v4_pure_proof.json` separately records **50 pure C2 command tests passed in0.15s**. That test count is distinct from50 raw feedback columns and does not establish native tracking/support/capture. The unchanged observer1d37ef46 and historical V3 harness85c485 had69 synthetic observer/bootstrap contracts; their original record/source remain explicitly historical. No standalone69 log was saved or synthesized. Independent reader tests and shared publisher byte-storage checks retain their own sources, counts and limits.

The closed independent reader verifies every original native row, the analytic C2 command and the archived pure observer replay. Its **17 output-only reader/mass arithmetic regressions** have separate frozen source, log and proof; they are not added to402, historical69 or C2-helper50. Original cold-start **1,999 unavailable table windows** and the inherited false table label remain unchanged. Right-hand world wrench/pad normals are original native values matched across raw and sampled records; individual right-pad local/frame records were not archived. Independent original-local-frame reconstruction covers thread, table and left records only. The original audit's final `scientific_scope` prose retained a stale V3 radial numeral150.848µm; all original numeric fields already correctly gave **150.0112715108498µm**. The separately frozen [v2 reader/report](independent_audits_v2/independent_cold_crest_brake_audit.json) correct only that explanatory text and its reader-source hash. The [narrow correction binding](independent_audits_v2/v1_prose_only_correction_binding.json) verifies all native fields and AST predicates unchanged; its separate17-test rerun remains the same proof scope, not another17 distinct tests. Both original and corrected audit trees are preserved verbatim. The corrected prose is the current explanation; original bytes remain historical provenance.

The separate [robot-inertia capacity diagnostic](independent_audits/independent_robot_inertia_feasibility.json) and [scope note](independent_audits/robot_inertia_scope_note.json) use **19 exact archived actual states**, with no interpolation, integration, collision or force replay. They compile the archived model and use `mj_kinematics`, `mj_comPos`, `mj_crb`, `mj_fullM` and analytic Jacobians. The right-arm mass subblock includes downstream finger inertias, while **off-arm finger accelerations are held zero**; native jaw/pad compliance is not solved and free-bolt inertia is not added. The five task rows cover two transverse translations plus three rotations; axial translation is excluded. Only scheduled orbital tangential/centripetal acceleration and yaw alpha are considered; bias, damping, Jdot and calibration derivatives are not reestimated. Saved post-q time t matches retained geometry for original command label t+100µs, distinct from the preintegration native-force timing above.

This approximate sampled calculation estimates feedforward-alone peaks **5.823872N /1.364297N·m**, but adding them to the already lagging archived PD command estimates **10.161006N /2.324890N·m**, exceeding the unchanged combined8N/2N·m caps. **No feedforward was executed in this V4 trial.** Those estimates do not predict whole-curve native tracking or prove the sole cause of radial drift; a future candidate must retain combined Cartesian and native motor caps, and its PD history will differ. The [equal-model passive-property binding](independent_audits/equal_model_passive_property_binding.json) retains a historical V3 compiled-property report because XML/assets/runtime match exactly. It is static model-property provenance, not a new V4 native-force proof.

Original independent audit bytes and their original relative sibling links are preserved. Exact root before/after aliases resolve both binders' historical `../crest_seat_search_v4_execution_{before,after}.json` records. A complete byte-identical `crest_seat_search_v4_audits/` alias tree resolves the narrow correction's original reader/report/binder references; `independent_audits/` also retains the complete original audit snapshot. Absolute paths and original execution HEAD remain historical provenance. Native/media repeat recipes below do not promise portable reexecution of that path-bound archived reader, which expects the original declared parent and run-name sibling layout. No reader portability adapter or rewritten declaration is supplied.

## Verify preserved bytes

```sh
python media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/reassemble_archives.py --verify-only
python media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/restore_original_run_layout.py --verify-only
```

Both are standard-library tools and invoke no simulator. They verify stored/chunk/whole hashes, exact original filename mappings and refuse to replace differing destinations. Source-only audit snapshots are provenance, not a complete standalone application/runtime.

## Native repeat in an isolated producer checkout

The old producer commit contains neither newer published package. Copy COMPLETE packages from the newer review checkout into ignored inputs of a fresh6e7 checkout, preserving parent model/assets/source/report/all raw ledgers and original closure sidecars. Do not replace the parent with a V3 abort or regenerated scene.

```sh
task_review=/workspace/astra-r2s
task_trial=/workspace/astra-r2s-crest-v4-review
git clone --no-hardlinks "$task_review" "$task_trial"
git -C "$task_trial" checkout --detach 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1
mkdir -p "$task_trial/outputs/m8_table_supported/cold_inputs"
cp -a "$task_review/media/m8_table_supported/full_canonical_v1_failed_evidence" \
  "$task_trial/outputs/m8_table_supported/cold_inputs/parent_v1_package"
cp -a "$task_review/media/m8_table_supported/diagnostics/crest_seat_search_v4_failed" \
  "$task_trial/outputs/m8_table_supported/cold_inputs/crest_v4_package"
cd "$task_trial"
task_parent_package=outputs/m8_table_supported/cold_inputs/parent_v1_package
task_crest_package=outputs/m8_table_supported/cold_inputs/crest_v4_package
python "$task_parent_package/reassemble_archives.py" --verify-only
python "$task_crest_package/reassemble_archives.py" --verify-only
python "$task_parent_package/restore_original_run_layout.py" \
  --output outputs/m8_table_supported/crest_v4_original_parent
mkdir -p outputs/m8_table_supported/diagnostics
cp "$task_crest_package/source_proof/crest_seat_brake_probe_v4.py" \
  outputs/m8_table_supported/diagnostics/crest_seat_brake_probe_v4.py
cp "$task_crest_package/source_proof/crest_seat_observer_v3.py" \
  outputs/m8_table_supported/diagnostics/crest_seat_observer_v3.py
cp "$task_crest_package/source_proof/reverse_brake_v4.py" \
  outputs/m8_table_supported/diagnostics/reverse_brake_v4.py
```

Those direct `diagnostics/*.py` placements are required by the frozen harness's `Path(__file__).resolve().parents[3]` ROOT and adjacent observer/brake imports. Prepare the matched native GCC CPU MuJoCo engine/plugin/bindings with `docs/m8_setup.md`; stock wheel physics is insufficient. Original runtime identities are preserved, and another-machine rebuild must be compared before claiming identical libraries.

The harness verifies all65 canonical source hashes, the immutable parent controller/commit/failed closure, complete hash-bound original model/force/report inputs and exact actually settled checkpoint row2651. It initializes a NEW cold native state without old warmstarts/windows. `--prepare-only` verifies source/layout/input and compiles the matched archived model without a native rollout. A fresh repeat uses:

```sh
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/crest_seat_brake_probe_v4.py \
  --parent outputs/m8_table_supported/crest_v4_original_parent/insertion_trace.npz \
  --checkpoint-time 13.1949 --brake-duration .15 \
  --output outputs/m8_table_supported/diagnostics/crest_v4_repeat
```

The output must not exist. This integrates a NEW branch and never changes the original recorded failure. It may reproduce the failure and does not retroactively qualify this trial.

## Geometry-only recorded-state replay and raw plotting

```sh
scripts/run_m8.sh "$task_crest_package/render/renderer_sources/render_crest_seat_search_v4.py" \
  --repository-root "$PWD" "$task_crest_package/native_v4" \
  outputs/m8_table_supported/diagnostics/crest_v4_geometry_replay
MPLCONFIGDIR=/tmp/m8-media-matplotlib python \
  "$task_crest_package/render/scientific_plot/plot_crest_seat_search_v4_boundary.py" \
  "$task_crest_package/native_v4" outputs/m8_table_supported/diagnostics/crest_v4_plot_replay
```

The portable renderer checks exact65 source bytes and original runtime before/after and original trial files afterward, records emitted media hashes and exact original source-state indices, and never integrates or drives physics. The plot needs NumPy/Matplotlib and original dense raw ledgers only. The separate original media-identity check verifies seven published stills/21frames and encoded durations; it makes no physical qualification claim.

Original full failure, V3 failure, prior cold packages and first carried-block demo remain immutable. This evidence does not certify material calibration, broad convergence, full pitch/flank capture, final seating/preload, passive reset or learned-policy robustness.
