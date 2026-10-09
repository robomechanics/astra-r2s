# Closed fresh C2/inertia attempt: open-reset recontact failure

This is one complete, unspliced fresh native attempt from independent table/rest spawns. Producer **da69a9cd44a8312cc7b97365faf5e09c27a646e2** has **672 software tests / 74 source files** unchanged before/after testing and the native attempt. The native run closes at **22.08285000017131 s / 441,657 ticks**, native exit **1**, after **3583.482 s / 59.72 min** launch wall time. Its original result remains **passed=false, partial=false, 19/27 checks**. Full schedule selection does not imply full completion.

The left arm acquires and stabilizes the table-supported free block; the right arm physically picks up and transports the separately supported bolt. Actual native entry, stopped direction search and shallow first forward motion execute. The first forward phase reaches only **21.60 µm** conservative formed overlap, with zero loaded fully interior contacts. Its later **0.29775 s** open-settle phase has zero entire-right-robot/bolt contacts, with maximum **65.42 nm axial / 0.162 mrad yaw** drift. The subsequent `reset_open_search_2` aborts after **1.0843 s** of the **1.47262 s** plan when one right pad recontacts the bolt at **1.487 N**. No captured thread, qualified full-pitch lead, completed passive reset, regrasp/qualified turns or complete assembly is established.

The three official audit CLIs execute with exit zero; that describes reader execution, not physical acceptance. The official primary audit remains **13/19 checks, overall false**. Original table/left-retention and passive-property reports pass their executed scopes. The additive contact/pose audit binder is **4be80e698c07137d6182dffa297586af18084569d594b4dc49f47ef187f9e605**, covering all **39 original native files / 36 derivative files**, with **10 pure reader regressions** separate from the 672 software proof. Software/identity success never substitutes for physical checks. The first carried-block demonstration and all earlier progress/cold/failure records remain unchanged.

## Actual closed media and artifact scope

[![Complete original fresh attempt at normal playback](render/demo.gif)](render/demo.mp4)

[Actual abort detail](render/endpoint_detail.png) · [Stopped first forward motion](render/first_forward_stopped_detail.png) · [First saved contact-free open state](render/first_contact_free_open_settle_detail.png) · [Separate ready open-support state](render/first_ready_open_settle_detail.png) · [Dense original reset tracking plot](execution_provenance/media_execution/failure_plots/open_reset_tracking_failure.png)

The clip selects **265 exact saved states at 12 fps**, normal 1×, with **22.083333333333332 s** encoded MP4 and **22,080 ms** GIF duration, versus **22.08285000017131 s** native duration. The exact endpoint occupies the last frame; timestamp quantization adds no interpolated state. All **19 PNG stills** bind exact original saved qpos/qvel. The contact-free still is the **first SAVED** fully-open zero-contact state at **20.70085 s**, not a claim about the first native tick; the later ready100ms still is a distinct state. Media verification `passed=true` means saved-state/encoding identity only. Original native `passed=false`, `partial=false`, exit1 are preserved.

Renderer source SHA is **43aff66a231b7c8ef4223c3bd38411523d700cb4fc70f5d22c246b949a30935e**, archived under both its executed v2 basename and a byte-identical canonical alias. Its render-manifest SHA is **79b3736db15b3fb52345404c70eaa4136b57b2c69a93e8f233f6971e41ce6e6e**. The complete old-source review mirror has all74 exact da69 bytes but GitHEAD9dc4; that mirror HEAD is not relabeled as the native producer. User reproduction below pins the actual da69 producer. Original failed render provenance and subsequent successful media execution remain separate under `execution_provenance/media_execution/`, with the approved four-file plot tree.

Exact artifact layout: `native_full/` for the complete original native run, `software_proof/` for the complete 672-test/74-source package, `independent_audits/` for original official executions and frozen supplemental binder/reports, `execution_provenance/` for root identity/workflow/helper source, `publication_dependencies/` for exact gates/restoration/publication helpers and `render/` for geometry-only closed media. Every original table/pad/native-feedback/inertia-command ledger and imported source/model/launch/log file is preserved. Large artifacts are contiguous, ordered, SHA-bound binary chunks of at most **45,000,000 bytes**; whole-file SHA checks verify lossless reconstruction. No quantization, force dropping, metadata repair, phase splice or body posing is allowed.

Before accepting this package, verify SHA256SUMS and `package_manifest.json`. The manifest binds every reconstructed artifact and all **39 original native files**; the stored-file/checksum counts are computed from the final package below, avoiding circular README-count/hash claims. Native AFTER identity is **481b930e3dadaddd54240ea9548f5decdb98abfdad9c41db05871c84cf5ccaa3**; original launch BEFORE is **f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3**; original trace is **6639eff093a6f1ba5d26540ecb77ca9cdd05e9d48ca17eb56efde4e7b95d0678**. Original software proof is **71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9**.

## Verify and restore without a simulator

From a newer review checkout containing the complete final package:

```sh
task_package=media/m8_table_supported/full_c2_inertia_v2_closed
python - "$task_package" <<'PY_PACKET_CHECK'
from pathlib import Path
import hashlib,json,sys
packet=Path(sys.argv[1]); lines=(packet/'SHA256SUMS').read_text().splitlines()
for line in lines:
    expected,name=line.split('  ',1)
    assert hashlib.sha256((packet/name).read_bytes()).hexdigest()==expected,name
manifest=json.loads((packet/'package_manifest.json').read_text())
assert len(manifest['original_run_file_identities'])==39
print('Stored files:',sum(p.is_file() for p in packet.rglob('*')))
print('Verified checksum entries:',len(lines),'reconstructed artifacts:',len(manifest['artifacts']))
PY_PACKET_CHECK
python "$task_package/reassemble_archives.py" --verify-only
python "$task_package/restore_original_run_layout.py" --verify-only
python "$task_package/reassemble_archives.py" --output outputs/m8_table_supported/closed_c2_artifacts
python "$task_package/restore_original_run_layout.py" --output outputs/m8_table_supported/closed_c2_original_layout
```

Both output destinations must be absent or contain identical already-restored bytes. The standard-library helpers invoke no simulator, verify every stored chunk and reconstructed whole artifact, preserve every original relative native filename, and refuse changed destinations. The original-layout restorer is distinct from full artifact reassembly: the latter also restores complete media/audit/proof/helper trees.

## Isolated source/runtime and original-layout handoff

Use a complete unused clone at the exact producer. The producer predates this evidence package, so copy the COMPLETE package from the newer review checkout into ignored inputs; the NPZ alone is insufficient.

```sh
task_review=/workspace/astra-r2s
task_clone=/workspace/astra-r2s-closed-c2-review
git clone --no-hardlinks "$task_review" "$task_clone"
git -C "$task_clone" switch --detach da69a9cd44a8312cc7b97365faf5e09c27a646e2
mkdir -p "$task_clone/outputs/m8_table_supported/closed_inputs"
cp -a "$task_review/media/m8_table_supported/full_c2_inertia_v2_closed" "$task_clone/outputs/m8_table_supported/closed_inputs/package"
cd "$task_clone"
task_package=outputs/m8_table_supported/closed_inputs/package
task_artifacts=outputs/m8_table_supported/closed_c2_artifacts
task_run=outputs/m8_table_supported/closed_c2_original_layout
python "$task_package/reassemble_archives.py" --verify-only
python "$task_package/reassemble_archives.py" --output "$task_artifacts"
python "$task_package/restore_original_run_layout.py" --output "$task_run"
python - "$task_artifacts" <<'PY_HELPER_COPY'
from pathlib import Path
import shutil,sys
artifacts=Path(sys.argv[1]); base=Path('outputs/m8_table_supported')
copies=[(artifacts/'execution_provenance/verify_closed_supported_run_identity_v2.py',base/'verify_closed_supported_run_identity_v2.py'),
        (artifacts/'publication_dependencies/full_c2_closed_bindings.py',base/'full_c2_closed_bindings.py')]
for src,dst in copies:
    if dst.exists() and dst.read_bytes()!=src.read_bytes():
        raise ValueError('Changed existing helper: '+str(dst))
    dst.parent.mkdir(parents=True,exist_ok=True)
    if not dst.exists(): shutil.copy2(src,dst)
PY_HELPER_COPY
```

Keep these helper bytes exact and refuse a differing existing destination. The root verifier/gate are output-only sources, outside the 74 tested canonical files; the gate expects the verifier at this original root depth. The renderer imports its archived adjacent gate. Complete archived source snapshots/frozen-five-auditor files are provenance; use the full matching producer checkout for Python imports.

On this cloud reuse the verified GCC CPU MuJoCo **3.15.0** runtime. On a new host follow `docs/m8_setup.md` and `scripts/setup.sh` when no other native experiment is active. Stock MuJoCo/MJLab/Newton do not match the contact engine/core/plugin ABI. Recorded core SHA is **58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433**; plugin source SHA is **1c8b5207c5f6c141cc034983ce76c6c1e9cca4114d16e17a4496b94cb637b42a**. Native binary identity is not inferred from source equality. Runtime relocation requires byte-identical recorded libraries and an explicit original-name-to-actual-path map via `--runtime-paths-json`; a different binary is a new experiment/local proof, not exact archived replay.

Verify original closure/source/runtime identities without decoding native arrays or importing a simulator:

```sh
python outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py "$task_run" --repository-root "$PWD" --proof-archive "$task_artifacts/software_proof" --expected-before-sha256 f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3 --expected-after-sha256 481b930e3dadaddd54240ea9548f5decdb98abfdad9c41db05871c84cf5ccaa3 --expected-producer da69a9cd44a8312cc7b97365faf5e09c27a646e2 --expected-proof-sha256 71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9 --output outputs/m8_table_supported/closed_c2_received_identity.json
```

The preserved original root identity remains immutable. The received derivative can bind explicit byte-identical relocated runtime paths and uses its own SHA when passed to the closed renderer. Both identities must retain original native exit 1 and failed acceptance. Do not execute helpers directly from the wrong media/provenance depth or overwrite canonical auditors.

## Geometry-only recorded-state replay

The original media uses archived old-source identity **73c09e18edb23ef55280a7ee631fd964a247540bb5ec689fa9dc8e204baaa49f** and binder **4be80e698c07137d6182dffa297586af18084569d594b4dc49f47ef187f9e605**. Their exact files are `independent_audits/replay_source_identity_old_clone.json` and `independent_audits/independent_audit_binding.json`. The canonical original root identity remains at `execution_provenance/root_identity_report.json`. A recipient's separately generated identity has a different path binding and must receive its own recomputed SHA; do not claim the archived mirror identity for a relocated checkout.

The archived renderer has explicit arguments `--repository-root`, `--proof-archive`, `--identity`, `--identity-sha256`, `--audit-binding`, `--audit-binding-sha256`, `--plugin-library`, `--plugin-sha256`, followed by restored run and fresh output. The library must already be built; replay must not build or overwrite it. The command below uses this cloud's supplied library by absolute path even from the isolated clone; on another host supply an actual byte-matching library path. A differing plugin binary is not silently relabeled as this supplied replay anchor. Set the audit path from `package_manifest.json.independent_audit_binding_artifact` under the reconstructed artifact root, retaining its COMPLETE companion folder. The plugin SHA is an explicit already-built REPLAY binary anchor. The native run records plugin source SHA, not plugin binary SHA, so this anchor must not be described as original native-binary equivalence.

```sh
task_identity=outputs/m8_table_supported/closed_c2_received_identity.json
task_identity_sha=$(python -c 'import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())' "$task_identity")
task_audit="$task_artifacts/independent_audits/independent_audit_binding.json"
task_audit_sha=4be80e698c07137d6182dffa297586af18084569d594b4dc49f47ef187f9e605
task_plugin=/workspace/astra-r2s/thread_lab/plugins/libm8_sdf.so
task_plugin_sha=53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18
scripts/run_m8.sh "$task_artifacts/render/renderer_sources/render_full_c2_inertia_v2_closed.py" --repository-root "$PWD" --proof-archive "$task_artifacts/software_proof" --identity "$task_identity" --identity-sha256 "$task_identity_sha" --audit-binding "$task_audit" --audit-binding-sha256 "$task_audit_sha" --plugin-library "$task_plugin" --plugin-sha256 "$task_plugin_sha" "$task_run" outputs/m8_table_supported/closed_c2_geometry_replay
```

The closure gate checks original AFTER before any trace/report read; identity and the complete frozen binder must match their explicit SHA anchors. The renderer compiles exact archived XML/assets with the already-loaded verified plugin and refreshes only `mj_kinematics`, `mj_comPos`, `mj_camlight`. It executes no `mj_forward`, collision discovery, contact/force solve, integration, interpolation, object drive or hidden/reposed body. Original solved contacts/geometry are **t−dt**, saved qpos/qvel are postintegration **t**, retained command M/J/Jdot/velocity inputs are **t−2dt** after the initialized rows. Displayed PD/FF/control quantities are original commanded records, not measured forces or replay solves.

## Serial official audits after closure

All derivative outputs must be new and outside the immutable run. Use the complete pinned da69 checkout/runtime; do not substitute old e468/abb4/65-source readers. Native integration is not repeated by these audit commands, but their model-based checks must run serially only after closure.

```sh
mkdir -p outputs/m8_table_supported/closed_c2_received_audits
scripts/run_m8.sh scripts/audit_m8_supported_trace.py "$task_run/insertion_trace.npz" --output outputs/m8_table_supported/closed_c2_received_audits/primary.json
scripts/run_m8.sh scripts/audit_m8_left_pad_force_history.py "$task_run/insertion_trace.npz" --output outputs/m8_table_supported/closed_c2_received_audits/left_pad.json
scripts/run_m8.sh scripts/audit_m8_free_joint_properties.py "$task_run/insertion_trace.npz" --output outputs/m8_table_supported/closed_c2_received_audits/passive_properties.json
```

Preserve every original log/return-code/report/source binder and all failed/unexecuted task stages. Exit zero confirms audit execution only. Saved-state kinematics can independently check collision geometry but cannot recreate historical contact forces; use the complete original every-step ledgers. The supplemental contact/pose analysis retains its no-solve and timing scope. At the failed endpoint, original uncapped open PD is149.9777N, bounded to8N; the reset has4,683 force-clipped ticks and zero torque/native-motor clips. Retained same-command pose error6.82512mm is distinct from saved post-step6.82718mm. Three65-point sampled reachable target branches do not prove full-clock or dynamic reachability. The actual native failure is the final left-pad/bolt1.487N contact, not a hypothetical IK failure. Do not infer causal tracking corrections or successful reset from a prospective approximation.

## New fresh native reproduction

The following repeats the original frozen source/configuration as a NEW failed-capable continuous attempt; it is not a replay, a successful-demo recipe or a cold checkpoint continuation. A new runtime requires a new local software proof as documented in `docs/m8_supported_agent_handoff.md`.

```sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --output outputs/m8_table_supported/agent_fresh_c2 --dt .00005 --starting-angular-speed 1 --angular-speed 2 --maximum-entry-dwell 10 --maximum-starting-strokes 5 --qualifying-strokes 2 --axial-damping 200
```

Use an absent output, no `--replay`, no `--maximum-phases` and no parent state. For source-bound orchestration invoke the complete software packet's portable `launch_supported_c2_inertia_v2.py` with exact producer, proof, `--repository-root` and absent output; `--prepare-only` verifies byte identities with zero model initialization/native steps, while removing it runs physics. Preserve new before/after closure, all 74 source/runtime bytes and new raw outputs. The recorded failed attempt cost59.72min; a longer bounded attempt can cost1–2h or more, not a promised runtime.

The later producer `9ae1a9fe76968a4013ea6c39e67718026622b9c0` exposes reset speed and starts a separate fresh `--reset-speed 2` attempt; when this package was published its native outcome was pending. Its new proof/CLI are not substituted into this original da69/default4 failure, and cannot qualify this record. Privileged perfect simulator pose at20kHz, a noncalibrated arm/finger inertia approximation and numerical materials remain. No supported Gym/policy wrapper, trained policy, hardware transfer or complete20kHz state/action dataset is provided. Capture, full-pitch lead, contact-free passive reset, full tightening/preload and robust complete assembly are separate unachieved qualifications.
