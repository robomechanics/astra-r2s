"""Package exact closed native pickup/entry prefix; never simulate or replay."""
from pathlib import Path
import hashlib
import json
import math
import subprocess

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/face120_prefix_v1'
AUDITS = ROOT/'outputs/m8_table_supported/face120_prefix_v1_audits'
RENDER = ROOT/'outputs/m8_table_supported/face120_prefix_v1_render'
PROOF = ROOT/'media/m8_table_supported/software_proof_281'
TARGET = ROOT/'media/m8_table_supported/face120_pickup_entry_v1'
COMMIT = 'b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def strict(raw):
    return json.loads(raw, parse_constant=lambda v:
        (_ for _ in ()).throw(ValueError(v)))


def finite_derivative(value, path='', replaced=None):
    if replaced is None:
        replaced = []
    if isinstance(value,float) and not math.isfinite(value):
        replaced.append({'path':path,'original':str(value),'exported_as':None})
        return None
    if isinstance(value,dict):
        return {k:finite_derivative(v,path+'/'+k,replaced) for k,v in value.items()}
    if isinstance(value,list):
        return [finite_derivative(v,path+'/'+str(i),replaced) for i,v in enumerate(value)]
    return value


def encoded(value):
    return (json.dumps(value,indent=2,allow_nan=False)+'\n').encode()


def main():
    if TARGET.exists():
        raise FileExistsError(TARGET)
    raw_validation = (RUN/'insertion_validation.json').read_bytes()
    report = json.loads(raw_validation)
    assert report['partial'] is True and report['passed'] is False and report['aborted'] is None
    assert report['phases'][-1]['phase'] == 'feed_to_entry'
    files = {}
    for path in sorted(RUN.rglob('*')):
        if not path.is_file():
            continue
        name = str(path.relative_to(RUN))
        if name == 'insertion_validation.json':
            files['validation_original.json.txt'] = raw_validation
        else:
            files[name] = path.read_bytes()
    converted = []
    finite_report = finite_derivative(report,replaced=converted)
    files['validation.json'] = encoded(finite_report)
    files['validation_serialization.json'] = encoded({
        'scope':'Strict JSON derivative only. Original report bytes and original NPZ metadata remain untouched.',
        'original_sha256':sha(raw_validation),
        'strict_derivative_sha256':sha(files['validation.json']),
        'nonfinite_original_values_exported_as_null':converted})
    for path in sorted(AUDITS.rglob('*')):
        if path.is_file():
            files['independent_audits/' + str(path.relative_to(AUDITS))] = path.read_bytes()
    for path in sorted(RENDER.rglob('*')):
        if path.is_file():
            files['render/' + str(path.relative_to(RENDER))] = path.read_bytes()
    for path in sorted(PROOF.iterdir()):
        if path.is_file():
            files['software_proof/' + path.name] = path.read_bytes()
    proof = strict(files['software_proof/software_proof.json'])
    assert proof['passed'] and proof['tests_passed'] == 281 and proof['source_hashes_unchanged']
    for name,digest in proof['source_hashes'].items():
        raw = subprocess.run(['git','show',COMMIT+':'+name],cwd=ROOT,
            check=True,stdout=subprocess.PIPE).stdout
        if sha(raw) != digest:
            raise ValueError('Published commit differs from proof: '+name)
        files['frozen_software_sources/' + name] = raw
    assert len(proof['source_hashes']) == 63
    assert files['controller_source.py'] == files['frozen_software_sources/yam_twin/m8_supported_simulation.py']
    assert sha(files['controller_source.py']) == report['controller_module_sha256']
    assert sha(files['scene_source.py']) == report['scene_source_sha256']
    assert sha(files['scene.xml']) == report['model_xml_sha256']
    for name,digest in report['recorded_source_dependencies_sha256'].items():
        assert sha(files[name]) == digest,name
    for field in ['left_pad_force_history','table_support_force_history']:
        identity = report[field]
        assert sha(files[identity['filename']]) == identity['sha256']
    binding = strict(files['independent_audits/audit_binding.json'])
    assert binding['trace_sha256'] == sha(files['insertion_trace.npz'])
    assert binding['original_validation_sha256'] == sha(raw_validation)
    assert binding['source_scope']['producer_controller_sha256'] == report['controller_module_sha256']
    assert binding['source_scope']['producer_snapshot_committed_at_run'] is False
    assert binding['source_scope']['historical213_proof_qualifies_this_new_producer'] is False
    for name,digest in binding['audits_sha256'].items():
        assert sha(files['independent_audits/'+name]) == digest,name
    for name,digest in binding['audit_sources_sha256'].items():
        assert sha(files['independent_audits/audit_sources/'+name]) == digest,name
    audit = strict(files['independent_audits/independent_supported_audit.json'])
    assert audit['passed'] is False
    assert audit['archive_and_actuation']['all_archived_whole_dependencies_match_recorded']
    assert audit['original_load_history']['identity_and_consecutive_full_step_coverage_verified']
    renderer = strict(files['render/render_manifest.json'])
    lift = strict(files['render/lift_bolt_manifest.json'])
    assert renderer['trajectory_sha256'] == sha(files['insertion_trace.npz'])
    assert renderer['original_validation_partial'] is True and renderer['original_validation_passed'] is False
    assert renderer['exact_saved_state_replay'] and not renderer['physics_integration_during_render']
    assert not renderer['state_interpolation'] and renderer['slow_motion'] == 1
    assert lift['trajectory_sha256'] == renderer['trajectory_sha256']
    assert renderer['model_xml_sha256'] == report['model_xml_sha256']
    assert renderer['runtime'] == renderer['original_recorded_runtime']
    for manifest in [renderer,lift]:
        for name,digest in manifest['media_sha256'].items():
            assert sha(files['render/'+name]) == digest,name
    gif = strict(files['render/gif_manifest.json'])
    assert gif['source_video_sha256'] == sha(files['render/demo.mp4'])
    assert gif['source_render_manifest_sha256'] == sha(files['render/render_manifest.json'])
    assert gif['source_sha256'] == sha(files['render/gif_source.py'])
    assert gif['gif_sha256'] == sha(files['render/demo.gif'])
    assert gif['frames'] == renderer['video_frames'] == 77 and gif['playback_speed'] == 1
    context = strict(files['render/table_context/table_view_manifest.json'])
    assert context['trajectory_sha256'] == renderer['trajectory_sha256']
    assert context['model_identity']['model_xml_sha256'] == report['model_xml_sha256']
    assert context['image_sha256'] == sha(files['render/table_context/table_view.png'])
    assert context['source_sha256'] == sha(files['render/table_context/render_face120_table_detail.py'])
    assert context['exact_saved_state_replay'] and not context['physics_integration']
    assert not context['physical_bodies_hidden_or_geometry_changed']
    for name,digest in context['dependency_sha256'].items():
        assert sha(files['render/table_context/'+name]) == digest,name
    for label in ['kinematic_face120_route_v2','kinematic_face120_open_reset_v1']:
        folder = ROOT/'outputs/m8_table_supported/diagnostics'/label
        for path in sorted(folder.iterdir()):
            if path.is_file():
                files['kinematic_rationale/'+label+'/'+path.name] = path.read_bytes()
    planner = strict(files['kinematic_rationale/kinematic_face120_route_v2/summary.json'])
    assert planner['study_sha256'] == sha(files['kinematic_rationale/kinematic_face120_route_v2/study.json'])
    margin = report['acceptance_checks']['native_joint_limits']['minimum_observed_margin_rad']
    producer_binding = {
        'scope':'Later published source/software binding for the exact earlier closed partial native prefix. This supplements the original uncommitted-at-run audit provenance; original audit binding and native data are untouched.',
        'published_producer_commit':COMMIT,
        'publication_was_after_native_trial_and_original_audit_binding':True,
        'producer_snapshot_committed_at_run':False,
        'original_audit_binding_sha256':sha(files['independent_audits/audit_binding.json']),
        'trace_sha256':sha(files['insertion_trace.npz']),
        'original_validation_sha256':sha(raw_validation),
        'controller_module_sha256':report['controller_module_sha256'],
        'archived_controller_equals_published_producer_source':True,
        'software_proof_sha256':sha(files['software_proof/software_proof.json']),
        'software_tests_passed':281,'software_wall_seconds':proof['wall_seconds'],
        'all_proof_source_hashes_match_published_commit':True,
        'proof_sources':proof['source_hashes'],
        'proof_source_count':63,
        'historical213_proof_qualifies_this_new_producer':False,
        'software_tests_are_physics_or_capture_qualification':False}
    files['published_producer_binding.json'] = encoded(producer_binding)
    files['package_source.py'] = Path(__file__).read_bytes()
    manifest = {
        'status':'closed_partial_native_pickup_transport_cone_entry_pilot',
        'producer_commit':COMMIT,
        'producer_binding_sha256':sha(files['published_producer_binding.json']),
        'original_report_partial':True,'original_report_passed':False,
        'original_abort':None,'independent_full_acceptance_passed':False,
        'thread_capture_qualified':False,'starting_thread_stroke_executed':False,
        'opening_release_or_passive_reset_executed':False,
        'trajectory_sha256':sha(files['insertion_trace.npz']),
        'first_physics_time_s':renderer['first_physics_time_s'],
        'last_physics_time_s':renderer['last_physics_time_s'],
        'native_phase_names':[phase['phase'] for phase in report['phases']],
        'exact_saved_state_replay':True,'state_interpolation':False,
        'physics_integration_during_render':False,
        'forces_from_original_solved_samples_only':True,
        'state_force_timing_scope':renderer['state_force_timing_scope'],
        'normal_playback_speed':1,'mp4_fps':12,'video_frames':77,
        'gif_centisecond_duration_ms':gif['requested_duration_ms'],
        'supplementary_table_view_scope':context['scope'],
        'supplementary_table_view_state_time_s':context['state_time_s'],
        'supplementary_table_view_state_index':context['sample_index'],
        'model_fingerprint':report['model_fingerprint'],
        'model_xml_sha256':report['model_xml_sha256'],
        'controller_module_sha256':report['controller_module_sha256'],
        'scene_source_sha256':report['scene_source_sha256'],
        'runtime':report['runtime'],
        'actual_native_minimum_joint_margin_rad':margin,
        'independent_saved_pose_minimum_right_joint_margin_rad':audit['saved_geometry']['sampled_native_joint_margins_rad']['right'],
        'kinematic_only_planner_parent_route_minimum_joint_margin_rad':planner['summary']['parent_route']['minimum_joint_margin_rad'],
        'planner_scope':'The separate sampled fixed-pose planner is not integrated physics, does not measure grip or forces, and does not certify a continuous path. Its 0.10156-rad planned route margin is not the actual native 0.017349-rad margin.',
        'software_tests_passed':281,'software_proof_source_count':63,
        'software_proof_is_separate_from_native_physics_acceptance':True,
        'independent_focused_auditor_tests_not_added_to_281':binding['auditor_focused_tests'],
        'serialization_nonfinite_to_null_paths':converted,
        'scope':renderer['scope'],
        'all_original_raw_bytes_preserved':True,
        'all_artifacts_below_45MB_no_chunks_required':True,
        'files':{name:{'bytes':len(raw),'sha256':sha(raw)} for name,raw in sorted(files.items())}}
    assert max(map(len,files.values())) < 45_000_000
    for name,raw in files.items():
        if name.endswith('.json'):
            strict(raw)
    files['package_manifest.json'] = encoded(manifest)
    files['README.md'] = make_readme(manifest).encode()
    TARGET.mkdir(parents=True)
    for name,raw in files.items():
        path=TARGET/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(files.items())))
    for name,raw in files.items():
        assert (TARGET/name).read_bytes() == raw,name
    print(json.dumps({'folder':str(TARGET.relative_to(ROOT)),
        'files':len(files)+1,'bytes':sum(map(len,files.values())),
        'largest_file_bytes':max(map(len,files.values())),
        'strict_json_replacements':len(converted),
        'native_duration_s':renderer['last_physics_time_s'],
        'actual_native_joint_margin_rad':margin,'thread_capture_qualified':False},indent=2))


def make_readme(manifest):
    return '''# Native +120-degree-face pickup and cone-entry pilot

This **6.3983-second closed native prefix** starts with both robot hands
separate from the free workpieces. The left hand acquires a side grip on the
female block and stabilizes it on the plain solid table. The right hand picks
up the separate male bolt using another hex-head face, clears the physical
rest, transports it above the bore and feeds to measured native cone entry.

**Original overall status: partial=true, passed=false, aborted=null.**
Pickup, grasp retention, rest clearance and table bearing are observed. This
pilot stops before any starting rotation. Formed overlap is zero; no captured
thread, qualified lead, opening/release, passive reset or completed assembly is
claimed. Cone entry is not formed-thread capture.

![Actual native entry endpoint](render/demo.png)

![Actual pre-entry block clamp and hex-head detail](render/table_context/table_view.png)

The supplementary detail uses two real camera angles of the **same actual
4.82-second saved state**: left side clamp on the table-supported block and
right jaws around the actual hex head. No physical bodies are hidden by render
settings or removed. Its original force caption, exact state hashes, camera
parameters and executed source are bound in `render/table_context/`.

[Normal 1x MP4](render/demo.mp4) · [Normal 1x GIF](render/demo.gif) ·
[Actual lift-complete still](render/lift_bolt.png)

The 77 video frames replay exact archived qpos/qvel at 12 fps using native
`mj_forward` only. No integration, state interpolation, manually posed free
objects or grasp welds are added. Displayed forces are the selected original
native solve records at saved time minus dt; replay forces are not used.
MP4 playback is normal 1x. GIF alternating 80/90 ms centisecond durations
preserve the same total time within one centisecond. The renderer/GIF source,
runtime, model and exact screenshot state hashes are in `render/`.

The minimum actual all-step native joint margin is **0.017349 rad**. The
independent saved-pose check gives **0.0173493 rad**. The separate sampled
kinematic planner reports **0.10156 rad** for its reconstructed parent route;
that value is not the native result. The archived planner is a fixed-pose
reach/collision rationale without native integration, forces or grip proof.
It does not certify a continuous trajectory or thread/reset success.

The original native whole-task table ledger confirms the table continues to
bear block weight while the left hand stabilizes it. The minimum active 100 ms
mean table share is **99.047%**, maximum mean positive left-hand upward share
is **0.953%**, and table loaded duty is **100%**. The bolt has no later world
support after pickup. Measured grip translation slip reaches **0.12717 mm**;
this is retained as an actual result, rather than replacing the original grasp
reference. Minimum whole-shaft/rest clearance during guarded transfer is
**14.939 mm**, above the original 10 mm guard. Read all original criteria in
`validation.json` and independent reports, including the incomplete thread and
reset criteria.

The trial and original audit binding were produced from an **uncommitted
orientation-only controller snapshot**. That exact snapshot was later
published as commit **b2b13ff39cd47c48afd19b38f83e9a405c9d6e32**. The separate
`published_producer_binding.json` verifies the archived controller bytes and
all **63 source hashes** against that commit. Its full software suite passed
**281 tests in 83.016 seconds**, with sources unchanged across execution.
This software proof does not turn the partial physics pilot into a successful
thread trajectory. The earlier audit provenance remains byte unchanged;
historical 213-test proof does not qualify this new controller. The focused
15-test auditor proof stays separate and is not added to 281.

- `insertion_trace.npz`: closed original native state/sample archive.
- `insertion_trace_partial.npz`: exact earlier original checkpoint archive,
  retained separately without splicing or relabeling.
- `table_support_force_history.npz`, `left_pad_force_history.npz`: complete
  original all-step ledgers; no rows removed or values quantized.
- `scene.xml`, `supported_scene.zip`, `scene_source.py`, `controller_source.py`,
  `recorded_sources/`: original model, meshes and producer source snapshots.
- `validation_original.json.txt`: unchanged original validation bytes.
  `validation.json` is a strict JSON derivative only, with nonfinite unobserved
  minima exported as null and each conversion recorded separately.
- `independent_audits/`: four closed independent physics/grasp/rest/free-body
  audits, their unchanged binding and exact source snapshots.
- `software_proof/`, `frozen_software_sources/`: full 281-test log/proof and
  exact published bytes for every proof-bound source.
- `kinematic_rationale/`: separate frozen sampled-route and open-workspace
  studies. These did not integrate a native opening/reset trajectory.
- `package_manifest.json`, `SHA256SUMS`: all trace, model, source, proof,
  native-versus-planner margin, presentation and artifact identities.

Every artifact is smaller than 45 MB, so no binary chunks are needed. Verify
the complete package from its directory:

```sh
sha256sum -c SHA256SUMS
```

Reconstruction uses the matched native CPU engine described in
`docs/m8_setup.md`, including its pinned source patch and plugin. A stock
MuJoCo wheel is insufficient. Core/plugin hashes are archived; independently
rebuilt native libraries require comparison before claiming identical physics.
Use an isolated checkout of the pinned producer. To replay this recording:

```sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --replay media/m8_table_supported/face120_pickup_entry_v1/insertion_trace.npz --output outputs/m8_table_supported/replayed_face120 --video --fps 12 --slow-motion 1
```

To regenerate a separate fresh native prefix, use a fresh output path and the
same archived source/configuration:

```sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --output outputs/m8_table_supported/repeated_face120 --dt .00005 --maximum-phases 12 --starting-angular-speed 1 --angular-speed 2 --axial-damping 50
```

The first carried-block demonstration remains at
`media/m8_table_pickup/full`. Earlier failed trials and cold diagnostics remain
separate immutable evidence. This package makes no calibrated-material,
full-seating/preload, numerical convergence or policy robustness claim.
'''


if __name__ == '__main__':
    main()
