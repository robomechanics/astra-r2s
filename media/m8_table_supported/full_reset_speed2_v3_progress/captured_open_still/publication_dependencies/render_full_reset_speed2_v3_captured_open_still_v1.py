"""One actual captured-OPEN endpoint from an approved immutable progress snapshot.

Source-only preparation until root approves one real snapshot/still execution.
Never reads the live trial. No plugin build/model regeneration/forward/collision/
contact force/integration. Reuses the exact approved archived-only compiler.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import types

SNAPSHOT_HELPER_SHA = '91db4f94c295585b8f0756c816dfe1179f65ee6529d0a260a24d55f5b9135f56'
APPROVED_RENDERER_SHA = '18c83b4d1747ec15988783dddf268b7e637e2cb8057bfa405931f552feacfa3a'
ARCHIVED_COMPILER_ORIGIN_SHA = 'ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8'
PLUGIN_SHA = '53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18'
BEFORE_SHA = '1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
PROOF_SHA = '9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
PRODUCER = '9ae1a9fe76968a4013ea6c39e67718026622b9c0'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value:
        (_ for _ in ()).throw(ValueError(value)))


def same_json(left, right):
    return json.dumps(left, sort_keys=True, separators=(',', ':'), allow_nan=False) == json.dumps(right, sort_keys=True, separators=(',', ':'), allow_nan=False)


def rooted(root, value):
    value = Path(value)
    return (value if value.is_absolute() else root/value).resolve()


def main(args):
    root = args.repository_root.resolve()
    snapshot = rooted(root, args.snapshot)
    output = rooted(root, args.output)
    immutable_extra = [root/'media', root/'outputs/m8_table_supported/full_reset_speed2_v3_audits',
        *[rooted(root,value) for value in args.immutable_root]]
    if (not output.is_relative_to(root/'outputs/m8_table_supported')
            or any(output.is_relative_to(folder) or folder.is_relative_to(output) for folder in immutable_extra)
            or output.exists() or output.is_relative_to(snapshot) or snapshot.is_relative_to(output)):
        raise ValueError('A fresh output outside the immutable snapshot is required')
    binding_path = snapshot/'snapshot_binding.json'
    if sha(binding_path) != args.snapshot_binding_sha256:
        raise ValueError('Immutable snapshot binding differs from its supplied SHA')
    binding = read(binding_path)
    if (binding['schema'] != 'fresh-full-reset-speed2-v3-phase-prefix-snapshot-v1'
            or binding['immutable_snapshot'] is not True
            or binding['historical_phase_end_progress'] is not True
            or binding['full_native_run_closed'] is not False):
        raise ValueError('Only an approved immutable historical progress snapshot is accepted')
    original_live_run = Path(binding['original_run']).resolve()
    if snapshot == original_live_run or output.is_relative_to(original_live_run) or original_live_run.is_relative_to(output):
        raise ValueError('Live/original trial is never a renderer input or output')
    if binding['helper_sha256'] != SNAPSHOT_HELPER_SHA or sha(snapshot/'snapshot_helper_source.py') != SNAPSHOT_HELPER_SHA:
        raise ValueError('Snapshot must bind the exact approved91db byte-copy helper')
    files = binding['file_sha256']
    actual_names = {str(p.relative_to(snapshot)) for p in snapshot.rglob('*') if p.is_file()}
    if actual_names != set(files)|{'snapshot_binding.json','snapshot_helper_source.py'}:
        raise ValueError('Immutable snapshot inventory differs')
    for name, digest in files.items():
        path = snapshot/name
        if path.is_symlink() or not path.resolve().is_relative_to(snapshot) or sha(path) != digest:
            raise ValueError('Immutable snapshot byte differs: '+name)
    launch, proof = read(snapshot/'run_publication_identity.json'), read(snapshot/'software_tests.json')
    if sha(snapshot/'run_publication_identity.json') != BEFORE_SHA or sha(snapshot/'software_tests.json') != PROOF_SHA:
        raise ValueError('Original producer/proof anchors differ')
    sources = launch['source_hashes_before']
    if (launch['producer_commit'] != PRODUCER or len(sources) != 74
            or not same_json(sources, binding['source74_sha256_before_after_copy'])
            or not same_json(sources, proof['source_hashes'])
            or proof['passed'] is not True or proof['tests_passed'] != 672):
        raise ValueError('Snapshot lacks the exact completed672/74 producer proof')
    runtime_files = launch['runtime_file_sha256_before']
    for name, digest in sources.items():
        if sha(root/name) != digest:
            raise ValueError('Current producer source differs: '+name)
    for name, digest in runtime_files.items():
        if sha(name) != digest:
            raise ValueError('Recorded native core/runtime bytes differ: '+name)
    approved = rooted(root, args.approved_renderer_source)
    compiler_origin = rooted(root, args.compiler_origin_source)
    if sha(approved) != APPROVED_RENDERER_SHA or sha(compiler_origin) != ARCHIVED_COMPILER_ORIGIN_SHA:
        raise ValueError('Exact18c8 compiler/ab16 source provenance differs')
    plugin = rooted(root, args.plugin_library)
    if args.plugin_sha256 != PLUGIN_SHA or sha(plugin) != PLUGIN_SHA:
        raise ValueError('Explicit existing replay535 plugin anchor differs; no builds')

    # All native/array/model imports occur only after immutable/source/core gates.
    os.environ['MUJOCO_GL'] = 'egl'
    os.environ['LP_NUM_THREADS'] = '1'
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(root))
    module = types.ModuleType('approved_saved_progress_geometry')
    module.__file__ = str(approved)
    exec(compile(approved.read_bytes(), str(approved), 'exec'), module.__dict__)
    np, mujoco = module.np, module.mujoco
    Image, ImageDraw, ImageFont = module.Image, module.ImageDraw, module.ImageFont
    trace = snapshot/binding['trace_filename']
    if sha(trace) != binding['trajectory_sha256']:
        raise ValueError('Snapshot trace SHA differs')
    with np.load(trace, allow_pickle=False) as archive:
        times = archive['time']
        qpos, qvel = archive['qpos'][-1].copy(), archive['qvel'][-1].copy()
        rows = json.loads(str(archive['info_json']))
        report = json.loads(str(archive['metadata_json']))
        index, time_s = len(times)-1, float(times[-1])
        if index < 0 or len(rows) != len(times):
            raise ValueError('Snapshot has no aligned endpoint')
    row = rows[index]
    endpoint = binding['original_endpoint']
    if (row['phase'] != args.expected_phase or row['phase'] != endpoint['phase']
            or time_s != endpoint['time_s'] or index != endpoint['sample_index']
            or not same_json(row, endpoint['original_sample'])
            or hashlib.sha256(qpos.tobytes()).hexdigest() != endpoint['qpos_sha256']
            or hashlib.sha256(qvel.tobytes()).hexdigest() != endpoint['qvel_sha256']
            or not np.isfinite(qpos).all() or not np.isfinite(qvel).all()):
        raise ValueError('Only the exact approved saved endpoint may be displayed')
    if ('search' in row['phase'] or not row['phase'].startswith(('reset_open_','open_hold_'))
            or row['fully_open_unassisted'] is not True or row['thread_engaged'] is not True
            or type(row['right_robot_bolt_contact_count']) is not int or row['right_robot_bolt_contact_count'] != 0
            or row['all_hard_guards_held'] is not True or row['external_drive_zero'] is not True
            or row['right_axial_float_active'] is not False or row['right_applied_axial_feed_N'] != 0.):
        raise ValueError('Endpoint is not an original guarded contact-free captured OPEN/reset state')
    if row['formed_flank_overlap_m'] < report['scene_config']['base']['thread']['pitch']:
        raise ValueError('Recorded endpoint lacks one-pitch formed-profile extent')
    if report['runtime'] != launch['runtime'] or report['control_config']['arm']['reset_speed_rad_s'] != 2.:
        raise ValueError('Original runtime/control metadata differs')
    module.require_micron_engine()
    runtime = module.engine_info()
    if runtime != launch['runtime']:
        raise ValueError('Actual replay runtime descriptor differs')
    mujoco.mj_loadPluginLibrary(str(plugin))
    model, model_identity = module.compile_archived_model(trace, report)
    if qpos.shape != (model.nq,) or qvel.shape != (model.nv,):
        raise ValueError('Saved endpoint shape differs from exact archived model')
    data = mujoco.MjData(model)
    data.qpos[:], data.qvel[:], data.time = qpos, qvel, time_s
    module.geometry_refresh(model, data)
    norms = module.command_norms(row)
    output.mkdir(parents=True)
    renderer = module.SupportedRenderer(model)
    try:
        whole = renderer.frame(data, row, report, slow_motion=1.)
        draw = ImageDraw.Draw(whole)
        draw.rectangle((0,96,780,124), fill='#101924')
        draw.text((20,100),'FRESH PROGRESS · recorded captured OPEN state · full audit pending',font=renderer.small,fill='#f1c46c')
        draw.rectangle((384,674,850,720), fill='#101924')
        draw.text((390,681),'Producer9ae1a9fe · full result/audit pending',font=renderer.small,fill='#f1c46c')
        whole.save(output/'captured_open.png')
    finally:
        renderer.close()
    detail = mujoco.Renderer(model, width=640, height=640)
    cameras = [dict(lookat_m=[.285,-.010,.020],distance_m=.22,azimuth_deg=70.,elevation_deg=-45.),
        dict(lookat_m=data.xpos[model.body('male_bolt').id].copy().tolist(),distance_m=.15,azimuth_deg=100.,elevation_deg=-30.)]
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big, small = ImageFont.truetype(font,25), ImageFont.truetype(font,18)
    try:
        image = Image.new('RGB',(1280,856),'#101924')
        for k, view in enumerate(cameras):
            camera = mujoco.MjvCamera(); camera.lookat[:] = view['lookat_m']
            camera.distance, camera.azimuth, camera.elevation = view['distance_m'],view['azimuth_deg'],view['elevation_deg']
            detail.update_scene(data,camera=camera)
            image.paste(Image.fromarray(detail.render()),(640*k,104))
        draw = ImageDraw.Draw(image)
        draw.text((16,10),'Fresh native progress · original capture flag / fully-open endpoint',font=big,fill='white')
        draw.text((16,43),f'{time_s:.5f} s · {row["phase"]} · formed {row["formed_flank_overlap_m"]*1000:.6f} mm · whole-right contacts0',font=small,fill='#f1c46c')
        draw.text((16,71),f'Original recorded thread_engaged=true · loaded interior {row["native_feedback"]["loaded_actual_interior_flank_contact_count"]} · full audit pending',font=small,fill='#c5d6e4')
        left,right = row['left_contact']['pad_normal_force_N'],row['contact']['pad_normal_force_N']
        draw.text((16,732),f'Original solve t−50µs: table {row["table_support"]["table_upward_force_N"]:.6f} N · pads L {left[0]:.2f}/{left[1]:.2f} N R {right[0]:.2f}/{right[1]:.2f} N',font=small,fill='#c5d6e4')
        ff_scope = 'ACTIVE' if norms['inertia_enabled'] else 'DISABLED (inverse/Jdot inputs absent)'
        draw.text((16,762),f'Original FF {ff_scope} · capped robot {norms["capped_total"]["force_N"]:.3f} N/{norms["capped_total"]["torque_Nm"]:.3f} Nm',font=small,fill='#c5d6e4')
        retained_time = float(row['inertia_control']['retained_native_state_time_s'])
        draw.text((16,792),f'Saved state {time_s:.5f}s; ORIGINAL retained command inputs {retained_time:.5f}s · no replay forces',font=small,fill='#c5d6e4')
        draw.text((16,822),'Producer9ae1a9fe · one stored state · dense ledgers/full audit pending · no force replay',font=small,fill='#c5d6e4')
        image.save(output/'captured_open_detail.png')
    finally:
        detail.close()
    for name,digest in files.items():
        if sha(snapshot/name) != digest:raise ValueError('Snapshot changed during still rendering: '+name)
    if sha(binding_path) != args.snapshot_binding_sha256:raise ValueError('Snapshot binding changed')
    for name,digest in sources.items():
        if sha(root/name) != digest:raise ValueError('Producer source changed: '+name)
    for name,digest in runtime_files.items():
        if sha(name) != digest:raise ValueError('Core/runtime changed: '+name)
    if module.engine_info() != runtime or sha(plugin) != PLUGIN_SHA or sha(approved) != APPROVED_RENDERER_SHA or sha(compiler_origin) != ARCHIVED_COMPILER_ORIGIN_SHA:
        raise ValueError('Runtime/plugin/compiler changed during still rendering')
    manifest = dict(scope='One original captured fully-OPEN saved endpoint from immutable fresh native prefix. Original flags/force samples retained; not full closed/native/passive-reset/qualified-lead/policy acceptance.',
        mode='progress_single_saved_state',full_native_run_closed=False,independent_full_audit_pending=True,
        snapshot_binding_sha256=args.snapshot_binding_sha256,immutable_snapshot_binding=binding,
        saved_endpoint=endpoint,original_metadata=report,producer_commit=PRODUCER,
        source74_before_after=sources,runtime_before_after=runtime,runtime_file_sha256_before_after=runtime_files,
        archived_model_identity=model_identity,actual_detail_cameras=cameras,
        canonical_overview_camera_and_original_geomgroup_filter_unchanged=True,
        actual_saved_state_count=1,media_file_count=2,video_frames=0,
        original_command_norms=norms,original_command_retained_native_state_time_s=float(row['inertia_control']['retained_native_state_time_s']),
        original_force_state_timing='Force/retained native geometry t−50µs; post-qpos/qvel t; actual ORIGINAL retained command timestamp archived separately, initialized exceptions retained. Disabled inverse/Jdot presence remains explicit. No replay forces.',
        replay_plugin_library=dict(path=str(plugin),sha256=PLUGIN_SHA,supplied_replay_binary_anchor=True,original_native_binary_identity_inferred=False,plugin_built=False),
        approved_compiler_module_sha256=APPROVED_RENDERER_SHA,archived_compiler_origin_sha256=ARCHIVED_COMPILER_ORIGIN_SHA,
        refresh=['mj_kinematics','mj_comPos','mj_camlight'],mj_forward_called=False,collision_discovery_called=False,force_solve_called=False,physics_integration=False,state_interpolation=False,model_regeneration=False,body_hiding_or_geometry_change=False,
        helper_sha256=sha(__file__),media_sha256={n:sha(output/n) for n in ('captured_open.png','captured_open_detail.png')})
    (output/'still_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    archive = output/'renderer_sources';archive.mkdir()
    for origin in (Path(__file__),approved,compiler_origin,snapshot/'snapshot_helper_source.py'):
        shutil.copyfile(origin,archive/origin.name)
    print(json.dumps(dict(output=str(output),phase=row['phase'],time_s=time_s,saved_states_rendered=1,media_files=2,manifest_sha256=sha(output/'still_manifest.json')),indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,required=True)
    p.add_argument('--snapshot-binding-sha256',required=True)
    p.add_argument('--expected-phase',required=True)
    p.add_argument('--immutable-root',type=Path,action='append',default=[],help='Additional immutable proof/audit/input tree excluded from render output')
    p.add_argument('--approved-renderer-source',type=Path,default=Path('outputs/m8_table_supported/render_full_reset_speed2_v3_regrasp_progress_v2.py'))
    p.add_argument('--compiler-origin-source',type=Path,default=Path('outputs/m8_table_supported/render_full_reset_speed2_v3_closed.py'))
    p.add_argument('--plugin-library',type=Path,required=True)
    p.add_argument('--plugin-sha256',required=True)
    p.add_argument('snapshot',type=Path);p.add_argument('output',type=Path)
    main(p.parse_args())
