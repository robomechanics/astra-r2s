"""CLOSED fresh C2/inertia media from the complete original saved trajectory.

Refuses LIVE/nonclosed evidence before reading any trace/report. Requires frozen original AFTER, root identity and audit SHA anchors. Refreshes archived geometry only.
No mj_forward, collision, force solve, interpolation or physics integration.
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import sys
from pathlib import Path

os.environ['MUJOCO_GL'] = 'egl'
os.environ['LP_NUM_THREADS'] = '1'
preparser = argparse.ArgumentParser(add_help=False)
preparser.add_argument('--repository-root', type=Path, default=Path('/workspace/astra-r2s'))
preargs, _ = preparser.parse_known_args()
ROOT = preargs.repository_root.resolve()
sys.path.insert(0, str(ROOT))

import xml.etree.ElementTree as ET
from full_c2_closed_bindings import sha, strict_json, verify_closed


def compile_archived_model(trace, report):
    """Compile only exact archived XML/assets, with an already-loaded plugin.

    This local compiler contains no plugin build path or dynamics/collision call.
    """
    from scripts.audit_m8_insertion_trace import archived_model_fingerprint, archived_asset_path
    directory = trace.parent
    xml = (directory/'scene.xml').read_bytes()
    assert hashlib.sha256(xml).hexdigest() == report['model_xml_sha256']
    fingerprint, hashes = archived_model_fingerprint(xml, directory)
    assert fingerprint == report['model_fingerprint']
    scene_sha = report.get('scene_source_sha256')
    if scene_sha is not None:
        assert sha(directory/'scene_source.py') == scene_sha
    tree, assets = ET.fromstring(xml), {}
    for mesh in tree.findall('asset/mesh'):
        name = mesh.get('file')
        if name:
            path = archived_asset_path(name, directory)
            value = path.read_bytes()
            assert hashlib.sha256(value).hexdigest() == hashes[path.name]
            assets[path.name] = value
            mesh.set('file', path.name)
    model = mujoco.MjSpec.from_string(ET.tostring(tree, encoding='unicode'), assets=assets).compile()
    return model, dict(model_fingerprint=fingerprint, model_xml_sha256=report['model_xml_sha256'],
        mesh_sha256=hashes, scene_source_sha256=scene_sha,
        method='Compile exact SHA-verified archived XML/assets with explicit already-built plugin. No current scene generation/plugin build/forward/collision/integration.')


def recorded_runtime_content_matches(actual, recorded):
    # Relocation can change mapped library paths, never engine/source/binary content.
    if actual['mujoco_version'] != recorded['mujoco_version'] or actual['thread_plugin_source_sha256'] != recorded['thread_plugin_source_sha256']:
        return False
    def descriptors(value):
        return sorted((dict((k,v) for k,v in row.items() if k != 'path') for row in value['libraries']), key=lambda x:x.get('sha256',''))
    return descriptors(actual) == descriptors(recorded)


def geometry_refresh(model, data):
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_camlight(model, data)


def command_norms(row):
    record = row['inertia_control']
    result = {}
    for label, key in (('PD', 'pd_wrench_world_N_Nm'), ('FF', 'ff_wrench_world_N_Nm'),
            ('capped_total', 'capped_wrench_world_N_Nm')):
        value = np.asarray(record[key], dtype=float)
        assert value.shape == (6,) and np.isfinite(value).all()
        result[label] = dict(force_N=float(np.linalg.norm(value[:3])),
            torque_Nm=float(np.linalg.norm(value[3:])))
    result['scope'] = 'Norms of ORIGINAL executed robot command records, not measured thread/contact forces or replay forces'
    result['inertia_enabled'] = record['enabled']
    result['inertia_inputs_present'] = record['inertia_inputs_present']
    result['jacobian_derivatives_present'] = record['jacobian_derivatives_present']
    if not record['enabled']:
        assert record['inertia_inputs_present'] is False and record['jacobian_derivatives_present'] is False
        assert np.all(np.asarray(record['ff_wrench_world_N_Nm']) == 0)
    return result


def alignment_caption(row):
    inactive_phases = {'settle_table', 'reach_left_block', 'close_left_block', 'settle_left_block',
        'secure_left', 'reach_bolt', 'close_bolt', 'settle_bolt', 'lift_bolt', 'transport_bolt', 'align_over_hole'}
    active = row['phase'] not in inactive_phases and row['thread_overlap_m'] > 0
    if active:
        return f'radial {row["radial_offset_m"]*1e6:.3f} µm /150 µm entry guard'
    return f'bolt/bore radial distance {row["radial_offset_m"]*1e3:.3f} mm · entry guard inactive'


def main(args):
    run, output = args.run.resolve(), args.output.resolve()
    assert not output.exists()
    bound = verify_closed(run, ROOT, args.proof_archive, args.identity, args.identity_sha256,
        args.audit_binding, args.audit_binding_sha256)
    # Gate above rejects a missing original AFTER before any mutable native data.
    trace = run/'insertion_trace.npz'
    if not trace.is_file() or not (run/'insertion_validation.json').is_file():
        raise ValueError('Final original trace/report absent: use explicit raw-only evidence; never substitute a partial trace')
    trace_sha = sha(trace)
    launch, proof, sources = bound['launch'], bound['proof'], bound['sources']
    original_report = strict_json(run/'insertion_validation.json')
    plugin = args.plugin_library.resolve()
    if sha(plugin) != args.plugin_sha256:
        raise ValueError('Already-built replay plugin differs from its explicit supplied binary SHA anchor')
    global imageio, mujoco, np, Image, ImageDraw, ImageFont, ImageSequence
    import imageio.v2 as imageio
    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont, ImageSequence
    from thread_lab.runtime import engine_info, require_micron_engine
    from yam_twin.m8_supported_demo import SupportedRenderer
    with np.load(trace, allow_pickle=False) as archive:
        times = archive['time'].copy()
        qpos, qvel = archive['qpos'].copy(), archive['qvel'].copy()
        samples = json.loads(str(archive['info_json']))
        report = json.loads(str(archive['metadata_json']))
    if report != original_report:
        raise ValueError('Final embedded metadata/report differs from untouched original validation')
    assert float(report['scene_config']['base']['thread']['timestep']) == .00005
    assert report['physical_feedback_controller'] == 'supported-crest-c2-hybrid-inertia-v2'
    assert report['starts_preengaged'] is False and report['starts_grasp_ready'] is False
    assert report['block_starts_on_table'] is True and report['bolt_starts_on_declared_fixed_rest'] is True
    assert report['controller_module_sha256'] == sha(run/'controller_source.py') == sources['yam_twin/m8_supported_simulation.py']
    assert report['robot_inertia_feedforward']['source_sha256'] == sha(run/'recorded_sources/yam_twin/m8_robot_inertia.py')
    for name, digest in sources.items():
        assert sha(ROOT/name) == digest, name
    originals = {run/name:digest for name,digest in bound['original_run_files_sha256'].items()}
    require_micron_engine()
    runtime = engine_info()
    assert report['runtime'] == launch['runtime']
    assert recorded_runtime_content_matches(runtime, report['runtime'])
    assert len(times) == len(samples) == len(qpos) == len(qvel) and len(times) > 0
    assert np.isfinite(times).all() and np.all(np.diff(times) > 0)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    assert all(float(row['time']) == float(t) for row, t in zip(samples, times))
    mujoco.mj_loadPluginLibrary(str(plugin))
    model, model_identity = compile_archived_model(trace, report)
    data = mujoco.MjData(model)
    renderer = SupportedRenderer(model)
    output.mkdir(parents=True)
    fps = 12
    indices = [] if args.stills_only else [min(len(times)-1, int(np.searchsorted(times, times[0]+i/fps)))
        for i in range(max(1, math.ceil(float(times[-1]-times[0])*fps)))]
    if indices:
        indices[-1] = len(times)-1
    status = f'CLOSED FRESH NATIVE · original passed={report.get("passed")} · partial={report.get("partial")} · exit={bound["after"]["native_exit_code"]}'
    stills = {'demo.png':len(times)-1, 'initial_state.png':0}
    for phase, name in (('secure_left', 'left_stabilized.png'), ('settle_bolt', 'bolt_grasped.png'),
            ('lift_bolt', 'bolt_lifted.png'), ('align_over_hole', 'bolt_aligned.png'),
            ('stop_reverse_seat_1', 'direction_stopped.png')):
        rows = [i for i, row in enumerate(samples) if row['phase'] == phase]
        if rows:
            stills[name] = rows[-1]
    # Every executed physical stage uses its exact last saved state; no synthetic events.
    for phase in dict.fromkeys(row['phase'] for row in samples):
        rows = [i for i, row in enumerate(samples) if row['phase'] == phase]
        if phase.startswith(('stop_start_', 'stop_turn_', 'open_settle_', 'reset_open_', 'open_hold_', 'regrip_', 'settle_regrip_', 'hold')):
            stills[f'phase_{phase}.png'] = rows[-1]
    milestone_indices = {}
    for phase in dict.fromkeys(row['phase'] for row in samples):
        if phase.startswith('open_settle_'):
            contact_free = [i for i,row in enumerate(samples) if row['phase']==phase
                and row['fully_open_unassisted'] and row['right_robot_bolt_contact_count']==0]
            if contact_free:
                milestone_indices['first_contact_free_open_settle'] = contact_free[0]
                ready = [i for i in contact_free if samples[i]['open_weight_window']['ready_for_diagnostic_release_attempt']]
                if ready:
                    milestone_indices['first_ready_open_settle'] = ready[0]
                break
    first_forward_stop = [i for i,row in enumerate(samples) if row['phase']=='stop_start_1']
    if first_forward_stop:
        milestone_indices['first_forward_stopped'] = first_forward_stop[-1]
    for name,index in milestone_indices.items():
        stills[name+'.png'] = index
    detail_stills = {'endpoint_detail.png':len(times)-1, 'initial_state_detail.png':0,
        **{name+'_detail.png':index for name,index in milestone_indices.items()}}
    if 'left_stabilized.png' in stills:
        detail_stills['left_stabilized_detail.png'] = stills['left_stabilized.png']

    def state(index):
        data.qpos[:] = qpos[index]
        data.qvel[:] = qvel[index]
        data.time = float(times[index])
        geometry_refresh(model, data)

    def canvas(index):
        state(index)
        image = renderer.frame(data, samples[index], report, slow_motion=1)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 96, 780, 124), fill='#101924')
        draw.text((20, 100), status,
            font=renderer.small, fill='#f1c46c')
        row, norms = samples[index], command_norms(samples[index])
        ff_status = 'ACTIVE' if row['inertia_control']['enabled'] else 'DISABLED (no inverse/Jdot inputs)'
        draw.rectangle((0, 676, 864, 720), fill='#101924')
        draw.text((20, 679), f'Original FF {ff_status} · capped ROBOT command {norms["capped_total"]["force_N"]:.3f} N / {norms["capped_total"]["torque_Nm"]:.3f} Nm',
            font=renderer.small, fill='#c5d6e4')
        draw.text((20, 700), f'{times[index]:.5f} s · normal 1× · native force t−50µs / retained command t−100µs / saved state t',
            font=renderer.small, fill='#c5d6e4')
        return image

    writer = None
    try:
        if indices:
            writer = imageio.get_writer(output/'demo.mp4', fps=fps, codec='libx264', quality=8,
                macro_block_size=1, ffmpeg_params=['-movflags', '+faststart', '-pix_fmt', 'yuv420p'])
            for index in indices:
                writer.append_data(np.asarray(canvas(index)))
        for name, index in stills.items():
            canvas(index).save(output/name)
    finally:
        if writer:
            writer.close()
        renderer.close()

    cameras = [dict(lookat_m=[.285, -.010, .020], distance_m=.22,
        azimuth_deg=70., elevation_deg=-45., scope='Fixed actual table/left-clamp view'),
        dict(lookat_m=None, distance_m=.20, azimuth_deg=130., elevation_deg=-30.,
        scope='Camera follows exact saved post-q bolt body position only; no workpiece pose change')]
    actual_detail_cameras = {}
    detail = mujoco.Renderer(model, width=640, height=640)
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big, small = ImageFont.truetype(font, 25), ImageFont.truetype(font, 18)
    try:
        for name, index in detail_stills.items():
            state(index)
            image = Image.new('RGB', (1280, 856), '#101924')
            actual_detail_cameras[name] = []
            for k, view in enumerate(cameras):
                camera = mujoco.MjvCamera()
                lookat = view['lookat_m'] if view['lookat_m'] is not None else data.xpos[model.body('male_bolt').id].copy().tolist()
                camera.lookat[:] = lookat
                actual_detail_cameras[name].append(dict(view, lookat_m=lookat))
                camera.distance, camera.azimuth, camera.elevation = (
                    view['distance_m'], view['azimuth_deg'], view['elevation_deg'])
                detail.update_scene(data, camera=camera)
                image.paste(Image.fromarray(detail.render()), (640*k, 104))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 1280, 104), fill='#101924')
            draw.rectangle((0, 720, 1280, 856), fill='#101924')
            row, norms = samples[index], command_norms(samples[index])
            draw.text((16, 10), 'CLOSED fresh table-supported native · exact original saved state', font=big, fill='white')
            draw.text((16, 43), f'{times[index]:.5f} s original native · {row["phase"]} · {alignment_caption(row)}', font=small, fill='#f1c46c')
            draw.text((16, 71), f'Actual saved state · formed {row["formed_flank_overlap_m"]*1e6:.3f} µm · loaded interior {row["native_feedback"]["loaded_actual_interior_flank_contact_count"]} · original passed={report.get("passed")}', font=small, fill='#c5d6e4')
            left, right = row['left_contact']['pad_normal_force_N'], row['contact']['pad_normal_force_N']
            draw.text((16, 732), f'Original solve t−50µs: table {row["table_support"]["table_upward_force_N"]:.6f} N · left pads {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N', font=small, fill='#c5d6e4')
            record = row['inertia_control']
            ff_scope = 'ACTIVE' if record['enabled'] else 'DISABLED (no inverse inputs present)'
            draw.text((16, 762), f'Inertia FF {ff_scope} · original capped robot command {norms["capped_total"]["force_N"]:.3f} N/{norms["capped_total"]["torque_Nm"]:.3f} Nm', font=small, fill='#c5d6e4')
            draw.text((16, 792), 'Command records are not contact forces; retained command geometry t−100µs · saved qpos/qvel at t', font=small, fill='#c5d6e4')
            draw.text((16, 822), 'Fresh original table/rest spawns · no cold checkpoint / splice / integration / force solve / body hiding', font=small, fill='#c5d6e4')
            image.save(output/name)
    finally:
        detail.close()

    actual_durations = []
    if indices:
        frames = []
        reader = imageio.get_reader(output/'demo.mp4')
        for frame in reader:
            frames.append(Image.fromarray(frame).resize((1080, 600)).convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
        reader.close()
        durations = [10*(round((i+1)*100/fps)-round(i*100/fps)) for i in range(len(frames))]
        frames[0].save(output/'demo.gif', save_all=True, append_images=frames[1:],
            duration=durations, loop=0, optimize=False, disposal=2)
        with Image.open(output/'demo.gif') as image:
            actual_durations = [frame.info['duration'] for frame in ImageSequence.Iterator(image)]
    for path, digest in originals.items():
        assert sha(path) == digest, path
    for name, digest in sources.items():
        assert sha(ROOT/name) == digest, name
    assert engine_info() == runtime
    assert sha(plugin) == args.plugin_sha256
    all_stills = dict(stills, **detail_stills)
    media_names = (['demo.mp4', 'demo.gif'] if indices else []) + list(all_stills)
    manifest = {
        'scope':'One CLOSED fresh full C2/inertia native attempt. Complete original trajectory from independent table/rest spawns; no cold checkpoint or splicing. Original success/failure/partial/abort/capture flags remain verbatim and independent audit meanings are not inferred from footage. Geometry-only exact-state refresh, no integration/forward/collision/force solve/interpolation/posing.',
        'mode':'closed_native',
        'identity_report_sha256':bound['identity_sha256'],
        'independent_audit_binding_sha256':bound['audit_sha256'],
        'original_after_identity_sha256':sha(run/'run_publication_identity_after.json'),
        'original_native_exit_code':bound['after']['native_exit_code'],
        'trajectory_sha256':trace_sha, 'trace_filename':trace.name,
        'original_report_sha256':sha(run/'insertion_validation.json'),
        'original_metadata_scope':'Exact metadata_json embedded in the original copied trace; no produced validation flags rewritten',
        'original_passed':report.get('passed'), 'original_partial':report.get('partial'), 'original_aborted':report.get('aborted'),
        'native_closed_result':True, 'progress_is_not_closed_rollout_proof':False,
        'original_acceptance_checks':report.get('acceptance_checks'),
        'original_runtime_plugin_source_sha256':report['runtime']['thread_plugin_source_sha256'],
        'replay_plugin_library':dict(path=str(plugin), sha256=args.plugin_sha256,
            supplied_replay_binary_anchor=True, original_native_binary_hash_inferred=False,
            plugin_built_during_replay=False),
        'fps':fps, 'slow_motion':1, 'frames':len(indices),
        'frame_original_inertia_enabled':[samples[i]['inertia_control']['enabled'] for i in indices],
        'frame_original_inertia_inputs_present':[samples[i]['inertia_control']['inertia_inputs_present'] for i in indices],
        'frame_original_jacobian_derivatives_present':[samples[i]['inertia_control']['jacobian_derivatives_present'] for i in indices],
        'encoded_video_duration_s':len(indices)/fps if indices else None, 'gif_frames':len(actual_durations),
        'gif_duration_ms':sum(actual_durations) if actual_durations else None,
        'native_first_time_s':float(times[0]), 'native_last_time_s':float(times[-1]),
        'frame_sample_indices':indices, 'frame_saved_times_s':[float(times[i]) for i in indices],
        'stills':{name:dict(sample_index=i, time_s=float(times[i]), phase=samples[i]['phase'],
            qpos_sha256=hashlib.sha256(qpos[i].tobytes()).hexdigest(),
            qvel_sha256=hashlib.sha256(qvel[i].tobytes()).hexdigest(), original_sample=samples[i],
            original_executed_command_norms=command_norms(samples[i])) for name, i in all_stills.items()},
        'force_state_command_timing':'Original solve/retained geometry t−50µs; saved qpos/qvel t. Command geometry/calibration from previous retained solve t−100µs, first-command initialization exception. Executed approximate arm-inertia command records are not thread/contact forces; no replay force calculated.',
        'frame_time_quantization':'Normal1x12fps ceiling; exact original endpoint in last frame, no interpolation. GIF centisecond rounding separate. Inherited canonical1xslowmotion footer is factor1 normalplayback.',
        'original_launch_identity_sha256':sha(run/'run_publication_identity.json'),
        'original_software672_proof_sha256':sha(run/'software_tests.json'),
        'original_inertia_helper_source_sha256':sha(run/'recorded_sources/yam_twin/m8_robot_inertia.py'),
        'original_inertia_metadata':report['robot_inertia_feedforward'],
        'source74_before_after_render':sources, 'source74_unchanged':True,
        'original_run_files_sha256_before_after':{str(p.relative_to(run)):d for p, d in originals.items()},
        'runtime_before':runtime, 'runtime_after':engine_info(), 'original_runtime':report['runtime'],
        'recorded_runtime_content_matches_before_after':True, 'mapped_library_path_relocation_does_not_change_binary_hash':True,
        'archived_model_identity':model_identity,
        'refresh':['mj_kinematics', 'mj_comPos', 'mj_camlight'],
        'mj_forward_called':False, 'collision_discovery_called':False, 'force_solve_called':False,
        'physics_integration':False, 'state_interpolation':False, 'body_hiding_or_geometry_change':False,
        'milestone_selection':'Exact first saved whole-right contact-free fully-open open_settle state, first live100ms ready saved event if observed, exact last saved first forward stop. No unsaved state/event synthesized.',
        'canonical_camera_and_original_geomgroup_filter_unchanged':True, 'detail_cameras':cameras, 'actual_per_still_detail_cameras':actual_detail_cameras,
        'helper_sha256':sha(__file__), 'executed_helper_filename':Path(__file__).name, 'media_sha256':{name:sha(output/name) for name in media_names}}
    (output/'render_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    archive = output/'renderer_sources'
    archive.mkdir()
    shutil.copyfile(__file__, archive/Path(__file__).name)
    # Byte-identical portable entry alias; the executed version remains archived.
    shutil.copyfile(__file__, archive/'render_full_c2_inertia_v2_closed.py')
    shutil.copyfile(Path(__file__).with_name('full_c2_closed_bindings.py'), archive/'full_c2_closed_bindings.py')
    for name in ('yam_twin/m8_supported_demo.py', 'yam_twin/m8_insertion_demo.py',
            'yam_twin/m8_demo.py', 'scripts/audit_m8_insertion_trace.py', 'thread_lab/runtime.py'):
        target = archive/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, target)
    print(json.dumps({key:manifest[key] for key in ('mode','frames','encoded_video_duration_s',
        'gif_duration_ms','native_last_time_s','trajectory_sha256','media_sha256')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository-root', type=Path, default=ROOT)
    parser.add_argument('--stills-only', action='store_true')
    parser.add_argument('--identity', type=Path, required=True)
    parser.add_argument('--identity-sha256', required=True)
    parser.add_argument('--audit-binding', type=Path, required=True)
    parser.add_argument('--audit-binding-sha256', required=True)
    parser.add_argument('--proof-archive', type=Path, required=True)
    parser.add_argument('--plugin-library', type=Path, required=True)
    parser.add_argument('--plugin-sha256', required=True)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
