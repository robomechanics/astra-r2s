"""Fresh reset-speed2-v3 OPEN/reset/regrasp media from an immutable original saved prefix.

Refreshes archived geometry only. Never reads a live trial's mutable NPZ/report.
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

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageSequence

import xml.etree.ElementTree as ET
from thread_lab.runtime import engine_info, require_micron_engine
from yam_twin.m8_supported_demo import SupportedRenderer


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strict_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))


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
    snapshot = strict_json(run/'snapshot_binding.json')
    assert snapshot['schema'] == 'fresh-full-reset-speed2-v3-phase-prefix-snapshot-v1'
    assert snapshot['immutable_snapshot'] is True and snapshot['historical_phase_end_progress'] is True
    assert snapshot['full_native_run_closed'] is False
    for name, digest in snapshot['file_sha256'].items():
        assert sha(run/name) == digest, name
    trace = run/snapshot['trace_filename']
    assert trace.is_file()
    trace_sha = sha(trace)
    launch = strict_json(run/'run_publication_identity.json')
    proof = strict_json(run/'software_tests.json')
    with np.load(trace, allow_pickle=False) as archive:
        times = archive['time'].copy()
        qpos, qvel = archive['qpos'].copy(), archive['qvel'].copy()
        samples = json.loads(str(archive['info_json']))
        report = json.loads(str(archive['metadata_json']))
    assert launch['producer_commit'] == '9ae1a9fe76968a4013ea6c39e67718026622b9c0'
    sources = launch['source_hashes_before']
    assert len(sources) == 74 and sources == snapshot['source74_sha256_before_after_copy'] == proof['source_hashes']
    assert proof['passed'] is True and proof['tests_passed'] == 672
    assert sha(run/'software_tests.json')=='9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
    assert sha(run/'run_publication_identity.json')=='1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
    assert report['control_config']['arm']['reset_speed_rad_s']==2.
    assert report['physical_feedback_controller'] == 'supported-crest-c2-hybrid-inertia-v2'
    assert report['starts_preengaged'] is False and report['starts_grasp_ready'] is False
    assert report['block_starts_on_table'] is True and report['bolt_starts_on_declared_fixed_rest'] is True
    assert report['controller_module_sha256'] == sha(run/'controller_source.py') == sources['yam_twin/m8_supported_simulation.py']
    assert report['robot_inertia_feedforward']['source_sha256'] == sha(run/'recorded_sources/yam_twin/m8_robot_inertia.py')
    for name, digest in sources.items():
        assert sha(ROOT/name) == digest, name
    originals = {p:sha(p) for p in run.rglob('*') if p.is_file()}
    require_micron_engine()
    runtime = engine_info()
    assert runtime == report['runtime'] == launch['runtime']
    assert len(times) == len(samples) == len(qpos) == len(qvel) and len(times) > 0
    assert np.isfinite(times).all() and np.all(np.diff(times) > 0)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    assert all(float(row['time']) == float(t) for row, t in zip(samples, times))
    plugin=args.plugin_library.resolve()
    assert plugin.is_file(), 'An existing supplied plugin binary is required; replay cannot build it'
    assert args.plugin_sha256=='53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18'
    assert sha(plugin)==args.plugin_sha256, 'Supplied replay plugin anchor mismatch'
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
    status = 'FRESH NATIVE PROGRESS'
    stills = {'demo.png':len(times)-1}
    for phase, name in (('secure_left', 'left_stabilized.png'), ('settle_bolt', 'bolt_grasped.png'),
            ('lift_bolt', 'bolt_lifted.png'), ('align_over_hole', 'bolt_aligned.png'),
            ('stop_reverse_seat_1', 'direction_stopped.png'),
            ('stop_start_1', 'first_forward_stopped.png'),
            ('open_settle_search_2', 'fully_open.png'),
            ('reset_open_search_2', 'open_reset.png'),
            ('settle_regrip_search_2', 'quiet_regrasp.png')):
        rows = [i for i, row in enumerate(samples) if row['phase'] == phase]
        if rows:
            stills[name] = rows[-1]
    detail_stills = {'endpoint_detail.png':len(times)-1}
    if 'left_stabilized.png' in stills:
        detail_stills['left_stabilized_detail.png'] = stills['left_stabilized.png']
    for label in ('fully_open', 'open_reset', 'quiet_regrasp'):
        if label+'.png' in stills:
            detail_stills[label+'_detail.png'] = stills[label+'.png']

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
        draw.text((20, 100), 'FRESH RESET-SPEED2 PROGRESS · actual native phase prefix · no full result claimed',
            font=renderer.small, fill='#f1c46c')
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
        dict(lookat_m=None, distance_m=.15, azimuth_deg=100., elevation_deg=-30.,
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
            draw.text((16, 10), 'Fresh reset-speed2 native progress · actual saved state', font=big, fill='white')
            draw.text((16, 43), f'{times[index]:.5f} s original native · {row["phase"]} · {alignment_caption(row)}', font=small, fill='#f1c46c')
            draw.text((16, 71), f'Actual saved state · formed {row["formed_flank_overlap_m"]*1e6:.3f} µm · loaded interior {row["native_feedback"]["loaded_actual_interior_flank_contact_count"]} · no full qualification', font=small, fill='#c5d6e4')
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
    assert sha(plugin)==args.plugin_sha256
    all_stills = dict(stills, **detail_stills)
    media_names = (['demo.mp4', 'demo.gif'] if indices else []) + list(all_stills)
    manifest = {
        'scope':'One fresh reset-speed2-v3 native attempt, exact immutable original phase-end prefix only. Original table/rest spawns; no cold checkpoint or splicing. Progress does not establish completed trajectory/capture/reset/policy qualification. Geometry-only refresh, no integration/forward/collision/force solve/interpolation/posing.',
        'mode':'progress', 'immutable_snapshot_binding':snapshot,
        'snapshot_binding_sha256':sha(run/'snapshot_binding.json') if snapshot else None,
        'trajectory_sha256':trace_sha, 'trace_filename':trace.name,
        'original_report_sha256':None,
        'original_metadata_scope':'Exact metadata_json embedded in the original copied trace; no produced validation flags rewritten',
        'original_passed':report.get('passed'), 'original_partial':report.get('partial'), 'original_aborted':report.get('aborted'),
        'native_closed_result':False, 'progress_is_not_closed_rollout_proof':True,
        'fps':fps, 'slow_motion':1, 'frames':len(indices),
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
        'runtime_before':runtime, 'runtime_after':engine_info(), 'archived_model_identity':model_identity,
        'original_runtime_plugin_source_sha256':report['runtime']['thread_plugin_source_sha256'],
        'replay_plugin_library':dict(path=str(plugin),sha256=args.plugin_sha256,
            scope='Explicit supplied already-built REPLAY binary anchor only. Native runtime records plugin SOURCE SHA, not native plugin binary SHA; no original native-binary equivalence inferred.',
            plugin_built_during_replay=False),
        'archived_compiler_source_origin':dict(path='outputs/m8_table_supported/render_full_reset_speed2_v3_closed.py',sha256='ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8'),
        'refresh':['mj_kinematics', 'mj_comPos', 'mj_camlight'],
        'mj_forward_called':False, 'collision_discovery_called':False, 'force_solve_called':False,
        'physics_integration':False, 'state_interpolation':False, 'body_hiding_or_geometry_change':False,
        'canonical_camera_and_original_geomgroup_filter_unchanged':True, 'detail_cameras':cameras, 'actual_per_still_detail_cameras':actual_detail_cameras,
        'helper_sha256':sha(__file__),
        'portable_renderer_alias':dict(basename='render_full_reset_speed2_v3_regrasp_progress.py',sha256=sha(__file__),byte_identical_to_actual_versioned_source=True),
        'media_sha256':{name:sha(output/name) for name in media_names}}
    (output/'render_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    archive = output/'renderer_sources'
    archive.mkdir()
    shutil.copyfile(__file__, archive/Path(__file__).name)
    shutil.copyfile(__file__, archive/'render_full_reset_speed2_v3_regrasp_progress.py')
    compiler_origin=Path(__file__).with_name('render_full_reset_speed2_v3_regrasp_compiler_origin.py')
    assert sha(compiler_origin)=='ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8'
    shutil.copyfile(compiler_origin, archive/compiler_origin.name)
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
    parser.add_argument('--plugin-library',type=Path,required=True)
    parser.add_argument('--plugin-sha256',required=True)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
