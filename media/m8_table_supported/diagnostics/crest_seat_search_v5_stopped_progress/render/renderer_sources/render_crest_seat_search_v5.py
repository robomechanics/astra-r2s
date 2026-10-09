"""V5 native cold-branch media from an immutable copied snapshot or CLOSED run.

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

from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import engine_info, require_micron_engine
from yam_twin.m8_supported_demo import SupportedRenderer


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strict_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))


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
    return result


def main(args):
    run, output = args.run.resolve(), args.output.resolve()
    assert not output.exists()
    assert run != ROOT/'outputs/m8_table_supported/diagnostics/crest_seat_search_v5' or args.mode == 'closed'
    snapshot = None
    if args.mode == 'progress':
        snapshot = strict_json(run/'snapshot_binding.json')
        assert snapshot['schema'] == 'm8-v5-native-media-snapshot-v1'
        assert snapshot['immutable_snapshot'] is True
        assert type(snapshot['native_run_still_active']) is bool
        assert snapshot['native_run_still_active'] is True
        assert snapshot['file_sha256']
        for name, digest in snapshot['file_sha256'].items():
            path = (run/name).resolve()
            assert path.is_relative_to(run) and path.is_file()
            assert sha(path) == digest, name
        trace = run/snapshot['trace_filename']
        assert snapshot['trace_filename'] in snapshot['file_sha256']
        assert snapshot['source_phase_scope']
    else:
        closure = strict_json(run/'diagnostic_execution_after.json')
        assert closure['source_65_after'] == strict_json(run/'diagnostic_declaration.json')['source_65_before']
        trace = run/'insertion_trace.npz'
    assert trace.is_file()
    trace_sha = sha(trace)
    declaration = strict_json(run/'diagnostic_declaration.json')
    with np.load(trace, allow_pickle=False) as archive:
        times = archive['time'].copy()
        qpos, qvel = archive['qpos'].copy(), archive['qvel'].copy()
        samples = json.loads(str(archive['info_json']))
        report = json.loads(str(archive['metadata_json']))
    if args.mode == 'closed':
        assert strict_json(run/'insertion_validation.json') == report
    assert report['full_fresh_trajectory_qualified'] is False if args.mode == 'closed' else not report.get('full_fresh_trajectory_qualified', False)
    assert report['capture_or_open_reset_qualified'] is False if args.mode == 'closed' else not report.get('capture_or_open_reset_qualified', False)
    assert report['physical_feedback_controller'] == 'cold-crest-return-c2-hybrid-inertia-diagnostic-v5'
    assert declaration['robot_inertia_source_sha256'] == sha(run/'robot_inertia_source.py')
    assert report['robot_inertia_feedforward']['source_sha256'] == sha(run/'robot_inertia_source.py')
    sources = declaration['source_65_before']
    assert len(sources) == 65
    for name, digest in sources.items():
        assert sha(ROOT/name) == digest, name
    originals = {p:sha(p) for p in run.rglob('*') if p.is_file()}
    require_micron_engine()
    runtime = engine_info()
    assert runtime == report['runtime'] == declaration['runtime']
    assert len(times) == len(samples) == len(qpos) == len(qvel) and len(times) > 0
    assert np.isfinite(times).all() and np.all(np.diff(times) > 0)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    assert all(float(row['time']) == float(t) for row, t in zip(samples, times))
    model, model_identity = recorded_model(trace, report)
    data = mujoco.MjData(model)
    renderer = SupportedRenderer(model)
    output.mkdir(parents=True)
    fps = 12
    indices = [] if args.stills_only else [min(len(times)-1, int(np.searchsorted(times, times[0]+i/fps)))
        for i in range(max(1, math.ceil(float(times[-1]-times[0])*fps)))]
    if indices:
        indices[-1] = len(times)-1
    status = 'PROGRESS SNAPSHOT' if args.mode == 'progress' else ('CLOSED FAILED' if report.get('aborted') else 'CLOSED DIAGNOSTIC')
    stills = {'demo.png':len(times)-1}
    if not args.stills_only:
        hold_rows = [i for i, row in enumerate(samples) if row['phase'] == 'cold_window_hold']
        if hold_rows:
            stills['search_start.png'] = hold_rows[-1]
        for phase, name in (('stop_reverse_seat_1', 'brake_start.png'), ('forward_scan_1', 'forward_start.png')):
            rows = [i for i, row in enumerate(samples) if row['phase'] == phase]
            if rows:
                stills[name] = rows[0]
    detail_stills = {'endpoint_detail.png':len(times)-1}

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
        draw.text((20, 100), f'COLD V5 {status} · bounded robot inertia FF · no full capture / trajectory claim',
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

    cameras = [dict(lookat_m=[.350, -.010, .029], distance_m=.15,
        azimuth_deg=100., elevation_deg=-30.), dict(lookat_m=[.350, -.010, .029],
        distance_m=.15, azimuth_deg=130., elevation_deg=-5.)]
    detail = mujoco.Renderer(model, width=640, height=640)
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big, small = ImageFont.truetype(font, 25), ImageFont.truetype(font, 18)
    try:
        for name, index in detail_stills.items():
            state(index)
            image = Image.new('RGB', (1280, 856), '#101924')
            for k, view in enumerate(cameras):
                camera = mujoco.MjvCamera()
                camera.lookat[:] = view['lookat_m']
                camera.distance, camera.azimuth, camera.elevation = (
                    view['distance_m'], view['azimuth_deg'], view['elevation_deg'])
                detail.update_scene(data, camera=camera)
                image.paste(Image.fromarray(detail.render()), (640*k, 104))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 1280, 104), fill='#101924')
            draw.rectangle((0, 720, 1280, 856), fill='#101924')
            row, norms = samples[index], command_norms(samples[index])
            draw.text((16, 10), f'Cold crest-search V5 · {status} · actual robot inertia FF', font=big, fill='white')
            draw.text((16, 43), f'{times[index]:.5f} s local native · {row["phase"]} · radial {row["radial_offset_m"]*1e6:.3f} µm / 150 µm guard', font=small, fill='#f1c46c')
            draw.text((16, 71), f'Actual saved state · formed {row["formed_flank_overlap_m"]*1e6:.3f} µm · loaded interior {row["native_feedback"]["loaded_actual_interior_flank_contact_count"]} · no full qualification', font=small, fill='#c5d6e4')
            left, right = row['left_contact']['pad_normal_force_N'], row['contact']['pad_normal_force_N']
            draw.text((16, 732), f'Original solve t−50µs: table {row["table_support"]["table_upward_force_N"]:.6f} N · left pads {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N', font=small, fill='#c5d6e4')
            draw.text((16, 762), f'Original command norms FF {norms["FF"]["force_N"]:.3f} N/{norms["FF"]["torque_Nm"]:.3f} Nm · capped total {norms["capped_total"]["force_N"]:.3f} N/{norms["capped_total"]["torque_Nm"]:.3f} Nm', font=small, fill='#c5d6e4')
            draw.text((16, 792), 'Command records are not contact forces; retained command geometry t−100µs · saved qpos/qvel at t', font=small, fill='#c5d6e4')
            draw.text((16, 822), 'Cold full-v1 13.1949s parent · no solver warm-start / old force-window reuse / splice / body hiding', font=small, fill='#c5d6e4')
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
    all_stills = dict(stills, **detail_stills)
    media_names = (['demo.mp4', 'demo.gif'] if indices else []) + list(all_stills)
    manifest = {
        'scope':'Separate V5 cold branch, original copied native states only. Snapshot progress or closed diagnostic scope remains explicit; no full capture/reset/fresh trajectory/policy qualification inferred. Geometry-only refresh, no integration/forward/collision/force solve/interpolation/posing/splice.',
        'mode':args.mode, 'immutable_snapshot_binding':snapshot,
        'snapshot_binding_sha256':sha(run/'snapshot_binding.json') if snapshot else None,
        'trajectory_sha256':trace_sha, 'trace_filename':trace.name,
        'original_report_sha256':sha(run/'insertion_validation.json') if args.mode == 'closed' else None,
        'original_metadata_scope':'Exact metadata_json embedded in the original copied trace; no produced validation flags rewritten',
        'original_passed':report.get('passed'), 'original_partial':report.get('partial'), 'original_aborted':report.get('aborted'),
        'native_closed_result':args.mode == 'closed', 'progress_is_not_closed_rollout_proof':args.mode == 'progress',
        'fps':fps, 'slow_motion':1, 'frames':len(indices),
        'encoded_video_duration_s':len(indices)/fps if indices else None, 'gif_frames':len(actual_durations),
        'gif_duration_ms':sum(actual_durations) if actual_durations else None,
        'native_first_time_s':float(times[0]), 'native_last_time_s':float(times[-1]),
        'frame_sample_indices':indices, 'frame_saved_times_s':[float(times[i]) for i in indices],
        'stills':{name:dict(sample_index=i, time_s=float(times[i]), phase=samples[i]['phase'],
            qpos_sha256=hashlib.sha256(qpos[i].tobytes()).hexdigest(),
            qvel_sha256=hashlib.sha256(qvel[i].tobytes()).hexdigest(), original_sample=samples[i],
            original_executed_command_norms=command_norms(samples[i])) for name, i in all_stills.items()},
        'force_state_command_timing':'Original solve/retained geometry t−50µs; saved qpos/qvel t. Command geometry/calibration from previous retained solve t−100µs, first-command initialization exception. Executed V5 approximate arm-inertia command records are not thread/contact forces; no replay force calculated.',
        'frame_time_quantization':'Normal1x12fps ceiling; exact original endpoint in last frame, no interpolation. GIF centisecond rounding separate. Inherited canonical1xslowmotion footer is factor1 normalplayback.',
        'cold_parent_declaration_sha256':sha(run/'diagnostic_declaration.json'),
        'original_inertia_helper_source_sha256':sha(run/'robot_inertia_source.py'),
        'original_inertia_metadata':report['robot_inertia_feedforward'],
        'source65_before_after_render':sources, 'source65_unchanged':True,
        'original_run_files_sha256_before_after':{str(p.relative_to(run)):d for p, d in originals.items()},
        'runtime_before':runtime, 'runtime_after':engine_info(), 'archived_model_identity':model_identity,
        'refresh':['mj_kinematics', 'mj_comPos', 'mj_camlight'],
        'mj_forward_called':False, 'collision_discovery_called':False, 'force_solve_called':False,
        'physics_integration':False, 'state_interpolation':False, 'body_hiding_or_geometry_change':False,
        'canonical_camera_and_original_geomgroup_filter_unchanged':True, 'detail_cameras':cameras,
        'helper_sha256':sha(__file__), 'media_sha256':{name:sha(output/name) for name in media_names}}
    (output/'render_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    archive = output/'renderer_sources'
    archive.mkdir()
    shutil.copyfile(__file__, archive/Path(__file__).name)
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
    parser.add_argument('--mode', choices=('progress','closed'), default='progress')
    parser.add_argument('--stills-only', action='store_true')
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
