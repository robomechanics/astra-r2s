"""Original closed cold crest-search V3 media; geometry refresh only."""
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


def geometry_refresh(model, data):
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_camlight(model, data)


def main(args):
    run, output = args.run.resolve(), args.output.resolve()
    assert not output.exists()
    report = json.loads((run/'insertion_validation.json').read_text())
    declaration = json.loads((run/'diagnostic_declaration.json').read_text())
    closure = json.loads((run/'diagnostic_execution_after.json').read_text())
    assert report['passed'] is False and report['partial'] is True
    assert report['aborted']['phase'] == 'stop_reverse_seat_1'
    assert report['full_fresh_trajectory_qualified'] is False
    assert report['capture_or_open_reset_qualified'] is False
    sources = declaration['source_65_before']
    assert len(sources) == 65 and sources == closure['source_65_after']
    for name, digest in sources.items():
        assert sha(ROOT/name) == digest, name
    originals = {p:sha(p) for p in run.rglob('*') if p.is_file()}
    require_micron_engine()
    runtime = engine_info()
    assert runtime == report['runtime']
    trace = run/'insertion_trace.npz'
    trace_sha = sha(trace)
    with np.load(trace, allow_pickle=False) as archive:
        times = archive['time'].copy()
        qpos, qvel = archive['qpos'].copy(), archive['qvel'].copy()
        samples = json.loads(str(archive['info_json']))
        assert json.loads(str(archive['metadata_json'])) == report
    assert len(times) == len(samples) == len(qpos) == len(qvel)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    request = next(event for event in report['physical_motion_events']
        if event.get('phase') == 'reverse_seat_1' and event.get('event') == 'Live physical phase readiness consumed')
    assert request['seat_direction_event']['stop_requested_from_measured_return'] is True
    request_indices = np.flatnonzero(abs(times-request['time_s']) < 1e-10)
    assert len(request_indices) == 1
    request_index = int(request_indices[0])
    initial_index = max(i for i, row in enumerate(samples) if row['phase'] == 'cold_window_hold')
    model, model_identity = recorded_model(trace, report)
    data = mujoco.MjData(model)
    renderer = SupportedRenderer(model)
    output.mkdir(parents=True)
    fps = 12
    indices = [min(len(times)-1, int(np.searchsorted(times, times[0]+i/fps)))
        for i in range(math.ceil(float(times[-1]-times[0])*fps))]
    indices[-1] = len(times)-1
    writer = imageio.get_writer(output/'demo.mp4', fps=fps, codec='libx264', quality=8,
        macro_block_size=1, ffmpeg_params=['-movflags', '+faststart', '-pix_fmt', 'yuv420p'])

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
        draw.text((20, 100), 'COLD V3 FAILED · radial guard during abrupt CLOSED stop · no capture / opening',
            font=renderer.small, fill='#f1c46c')
        return image

    stills = {'demo.png':len(times)-1, 'search_start.png':initial_index,
        'stop_request.png':request_index}
    try:
        for index in indices:
            writer.append_data(np.asarray(canvas(index)))
        for name, index in stills.items():
            canvas(index).save(output/name)
    finally:
        writer.close()
        renderer.close()

    cameras = [dict(lookat_m=[.350, -.010, .029], distance_m=.15,
        azimuth_deg=100., elevation_deg=-30.), dict(lookat_m=[.350, -.010, .029],
        distance_m=.15, azimuth_deg=130., elevation_deg=-5.)]
    detail = mujoco.Renderer(model, width=640, height=640)
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big, small = ImageFont.truetype(font, 25), ImageFont.truetype(font, 18)
    detail_stills = {'stop_request_detail.png':request_index, 'abort_detail.png':len(times)-1}
    try:
        for name, index in detail_stills.items():
            state(index)
            image = Image.new('RGB', (1280, 832), '#101924')
            for k, view in enumerate(cameras):
                camera = mujoco.MjvCamera()
                camera.lookat[:] = view['lookat_m']
                camera.distance, camera.azimuth, camera.elevation = (
                    view['distance_m'], view['azimuth_deg'], view['elevation_deg'])
                detail.update_scene(data, camera=camera)
                image.paste(Image.fromarray(detail.render()), (640*k, 104))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 1280, 104), fill='#101924')
            draw.rectangle((0, 720, 1280, 832), fill='#101924')
            row = samples[index]
            draw.text((16, 10), 'Cold crest-search V3 · original CLOSED stop-request / radial failure', font=big, fill='white')
            draw.text((16, 43), f'{times[index]:.5f} s local native · {row["phase"]} · radial {row["radial_offset_m"]*1e6:.3f} µm / 150 µm guard', font=small, fill='#f1c46c')
            draw.text((16, 71), 'Actual saved state · no formed capture, quiet stop, forward scan or opening', font=small, fill='#c5d6e4')
            left, right = row['left_contact']['pad_normal_force_N'], row['contact']['pad_normal_force_N']
            draw.text((16, 732), f'Original solve t−50µs: table {row["table_support"]["table_upward_force_N"]:.6f} N · left pads {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N', font=small, fill='#c5d6e4')
            if index == request_index:
                event = row['seat_direction_event']
                draw.text((16, 762), f'Original request true: crest return {event["actual_crest_return_m"]*1e6:.4f} µm · confirmation {event["confirmed_search_direction_event"]} · geometry-only refresh', font=small, fill='#c5d6e4')
            else:
                draw.text((16, 762), 'Original final observer epoch cleared after radial failure; earlier request retained in phase-event metadata', font=small, fill='#c5d6e4')
            draw.text((16, 792), 'Cold init from full-v1 13.1949s post-state · no solver warm-start / old force-window reuse / splice / body hiding', font=small, fill='#c5d6e4')
            image.save(output/name)
    finally:
        detail.close()

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
    media_names = ['demo.mp4', 'demo.gif', *all_stills]
    manifest = {
        'scope':'Original closed FAILED cold crest-return V3 branch only. Request1.61915s, radial abort1.6254s; no quiet-stop confirmation, forward scan, formed capture/open/reset/full trajectory. Original flags retained. Exact saved qpos/qvel with geometry-only refresh, no integration/forward/collision/force solve/interpolation/posing/splice.',
        'trajectory_sha256':trace_sha, 'original_report_sha256':sha(run/'insertion_validation.json'),
        'original_passed':False, 'original_partial':True, 'original_aborted':report['aborted'],
        'original_stop_request_event':request,
        'original_final_cleared_event':report['final_experimental_crest_direction_event'],
        'fps':fps, 'slow_motion':1, 'frames':len(indices),
        'encoded_video_duration_s':len(indices)/fps, 'gif_frames':len(actual_durations),
        'gif_duration_ms':sum(actual_durations), 'native_first_time_s':float(times[0]),
        'native_last_time_s':float(times[-1]),
        'frame_sample_indices':indices, 'frame_saved_times_s':[float(times[i]) for i in indices],
        'stills':{name:dict(sample_index=i, time_s=float(times[i]), phase=samples[i]['phase'],
            qpos_sha256=hashlib.sha256(qpos[i].tobytes()).hexdigest(),
            qvel_sha256=hashlib.sha256(qvel[i].tobytes()).hexdigest(), original_sample=samples[i])
            for name, i in all_stills.items()},
        'force_state_command_timing':'Original preintegration solve/retained geometry t−50µs; postintegration qpos/qvel/state t. Executed command calibration prior retained solve t−100µs with first-command initialization exception. No replay force calculated.',
        'frame_time_quantization':'Normal1x12fps ceiling; exact native endpoint in last frame, no interpolation. GIF centisecond rounding separate. Inherited canonical1xslowmotion footer is factor1 normalplayback.',
        'cold_parent_declaration_sha256':sha(run/'diagnostic_declaration.json'),
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
    print(json.dumps({key:manifest[key] for key in ('frames','encoded_video_duration_s',
        'gif_duration_ms','native_last_time_s','trajectory_sha256','media_sha256')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository-root', type=Path, default=ROOT)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
