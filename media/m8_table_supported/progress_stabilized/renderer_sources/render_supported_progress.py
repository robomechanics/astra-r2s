"""Replay exact native supported-scene states; never integrate or pose free parts.

Output-only progress renderer. Existing published pickup/carry renderer and media
are not edited. Original saved force diagnostics remain original-solve evidence;
forces recomputed by mj_forward are never used for the overlay or manifest.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path('/workspace/astra-r2s')
sys.path.insert(0, str(ROOT))
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('MESA_SHADER_CACHE_DIR', '/workspace/.cache/mesa')
os.environ.setdefault('LP_NUM_THREADS', '2')
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from yam_twin.m8_insertion_demo import InsertionRenderer
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import engine_info, require_micron_engine


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plain_number(value, precision=3):
    if value is None:
        return '?'
    return f'{float(value):.{precision}f}'


class SupportedRenderer(InsertionRenderer):
    def __init__(self, model):
        super().__init__(model)
        self.overview_camera.lookat[:] = [.30, -.035, .13]
        self.overview_camera.distance = 1.05
        self.overview_camera.azimuth, self.overview_camera.elevation = 160., -42.
        self.hole_camera.distance = .075
        self.hole_camera.elevation = -50.

    def frame(self, data, sample=None, report=None, *, static=False, slow_motion=1):
        sample, report = sample or {}, report or {}
        canvas = super().frame(data, sample, report, static=static, slow_motion=slow_motion)
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((0, 0, 610, 96), fill='#101924')
        draw.text((20, 20), 'M8 assembly on the table', font=self.big, fill='white')
        draw.text((20, 59), 'Left stabilizes free block · right picks up free bolt',
                  font=self.small, fill='#c5d6e4')
        draw.rectangle((0, 620, 864, 720), fill='#101924')
        phase = str(sample.get('phase', 'Scene setup')).replace('_', ' ')
        draw.text((20, 629), phase, font=self.big, fill='white')
        note = ('STATIC SCENE PREVIEW · NO ROLLOUT' if static else
                f'{data.time:.2f} s native physics · {slow_motion:g}× playback')
        draw.text((20, 680), note, font=self.small, fill='#c5d6e4')
        if not static:
            left = sample.get('left_contact', {})
            pads = left.get('pad_normal_force_N', [])
            table = sample.get('table_support', sample.get('table_support_state', {}))
            upward = table.get('table_upward_force_N', sample.get('table_upward_force_N'))
            draw.text((20, 655), f'Table upward force: {plain_number(upward)} N',
                      font=self.small, fill='#c5d6e4')
            if len(pads) == 2:
                draw.text((420, 632), f'Left pads: {pads[0]:.2f} / {pads[1]:.2f} N',
                          font=self.small, fill='#c5d6e4')
            right = sample.get('contact', {}).get('pad_normal_force_N', [])
            if len(right) == 2:
                draw.text((420, 655), f'Right pads: {right[0]:.2f} / {right[1]:.2f} N',
                          font=self.small, fill='#c5d6e4')
            status = report.get('media_status', 'Progress only · full trajectory pending')
            draw.text((420, 680), status, font=self.small, fill='#f1c46c')
        draw.rectangle((864, 630, 1296, 720), fill='#101924')
        draw.text((882, 638), 'Real solid table bears block weight', font=self.small, fill='#c5d6e4')
        draw.text((882, 660), 'Native joints + fingers · no grasp weld', font=self.small, fill='#c5d6e4')
        draw.text((882, 682), 'M8 × 1.25 · short male shaft', font=self.small, fill='#c5d6e4')
        return canvas


def render(trace, output, *, still_time=None, video=False, fps=12):
    require_micron_engine()
    source_paths = [Path(__file__), ROOT/'yam_twin/m8_insertion_demo.py',
        ROOT/'yam_twin/m8_demo.py', ROOT/'scripts/audit_m8_insertion_trace.py',
        ROOT/'thread_lab/runtime.py']
    source_bytes = {p: p.read_bytes() for p in source_paths}
    trace, output = Path(trace).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with np.load(trace, allow_pickle=False) as saved:
        times, qpos, qvel = (saved[key].copy() for key in ('time', 'qpos', 'qvel'))
        samples = json.loads(str(saved['info_json']))
        report = json.loads(str(saved['metadata_json']))
    model, model_identity = recorded_model(trace, report)
    if (qpos.shape != (len(times), model.nq) or qvel.shape != (len(times), model.nv)
            or len(samples) != len(times) or not len(times) or not np.isfinite(qpos).all()
            or not np.isfinite(qvel).all() or not np.isfinite(times).all()):
        raise ValueError('Finite exact native state rows must align')
    index = len(times)-1 if still_time is None else min(len(times)-1,
                int(np.searchsorted(times, still_time)))
    indices = []
    if video:
        duration = float(times[-1]-times[0])
        indices = [min(len(times)-1, int(np.searchsorted(times, times[0]+i/fps)))
                   for i in range(max(1, int(np.ceil(duration*fps))))]
    data = mujoco.MjData(model)
    renderer = SupportedRenderer(model)
    writer = imageio.get_writer(output/'demo.mp4', fps=fps, codec='libx264', quality=8,
        macro_block_size=1, ffmpeg_params=['-pix_fmt', 'yuv420p', '-movflags', '+faststart']) if video else None
    try:
        for position, row in enumerate([*indices, index]):
            data.qpos[:], data.qvel[:], data.time = qpos[row], qvel[row], times[row]
            mujoco.mj_forward(model, data)
            image = renderer.frame(data, samples[row], report, slow_motion=1)
            if position == len(indices):
                image.save(output/'demo.png')
            else:
                writer.append_data(np.asarray(image))
    finally:
        if writer:
            writer.close()
        renderer.close()
    for path, value in source_bytes.items():
        if path.read_bytes() != value:
            raise ValueError(f'Render source changed while rendering: {path}')
    archived_renderers = output/'renderer_sources'
    archived_renderers.mkdir(exist_ok=True)
    for path, value in source_bytes.items():
        (archived_renderers/path.name).write_bytes(value)
    runtime = engine_info()
    manifest = {'status': 'actual_native_progress', 'trajectory_sha256': sha(trace),
        'source_recording': str(trace.relative_to(ROOT)), 'exact_saved_state_replay': True,
        'state_interpolation': False, 'physics_integration_during_render': False,
        'forces_from_original_solved_sample_only': True,
        'state_force_timing_scope': 'qpos/qvel are postintegration; original force samples refer to the immediately preceding native solve (time minus dt). mj_forward forces are not reported.',
        'fps': fps if video else None, 'slow_motion': 1, 'video_frames': len(indices),
        'first_physics_time_s': float(times[0]), 'last_physics_time_s': float(times[-1]),
        'screenshot_sample_index': index, 'screenshot_time_s': float(times[index]),
        'screenshot_phase': samples[index]['phase'],
        'screenshot_qpos_sha256': hashlib.sha256(qpos[index].tobytes()).hexdigest(),
        'screenshot_qvel_sha256': hashlib.sha256(qvel[index].tobytes()).hexdigest(),
        'original_saved_sample': samples[index],
        'model_xml_sha256': report['model_xml_sha256'],
        'model_fingerprint': report['model_fingerprint'],
        'scene_source_sha256': report.get('scene_source_sha256'),
        'controller_sha256': report.get('controller_sha256'),
        'archived_model_identity': model_identity, 'renderer_source_sha256': sha(__file__),
        'renderer_dependency_sha256': sha(ROOT/'yam_twin/m8_insertion_demo.py'),
        'recorded_model_loader_sha256': sha(ROOT/'scripts/audit_m8_insertion_trace.py'),
        'runtime': runtime, 'original_recorded_runtime': report.get('runtime'),
        'renderer_sources_sha256': {p.name: hashlib.sha256(v).hexdigest() for p, v in source_bytes.items()},
        'scope': 'Actual native left stabilization progress. Complete pickup/thread qualification is pending. No capture or completed trajectory is claimed by this media.'}
    manifest['media_sha256'] = {p.name: sha(p) for p in output.glob('demo.*')}
    (output/'render_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    (output/'renderer_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k: manifest[k] for k in ('status', 'screenshot_time_s', 'screenshot_phase', 'video_frames', 'media_sha256')}, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trace', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--still-time', type=float)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--fps', type=int, default=12)
    args = parser.parse_args()
    render(args.trace, args.output, still_time=args.still_time, video=args.video, fps=args.fps)

if __name__ == '__main__':
    main()
