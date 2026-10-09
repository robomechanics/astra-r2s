"""Validate a closed failed-trial replay without altering the native outcome."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile
import imageio.v2 as imageio
from PIL import Image
import numpy as np
sys.path.insert(0, str(Path.cwd()))
from scripts.audit_m8_insertion_trace import recorded_model, inspected_source_identity

directory = Path('outputs/m8_table_pickup/damped_failure_media')
run = Path('outputs/m8_table_pickup/full_damped')
trace = run/'insertion_trace.npz'
ffmpeg = '/workspace/.venvs/m8-contact/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
renderer_log = Path('outputs/m8_table_pickup/damped_failure_render.log')
render_report = json.loads(renderer_log.read_text().strip().splitlines()[-1])
shutil.copyfile(renderer_log, directory/'renderer_output.json')
shutil.copyfile(__file__, directory/'container_check_source.py')
subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
    '-i', str(directory/'insertion_demo.mp4'), '-vf',
    'fps=6,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
    str(directory/'insertion_demo.gif')], check=True)
subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error',
    '-i', str(directory/'insertion_demo.mp4'), '-f', 'null', '-'], check=True)
reader = imageio.get_reader(directory/'insertion_demo.mp4')
video_metadata = reader.get_meta_data()
frames = reader.count_frames()
reader.close()
assert video_metadata['codec'] == 'h264'
assert video_metadata['pix_fmt'].startswith('yuv420p')
assert video_metadata['fps'] == 12
atoms = []
raw = (directory/'insertion_demo.mp4').read_bytes()
offset = 0
while offset + 8 <= len(raw):
    length = int.from_bytes(raw[offset:offset+4], 'big')
    name = raw[offset+4:offset+8].decode('ascii')
    if length == 1:
        length = int.from_bytes(raw[offset+8:offset+16], 'big')
    elif length == 0:
        length = len(raw)-offset
    assert length >= 8 and offset+length <= len(raw)
    atoms.append({'name': name, 'offset': offset, 'length': length})
    offset += length
assert offset == len(raw)
assert next(x['offset'] for x in atoms if x['name'] == 'moov') < next(x['offset'] for x in atoms if x['name'] == 'mdat')
with Image.open(directory/'insertion_demo.gif') as animated:
    gif_frames = animated.n_frames
    gif_duration_ms = 0
    for index in range(gif_frames):
        animated.seek(index)
        animated.load()
        gif_duration_ms += animated.info.get('duration', 0)
with Image.open(directory/'insertion_demo.png') as still:
    still.load()
    still_size = list(still.size)
with zipfile.ZipFile(trace) as zipped:
    assert zipped.testzip() is None
with np.load(trace, allow_pickle=False) as saved:
    arrays = {name: saved[name].copy() for name in saved.files}
report = json.loads(str(arrays['metadata_json']))
rows = json.loads(str(arrays['info_json']))
model, model_identity = recorded_model(trace, report)
assert arrays['qpos'].shape == (len(arrays['time']), model.nq)
assert arrays['qvel'].shape == (len(arrays['time']), model.nv)
assert len(rows) == len(arrays['time'])
assert np.isfinite(arrays['qpos']).all() and np.isfinite(arrays['qvel']).all()
trace_sha = hashlib.sha256(trace.read_bytes()).hexdigest()
assert trace_sha == 'fbb07d1dc6f9931ace2cd4a68b1cef13d6f747b6a6807bb7bb597c4b06d0c166'
assert trace_sha == render_report['trajectory_sha256']
assert frames == render_report['frames'] == 312
assert report['passed'] is False and report['aborted']['phase'] == 'release_search_3'
controller_identity = inspected_source_identity(run/'controller_source.py')
assert controller_identity['controller_sha256'] == report['controller_sha256']
chart = json.loads((directory/'insertion_trajectory.json').read_text())
assert chart['source_trace_sha256'] == trace_sha
assert chart['source_report_sha256'] == hashlib.sha256((run/'insertion_validation.json').read_bytes()).hexdigest()
assert chart['lead_qualified_turn_segments'] == [] and chart['capture_tag_time_s'] is None
still_index = min(len(arrays['time'])-1, int(np.searchsorted(arrays['time'], 17.28)))
manifest = {
    'video': 'insertion_demo.mp4', 'screenshot': 'insertion_demo.png',
    'gif': 'insertion_demo.gif', 'frames': frames, 'fps': 12, 'slow_motion': 1.5,
    'source_recording': str(trace), 'trajectory_sha256': trace_sha,
    'recorded_validation_passed': False,
    'status': 'aborted_trial', 'finished_rollout': False,
    'partial_task_scope': report['partial'],
    'partial_flag_note': 'The recorded partial flag records requested phase selection, not completion. The real trajectory aborted.',
    'actual_abort': report['aborted'],
    'source_end_time_s': float(arrays['time'][-1]), 'source_end_phase': rows[-1]['phase'],
    'formed_thread_support_claim': False, 'capture_claim': False, 'qualified_thread_claim': False,
    'scope_note': 'Both genuine pickups and first cone-start opening/reset/regrasp recovery are shown. The second starting stroke was followed by a second release that exceeded the unchanged 10 micrometre SDF depth guard. The rollout stopped before formed-thread capture or qualified lead.',
    'render_method': 'Native recorded qpos/qvel samples selected at frame time with searchsorted. mj_forward computes display transforms; no dynamics integration, no q interpolation, no reconstructed-force claim.',
    'still': {'requested_time_s': 17.28, 'recorded_time_s': float(arrays['time'][still_index]),
              'recorded_phase': rows[still_index]['phase'], 'size_pixels': still_size},
    'video_metadata': video_metadata, 'video_top_level_atoms': atoms,
    'faststart_moov_before_mdat': True,
    'closed_container_and_all_frames_decoded': True,
    'gif_frames': gif_frames, 'gif_fps_filter': 6, 'gif_width_pixels': 800,
    'gif_duration_ms': gif_duration_ms, 'all_gif_frames_decoded': True,
    'model_identity': model_identity,
    'controller_source_identity': controller_identity,
    'renderer_source_sha256': hashlib.sha256((directory/'renderer_source.py').read_bytes()).hexdigest(),
    'chart_metadata': 'insertion_trajectory.json',
    'source_private_videos_included': False,
}
(directory/'render_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
checksums = {str(path.relative_to(directory)): {'bytes': path.stat().st_size,
    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    for path in sorted(directory.rglob('*')) if path.is_file() and path.name != 'checksums.json'}
(directory/'checksums.json').write_text(json.dumps(checksums, indent=2)+'\n')
for name, identity in checksums.items():
    path = directory/name
    assert path.stat().st_size == identity['bytes']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256']
print(json.dumps({'directory': str(directory), 'files_verified': len(checksums),
    'trajectory_sha256': trace_sha, 'video_frames': frames, 'gif_frames': gif_frames,
    'total_bytes_excluding_checksums': sum(x['bytes'] for x in checksums.values()),
    'sizes': {name: checksums[name]['bytes'] for name in ('insertion_demo.mp4', 'insertion_demo.gif', 'insertion_demo.png')}}, indent=2))
