"""Check closed renders and bind their exact trace and renderer sources."""
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

directory = Path('media/m8_table_pickup/progress_first_start')
rendered = Path('outputs/m8_table_pickup/progress_first_start')
log = Path('outputs/m8_table_pickup/progress_first_start_render.log')
ffmpeg = Path('/workspace/.venvs/m8-contact/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')
render_report = json.loads(log.read_text().strip().splitlines()[-1])
for source, name in ((rendered/'insertion_demo.mp4', 'demo.mp4'),
                     (rendered/'insertion_demo.png', 'demo.png'),
                     (log, 'renderer_output.json'),
                     (Path(__file__), 'container_check_source.py')):
    shutil.copyfile(source, directory/name)
subprocess.run([str(ffmpeg), '-hide_banner', '-loglevel', 'error', '-y',
    '-i', str(directory/'demo.mp4'), '-vf',
    'fps=6,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
    str(directory/'demo.gif')], check=True)
# Decode every video frame through ffmpeg; a readable header alone is insufficient.
subprocess.run([str(ffmpeg), '-hide_banner', '-loglevel', 'error',
    '-i', str(directory/'demo.mp4'), '-f', 'null', '-'], check=True)
reader = imageio.get_reader(directory/'demo.mp4')
video_metadata = reader.get_meta_data()
frames = reader.count_frames()
reader.close()
assert video_metadata['codec'] == 'h264'
assert video_metadata['pix_fmt'].startswith('yuv420p')
atoms = []
raw_video = (directory/'demo.mp4').read_bytes()
offset = 0
while offset + 8 <= len(raw_video):
    length = int.from_bytes(raw_video[offset:offset+4], 'big')
    name = raw_video[offset+4:offset+8].decode('ascii')
    if length == 1:
        length = int.from_bytes(raw_video[offset+8:offset+16], 'big')
    elif length == 0:
        length = len(raw_video)-offset
    assert length >= 8 and offset+length <= len(raw_video)
    atoms.append({'name': name, 'offset': offset, 'length': length})
    offset += length
assert offset == len(raw_video)
assert next(x['offset'] for x in atoms if x['name'] == 'moov') < next(x['offset'] for x in atoms if x['name'] == 'mdat')
with Image.open(directory/'demo.gif') as animated:
    gif_frames = animated.n_frames
    gif_duration_ms = 0
    for index in range(gif_frames):
        animated.seek(index)
        animated.load()
        gif_duration_ms += animated.info.get('duration', 0)
with Image.open(directory/'demo.png') as still:
    still.load()
    still_size = still.size
with zipfile.ZipFile(directory/'insertion_trace.npz') as saved:
    assert saved.testzip() is None
with np.load(directory/'insertion_trace.npz', allow_pickle=False) as saved:
    arrays = {name: saved[name].copy() for name in saved.files}
report = json.loads(str(arrays['metadata_json']))
rows = json.loads(str(arrays['info_json']))
model, model_identity = recorded_model(directory/'insertion_trace.npz', report)
assert arrays['qpos'].shape == (len(arrays['time']), model.nq)
assert arrays['qvel'].shape == (len(arrays['time']), model.nv)
assert len(rows) == len(arrays['time'])
assert np.isfinite(arrays['qpos']).all() and np.isfinite(arrays['qvel']).all()
trace_sha = hashlib.sha256((directory/'insertion_trace.npz').read_bytes()).hexdigest()
assert trace_sha == render_report['trajectory_sha256']
assert frames == render_report['frames'] == 142
controller_identity = inspected_source_identity(directory/'controller_source.py')
assert controller_identity['controller_sha256'] == report['controller_sha256']
manifest = {
    'status': 'incomplete_progress_prefix',
    'passed': False, 'finished_rollout': False,
    'formed_thread_support_claim': False, 'qualified_thread_claim': False,
    'trajectory': 'insertion_trace.npz', 'trajectory_sha256': trace_sha,
    'source_end_time_s': float(arrays['time'][-1]),
    'source_end_phase': rows[-1]['phase'],
    'video': {'file': 'demo.mp4', 'frames': frames, 'fps': 10,
              'slow_motion': 1, 'metadata': video_metadata,
              'top_level_atoms': atoms, 'faststart_moov_before_mdat': True,
              'closed_container_and_all_frames_decoded': True},
    'gif': {'file': 'demo.gif', 'frames': gif_frames, 'fps_filter': 6,
            'width_pixels': 800, 'duration_ms': gif_duration_ms,
            'all_frames_decoded': True},
    'still': {'file': 'demo.png', 'size_pixels': list(still_size),
              'requested_time_s': 12.3,
              'recorded_time_s': float(arrays['time'][np.searchsorted(arrays['time'], 12.3)]),
              'recorded_phase': rows[np.searchsorted(arrays['time'], 12.3)]['phase']},
    'method': 'Native recorded qpos/qvel samples only. mj_forward computes display transforms. No dynamics integration and no q interpolation.',
    'model_identity': model_identity,
    'controller_source_identity': controller_identity,
    'renderer_source_sha256': hashlib.sha256((directory/'renderer_source.py').read_bytes()).hexdigest(),
    'scope_note': 'Both actual pickups and first cone-start opening/reset/regrasp recovery. This prefix is not a final rollout or a formed-thread support/capture/qualified lead proof.',
}
(directory/'render_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
checksums = {str(path.relative_to(directory)): {'bytes': path.stat().st_size,
    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    for path in sorted(directory.rglob('*')) if path.is_file()
    and path.name != 'checksums.json'}
(directory/'checksums.json').write_text(json.dumps(checksums, indent=2)+'\n')
for name, identity in checksums.items():
    path = directory/name
    assert path.stat().st_size == identity['bytes']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256']
print(json.dumps({'package': str(directory), 'bytes': sum(x['bytes'] for x in checksums.values()),
    'files_verified': len(checksums), 'trace_sha256': trace_sha,
    'video_frames': frames, 'gif_frames': gif_frames,
    'largest_files': sorted([(name, x['bytes']) for name,x in checksums.items()], key=lambda x:x[1], reverse=True)[:5]}, indent=2))
