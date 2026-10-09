"""Package a closed recorded-state progress prefix; never infer completion."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
import numpy as np

root = Path.cwd()
recording = root / 'outputs/m8_table_pickup/full_damped'
destination = root / 'media/m8_table_pickup/progress_first_start'
destination.mkdir(parents=True, exist_ok=False)
snapshot = recording / 'progress_first_start.npz'
sources = ['scene.xml', 'scene_source.py', 'controller_source.py', 'engagement_observer_source.py']
for name in sources:
    shutil.copyfile(recording/name, destination/name)
shutil.copytree(recording/'recorded_sources', destination/'recorded_sources')
shutil.copyfile(snapshot, destination/'insertion_trace.npz')
shutil.copyfile(recording/'progress_first_start.json', destination/'snapshot_manifest.json')
for path, filename in [
    (root/'yam_twin/m8_insertion_demo.py', 'renderer_source.py'),
    (root/'scripts/audit_m8_insertion_trace.py', 'recorded_model_auditor_source.py'),
    (root/'outputs/m8_table_pickup/create_progress_snapshot.py', 'snapshot_source.py'),
    (Path(__file__), 'package_source.py'),
]:
    shutil.copyfile(path, destination/filename)
with np.load(snapshot, allow_pickle=False) as saved:
    times = saved['time'].copy()
    rows = json.loads(str(saved['info_json']))
    metadata = json.loads(str(saved['metadata_json']))
still_index = min(len(times)-1, int(np.searchsorted(times, 12.3)))
snapshot_info = json.loads((destination/'snapshot_manifest.json').read_text())
dependencies = {}
for mesh in ET.fromstring((destination/'scene.xml').read_bytes()).findall('asset/mesh'):
    value = mesh.get('file')
    if not value:
        continue
    path = Path(value)
    if not path.is_absolute():
        path = root/path
    name = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
    dependencies[name] = hashlib.sha256(path.read_bytes()).hexdigest()
(destination/'model_dependencies.json').write_text(json.dumps({
    'scope': 'Exact archived XML and sources are included. Hash-listed robot mesh dependencies are the repository assets; replay from this repository with scripts/run_m8.sh.',
    'mesh_sha256': dependencies,
    'runtime': metadata['runtime'],
    'model_fingerprint': metadata['model_fingerprint'],
    'model_xml_sha256': metadata['model_xml_sha256'],
}, indent=2)+'\n')
(destination/'progress_manifest.json').write_text(json.dumps({
    **snapshot_info,
    'immutable_snapshot': 'insertion_trace.npz',
    'first_still_requested_time_s': 12.3,
    'first_still_recorded_time_s': float(times[still_index]),
    'first_still_phase': rows[still_index]['phase'],
    'original_recorded_metadata': metadata,
    'completion_validation_available': False,
    'passed': False,
    'partial_progress': True,
    'replay': 'scripts/run_m8.sh -m yam_twin.m8_insertion_demo --replay media/m8_table_pickup/progress_first_start/insertion_trace.npz --output outputs/m8_table_pickup/replay_progress_first_start --fps 10 --slow-motion 1 --still-time 12.3',
    'render_method': 'Exact native recorded qpos/qvel states selected by searchsorted at frame time. mj_forward for display only; no integration and no interpolation.',
    'source_private_videos_included': False,
}, indent=2)+'\n')
(destination/'README.md').write_text('''This is an incomplete progress prefix from the fresh table-pickup rollout, not a completed threading demonstration.

It shows the left arm picking up and rolling the free block, the right arm picking up the separately supported bolt, and the first cone-start attempt followed by opening, reset and regrasp. The prefix ends at 14.18785 s before the second starting stroke. Cone contact and this opening/reset sequence do not establish formed-thread support, thread capture or qualified lead.

`demo.mp4` and `demo.gif` replay native recorded states. No physics is integrated, interpolated or posed for these renders. `demo.png` is the actual recorded state at the nearest saved sample after 12.3 s. The original trace metadata's `partial` field reflects requested phase selection rather than completion; `progress_manifest.json` explicitly marks this artifact as incomplete.

`insertion_trace.npz` is a byte-exact, closed snapshot of the running phase-prefix archive. Exact archived scene/controller/observer sources and imported controller helper sources accompany it. Robot mesh assets remain in this repository and their hashes are listed in `model_dependencies.json`. Replay with the command in `progress_manifest.json` using the pinned native runtime selected by `scripts/run_m8.sh`.

This progress package is not an all-substep force audit, a completed acceptance report, a formed-thread support proof, or a training-ready certification. No private input video is included.
''')
print(json.dumps({'destination': str(destination), 'trace_sha256': snapshot_info['trajectory_sha256'], 'files_so_far': len(list(destination.rglob('*')))}, indent=2))
