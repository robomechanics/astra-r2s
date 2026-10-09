"""Archive an incomplete, genuine faster-entry prefix for recorded replay."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from scripts.audit_m8_insertion_trace import archived_asset_path, recorded_model, inspected_source_identity

run=Path('outputs/m8_table_pickup/full_faster_entry')
destination=Path('media/m8_table_pickup/progress_faster_start')
destination.mkdir(parents=True,exist_ok=False)
trace=run/'progress_first_faster_start.npz'
with np.load(trace,allow_pickle=False) as saved:
    arrays={name:saved[name].copy() for name in saved.files}
rows=json.loads(str(arrays['info_json']))
metadata=json.loads(str(arrays['metadata_json']))
assert rows[-1]['phase']=='settle_regrip_search_2'
assert not any(row['phase']=='start_thread_2' for row in rows)
assert not any(row['thread_engaged'] for row in rows)
for name in ('scene.xml','scene_source.py','controller_source.py','engagement_observer_source.py'):
    shutil.copyfile(run/name,destination/name)
shutil.copytree(run/'recorded_sources',destination/'recorded_sources')
shutil.copyfile(trace,destination/'insertion_trace.npz')
shutil.copyfile(trace.with_suffix('.json'),destination/'snapshot_manifest.json')
for source,name in ((Path('yam_twin/m8_insertion_demo.py'),'renderer_source.py'),
    (Path('scripts/audit_m8_insertion_trace.py'),'recorded_model_auditor_source.py'),
    (Path('outputs/m8_table_pickup/snapshot_faster_progress.py'),'snapshot_source.py'),
    (Path(__file__),'package_source.py')):
    shutil.copyfile(source,destination/name)
model,model_identity=recorded_model(destination/'insertion_trace.npz',metadata)
controller_identity=inspected_source_identity(destination/'controller_source.py')
assert controller_identity['controller_sha256']==metadata['controller_sha256']
assert metadata['controller_sha256']=='dce8a1e1a8bc53d6caf4b717bf846a04fee1c5ebfafac9428b94f8e705385121'
assert hashlib.sha256((destination/'controller_source.py').read_bytes()).hexdigest()=='199ac49380e95b5849caf98e6bbfda1c0e85d1626fc97e468a421463a03394e8'
dependencies={}
for mesh in ET.fromstring((destination/'scene.xml').read_bytes()).findall('asset/mesh'):
    name=mesh.get('file')
    if not name:
        continue
    path=archived_asset_path(name,destination).resolve()
    label=str(path.relative_to(Path.cwd())) if path.is_relative_to(Path.cwd()) else str(path)
    dependencies[label]=hashlib.sha256(path.read_bytes()).hexdigest()
(destination/'model_dependencies.json').write_text(json.dumps({
    'scope':'Exact archived XML and source modules are included. Mesh dependencies are repository assets listed by SHA; replay from this repository with scripts/run_m8.sh.',
    'mesh_sha256':dependencies,'runtime':metadata['runtime'],'model_identity':model_identity
},indent=2,allow_nan=False)+'\n')
commit=subprocess.run(['git','rev-parse','HEAD'],check=True,capture_output=True,text=True).stdout.strip()
still_index=min(len(rows)-1,int(np.searchsorted(arrays['time'],17.1)))
manifest={
    'status':'incomplete_faster_start_progress','passed':False,
    'full_acceptance_claim':False,'capture_claim':False,'qualified_lead_claim':False,
    'formed_thread_support_claim':False,'finished_rollout':False,
    'scope':'Both genuine table pickups, settled native entry, first starting stroke at 1 rad/s and physical release/opening/reset/regrasp recovery. This prefix ends before the second starting stroke. Partial formed geometry is below the capture threshold; no formed capture, qualified pitch or full acceptance is claimed.',
    'source_commit':commit,'render_time_prefix_sha256':hashlib.sha256(trace.read_bytes()).hexdigest(),
    'snapshot_last_time_s':float(arrays['time'][-1]),'snapshot_last_phase':rows[-1]['phase'],
    'phase_sequence':list(dict.fromkeys(row['phase'] for row in rows)),
    'starting_angular_speed_rad_s':metadata['control_config']['starting_angular_speed_rad_s'],
    'video_playback_slow_motion_factor':1,
    'first_still_requested_time_s':17.1,
    'first_still_recorded_time_s':float(arrays['time'][still_index]),
    'first_still_phase':rows[still_index]['phase'],
    'first_still_sample':rows[still_index],
    'last_native_sample':rows[-1],
    'source_recorded_metadata':metadata,
    'metadata_partial_note':'The native metadata partial field records requested phase selection and cannot establish completion. This prefix remains incomplete.',
    'model_identity':model_identity,'controller_identity':controller_identity,
    'replay_command':'scripts/run_m8.sh -m yam_twin.m8_insertion_demo --replay media/m8_table_pickup/progress_faster_start/insertion_trace.npz --output outputs/m8_table_pickup/replay_faster_progress --fps 12 --slow-motion 1 --still-time 17.1',
    'render_method':'Native recorded qpos/qvel/time samples only; mj_forward computes display transforms. No dynamics integration, state interpolation or pose editing.',
    'source_private_videos_included':False,
}
(destination/'progress_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
(destination/'README.md').write_text('''This is an incomplete progress prefix of the faster native M8 table-pickup rollout, shown at normal 1× playback speed.

Both arms start open with the block and bolt separately supported on the table. The video shows the left arm picking up and rolling the free block, the right arm picking up the free bolt, settled cone/lead-in contact, the first starting stroke at 1 rad/s, and physical release/opening/reset/regrasp recovery. The prefix ends at 19.0268 s after the first regrasp and before the second starting stroke.

Capture remains false throughout this prefix. The final potential formed-flank overlap is about 0.125 mm, below the 1.25 mm pitch threshold. This is a contact-driven starting attempt, not a completed capture or qualified-thread lead demonstration. The original native metadata's `partial` flag records requested phase selection rather than completion; this artifact explicitly remains incomplete.

`demo.mp4` is H264/yuv420p at 12 fps with normal 1× playback. `demo.gif` is an 800 px, 6 fps GitHub preview. `demo.png` shows the saved first-release/recovery state around 17.10 s. All renders select exact native recorded qpos/qvel states and use `mj_forward` for display only, without integration, interpolation, pose editing or a grasp weld.

The closed immutable prefix trace, exact XML, controller/scene/observer and imported controller helper sources are included. `model_dependencies.json` binds repository robot-mesh assets and runtime identity. `render_manifest.json` binds the completed media to this exact trace; all JSON uses strict finite serialization and `checksums.json` lists the final file bytes.

No formed-thread support, capture, qualified pitch, full acceptance or training-ready certification is claimed. No private source video is included.
''')
print(json.dumps({'package':str(destination),'source_commit':commit,
    'trace_sha256':manifest['render_time_prefix_sha256'],'still_time_s':manifest['first_still_recorded_time_s'],
    'still_phase':manifest['first_still_phase']},indent=2,allow_nan=False))
