"""Close-check and snapshot a genuine native phase prefix without changing it."""
from pathlib import Path
import hashlib,json,os,shutil,tempfile,zipfile
import numpy as np
ROOT=Path('/workspace/astra-r2s')
run=ROOT/'outputs/m8_table_supported/full_v2'
target=ROOT/'outputs/m8_table_supported/progress_bolt_over_bore'
if target.exists():raise FileExistsError(target)
source=run/'insertion_trace_partial.npz'
raw=source.read_bytes()
with tempfile.NamedTemporaryFile(suffix='.npz',dir=ROOT/'outputs/m8_table_supported',delete=False) as f:
 scratch=Path(f.name);f.write(raw);f.flush();os.fsync(f.fileno())
try:
 with zipfile.ZipFile(scratch) as z:
  if z.testzip() is not None:raise ValueError('Invalid closed prefix ZIP')
 with np.load(scratch,allow_pickle=False) as saved:
  times=saved['time'].copy();qpos=saved['qpos'].copy();qvel=saved['qvel'].copy()
  rows=json.loads(str(saved['info_json']));metadata=json.loads(str(saved['metadata_json']))
 if not (len(times)==len(rows)==len(qpos)==len(qvel) and np.isfinite(qpos).all() and np.isfinite(qvel).all()):
  raise ValueError('Nonfinite or unaligned native saved rows')
 candidates=[i for i,r in enumerate(rows) if r['phase']=='align_over_hole']
 if not candidates:raise ValueError('Required real align_over_hole phase has not been saved')
 selected=candidates[-1]
 target.mkdir()
 scratch.rename(target/'trace.npz');scratch=None
 for name in ('scene.xml','supported_scene.zip','controller_source.py','scene_source.py',
  'engagement_observer_source.py','renderer_source.py','software_tests.json','software_tests.log',
  'manifest.json','verify_sources.py','run_publication_identity.json'):
  shutil.copyfile(run/name,target/name)
 for name in ('recorded_sources','frozen_audit_sources'):
  shutil.copytree(run/name,target/name)
 note={'status':'incomplete_native_phase_prefix','source_recording':str(source.relative_to(ROOT)),
  'immutable_prefix':'trace.npz','trajectory_sha256':hashlib.sha256(raw).hexdigest(),
  'byte_exact_copy':True,'closed_zip_crc_verified':True,'samples':len(rows),
  'first_physics_time_s':float(times[0]),'last_physics_time_s':float(times[-1]),
  'last_phase':rows[-1]['phase'],'screenshot_sample_index':selected,
  'screenshot_time_s':float(times[selected]),'screenshot_phase':rows[selected]['phase'],
  'prefix_metadata_partial':metadata.get('partial'),
  'partial_field_scope':'This original field describes requested phase selection, not full native trajectory completion; the copied phase prefix is incomplete.',
  'scope':'Fresh full_v2 native progress after separate bolt pickup, guarded rest-clear transport and alignment above the table-supported bore. No thread contact/capture, qualified turn or completed task is claimed. Original all-step table/left ledgers are still open in the continuing parent trial.',
  'original_force_timing':metadata.get('native_force_recording_note'),
  'model_xml_sha256':metadata['model_xml_sha256'],'model_fingerprint':metadata['model_fingerprint'],
  'scene_source_sha256':metadata['scene_source_sha256'],'controller_sha256':metadata['controller_sha256'],
  'controller_module_sha256':metadata['controller_module_sha256'],
  'original_selected_sample':rows[selected],
  'selected_qpos_sha256':hashlib.sha256(qpos[selected].tobytes()).hexdigest(),
  'selected_qvel_sha256':hashlib.sha256(qvel[selected].tobytes()).hexdigest()}
 (target/'prefix_identity.json').write_text(json.dumps(note,indent=2,allow_nan=False)+'\n')
 print(json.dumps({k:note[k] for k in ('status','samples','last_physics_time_s','last_phase',
  'screenshot_time_s','screenshot_phase','trajectory_sha256')},indent=2))
finally:
 if scratch is not None:scratch.unlink(missing_ok=True)
