"""Byte-copy a closed native stabilization pilot as immutable progress evidence."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import subprocess
import zipfile
import numpy as np
from PIL import Image

ROOT=Path('/workspace/astra-r2s')
run=ROOT/'outputs/m8_table_supported/stabilization_v1'
render=ROOT/'outputs/m8_table_supported/stabilization_v1_render'
target=ROOT/'media/m8_table_supported/progress_stabilized'
if target.exists():
    raise FileExistsError(target)

def sha(value):
    return hashlib.sha256(value).hexdigest()

files={}
for source,name in [(run/'insertion_trace.npz','trace.npz'),
                    (run/'insertion_validation.json','validation.json'),
                    (run/'supported_scene.zip','supported_scene.zip'),
                    (run/'renderer_source.py','recorded_app_renderer_source.py')]:
    files[name]=source.read_bytes()
for name in ('scene.xml','controller_source.py','scene_source.py','engagement_observer_source.py',
             'left_pad_force_history.npz','table_support_force_history.npz'):
    files[name]=(run/name).read_bytes()
for path in sorted((run/'recorded_sources').rglob('*.py')):
    files[str(path.relative_to(run))]=path.read_bytes()
for name in ('demo.png','demo.mp4','demo.gif','render_manifest.json','renderer_source.py'):
    files[name]=(render/name).read_bytes()
for path in sorted((render/'renderer_sources').rglob('*.py')):
    files[str(path.relative_to(render))]=path.read_bytes()
report=json.loads(files['validation.json'])
manifest=json.loads(files['render_manifest.json'])
with np.load(run/'insertion_trace.npz',allow_pickle=False) as saved:
    rows=json.loads(str(saved['info_json']))
    metadata=json.loads(str(saved['metadata_json']))
    times=saved['time'].copy()
    qpos,qvel=saved['qpos'].copy(),saved['qvel'].copy()
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
assert metadata==report
original_report_bytes=files['validation.json']
converted_paths=[]
def strict_unobserved(value,path=''):
    if isinstance(value,dict):
        return {k:strict_unobserved(v,path+'/'+k) for k,v in value.items()}
    if isinstance(value,list):
        return [strict_unobserved(v,path+'/'+str(i)) for i,v in enumerate(value)]
    if isinstance(value,float) and not math.isfinite(value):
        converted_paths.append(path)
        return None
    return value
strict_report=strict_unobserved(report)
assert converted_paths==[
    '/acceptance_checks/picked_up_free_bolt/sampled_minimum_closed_transport_pad_normals_N/0',
    '/acceptance_checks/picked_up_free_bolt/sampled_minimum_closed_transport_pad_normals_N/1']
files['validation_original.json.txt']=original_report_bytes
files['validation.json']=(json.dumps(strict_report,indent=2,allow_nan=False)+'\n').encode()
files['validation_serialization.json']=(json.dumps({
    'original_report_sha256':sha(original_report_bytes),
    'strict_report_sha256':sha(files['validation.json']),
    'unobserved_value_paths_converted_from_infinity_to_null':converted_paths,
    'original_report_preserved_as':'validation_original.json.txt',
    'trace_preserved_byte_exact':True,
    'scope':'Only unobserved minima of a phase that was not selected are represented as null; no measured force or acceptance result is changed.'
},indent=2,allow_nan=False)+'\n').encode()
assert report['partial'] is True and report['aborted'] is None
assert rows[-1]['phase']=='settle_left_block'
assert rows[-1]['left_stabilization_verified'] is True
assert report['left_acquisition']['table_load_window']['ready'] is True
assert manifest['trajectory_sha256']==sha(files['trace.npz'])
assert manifest['screenshot_sample_index']==len(rows)-1
assert manifest['screenshot_qpos_sha256']==sha(qpos[-1].tobytes())
assert manifest['screenshot_qvel_sha256']==sha(qvel[-1].tobytes())
assert report['model_xml_sha256']==sha(files['scene.xml'])
assert report['scene_source_sha256']==sha(files['scene_source.py'])
for name,value in files.items():
    if name.endswith('.json'):
        json.loads(value,parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
media={}
for name in ('demo.png','demo.gif'):
    with Image.open(render/name) as img:
        for frame in range(getattr(img,'n_frames',1)):
            img.seek(frame)
            img.load()
        media[name]={'sha256':sha(files[name]),'bytes':len(files[name]),
                     'frames':getattr(img,'n_frames',1),'width':img.width,'height':img.height}
media['demo.mp4']={'sha256':sha(files['demo.mp4']),'bytes':len(files['demo.mp4']),
    'fps':12,'frames':manifest['video_frames'],'playback_speed':1,
    'container_decoded':True,'codec':'H264','pixel_format':'yuv420p','faststart':True}
manifest['media_sha256']={name:item['sha256'] for name,item in media.items()}
manifest['closed_media']=media
manifest['scope']='Actual native left stabilization only, ending at 1.65 s before any right-hand pickup or M8 engagement. The original solved 100 ms window confirms the real table bears the block weight while left fingers stabilize it. Later threading and complete policy-training qualification remain pending.'
files['render_manifest.json']=(json.dumps(manifest,indent=2,allow_nan=False)+'\n').encode()
identity={
    'status':'actual_native_stabilization_progress',
    'trajectory_sha256':sha(files['trace.npz']),
    'original_run_directory':str(run.relative_to(ROOT)),
    'first_physics_time_s':float(times[0]),'last_physics_time_s':float(times[-1]),
    'samples':len(rows),'phases':list(dict.fromkeys(row['phase'] for row in rows)),
    'aborted':None,'original_validation_passed':report['passed'],
    'original_validation_partial':report['partial'],
    'validation_scope':'Original validation bytes remain in validation_original.json.txt. Strict validation.json maps only two unobserved right-transport minima from Infinity to null; full-demo checks are not fulfilled by this selected four-phase pilot.',
    'original_report_sha256':sha(original_report_bytes),
    'strict_report_sha256':sha(files['validation.json']),
    'left_acquisition_time_s':report['left_acquisition']['time_s'],
    'table_load_window':report['left_acquisition']['table_load_window'],
    'screenshot_original_sample':rows[-1],
    'model_fingerprint':report['model_fingerprint'],'model_xml_sha256':report['model_xml_sha256'],
    'controller_sha256':report['controller_sha256'],
    'controller_module_sha256':sha(files['controller_source.py']),
    'scene_source_sha256':report['scene_source_sha256'],
    'runtime':report['runtime'],'publication_source_commit':subprocess.check_output(
        ['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
    'publication_source_note':'New supported modules may be pending commit; exact archived source identities are authoritative for this pilot.',
    'scope':manifest['scope'],
    'first_demo_preserved':'media/m8_table_pickup/full is unchanged.'}
files['progress_identity.json']=(json.dumps(identity,indent=2,allow_nan=False)+'\n').encode()
files['package_source.py']=Path(__file__).read_bytes()
files['README.md']=('''# Table-supported M8 stabilization progress\n\nThis is an actual native four-phase stabilization pilot, not a complete threading demo.\nThe free block remains on a plain solid table. Left fingers approach and close from\nthe side. The side bolt remains on its separate rest; the right hand has not picked\nit up and the M8 threads have not engaged.\n\nThe saved 100 ms load window measures 0.989528 N mean upward table force for\n0.998797 N block weight, with only 0.009269 N mean positive upward hand force\nand 100% loaded table substep duty. Original solved forces are recorded before\nintegration; the displayed saved qpos/qvel are immediately postintegration.\nReplay uses mj_forward only, with no integration, interpolation, or manual edits\nto either free part.\n\n- `demo.png`: exact final saved native state at 1.65 s.\n- `demo.gif` and `demo.mp4`: actual pilot at normal 1× playback.\n- `trace.npz`: byte-exact original recording.
- `validation_original.json.txt`: byte-exact original report; it contains two Infinity values for unobserved right-transport pad minima.
- `validation.json` and `validation_serialization.json`: strict report with just those unobserved minima represented as null, plus both hashes and exact conversion paths.\n- `progress_identity.json`: endpoint, original force sample, model/source/runtime IDs.\n- `table_support_force_history.npz`: original native per-substep table/hand force ledger.\n- `render_manifest.json` and `renderer_sources/`: rendering/state identities and sources.\n- `SHA256SUMS`: every primary and supporting file.\n\nThe original validation remains `passed: false`, `partial: true` because this\nselected pilot does not attempt later right pickup, capture, reset, or qualified\nturns. No completed assembly, hardware calibration, or training readiness is claimed.\n\nThe first carried-block demonstration stays preserved at `media/m8_table_pickup/full`.\n''').encode()
target.mkdir(parents=True)
for name,value in files.items():
    path=target/name
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(value)
ledger=''.join(f'{sha(value)}  {name}\n' for name,value in sorted(files.items()))
(target/'SHA256SUMS').write_text(ledger)
for name,value in files.items():
    assert (target/name).read_bytes()==value
print(json.dumps({'target':str(target.relative_to(ROOT)),'files':len(files),
    'total_bytes':sum(map(len,files.values())),'trace_sha256':sha(files['trace.npz']),
    'end_time_s':float(times[-1]),'phase':rows[-1]['phase'],'media':media},indent=2))
