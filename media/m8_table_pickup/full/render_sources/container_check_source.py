"""Validate exact normal-speed final replay while retaining the failed gate."""
from pathlib import Path
import hashlib, json, math, shutil, subprocess, sys, zipfile
import imageio.v2 as imageio
from PIL import Image
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from scripts.audit_m8_insertion_trace import recorded_model, inspected_source_identity

directory=Path('outputs/m8_table_pickup/final_faster_media')
run=Path('outputs/m8_table_pickup/full_faster_entry')
trace=run/'insertion_trace.npz'
report_path=run/'insertion_validation.json'
log=Path('outputs/m8_table_pickup/final_faster_render.log')
ffmpeg='/workspace/.venvs/m8-contact/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
render_report=json.loads(log.read_text().strip().splitlines()[-1])
shutil.copyfile(log,directory/'renderer_output.json')
shutil.copyfile(__file__,directory/'container_check_source.py')
subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-y',
    '-i',str(directory/'insertion_demo.mp4'),'-vf',
    'fps=5,scale=700:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
    str(directory/'insertion_demo.gif')],check=True)
subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-i',str(directory/'insertion_demo.mp4'),
    '-f','null','-'],check=True)
reader=imageio.get_reader(directory/'insertion_demo.mp4')
video_metadata=reader.get_meta_data()
frames=reader.count_frames()
reader.close()
assert video_metadata['codec']=='h264' and video_metadata['pix_fmt'].startswith('yuv420p')
assert video_metadata['fps']==12
normalization=[]
def finite_container_metadata(value,path):
    if isinstance(value,dict):
        return {key:finite_container_metadata(child,path+'/'+key) for key,child in value.items()}
    if isinstance(value,list):
        return [finite_container_metadata(child,path+'/'+str(index)) for index,child in enumerate(value)]
    if isinstance(value,float) and not math.isfinite(value):
        normalization.append({'path':path,'diagnostic_original':str(value),'strict_json_value':None})
        return None
    return value
video_metadata=finite_container_metadata(video_metadata,'/video_metadata')
raw=(directory/'insertion_demo.mp4').read_bytes()
atoms=[]
offset=0
while offset+8<=len(raw):
    length=int.from_bytes(raw[offset:offset+4],'big')
    name=raw[offset+4:offset+8].decode('ascii')
    if length==1:
        length=int.from_bytes(raw[offset+8:offset+16],'big')
    elif length==0:
        length=len(raw)-offset
    assert length>=8 and offset+length<=len(raw)
    atoms.append({'name':name,'offset':offset,'length':length})
    offset+=length
assert offset==len(raw)
assert next(x['offset'] for x in atoms if x['name']=='moov')<next(x['offset'] for x in atoms if x['name']=='mdat')
with Image.open(directory/'insertion_demo.gif') as animated:
    gif_frames=animated.n_frames
    gif_size=list(animated.size)
    gif_duration_ms=0
    for index in range(gif_frames):
        animated.seek(index)
        animated.load()
        gif_duration_ms+=animated.info.get('duration',0)
with Image.open(directory/'insertion_demo.png') as still:
    still.load()
    still_size=list(still.size)
with zipfile.ZipFile(trace) as zipped:
    assert zipped.testzip() is None
with np.load(trace,allow_pickle=False) as saved:
    arrays={name:saved[name].copy() for name in saved.files}
report=json.loads(str(arrays['metadata_json']))
assert report==json.loads(report_path.read_text())
rows=json.loads(str(arrays['info_json']))
model,model_identity=recorded_model(trace,report)
controller_identity=inspected_source_identity(run/'controller_source.py')
assert controller_identity['controller_sha256']==report['controller_sha256']
assert arrays['qpos'].shape==(len(rows),model.nq)
assert arrays['qvel'].shape==(len(rows),model.nv)
assert np.isfinite(arrays['qpos']).all() and np.isfinite(arrays['qvel']).all()
trace_sha=hashlib.sha256(trace.read_bytes()).hexdigest()
report_sha=hashlib.sha256(report_path.read_bytes()).hexdigest()
assert trace_sha==render_report['trajectory_sha256']=='c25daaf438f36c8b1617c989c8441b0414305a1c90648b397fb096cab484b9db'
assert report_sha=='c2647298f59e7b8d832ac140df243a26d4d403ac62e0de2353ab0760da693a3d'
assert frames==render_report['frames']==524
assert report['passed'] is False and report['partial'] is False and report['aborted'] is None
assert len(report['phases'])==49 and rows[-1]['phase']=='stop_2'
failed_checks=[name for name,value in report['acceptance_checks'].items() if not value['passed']]
assert failed_checks==['all_substep_left_pad_contact_retention']
turns=[phase for phase in report['phases'] if phase['phase'].startswith('turn_')]
assert [phase['phase'] for phase in turns]==['turn_1','turn_2']
assert all(phase['started_engaged'] and phase['ended_engaged'] for phase in turns)
chart=json.loads((directory/'insertion_trajectory.json').read_text())
assert chart['source_trace_sha256']==trace_sha and chart['source_report_sha256']==report_sha
assert chart['lead_qualified_turn_segments']==['turn_1','turn_2']
assert chart['capture_tag_time_s']==report['acceptance_checks']['started_previously_separate_threads']['engagement_time_s']
renderer_sha=hashlib.sha256((directory/'renderer_source.py').read_bytes()).hexdigest()
assert renderer_sha=='fcf1cb1ce92bfcbe1a91777af1a92ee49fa2bd97a6c5f6225cfb40c03a06d6a9'
manifest={
    'video':'insertion_demo.mp4','screenshot':'insertion_demo.png','gif':'insertion_demo.gif',
    'frames':frames,'fps':12,'slow_motion':1,'normal_speed_playback':True,
    'source_recording':str(trace),'trajectory_sha256':trace_sha,
    'source_report':str(report_path),'source_report_sha256':report_sha,
    'status':'complete_native_sequence_with_failed_strict_preload_gate',
    'passed':False,'recorded_validation_passed':False,'partial_task_scope':False,
    'native_sequence_completed':True,'aborted':None,'executed_phases':49,
    'source_end_time_s':float(arrays['time'][-1]),'source_end_phase':rows[-1]['phase'],
    'acceptance_checks_passed':22,'acceptance_checks_total':23,
    'failed_acceptance_checks':failed_checks,
    'failed_strict_left_pad_preload_check':report['acceptance_checks'][failed_checks[0]],
    'capture_tag_time_s':chart['capture_tag_time_s'],
    'qualified_turn_phases':[phase['phase'] for phase in turns],
    'qualified_turn_insertion_advance_m':sum(phase['bolt_insertion_advance_m'] for phase in turns),
    'qualified_turn_clockwise_rotation_rad':sum(phase['bolt_clockwise_rotation_rad'] for phase in turns),
    'qualified_turn_observed_helix_residuals_m':[phase['observed_helix_residual_m'] for phase in turns],
    'scope':'The actual complete native sequence contains both table pickups, settled entry, three starting strokes, formed-flank observer capture, physical opening/reset/regrasp and two qualifying half-turns. Overall validation remains false because the strict left-pad preload gate observed 9/6 isolated 50-microsecond unloaded substeps. The gate, forces and original report are preserved.',
    'qualification_scope':'This manifest verifies recorded replay identities and closed renders. Independent capture, reset, raw-force and free-body audits accompany the canonical package separately. No hardware calibration, full bolt seating or trained-policy claim is made.',
    'chart_metadata':'insertion_trajectory.json',
    'chart_pitch_reference_scope':'Only turn_1 and turn_2, after the recorded formed-flank capture; no starting/cone travel receives a pitch reference.',
    'video_metadata':video_metadata,'video_metadata_normalization':normalization,
    'normalization_note':'Nonfinite diagnostic imageio container estimates are null. The independently decoded finite frame count is frames. All JSON is serialized with allow_nan=false and parse_constant rejection.',
    'video_top_level_atoms':atoms,'faststart_moov_before_mdat':True,
    'closed_container_and_all_frames_decoded':True,
    'gif_frames':gif_frames,'gif_size_pixels':gif_size,'gif_fps_filter':5,
    'gif_duration_ms':gif_duration_ms,'all_gif_frames_decoded':True,
    'still':{'requested_time_s':43.6361000008868,'recorded_time_s':float(arrays['time'][-1]),
        'recorded_phase':rows[-1]['phase'],'size_pixels':still_size},
    'model_identity':model_identity,'runtime':report['runtime'],
    'controller_archived_identity':controller_identity,
    'controller_whole_source_sha256':hashlib.sha256((run/'controller_source.py').read_bytes()).hexdigest(),
    'scene_source_sha256':report['scene_source_sha256'],
    'observer_source_sha256':report['engagement_observer_source_sha256'],
    'renderer_whole_source_sha256':renderer_sha,
    'renderer_source_identity_note':'Exact module bytes frozen at this renderer process start as renderer_source.py. Later CLI default edits are not loaded by the running renderer and are not used as its source identity.',
    'method':'Native recorded qpos/qvel/time samples selected by searchsorted. mj_forward computes display transforms only. No dynamics integration, state/object interpolation, pose editing or reconstructed-force claim.',
    'source_private_videos_included':False,
}
(directory/'render_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
def reject_constant(value):
    raise ValueError('Invalid nonfinite JSON constant: '+value)
strict_json_files=[]
for path in sorted(directory.rglob('*.json')):
    json.loads(path.read_text(),parse_constant=reject_constant)
    strict_json_files.append(str(path.relative_to(directory)))
checksums={str(path.relative_to(directory)):{'bytes':path.stat().st_size,
    'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    for path in sorted(directory.rglob('*')) if path.is_file() and path.name!='checksums.json'}
(directory/'checksums.json').write_text(json.dumps(checksums,indent=2,allow_nan=False)+'\n')
json.loads((directory/'checksums.json').read_text(),parse_constant=reject_constant)
strict_json_files.append('checksums.json')
for name,identity in checksums.items():
    path=directory/name
    assert path.stat().st_size==identity['bytes']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==identity['sha256']
print(json.dumps({'directory':str(directory),'trace_sha256':trace_sha,'report_sha256':report_sha,
    'frames':frames,'gif_frames':gif_frames,'normal_speed':True,
    'recorded_validation_passed':False,'native_sequence_completed':True,
    'files_verified':len(checksums),'strict_json_files_verified':strict_json_files,
    'sizes':{name:checksums[name]['bytes'] for name in ('insertion_demo.mp4','insertion_demo.gif','insertion_demo.png')},
    'total_bytes_excluding_checksums':sum(x['bytes'] for x in checksums.values())},indent=2,allow_nan=False))
