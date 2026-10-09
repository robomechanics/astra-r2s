"""Close-check and strictly serialize an exact normal-speed progress replay."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import subprocess
import sys
import zipfile
import imageio.v2 as imageio
from PIL import Image
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from scripts.audit_m8_insertion_trace import recorded_model, inspected_source_identity

package=Path('media/m8_table_pickup/progress_faster_start')
rendered=Path('outputs/m8_table_pickup/faster_progress_media')
log=Path('outputs/m8_table_pickup/faster_progress_render.log')
ffmpeg='/workspace/.venvs/m8-contact/lib/python3.12/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
render_report=json.loads(log.read_text().strip().splitlines()[-1])
for source,name in ((rendered/'insertion_demo.mp4','demo.mp4'),
    (rendered/'insertion_demo.png','demo.png'),(log,'renderer_output.json'),
    (Path(__file__),'container_check_source.py')):
    shutil.copyfile(source,package/name)
subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-y',
    '-i',str(package/'demo.mp4'),'-vf',
    'fps=6,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
    str(package/'demo.gif')],check=True)
subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-i',str(package/'demo.mp4'),
    '-f','null','-'],check=True)
reader=imageio.get_reader(package/'demo.mp4')
video_metadata=reader.get_meta_data()
frames=reader.count_frames()
reader.close()
assert video_metadata['codec']=='h264' and video_metadata['pix_fmt'].startswith('yuv420p')
assert video_metadata['fps']==12
normalization=[]
def finite_metadata(value,path):
    if isinstance(value,dict):
        return {key:finite_metadata(child,path+'/'+key) for key,child in value.items()}
    if isinstance(value,list):
        return [finite_metadata(child,path+'/'+str(index)) for index,child in enumerate(value)]
    if isinstance(value,float) and not math.isfinite(value):
        normalization.append({'path':path,'diagnostic_original':str(value),'strict_json_value':None})
        return None
    return value
video_metadata=finite_metadata(video_metadata,'/video_metadata')
atoms=[]
raw=(package/'demo.mp4').read_bytes()
offset=0
while offset+8<=len(raw):
    size=int.from_bytes(raw[offset:offset+4],'big')
    name=raw[offset+4:offset+8].decode('ascii')
    if size==1:
        size=int.from_bytes(raw[offset+8:offset+16],'big')
    elif size==0:
        size=len(raw)-offset
    assert size>=8 and offset+size<=len(raw)
    atoms.append({'name':name,'offset':offset,'length':size})
    offset+=size
assert offset==len(raw)
assert next(x['offset'] for x in atoms if x['name']=='moov')<next(x['offset'] for x in atoms if x['name']=='mdat')
with Image.open(package/'demo.gif') as animated:
    gif_frames=animated.n_frames
    gif_duration_ms=0
    gif_size=list(animated.size)
    for index in range(gif_frames):
        animated.seek(index)
        animated.load()
        gif_duration_ms+=animated.info.get('duration',0)
with Image.open(package/'demo.png') as still:
    still.load()
    still_size=list(still.size)
trace=package/'insertion_trace.npz'
with zipfile.ZipFile(trace) as zipped:
    assert zipped.testzip() is None
with np.load(trace,allow_pickle=False) as saved:
    arrays={name:saved[name].copy() for name in saved.files}
metadata=json.loads(str(arrays['metadata_json']))
rows=json.loads(str(arrays['info_json']))
model,model_identity=recorded_model(trace,metadata)
controller_identity=inspected_source_identity(package/'controller_source.py')
assert controller_identity['controller_sha256']==metadata['controller_sha256']
assert arrays['qpos'].shape==(len(rows),model.nq)
assert arrays['qvel'].shape==(len(rows),model.nv)
assert np.isfinite(arrays['qpos']).all() and np.isfinite(arrays['qvel']).all()
trace_sha=hashlib.sha256(trace.read_bytes()).hexdigest()
assert trace_sha==render_report['trajectory_sha256']=='b5f8e812b0fee489d4c8f4ad2865c06e38b30efa0b6fa5be6a11d31dace1eb02'
assert frames==render_report['frames']==229
assert rows[-1]['phase']=='settle_regrip_search_2'
assert not any(row['thread_engaged'] for row in rows)
assert not any(row['phase']=='start_thread_2' for row in rows)
renderer_sha=hashlib.sha256((package/'renderer_source.py').read_bytes()).hexdigest()
assert renderer_sha=='fcf1cb1ce92bfcbe1a91777af1a92ee49fa2bd97a6c5f6225cfb40c03a06d6a9'
still_index=min(len(rows)-1,int(np.searchsorted(arrays['time'],17.1)))
manifest={
    'status':'incomplete_faster_start_progress','passed':False,
    'video':'demo.mp4','gif':'demo.gif','screenshot':'demo.png',
    'frames':frames,'fps':12,'slow_motion':1,'normal_speed_playback':True,
    'trajectory':'insertion_trace.npz','trajectory_sha256':trace_sha,
    'source_end_time_s':float(arrays['time'][-1]),'source_end_phase':rows[-1]['phase'],
    'starting_angular_speed_rad_s':metadata['control_config']['starting_angular_speed_rad_s'],
    'finished_rollout':False,'full_acceptance_claim':False,
    'capture_claim':False,'qualified_lead_claim':False,'formed_thread_support_claim':False,
    'scope':'Genuine table pickups, settled native cone/lead-in contact, first 1 rad/s starting stroke and physical release/opening/reset/regrasp recovery. No second starting stroke is included. Capture remains false; no qualified pitch or full acceptance is claimed.',
    'partial_flag_note':'The recorded metadata partial flag records requested phase selection rather than completion. This is an incomplete prefix.',
    'video_metadata':video_metadata,'video_metadata_normalization':normalization,
    'normalization_note':'Only nonfinite diagnostic imageio container metadata is null. The independently decoded exact finite frame count is the top-level frames value. Every JSON file is serialized with allow_nan=false and checked with parse_constant rejection before publication.',
    'closed_container_and_all_frames_decoded':True,
    'video_top_level_atoms':atoms,'faststart_moov_before_mdat':True,
    'gif_frames':gif_frames,'gif_duration_ms':gif_duration_ms,'gif_fps_filter':6,
    'gif_size_pixels':gif_size,'all_gif_frames_decoded':True,
    'still':{'requested_time_s':17.1,'recorded_time_s':float(arrays['time'][still_index]),
        'recorded_phase':rows[still_index]['phase'],'size_pixels':still_size},
    'model_identity':model_identity,'runtime':metadata['runtime'],
    'controller_identity':controller_identity,'renderer_whole_source_sha256':renderer_sha,
    'method':'Exact native saved qpos/qvel/time samples selected by searchsorted. mj_forward computes display transforms only. No integration, object/state interpolation, pose editing or reconstructed-force claim.',
    'source_private_videos_included':False,
}
(package/'render_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
def reject_constant(value):
    raise ValueError('Invalid nonfinite JSON constant: '+value)
json_files=[]
for path in sorted(package.rglob('*.json')):
    json.loads(path.read_text(),parse_constant=reject_constant)
    json_files.append(str(path.relative_to(package)))
checksums={str(path.relative_to(package)):{'bytes':path.stat().st_size,
    'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    for path in sorted(package.rglob('*')) if path.is_file() and path.name!='checksums.json'}
(package/'checksums.json').write_text(json.dumps(checksums,indent=2,allow_nan=False)+'\n')
json.loads((package/'checksums.json').read_text(),parse_constant=reject_constant)
json_files.append('checksums.json')
for name,identity in checksums.items():
    path=package/name
    assert path.stat().st_size==identity['bytes']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==identity['sha256']
print(json.dumps({'package':str(package),'files_verified':len(checksums),
    'strict_json_files_verified':json_files,'trajectory_sha256':trace_sha,
    'video_frames':frames,'gif_frames':gif_frames,
    'normal_speed':True,'total_bytes_excluding_checksums':sum(x['bytes'] for x in checksums.values()),
    'sizes':{name:checksums[name]['bytes'] for name in ('demo.mp4','demo.gif','demo.png','insertion_trace.npz')}},indent=2,allow_nan=False))
