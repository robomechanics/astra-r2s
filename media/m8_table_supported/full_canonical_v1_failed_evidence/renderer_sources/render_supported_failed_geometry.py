"""Exact CLOSED failed native trial, canonical renderer; geometry refresh only."""
from pathlib import Path
import argparse,hashlib,json,math,os,sys
os.environ['MUJOCO_GL']='egl';os.environ['LP_NUM_THREADS']='1'
ROOT=Path('/workspace/astra-r2s');sys.path.insert(0,str(ROOT))
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont,ImageSequence
from yam_twin.m8_supported_demo import SupportedRenderer
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine,engine_info
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def geometry_refresh(model,data):
    mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data)
def main(a):
    run=a.run.resolve();output=a.output.resolve();assert not output.exists()
    report=json.loads((run/'insertion_validation.json').read_text());identity=json.loads((run/'run_publication_identity.json').read_text());after=json.loads((run/'run_publication_identity_after.json').read_text())
    assert not report['passed'] and report['aborted']['phase']=='stop_reverse_seat_1'
    source_hashes=identity['source_hashes_before'];assert len(source_hashes)==65
    assert after.get('source_hashes_after',after.get('source_hashes_before'))==source_hashes
    for n,d in source_hashes.items():assert sha(ROOT/n)==d,n
    originals={p:p.read_bytes() for p in run.rglob('*') if p.is_file()}
    source=run/'insertion_trace.npz';trace_sha=sha(source)
    require_micron_engine();runtime_before=engine_info();assert runtime_before==report['runtime']
    with np.load(source,allow_pickle=False) as z:
        times=z['time'].copy();qpos=z['qpos'].copy();qvel=z['qvel'].copy();samples=json.loads(str(z['info_json']));metadata=json.loads(str(z['metadata_json']))
    assert metadata==report and len(times)==len(samples)==len(qpos)==len(qvel)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    model,model_identity=recorded_model(source,metadata);data=mujoco.MjData(model);renderer=SupportedRenderer(model)
    output.mkdir(parents=True);fps=12
    indices=[min(len(times)-1,int(np.searchsorted(times,times[0]+i/fps))) for i in range(math.ceil(float(times[-1]-times[0])*fps))];indices[-1]=len(times)-1
    writer=imageio.get_writer(output/'demo.mp4',fps=fps,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-movflags','+faststart','-pix_fmt','yuv420p'])
    def state(i):
        data.qpos[:]=qpos[i];data.qvel[:]=qvel[i];data.time=float(times[i]);geometry_refresh(model,data)
    def canvas(i):
        state(i);im=renderer.frame(data,samples[i],metadata,slow_motion=1)
        d=ImageDraw.Draw(im);d.rectangle((0,96,610,124),fill='#101924');d.text((20,100),'CLOSED FAILED ATTEMPT · no formed capture / full completion',font=renderer.small,fill='#f1c46c')
        return im
    stills={'demo.png':len(times)-1}
    for phase,name in [('transfer_bolt_weight','search_start.png'),('reverse_seat_1','reverse_search_endpoint.png'),('stop_reverse_seat_1','stop_search_endpoint.png')]:
        ids=[i for i,s in enumerate(samples) if s['phase']==phase]
        if ids:stills[name]=ids[-1]
    try:
        for i in indices:writer.append_data(np.asarray(canvas(i)))
        for name,i in stills.items():canvas(i).save(output/name)
    finally:writer.close();renderer.close()
    cameras=[{'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':100.,'elevation_deg':-30.},
        {'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':130.,'elevation_deg':-5.}]
    detail=mujoco.Renderer(model,width=640,height=640);font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';big=ImageFont.truetype(font,25);small=ImageFont.truetype(font,18)
    def detail_canvas(i):
        state(i);im=Image.new('RGB',(1280,808),'#101924')
        for k,v in enumerate(cameras):
            cam=mujoco.MjvCamera();cam.lookat[:]=v['lookat_m'];cam.distance=v['distance_m'];cam.azimuth=v['azimuth_deg'];cam.elevation=v['elevation_deg']
            detail.update_scene(data,camera=cam);im.paste(Image.fromarray(detail.render()),(640*k,104))
        d=ImageDraw.Draw(im);d.rectangle((0,0,1280,104),fill='#101924');d.rectangle((0,722,1280,808),fill='#101924');s=samples[i]
        d.text((16,10),'Fresh native attempt · CLOSED FAILED direction gate',font=big,fill='white')
        d.text((16,43),f'{times[i]:.5f} s actual saved state · {s["phase"]} · formed {s["formed_flank_overlap_m"]*1e6:.3f} µm',font=small,fill='#f1c46c')
        d.text((16,71),'Original state/forces preserved · no capture / full pitch / qualified reset / full completion',font=small,fill='#c5d6e4')
        left=s['left_contact']['pad_normal_force_N'];right=s['contact']['pad_normal_force_N']
        d.text((16,732),f'Original solve at t−50µs: table {s["table_support"]["table_upward_force_N"]:.6f} N · left {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N',font=small,fill='#c5d6e4')
        event=s.get('seat_direction_event')
        if event:d.text((16,762),f'Original measured drop {event["actual_axial_drop_m"]*1e6:.4f} µm / required {event["minimum_actual_drop_m"]*1e6:g} µm · event {event["confirmed_search_direction_event"]} · geometry refresh only',font=small,fill='#c5d6e4')
        else:d.text((16,762),'Original measured support before direction search · geometry refresh only, no collision/force solve',font=small,fill='#c5d6e4')
        return im
    details={'endpoint_detail.png':len(times)-1,'search_start_detail.png':stills['search_start.png'],'reverse_search_detail.png':stills['reverse_search_endpoint.png']}
    try:
        for name,i in details.items():detail_canvas(i).save(output/name)
    finally:detail.close()
    gif=[];reader=imageio.get_reader(output/'demo.mp4')
    for im in reader:gif.append(Image.fromarray(im).resize((1080,600)).convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
    reader.close();duration=[10*(round((i+1)*100/fps)-round(i*100/fps)) for i in range(len(gif))]
    gif[0].save(output/'demo.gif',save_all=True,append_images=gif[1:],duration=duration,loop=0,optimize=False,disposal=2)
    with Image.open(output/'demo.gif') as im:gif_duration=[x.info['duration'] for x in ImageSequence.Iterator(im)]
    for p,raw in originals.items():assert p.read_bytes()==raw,p
    for n,d in source_hashes.items():assert sha(ROOT/n)==d,n
    runtime_after=engine_info();assert runtime_after==runtime_before
    media_names=['demo.mp4','demo.gif',*stills,*details]
    result={'scope':'Original CLOSED failed whole fresh native attempt. Pickup/align/entry/weight transfer/reverse search/stopped endpoint executed. Direction gate failed measured19.1214um<50um; formed0/interior0 and no capture/reset/full completion. Canonical fixed-camera saved-state replay uses ONLY mj_kinematics/comPos/camlight; no mj_forward/collision/force solve/integration/interpolation/object posing/splice. Original failure/acceptance flags retained.',
        'trajectory_sha256':trace_sha,'source_recording':str(source),'producer_commit':identity['producer_commit'],
        'original_report_sha256':sha(run/'insertion_validation.json'),'original_report_passed':False,'original_report_partial':report['partial'],'original_abort':report['aborted'],
        'fps':fps,'slow_motion':1,'frames':len(indices),'native_first_time_s':float(times[0]),'native_last_time_s':float(times[-1]),
        'encoded_video_duration_s':len(indices)/fps,'gif_frames':len(gif_duration),'gif_duration_ms':sum(gif_duration),
        'frame_sample_indices':indices,'frame_saved_times_s':[float(times[i]) for i in indices],
        'screenshot_sample_index':len(times)-1,'screenshot_qpos_sha256':hashlib.sha256(qpos[-1].tobytes()).hexdigest(),'screenshot_qvel_sha256':hashlib.sha256(qvel[-1].tobytes()).hexdigest(),'original_saved_sample':samples[-1],
        'frame_time_quantization_scope':'Normal1x12fps ceiling; exact endpoint in last slot. Native timestamps/states remain unchanged; canonical1xslowmotion footer means factor1 normalplayback.',
        'force_state_timing':'Original preintegration force solve/retained kinematics at t−dt (50us); saved qpos/qvel and explicit aperture at postintegrationt. Original motor command calibration is prior solve t−2dt, with initialized first-command exception. No force recomputation.',
        'refresh':['mj_kinematics','mj_comPos','mj_camlight'],'mj_forward_called':False,'collision_discovery_called':False,'dynamics_or_contact_force_solve_called':False,'physics_integration':False,'state_interpolation':False,
        'canonical_fixed_cameras_and_view_options_unchanged':True,'geomgroup_filter_scope':'Original canonical geomgroup[3:] filter; no additional object/body hiding. Real detail cameras/default visible geometry are separate.',
        'physical_geometry_changed_or_body_hidden':False,'detail_cameras':cameras,
        'stills':{n:{'sample_index':i,'time_s':float(times[i]),'phase':samples[i]['phase'],'qpos_sha256':hashlib.sha256(qpos[i].tobytes()).hexdigest(),'qvel_sha256':hashlib.sha256(qvel[i].tobytes()).hexdigest(),'original_sample':samples[i]} for n,i in {**stills,**details}.items()},
        'archived_model_identity':model_identity,'runtime_before':runtime_before,'runtime_after':runtime_after,
        'producer_source_hashes_before_after_render':source_hashes,'65_sources_unchanged_before_after':True,
        'original_closed_run_files_unchanged_after_render':True,'original_closed_run_file_sha256':{str(p.relative_to(run)):hashlib.sha256(raw).hexdigest() for p,raw in originals.items()},
        'render_helper_sha256':sha(__file__),'media_sha256':{n:sha(output/n) for n in media_names},'media_bytes':{n:(output/n).stat().st_size for n in media_names}}
    (output/'render_manifest.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (output/'renderer_sources').mkdir();(output/'renderer_sources'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    for n in ('yam_twin/m8_supported_demo.py','yam_twin/m8_insertion_demo.py','yam_twin/m8_demo.py','scripts/audit_m8_insertion_trace.py','thread_lab/runtime.py'):
        p=output/'renderer_sources'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes((ROOT/n).read_bytes())
    print(json.dumps({'native_last_time_s':float(times[-1]),'frames':len(indices),'encoded_duration_s':len(indices)/fps,'gif_duration_ms':sum(gif_duration),'stills':{n:v['time_s'] for n,v in result['stills'].items()},'media_sha256':result['media_sha256']},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);main(p.parse_args())
