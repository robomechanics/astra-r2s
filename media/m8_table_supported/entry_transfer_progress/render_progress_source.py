"""Canonical renderer on immutable saved prefix; kinematics/light only."""
from pathlib import Path
import argparse,hashlib,json,math,os,sys
os.environ['MUJOCO_GL']='egl';os.environ['LP_NUM_THREADS']='1'
ROOT=Path('/workspace/astra-r2s');sys.path.insert(0,str(ROOT))
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageSequence
from yam_twin.m8_supported_demo import SupportedRenderer
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine,engine_info
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def geometry_refresh(model,data):
    mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data)
def main(a):
    directory=a.snapshot.resolve();manifest=json.loads((directory/'progress_snapshot.json').read_text());source=directory/'insertion_trace_partial.npz'
    assert sha(source)==manifest['snapshot_trajectory_sha256']
    for n,d in manifest['source_hashes_before'].items():assert sha(ROOT/n)==d and sha(directory/'producer_sources'/n)==d,n
    before_files={p:p.read_bytes() for p in directory.rglob('*') if p.is_file()}
    require_micron_engine();runtime_before=engine_info();assert runtime_before==manifest['runtime']
    with np.load(source,allow_pickle=False) as z:
        times=z['time'].copy();qpos=z['qpos'].copy();qvel=z['qvel'].copy();samples=json.loads(str(z['info_json']));metadata=json.loads(str(z['metadata_json']))
    model,identity=recorded_model(source,metadata);data=mujoco.MjData(model);renderer=SupportedRenderer(model)
    output=directory/'entry_transfer_progress.mp4';assert not output.exists();fps=12
    indices=[min(len(times)-1,int(np.searchsorted(times,times[0]+i/fps))) for i in range(math.ceil(float(times[-1]-times[0])*fps))]
    indices[-1]=len(times)-1
    writer=imageio.get_writer(output,fps=fps,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-movflags','+faststart','-pix_fmt','yuv420p'])
    def canvas(i):
        data.qpos[:]=qpos[i];data.qvel[:]=qvel[i];data.time=float(times[i]);geometry_refresh(model,data)
        im=renderer.frame(data,samples[i],metadata,slow_motion=1)
        draw=ImageDraw.Draw(im);draw.rectangle((0,96,610,124),fill='#101924')
        draw.text((20,100),'PHASE-ONLY PROGRESS · no capture / full qualification yet',font=renderer.small,fill='#f1c46c')
        return im
    try:
        for i in indices:writer.append_data(np.asarray(canvas(i)))
        canvas(len(times)-1).save(output.with_suffix('.png'))
    finally:writer.close();renderer.close()
    detail_cameras=[{'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':100.,'elevation_deg':-30.},
        {'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':130.,'elevation_deg':-5.}]
    detail=mujoco.Renderer(model,width=640,height=640)
    try:
        geometry_refresh(model,data);im=Image.new('RGB',(1280,808),'#101924')
        for k,v in enumerate(detail_cameras):
            cam=mujoco.MjvCamera();cam.lookat[:]=v['lookat_m'];cam.distance=v['distance_m'];cam.azimuth=v['azimuth_deg'];cam.elevation=v['elevation_deg']
            detail.update_scene(data,camera=cam);im.paste(Image.fromarray(detail.render()),(640*k,104))
        draw=ImageDraw.Draw(im);font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        from PIL import ImageFont
        big=ImageFont.truetype(font_path,25);small=ImageFont.truetype(font_path,18)
        draw.rectangle((0,0,1280,104),fill='#101924');draw.rectangle((0,722,1280,808),fill='#101924')
        draw.text((16,10),'Fresh native pickup → entry → measured weight transfer',font=big,fill='white')
        draw.text((16,43),f'{times[-1]:.5f} s actual saved time · PHASE-ONLY PROGRESS · formed 0 / interior 0',font=small,fill='#f1c46c')
        draw.text((16,71),'Exact original post-state · geometry refresh only · no collision / force solve / integration',font=small,fill='#c5d6e4')
        s=samples[-1];w=s['weight_window'];left=s['left_contact']['pad_normal_force_N'];right=s['contact']['pad_normal_force_N']
        draw.text((16,732),f'Original solve: table {s["table_support"]["table_upward_force_N"]:.6f} N · left pads {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N',font=small,fill='#c5d6e4')
        draw.text((16,762),f'Original {w["observed_window_s"]*1000:.2f} ms window: thread {w["mean_thread_weight_fraction"]*100:.6f}% mg · positive hand {w["mean_positive_hand_upward_weight_fraction"]*100:.6f}% · capture unqualified',font=small,fill='#c5d6e4')
        im.save(directory/'entry_transfer_detail.png')
    finally:detail.close()
    gif=[];video=imageio.get_reader(output)
    for im in video:gif.append(Image.fromarray(im).resize((1080,600)).convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
    video.close();duration=[10*(round((i+1)*100/fps)-round(i*100/fps)) for i in range(len(gif))]
    gif_path=output.with_suffix('.gif');gif[0].save(gif_path,save_all=True,append_images=gif[1:],duration=duration,loop=0,optimize=False,disposal=2)
    with Image.open(gif_path) as im:gif_duration=[x.info['duration'] for x in ImageSequence.Iterator(im)]
    for p,raw in before_files.items():assert p.read_bytes()==raw,p
    for n,d in manifest['source_hashes_before'].items():assert sha(ROOT/n)==d,n
    runtime_after=engine_info();assert runtime_before==runtime_after
    result={'scope':manifest['scope'],'producer_commit':manifest['producer_commit'],'source_recording':str(source),'trajectory_sha256':sha(source),
        'render_helper_sha256':sha(__file__),'archived_model_identity':identity,'runtime_before':runtime_before,'runtime_after':runtime_after,
        '65_producer_source_hashes_matched_before_after_render':True,'original_snapshot_files_unchanged_after_render':True,
        'renderer_module_sha256':sha(ROOT/'yam_twin/m8_supported_demo.py'),'base_renderer_sha256':sha(ROOT/'yam_twin/m8_insertion_demo.py'),
        'canonical_renderer_cameras_and_view_options_unchanged':True,'canonical_geomgroup_filter_scope':'Original canonical geomgroup[3:] filter only, with no additional object/body hiding. Supplemental detail uses default visible geometry options and real cameras.',
        'additional_physical_body_hiding':False,'geometry_changed':False,'refresh':['mj_kinematics','mj_comPos','mj_camlight'],
        'mj_forward_called':False,'collision_discovery_called':False,'dynamics_or_contact_force_solve_called':False,'integration_called':False,'pose_interpolation_called':False,
        'force_scope':'Original preintegration solve at saved time−50us; original observer window copied verbatim. No forces inferred from saved qpos or recomputed for replay.',
        'fps':fps,'slow_motion':1,'native_first_time_s':float(times[0]),'native_last_time_s':float(times[-1]),'native_elapsed_between_saved_states_s':float(times[-1]-times[0]),
        'frames':len(indices),'encoded_video_duration_s':len(indices)/fps,'gif_frames':len(gif_duration),'gif_duration_ms':sum(gif_duration),
        'frame_time_quantization_scope':'12fps ceiling; original exact prefix endpoint occupies last display slot. No state interpolation, retiming of original timestamps or splice.',
        'frame_sample_indices':indices,'frame_saved_times_s':[float(times[i]) for i in indices],
        'screenshot_saved_sample_index':len(times)-1,'screenshot_saved_time_s':float(times[-1]),'screenshot_original_phase':samples[-1]['phase'],
        'screenshot_qpos_sha256':hashlib.sha256(qpos[-1].tobytes()).hexdigest(),'screenshot_qvel_sha256':hashlib.sha256(qvel[-1].tobytes()).hexdigest(),
        'original_saved_sample':samples[-1],'supplemental_detail_cameras':detail_cameras,'supplemental_detail_is_same_saved_state':True,
        'full_raw_ledgers_closed':False,'full_qualification_audited':False,
        'media':{name:{'sha256':sha(directory/name),'bytes':(directory/name).stat().st_size} for name in ('entry_transfer_progress.mp4','entry_transfer_progress.gif','entry_transfer_progress.png','entry_transfer_detail.png')}}
    (directory/'render_manifest.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (directory/'render_progress_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({'frames':len(indices),'native_last_time_s':float(times[-1]),'encoded_duration_s':len(indices)/fps,'gif_duration_ms':sum(gif_duration),'media':result['media']},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--snapshot',type=Path,required=True);main(p.parse_args())
