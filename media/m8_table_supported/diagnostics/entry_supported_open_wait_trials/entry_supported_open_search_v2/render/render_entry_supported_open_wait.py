"""Exact saved-state media for closed entry-supported opening/wait diagnostics."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import sys

ROOT=Path('/workspace/astra-r2s')
sys.path.insert(0,str(ROOT))
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('MESA_SHADER_CACHE_DIR','/workspace/.cache/mesa')
os.environ.setdefault('LP_NUM_THREADS','2')
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont,ImageSequence
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine,engine_info

TRIALS={'entry_supported_open_search_v1':('Open wait v1','Strict quiet-window timeout · no reset / regrasp'),
'entry_supported_open_search_v2':('Open wait v2','Earlier strict readiness · fixed endpoint missed · no reset')}
PARENT=ROOT/'media/m8_table_supported/face120_pickup_entry_v1/insertion_trace.npz'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(name,output,fps=40):
    require_micron_engine()
    directory=ROOT/'outputs/m8_table_supported/diagnostics'/name
    trace=directory/'checkpoint_trace.npz'
    declaration=json.loads((directory/'declaration.json').read_text())
    report=json.loads((directory/'report.json').read_text())
    assert report['trace_sha256']==sha(trace)
    assert declaration['parent_trace_sha256']==sha(PARENT)
    source_paths=[Path(__file__),ROOT/'scripts/audit_m8_insertion_trace.py',ROOT/'thread_lab/runtime.py']
    source_bytes={p:p.read_bytes() for p in source_paths}
    with np.load(PARENT,allow_pickle=False) as saved:
        parent_metadata=json.loads(str(saved['metadata_json']))
    model,identity=recorded_model(PARENT,parent_metadata)
    assert identity['model_xml_sha256']==declaration['parent_model']['model_xml_sha256']==sha(directory/'scene.xml')
    with np.load(trace,allow_pickle=False) as saved:
        qpos=saved['qpos'].copy();qvel=saved['qvel'].copy()
        samples=json.loads(str(saved['info_json']))
        assert json.loads(str(saved['metadata_json']))==declaration
    assert len(samples)==len(qpos)==len(qvel)
    assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
    times=np.asarray([s['time_s'] for s in samples])
    elapsed=np.asarray([s['elapsed_s'] for s in samples])
    assert np.all(np.diff(times)>=0) and np.all(np.diff(elapsed)>=0)
    open_audit=json.loads((directory/'independent_open_search_audit.json').read_text())
    assert open_audit['original_trace_sha256']==sha(trace)
    missing_tail=float(report['actual_duration_s']-elapsed[-1])
    assert abs(missing_tail-open_audit['timing']['saved_state_tail_gap_s'])<1e-12
    data=mujoco.MjData(model)
    cameras=[
      {'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':100.,'elevation_deg':-30.,'label':'Actual jaw gap / head / thread shaft'},
      {'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':130.,'elevation_deg':-5.,'label':'Same native state / low real view'}]
    renderer=mujoco.Renderer(model,width=640,height=640)
    font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big=ImageFont.truetype(font,25);small=ImageFont.truetype(font,19)
    title,outcome=TRIALS[name]
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    def frame(index):
        data.qpos[:]=qpos[index];data.qvel[:]=qvel[index];data.time=float(times[index])
        mujoco.mj_forward(model,data)
        canvas=Image.new('RGB',(1280,808),'#101924')
        for column,view in enumerate(cameras):
            camera=mujoco.MjvCamera();camera.lookat[:]=view['lookat_m'];camera.distance=view['distance_m']
            camera.azimuth=view['azimuth_deg'];camera.elevation=view['elevation_deg']
            renderer.update_scene(data,camera=camera)
            canvas.paste(Image.fromarray(renderer.render()),(column*640,104))
        sample=samples[index];draw=ImageDraw.Draw(canvas)
        draw.rectangle((0,0,1280,138),fill='#101924')
        draw.text((16,10),f'COLD entry-supported SEARCH · {title} · capture unqualified',font=big,fill='white')
        draw.text((16,43),f'{elapsed[index]:.5f} s local native time · {sample["physical_phase"]} · normal 1×',font=small,fill='#c5d6e4')
        draw.text((16,71),outcome+(f' · saved tail {missing_tail*1000:.2f} ms absent' if missing_tail>1e-8 else ''),font=small,fill='#f1c46c')
        for column,view in enumerate(cameras):
            draw.text((column*640+12,111),view['label'],font=small,fill='#c5d6e4')
        draw.rectangle((0,722,1280,808),fill='#101924')
        left=sample['left_pad_normal_force_N'];right=sample['right_pad_normal_force_N']
        draw.text((16,732),f'Original solve: table {sample["table_upward_force_N"]:.3f} N · left pads {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N',font=small,fill='#c5d6e4')
        draw.text((16,762),f'Whole right/bolt contacts {sample["right_bolt_contact_count"]} · hand load {sample["hand_gravity_opposing_force_N"]:.4f} N · fully open {sample["fully_open_unassisted"]} · exact saved state',font=small,fill='#c5d6e4')
        return canvas
    indices=[min(len(times)-1,int(np.searchsorted(elapsed,elapsed[0]+i/fps)))
             for i in range(max(1,math.ceil(float(elapsed[-1])*fps)))]
    writer=imageio.get_writer(output/'demo.mp4',fps=fps,codec='libx264',quality=8,
        macro_block_size=1,ffmpeg_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
    try:
        for index in indices:writer.append_data(np.asarray(frame(index)))
        frame(0).save(output/'entry_detail.png')
        frame(len(times)-1).save(output/'open_jaw_gap.png')
        frame(len(times)-1).save(output/'endpoint_detail.png')
    finally:
        writer.close();renderer.close()
    frames=[]
    reader=imageio.get_reader(output/'demo.mp4')
    for image in reader:
        image=Image.fromarray(image).resize((960,606))
        frames.append(image.convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
    reader.close()
    durations=[10*(round((i+1)*100/fps)-round(i*100/fps)) for i in range(len(frames))]
    frames[0].save(output/'demo.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0,optimize=False,disposal=2)
    with Image.open(output/'demo.gif') as gif:
        encoded_durations=[f.info['duration'] for f in ImageSequence.Iterator(gif)]
    states={label:{'sample_index':index,'state_time_s':float(times[index]),
        'elapsed_native_time_s':float(elapsed[index]),'phase':samples[index]['physical_phase'],
        'qpos_sha256':hashlib.sha256(qpos[index].tobytes()).hexdigest(),
        'qvel_sha256':hashlib.sha256(qvel[index].tobytes()).hexdigest(),
        'original_sample':samples[index]} for label,index in [('entry_detail',0),('endpoint_detail',len(times)-1),('open_jaw_gap',len(times)-1)]}
    manifest={
        'scope':'Separate closed cold entry-supported SEARCH opening/wait replay only. Original strict fixed-endpoint readiness failure remains failed. No qualified full-flank capture, reset/regrasp/full-pitch lead or continuous trajectory. V1 absent saved-state tail remains absent, with no fabricated endpoint.',
        'trial':name,'parent_canonical_trace_sha256':sha(PARENT),
        'state_parent_trace_sha256':declaration.get('state_parent_trace_sha256',declaration['parent_trace_sha256']),
        'trace_sha256':sha(trace),'declaration_sha256':sha(directory/'declaration.json'),
        'original_report_sha256':sha(directory/'report.json'),
        'all_original_guards_held':report['all_original_guards_held'],'original_abort':report['aborted'],
        'original_termination':report['physical_closed_turn'].get('termination'),
        'exact_saved_state_replay':True,'physics_integration':False,'state_interpolation':False,
        'physical_bodies_hidden_or_geometry_changed':False,
        'original_force_scope':'Original recorded native solve samples only; solve geometry/forces time_s-dt versus postintegration saved qpos/qvel. No replay force values reported.',
        'native_timestep_s':float(model.opt.timestep),'native_duration_s':report['actual_duration_s'],
        'actual_last_saved_elapsed_s':float(elapsed[-1]),'missing_saved_state_tail_s':missing_tail,
        'missing_state_tail_synthesized_or_integrated':False,
        'last_integrated_vs_precommand_abort_timing':open_audit['timing'],
        'strict_readiness_history':open_audit['strict_open_ready_original_arithmetic'],
        'recorded_saved_state_rows':len(times),'first_state_time_s':float(times[0]),'last_state_time_s':float(times[-1]),
        'fps':fps,'playback_speed':1,'video_frames':len(indices),'video_exact_saved_state_indices':indices,
        'video_exact_saved_state_times_s':[float(times[i]) for i in indices],
        'gif_source_video_sha256':sha(output/'demo.mp4'),'gif_stored_frames':len(encoded_durations),
        'gif_centisecond_frame_durations_ms':encoded_durations,'gif_duration_ms':sum(encoded_durations),
        'fixed_cameras':cameras,'camera_scope':'Two fixed real open-side/low cameras approved for the identical closed-v3 endpoint; now show actual opening states. Cameras do not track/manipulate state or hide geometry; physical occlusion remains.',
        'screenshot_states':states,'model_identity':identity,
        'runtime':engine_info(),'original_runtime':declaration['runtime'],
        'source_sha256':sha(__file__),
        'source_dependencies_sha256':{p.name:hashlib.sha256(raw).hexdigest() for p,raw in source_bytes.items()},
        'media':{n:{'sha256':sha(output/n),'bytes':(output/n).stat().st_size} for n in ['demo.mp4','demo.gif','entry_detail.png','endpoint_detail.png','open_jaw_gap.png']}}
    for p,raw in source_bytes.items():
        if p.read_bytes()!=raw:raise ValueError('Render dependency changed: '+str(p))
        (output/p.name).write_bytes(raw)
    (output/'render_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'trial':name,'video_frames':len(indices),'native_duration_s':report['actual_duration_s'],'media':manifest['media']},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial',choices=TRIALS);p.add_argument('output',type=Path);p.add_argument('--fps',type=int,default=40)
    args=p.parse_args();run(args.trial,args.output,args.fps)
