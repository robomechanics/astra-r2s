"""Portable exact native-state geometry replay for closed entry-supported trials."""
from pathlib import Path
import argparse,hashlib,json,math,os,sys

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def strict(raw):return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))
def run(args):
    repo=args.repository_root.resolve();parent=args.canonical_prefix.resolve();directory=args.trial_dir.resolve();output=args.output.resolve()
    if output.exists():raise FileExistsError(output)
    declaration=strict((directory/'declaration.json').read_text());report=strict((directory/'report.json').read_text())
    assert sha(directory/'checkpoint_trace.npz')==report['trace_sha256']
    assert sha(directory/'original_native_force_ledger.npz')==report['ledger_sha256']
    assert sha(parent)==declaration['parent_trace_sha256']
    producer={**declaration['parent_source'],'yam_twin/m8_supported_start.py':declaration['observer_sha256']}
    for name,digest in producer.items():assert sha(repo/name)==digest,('Frozen producer mismatch',name)
    dependency_paths=[Path(__file__).resolve(),repo/'scripts/audit_m8_insertion_trace.py',repo/'thread_lab/runtime.py']
    dependencies={p:p.read_bytes() for p in dependency_paths}
    original_files={p:p.read_bytes() for p in directory.rglob('*') if p.is_file()}
    sys.path.insert(0,str(repo));os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('LP_NUM_THREADS','1')
    import numpy as np,mujoco,imageio.v2 as imageio
    from PIL import Image,ImageDraw,ImageFont,ImageSequence
    from scripts.audit_m8_insertion_trace import recorded_model
    from thread_lab.runtime import require_micron_engine,engine_info
    require_micron_engine();assert engine_info()==declaration['runtime']
    with np.load(parent,allow_pickle=False) as z:parent_metadata=strict(str(z['metadata_json']))
    model,identity=recorded_model(parent,parent_metadata)
    assert identity['model_xml_sha256']==sha(directory/'scene.xml')==declaration['parent_model']['model_xml_sha256']
    with np.load(directory/'checkpoint_trace.npz',allow_pickle=False) as z:
        qpos=z['qpos'].copy();qvel=z['qvel'].copy();samples=strict(str(z['info_json']));metadata_raw=str(z['metadata_json']);metadata=strict(metadata_raw)
    # Adaptive producer finalizes only observed timing/event facts in trace
    # metadata. Its original declaration remains the planned inputs.
    assert all(metadata.get(k)==v for k,v in declaration.items()),'Shared declaration/trace metadata changed'
    observed_fields={k:v for k,v in metadata.items() if k not in declaration}
    assert set(observed_fields)<= {'actual_adaptive_open_readiness_event','actual_executed_native_duration_s','actual_shifted_phase_schedule','actual_shifted_planned_duration_s'},set(observed_fields)
    assert len(samples)==len(qpos)==len(qvel) and np.isfinite(qpos).all() and np.isfinite(qvel).all()
    times=np.asarray([s['time_s'] for s in samples]);elapsed=np.asarray([s['elapsed_s'] for s in samples])
    assert np.all(np.diff(times)>0) and np.all(np.diff(elapsed)>0)
    data=mujoco.MjData(model);output.mkdir(parents=True)
    cameras=[{'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':100.,'elevation_deg':-30.,'label':'Actual head / thread / jaw gap'},
             {'lookat_m':[.350,-.010,.029],'distance_m':.15,'azimuth_deg':130.,'elevation_deg':-5.,'label':'Same recorded native state, low real view'}]
    renderer=mujoco.Renderer(model,width=640,height=640)
    font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';big=ImageFont.truetype(font,25);small=ImageFont.truetype(font,18)
    def frame(i):
        data.qpos[:]=qpos[i];data.qvel[:]=qvel[i];data.time=float(times[i]);mujoco.mj_forward(model,data)
        canvas=Image.new('RGB',(1280,808),'#101924')
        for column,view in enumerate(cameras):
            cam=mujoco.MjvCamera();cam.lookat[:]=view['lookat_m'];cam.distance=view['distance_m'];cam.azimuth=view['azimuth_deg'];cam.elevation=view['elevation_deg']
            renderer.update_scene(data,camera=cam);canvas.paste(Image.fromarray(renderer.render()),(column*640,104))
        s=samples[i];draw=ImageDraw.Draw(canvas);draw.rectangle((0,0,1280,138),fill='#101924')
        draw.text((16,10),f'COLD native diagnostic · {args.name}',font=big,fill='white')
        draw.text((16,43),f'{elapsed[i]:.5f} s local native time · {s["physical_phase"]} · normal 1×',font=small,fill='#c5d6e4')
        draw.text((16,71),'Capture / full-pitch lead / full trajectory unqualified · exact original trial',font=small,fill='#f1c46c')
        for column,view in enumerate(cameras):draw.text((column*640+12,111),view['label'],font=small,fill='#c5d6e4')
        draw.rectangle((0,722,1280,808),fill='#101924');left=s['left_pad_normal_force_N'];right=s['right_pad_normal_force_N']
        draw.text((16,732),f'Original solve: table {s["table_upward_force_N"]:.3f} N · left {left[0]:.2f}/{left[1]:.2f} N · right {right[0]:.2f}/{right[1]:.2f} N · robot/bolt contacts {s["right_bolt_contact_count"]}',font=small,fill='#c5d6e4')
        draw.text((16,762),f'Formed {s["formed_flank_overlap_m"]*1e6:.3f} µm · loaded interior {s["loaded_actual_interior_flank_contact_count"]} · original checks {s["all_checks_held"]} · no integration',font=small,fill='#c5d6e4')
        return canvas
    indices=[min(len(times)-1,int(np.searchsorted(elapsed,elapsed[0]+i/args.fps))) for i in range(max(1,math.ceil(float(elapsed[-1])*args.fps)))]
    # Retain the exact terminal native endpoint, especially a real guard abort,
    # in the last display slot. This is finite frame-time quantization only.
    indices[-1]=len(times)-1
    writer=imageio.get_writer(output/'demo.mp4',fps=args.fps,codec='libx264',quality=8,macro_block_size=1,ffmpeg_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
    stills={'entry_detail':0,'endpoint_detail':len(times)-1}
    for phase,label in [('reset_open_search_2','open_reset'),('settle_regrip_search_2','quiet_regrasp'),('stop_start_2','next_turn_stopped')]:
        ids=[i for i,s in enumerate(samples) if s['physical_phase']==phase]
        if ids:stills[label]=ids[-1]
    try:
        for i in indices:writer.append_data(np.asarray(frame(i)))
        for name,i in stills.items():frame(i).save(output/(name+'.png'))
    finally:writer.close();renderer.close()
    gif=[];reader=imageio.get_reader(output/'demo.mp4')
    for im in reader:gif.append(Image.fromarray(im).resize((960,606)).convert('P',palette=Image.Palette.ADAPTIVE,colors=128))
    reader.close();durations=[10*(round((i+1)*100/args.fps)-round(i*100/args.fps)) for i in range(len(gif))]
    gif[0].save(output/'demo.gif',save_all=True,append_images=gif[1:],duration=durations,loop=0,optimize=False,disposal=2)
    with Image.open(output/'demo.gif') as im:encoded_duration=[f.info['duration'] for f in ImageSequence.Iterator(im)]
    media_names=['demo.mp4','demo.gif']+[n+'.png' for n in stills]
    manifest={
        'scope':'One exact closed cold native diagnostic replay, no splice with predecessor trial/kinematic preflight. Capture, full-pitch lead and continuous full trajectory are not qualified. Native original reports/audits remain authoritative.',
        'trial_name':args.name,'original_trial_directory':str(directory),'frozen_repository_root':str(repo),
        'original_declared_execution_root':declaration.get('execution_root'),
        'canonical_prefix_trace_sha256':sha(parent),'actual_state_parent_trace_sha256':declaration['state_parent_trace_sha256'],
        'original_trace_sha256':report['trace_sha256'],'original_ledger_sha256':report['ledger_sha256'],
        'original_declaration_sha256':sha(directory/'declaration.json'),'original_report_sha256':sha(directory/'report.json'),
        'original_trace_metadata_utf8_sha256':hashlib.sha256(metadata_raw.encode()).hexdigest(),
        'all_original_declared_fields_equal_trace_metadata':True,
        'trace_only_finalized_observed_fields':observed_fields,
        'original_overall_guards_held':report['all_original_guards_held'],'original_abort':report['aborted'],
        'physical_bodies_hidden_or_geometry_changed':False,'source_model_regeneration':False,
        'model_runtime_ranges_or_geometry_overrides_applied':False,
        'exact_saved_state_replay':True,'physics_integration':False,'state_interpolation':False,
        'original_force_scope':'Original preintegration solves at logged state time−50us only; saved qpos/qvel are postintegration. Replay forces are never reported.',
        'native_duration_s':report['actual_duration_s'],'actual_last_saved_elapsed_s':float(elapsed[-1]),
        'missing_saved_state_tail_s':float(report['actual_duration_s']-elapsed[-1]),'missing_tail_reconstructed':False,
        'saved_state_rows':len(samples),'fps':args.fps,'playback_speed':1,'video_frames':len(indices),
        'video_exact_saved_indices':indices,'video_exact_saved_times_s':[float(times[i]) for i in indices],
        'last_video_frame_is_original_exact_terminal_saved_state':True,
        'display_quantization_scope':'12fps ceiling and original exact endpoint in last display slot; no state interpolation or synthetic motion. Encoded duration can differ by less than one frame from native duration.',
        'gif_source_video_sha256':sha(output/'demo.mp4'),'gif_stored_frames':len(encoded_duration),
        'gif_centisecond_durations_ms':encoded_duration,'gif_duration_ms':sum(encoded_duration),
        'fixed_real_cameras':cameras,
        'stills':{name:{'state_index':i,'state_time_s':float(times[i]),'elapsed_s':float(elapsed[i]),
            'qpos_sha256':hashlib.sha256(qpos[i].tobytes()).hexdigest(),'qvel_sha256':hashlib.sha256(qvel[i].tobytes()).hexdigest(),'original_sample':samples[i]} for name,i in stills.items()},
        'original_command_open_aperture_m':declaration.get('right_open_aperture_m',.024),
        'frozen_producer_source_hashes':producer,'frozen_sources_equal_before_after':True,
        'renderer_sha256':sha(__file__),'dependency_sha256':{str(p.relative_to(repo)) if p.is_relative_to(repo) else p.name:hashlib.sha256(raw).hexdigest() for p,raw in dependencies.items()},
        'original_trial_files_equal_before_after':True,'original_trial_files_before_after_sha256':{str(p.relative_to(directory)):hashlib.sha256(raw).hexdigest() for p,raw in original_files.items()},
        'model_identity':identity,'runtime_before':declaration['runtime'],'runtime_after':engine_info(),
        'media':{n:{'sha256':sha(output/n),'bytes':(output/n).stat().st_size} for n in media_names}}
    assert manifest['runtime_after']==manifest['runtime_before']
    for name,digest in producer.items():assert sha(repo/name)==digest,name
    for p,raw in original_files.items():assert p.read_bytes()==raw,p
    for p,raw in dependencies.items():
        assert p.read_bytes()==raw,p
        destination=output/'renderer_sources'/(str(p.relative_to(repo)) if p.is_relative_to(repo) else p.name)
        destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(raw)
    (output/'render_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'name':args.name,'video_frames':len(indices),'native_duration_s':report['actual_duration_s'],'stills':{k:v['elapsed_s'] for k,v in manifest['stills'].items()},'media':manifest['media']},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repository-root',type=Path,required=True);p.add_argument('--canonical-prefix',type=Path,required=True);p.add_argument('--trial-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--name',required=True);p.add_argument('--fps',type=int,default=12);run(p.parse_args())
