"""Render one exact native pre-entry state with an unobstructed table camera."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import sys

ROOT=Path('/workspace/astra-r2s')
sys.path.insert(0,str(ROOT))
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('MESA_SHADER_CACHE_DIR','/workspace/.cache/mesa')
os.environ.setdefault('LP_NUM_THREADS','2')
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine,engine_info


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--azimuth',type=float,default=70)
    parser.add_argument('--elevation',type=float,default=-45)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    trace=ROOT/'outputs/m8_table_supported/face120_prefix_v1/insertion_trace.npz'
    dependencies=[Path(__file__),ROOT/'scripts/audit_m8_insertion_trace.py',ROOT/'thread_lab/runtime.py']
    source_bytes={p:p.read_bytes() for p in dependencies}
    require_micron_engine()
    with np.load(trace,allow_pickle=False) as saved:
        rows=json.loads(str(saved['info_json']))
        metadata=json.loads(str(saved['metadata_json']))
        index=[i for i,row in enumerate(rows) if row['phase']=='align_over_hole'][-1]
        qpos=saved['qpos'][index].copy();qvel=saved['qvel'][index].copy()
        timestamp=float(saved['time'][index])
    sample=rows[index]
    model,model_identity=recorded_model(trace,metadata)
    data=mujoco.MjData(model)
    data.qpos[:]=qpos;data.qvel[:]=qvel;data.time=timestamp
    mujoco.mj_forward(model,data)
    views=[
        {'lookat_m':[.285,-.010,.020],'distance_m':.22,'azimuth_deg':70.,'elevation_deg':-45.,'label':'Left side clamp / table-supported block'},
        {'lookat_m':[.35,-.010,.031],'distance_m':.17,'azimuth_deg':-30.,'elevation_deg':-35.,'label':'Right jaws / actual hex head and shaft'}]
    renderer=mujoco.Renderer(model,width=640,height=640)
    frame=Image.new('RGB',(1280,760),'#101924')
    try:
        for column,view in enumerate(views):
            camera=mujoco.MjvCamera()
            camera.lookat[:]=view['lookat_m'];camera.distance=view['distance_m']
            camera.azimuth=view['azimuth_deg'];camera.elevation=view['elevation_deg']
            renderer.update_scene(data,camera=camera)
            frame.paste(Image.fromarray(renderer.render()),(column*640,88))
    finally:
        renderer.close()
    draw=ImageDraw.Draw(frame)
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    big=ImageFont.truetype(font_path,27);small=ImageFont.truetype(font_path,21)
    draw.rectangle((0,0,1280,88),fill='#101924')
    draw.text((20,12),'Actual table details · two cameras, same saved native state',font=big,fill='white')
    draw.text((20,51),f'{timestamp:.2f} s · before entry · no hidden geometry / no formed-thread capture',font=small,fill='#c5d6e4')
    for column,view in enumerate(views):
        draw.rectangle((column*640,88,column*640+640,122),fill='#101924')
        draw.text((column*640+12,93),view['label'],font=small,fill='#c5d6e4')
    draw.rectangle((0,716,1280,760),fill='#101924')
    table=sample['table_support']['table_upward_force_N']
    pads=sample['left_contact']['pad_normal_force_N']
    draw.text((20,726),f'Original solve: table {table:.3f} N · left pads {pads[0]:.2f}/{pads[1]:.2f} N',font=small,fill='#c5d6e4')
    args.output.mkdir(parents=True,exist_ok=True)
    frame.save(args.output/'table_view.png')
    manifest={
        'scope':'Supplementary two-camera detail of the same actual saved pre-entry native state. Only viewing cameras and panel layout change. No physical bodies are hidden by render settings or removed; no integration, state interpolation, free-object posing or force reconstruction.',
        'trajectory_sha256':sha(trace),'sample_index':index,'state_time_s':timestamp,
        'state_phase':sample['phase'],
        'qpos_sha256':hashlib.sha256(qpos.tobytes()).hexdigest(),
        'qvel_sha256':hashlib.sha256(qvel.tobytes()).hexdigest(),
        'cameras':views,
        'model_identity':model_identity,'original_sample':sample,
        'original_force_time_s':timestamp-model.opt.timestep,
        'original_force_scope':'Caption uses original native solve records. Saved qpos/qvel are postintegration; mj_forward forces are not reported.',
        'exact_saved_state_replay':True,'physics_integration':False,'state_interpolation':False,
        'physical_bodies_hidden_or_geometry_changed':False,
        'runtime':engine_info(),'original_recorded_runtime':metadata['runtime'],
        'source_sha256':sha(__file__),
        'dependency_sha256':{p.name:hashlib.sha256(raw).hexdigest() for p,raw in source_bytes.items()},
        'image_sha256':sha(args.output/'table_view.png')}
    for p,raw in source_bytes.items():
        if p.read_bytes()!=raw:raise ValueError('Render source changed')
        (args.output/p.name).write_bytes(raw)
    (args.output/'table_view_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:manifest[k] for k in ['sample_index','state_time_s','state_phase','cameras','image_sha256']},indent=2))


if __name__=='__main__':main()
