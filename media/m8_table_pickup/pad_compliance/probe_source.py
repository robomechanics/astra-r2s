"""Separate native-jaw quasistatic contact probe; not actual-demo qualification."""
from pathlib import Path
import hashlib,importlib.util,json,sys,time
import numpy as np,mujoco
ROOT=Path('/workspace/astra-r2s');sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location('audit',ROOT/'scripts/audit_m8_insertion_trace.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
from yam_twin.m8_simulation import YamCartesianController,YamM8ControlConfig
p=ROOT/'outputs/m8_insertion/table_left_smoke_v2/insertion_trace.npz';z=np.load(p);metadata=json.loads(str(z['metadata_json']));rows=json.loads(str(z['info_json']));ix=max(i for i,r in enumerate(rows) if r['phase']=='settle_left_block');cfg=audit.recorded_configuration(metadata); ctrlcfg=YamM8ControlConfig(**metadata['control_config']['arm'])
results=[]
for case in ['original_normal_0p8ms','direct_normal_K31250_B2500']:
 m,identity=audit.recorded_model(p,metadata);d=mujoco.MjData(m);pads=[m.geom('left_m8_pad_'+s).id for s in ['left','right']];block=m.body('fixture_block').id;bg=m.geom('fixture_block_geom').id
 changed=[]
 if case.startswith('direct'):
  for j in range(m.npair):
   if bg in [m.pair_geom1[j],m.pair_geom2[j]] and set([m.pair_geom1[j],m.pair_geom2[j]]).intersection(pads):
    m.pair_solref[j]=[-31250.,-2500.];m.pair_solreffriction[j]=[.0008,1.];changed.append(j)
 d.qpos[:]=z['qpos'][ix];d.qvel[:]=z['qvel'][ix];d.ctrl[:]=z['controller'][ix];mujoco.mj_forward(m,d)
 controllers={side:YamCartesianController(m,d,side,ctrlcfg,cfg.base) for side in ['left','right']};targets={side:c.pose() for side,c in controllers.items()};measures=[]
 for step in range(round(.5/m.opt.timestep)):
  for side,c in controllers.items():c.command(*targets[side],ctrlcfg.left_closed_aperture if side=='left' else ctrlcfg.open_aperture)
  mujoco.mj_step(m,d)
  if d.time<.4:continue
  forces=np.zeros(2);weighted=np.zeros(2);depthmax=np.zeros(2);counts=np.zeros(2,int);cf=np.zeros(6);details=[]
  for k,contact in enumerate(d.contact):
   geoms=list(map(int,contact.geom))
   if bg not in geoms:continue
   for pad_i,pad in enumerate(pads):
    if pad in geoms:
     mujoco.mj_contactForce(m,d,k,cf);forces[pad_i]+=cf[0];weighted[pad_i]+=cf[0]*max(-float(contact.dist),0);depthmax[pad_i]=max(depthmax[pad_i],-float(contact.dist));counts[pad_i]+=1
     if step==round(.5/m.opt.timestep)-1:
      adr=contact.efc_address;details.append({'pad':m.geom(pad).name,'force_N':float(cf[0]),'distance_m':float(contact.dist),'solref':contact.solref.tolist(),'solreffriction':contact.solreffriction.tolist(),'solimp':contact.solimp.tolist(),'normal_KBIP':d.efc_KBIP[adr].tolist(),'normal_diagA':float(d.efc_diagA[adr]),'normal_R':float(d.efc_R[adr])})
  indentation=weighted/np.maximum(forces,1e-300)
  measures.append(np.r_[d.time,forces,indentation,depthmax,counts])
  if step==round(.5/m.opt.timestep)-1:final_details=details
 values=np.array(measures);fmean=values[:,1:3].mean(0);indentmean=values[:,3:5].mean(0)
 item={'case':case,'modified_pair_ids':changed,'measurement_scope':'Separate native 12 bounded arm motors plus 4 finite jaw position actuators holding original table-acquisition configuration; no object actuator/weld/applied wrench. Mean of original native pre-integration contact force/depth pairs over final 0.1 s of 0.5 s integration; not actual-demo qualification or material calibration.','mean_pad_normal_force_N':fmean.tolist(),'mean_force_weighted_normal_indentation_m':indentmean.tolist(),'force_weighted_indentation_range_m':[[float(values[:,3+j].min()),float(values[:,3+j].max())] for j in range(2)],'mean_pad_secant_stiffness_N_per_m':(fmean/indentmean).tolist(),'unloaded_measurement_steps':(values[:,1:3]<=.1).sum(0).tolist(),'final_contacts':final_details,'maximum_object_applied_force':float(np.max(abs(d.xfrc_applied[[block,m.body('male_bolt').id]]))),'warnings':[int(w.number) for w in d.warning]}
 results.append(item);print(json.dumps(item),flush=True)
output={'source_trace':str(p),'source_trace_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'compiled_archived_model_identity':identity,'initial_saved_time_s':float(z['time'][ix]),'probe_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'rubber_comparison_assumptions':{'face_area_m2':.009*.0055,'thickness_m':.003,'assumed_Young_modulus_Pa':1e7,'nominal_compression_at_16N_m':16*.003/(1e7*.009*.0055),'scope':'Simple uniform uniaxial compression reference only; uncalibrated assumed modulus, no shape-factor correction.'},'cases':results}
out=ROOT/'outputs/m8_insertion/pad_compliance_review';out.mkdir(exist_ok=True);(out/'native_pad_compliance_probe.json').write_text(json.dumps(output,indent=2)+'\n');(out/'probe_source.py').write_bytes(Path(__file__).read_bytes())
