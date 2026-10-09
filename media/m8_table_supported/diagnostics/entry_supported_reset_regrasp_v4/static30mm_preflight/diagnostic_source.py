"""Output-only native geometry/reach study; no integration or force solve."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'thread_lab/runtime.py').is_file())
sys.path.insert(0,str(ROOT))
from scripts.audit_m8_insertion_trace import recorded_model,archived_source_object
from yam_twin.kinematics import ArmIK
from thread_lab.runtime import require_micron_engine
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    out=ROOT/'outputs/m8_table_supported/diagnostics/open30_static_preflight_v1';out.mkdir(exist_ok=False)
    source=Path(__file__);(out/'diagnostic_source.py').write_bytes(source.read_bytes())
    parent=ROOT/'outputs/m8_table_supported/face120_prefix_v1/insertion_trace.npz'
    v3=ROOT/'outputs/m8_table_supported/diagnostics/entry_supported_open_search_v3'
    original=np.load(parent,allow_pickle=False);metadata=json.loads(str(original['metadata_json'].item()))
    model,identity=recorded_model(parent,metadata)
    declaration=json.loads((v3/'declaration.json').read_text())
    checkpoint=np.load(v3/'checkpoint_trace.npz',allow_pickle=False)
    samples=json.loads(str(checkpoint['info_json'].item()))
    cold=np.load(v3/'declared_cold_initialization.npz',allow_pickle=False);qbase=cold['qpos'].copy()
    frozen=parent.parent/'controller_source.py';ns={};exec(archived_source_object(frozen.read_text(),'_unexpected_native_contacts'),ns)
    classifier=ns['_unexpected_native_contacts']
    copies=[frozen,ROOT/'yam_twin/kinematics.py',ROOT/'scripts/audit_m8_insertion_trace.py',v3/'diagnostic_source.py',v3/'declaration.json']
    inputs=out/'inputs';inputs.mkdir()
    for p in copies:(inputs/p.name).write_bytes(p.read_bytes())
    site=model.site('right_grasp_site').id;bolt=model.body('male_bolt').id;block=model.body('fixture_block').id;hole=model.body('female_frame').id
    joints=[model.joint('right_joint'+str(i)).id for i in range(1,7)];qadr=model.jnt_qposadr[joints]
    finger_joints=[model.joint('right_'+f+'_finger').id for f in ('left','right')];fadr=model.jnt_qposadr[finger_joints]
    targets=np.array([.0155,-.0155]);actuators=[model.actuator('right_grip_'+f).id for f in ('left','right')]
    native_fingers=[{'joint':model.joint(j).name,'joint_range_m':model.jnt_range[j].tolist(),'joint_limited':bool(model.jnt_limited[j]),'ctrl_range_m':model.actuator_ctrlrange[a].tolist(),'ctrl_limited':bool(model.actuator_ctrllimited[a]),'force_range_N':model.actuator_forcerange[a].tolist(),'force_limited':bool(model.actuator_forcelimited[a]),'command_m':float(q)} for j,a,q in zip(finger_joints,actuators,targets)]
    if not all(model.jnt_range[j,0]<=q<=model.jnt_range[j,1] and model.actuator_ctrlrange[a,0]<=q<=model.actuator_ctrlrange[a,1] for j,a,q in zip(finger_joints,actuators,targets)):raise ValueError('30mm exceeds native bounds')
    pads=[frozenset(model.geom(s+'_m8_pad_'+f).id for f in ('left','right')) for s in ('left','right')]
    tables=[model.geom(n).id for n in metadata['table_support_geom_names']]
    thread_geoms=tuple(model.geom(n).id for n in ('bolt_thread','female_thread'));head=model.geom('bolt_head').id
    data=mujoco.MjData(model);records=[];solved=[]
    def record(kind,q,clock=None,time=None,pe=None,re=None):
        data.qpos[:]=q;data.qpos[fadr]=targets;mujoco.mj_kinematics(model,data);mujoco.mj_collision(model,data)
        unchanged=np.setdiff1d(np.arange(model.nq),np.r_[qadr,fadr])
        if kind=='nominal_frozen_open_route' and not np.array_equal(data.qpos[unchanged],qbase[unchanged]):raise ValueError('Non-right-robot coordinate changed')
        unexpected=classifier(model,data,block,bolt,tables,*pads,thread_geoms,allow_bolt_rest=False,head_geom=head)
        right_bolt=[{'geom1':model.geom(int(c.geom1)).name,'geom2':model.geom(int(c.geom2)).name,'dist_m':float(c.dist)} for c in data.contact if any(int(model.body_weldid[int(model.geom_bodyid[int(g)])])==bolt for g in (c.geom1,c.geom2)) and any((model.geom(int(g)).name or '').startswith('right_') or (model.body(int(model.geom_bodyid[int(g)])).name or '').startswith('right_') for g in (c.geom1,c.geom2))]
        distances={model.geom(g).name:float(mujoco.mj_geomDistance(model,data,g,head,.05,np.zeros(6))) for g in pads[1]}
        margin=float(np.min(np.minimum(data.qpos[qadr]-model.jnt_range[joints,0],model.jnt_range[joints,1]-data.qpos[qadr])))
        passed=not unexpected and not right_bolt and min(distances.values())>0 and margin>=.05 and (pe is None or pe<=50e-6) and (re is None or re<=.0005)
        records.append({'scope':kind,'clock_rad':clock,'original_post_state_time_s':time,'right_joint_margin_rad':margin,'position_error_m':pe,'rotation_error_rad':re,'unexpected_native_contacts':unexpected,'right_robot_bolt_contacts':right_bolt,'pad_head_distances_m':distances,'passed':bool(passed)})
        solved.append(data.qpos.copy())
    # All original sampled native post-states including the exact failed final
    # state; only hypothetical right finger opening changes in these copies.
    for q,s in zip(checkpoint['qpos'],samples):record('observed_v3_post_state_with_hypothetical30mm',q,time=float(s['time_s']))
    ref=mujoco.MjData(model);ref.qpos[:]=qbase;mujoco.mj_kinematics(model,ref)
    hp=ref.xpos[hole].copy();hr=ref.xmat[hole].reshape(3,3).copy()
    frozen_r=np.asarray(declaration['frozen_open_tool_rotation_relative_hole']);frozen_p=np.asarray(declaration['frozen_open_tool_position_relative_hole_m']);offset=np.asarray(declaration['frozen_open_head_to_tool_offset_m']);anchor=frozen_p+frozen_r@offset
    start=declaration['actual_open_start_tool_clock_rad'];end=declaration['actual_reset_clock_rad']
    ik=ArmIK(model,'right');ik.bounds[0]+=.05;ik.bounds[1]-=.05;ik.q=qbase[qadr].copy()
    for clock in np.linspace(start,end,127):
        ik.data.qpos[:]=qbase;ik.data.qpos[fadr]=targets
        r=hr@Rotation.from_euler('z',clock-start).as_matrix()@frozen_r;p=hp+hr@anchor-r@offset
        ik.solve(p,r);actual_p,actual_r=ik.pose();pe=float(np.linalg.norm(p-actual_p));re=float(np.linalg.norm(Rotation.from_matrix(r@actual_r.T).as_rotvec()))
        q=qbase.copy();q[qadr]=ik.q;record('nominal_frozen_open_route',q,float(clock),pe=pe,re=re)
    np.savez_compressed(out/'sampled_kinematic_coordinates.npz',qpos=np.asarray(solved))
    report={'scope':'Native static reach/geometry only; no integration, mj_forward, force solve, controller command, dynamic clearance or physical30mm actuation proof. Hypothetical finger coordinates changed only in scratch copied states. Native asset joint bounds are not independently measured real hardware certification. Nominal127-point reset route is sampled, not a continuous swept-volume certificate.','opening_m':.03,'nominal_native_finger_targets_m':targets.tolist(),'native_finger_ranges_caps':native_fingers,'parent_model':identity,'runtime':require_micron_engine(),'parent_trace_sha256':sha(parent),'v3_checkpoint_sha256':sha(v3/'checkpoint_trace.npz'),'v3_cold_initialization_sha256':sha(v3/'declared_cold_initialization.npz'),'v3_declaration_sha256':sha(v3/'declaration.json'),'v3_native_force_ledger_sha256':sha(v3/'original_native_force_ledger.npz'),'input_source_sha256':{p.name:sha(p) for p in inputs.iterdir()},'diagnostic_source_sha256':sha(source),'coordinates_sha256':sha(out/'sampled_kinematic_coordinates.npz'),'sample_count':len(records),'all_sampled_states_passed':all(r['passed'] for r in records),'minimum_pad_head_distance_m':min(min(r['pad_head_distances_m'].values()) for r in records),'minimum_right_joint_margin_rad':min(r['right_joint_margin_rad'] for r in records),'records':records}
    (out/'study.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('sample_count','all_sampled_states_passed','minimum_pad_head_distance_m','minimum_right_joint_margin_rad','native_finger_ranges_caps')},indent=2))
    print('failed_states',json.dumps([r for r in records if not r['passed']][:10]))
if __name__=='__main__':main()
