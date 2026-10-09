"""Archived-state geometry/mass feasibility; no forward/contact/force solve/step."""
from pathlib import Path
import sys,json,hashlib,argparse
import numpy as np
import mujoco
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'thread_lab/runtime.py').exists())
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT))
from audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine


def hybrid_inertia_feedforward(mass, jacobian, acceleration):
    """Minimum kinetic energy robot acceleration for 5 measured task rows."""
    inverse_task=jacobian@np.linalg.solve(mass,jacobian.T)
    lam=np.linalg.solve(inverse_task,np.eye(5))
    wrench=lam@acceleration
    tau=jacobian.T@wrench
    return lam,wrench,tau


def rz(theta):
    c,s=np.cos(theta),np.sin(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def audit(run):
    runtime=require_micron_engine()
    outer=json.loads(run.with_name(run.name+'_execution_after.json').read_text())
    assert outer['actual_native_exit_code']==1 and outer['runtime_files_unchanged']
    with np.load(run/'insertion_trace.npz',allow_pickle=False) as z:
        metadata=json.loads(str(z['metadata_json']));rows=json.loads(str(z['info_json']))
        times=z['time'].copy();qs=z['qpos'].copy();vs=z['qvel'].copy()
    with np.load(run/'native_feedback_force_history.npz',allow_pickle=False) as z:
        raw={k:z[k].copy() for k in z.files if k not in ('metadata_json','phase_labels_json')}
    model,identity=recorded_model(run/'insertion_trace.npz',metadata)
    assert sorted(x['sha256'] for x in runtime['libraries'])==sorted(x['sha256'] for x in metadata['runtime']['libraries'])
    data=mujoco.MjData(model)
    arm=np.array([int(model.jnt_dofadr[model.joint('right_joint'+str(i)).id]) for i in range(1,7)])
    motor=np.array([model.actuator('right_servo'+str(i)).id for i in range(1,7)])
    caps=model.actuator_ctrlrange[motor,1]
    site=model.site('right_grasp_site').id;bolt=model.body('male_bolt').id;hole=model.body('female_frame').id
    jhp=np.zeros((3,model.nv));jhr=jhp.copy();jbp=jhp.copy();jbr=jhp.copy();jfp=jhp.copy();jfr=jhp.copy()
    mass=np.zeros((model.nv,model.nv));dt=model.opt.timestep
    decl=metadata['cold_initialization'];event=next(e for e in metadata['physical_motion_events'] if e['event']=='Measured crest return requests C2 CLOSED robot-yaw braking')
    result=[]
    for t,q,v,r in zip(times,qs,vs,rows):
        if t<event['time_s']-1e-9:continue
        # The controller command at label(t+2dt) uses retained derived pose t.
        # This matches the saved post-q pose exactly, without a force replay.
        j=int(np.searchsorted(raw['time'],t+2*dt-1e-10))
        if j>=len(raw['time']):continue
        assert abs(raw['time'][j]-(t+2*dt))<1e-9
        data.qpos[:]=q;data.qvel[:]=v
        mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
        mujoco.mj_crb(model,data);mujoco.mj_fullM(model,data,mass)
        mujoco.mj_jacSite(model,data,jhp,jhr,site)
        hp=data.site_xpos[site].copy();hr=data.site_xmat[site].reshape(3,3).copy()
        bp=data.xpos[bolt].copy();br=data.xmat[bolt].reshape(3,3).copy()
        fp=data.xpos[hole].copy();fr=data.xmat[hole].reshape(3,3).copy();axis=fr[:,2]
        head=bp+br@np.array([0.,0.,-metadata['scene_config']['head_height']/2])
        mujoco.mj_jac(model,data,jbp,jbr,head,bolt);mujoco.mj_jacBody(model,data,jfp,jfr,hole)
        clock=raw['desired_independent_clock_rad'][j];omega=raw['desired_independent_angular_speed_rad_s'][j];alpha=raw['desired_independent_angular_acceleration_rad_s2'][j]
        cp=hr.T@(head-hp);cr=hr.T@br
        desired=fr@rz(decl['original_closed_yaw_anchor_rad']+clock-decl['original_independent_clock_anchor_rad'])
        tool_target=desired@cr.T;lever=tool_target@cp
        orbital=-np.cross(alpha*axis,lever)-np.cross(omega*axis,np.cross(omega*axis,lever))
        J=np.vstack([fr[:,:2].T@jhp[:,arm],jhr[:,arm]])
        Maa=mass[np.ix_(arm,arm)]
        acceleration=np.r_[fr[:,:2].T@orbital,alpha*axis]
        lam,ff,tau=hybrid_inertia_feedforward(Maa,J,acceleration)
        fworld=fr[:,:2]@ff[:2];wff=np.r_[fworld,ff[2:]]
        original=raw['right_command_wrench_N_Nm'][j]
        combined=original+wff
        # The original bias/drag compensation and PD remain unchanged in this
        # capacity estimate. Commands would still be capped before application.
        combined_tau=raw['right_motor_torques_Nm'][j]+tau
        head_rel=fr.T@(head-fp);base_rel=fr.T@(bp-fp)
        hvel=jhp@v;headvel=jbp@v;holevel=jfp@v;holeomega=jfr@v
        headrelvel=headvel-holevel-np.cross(holeomega,head-fp)
        target_linear=np.cross(-omega*axis,lever)
        result.append({'saved_post_state_time_s':float(t),'original_command_label_time_s':float(raw['time'][j]),'command_retained_geometry_time_s':float(raw['time'][j]-2*dt),
            'clock_rad':float(clock),'omega_rad_s':float(omega),'alpha_rad_s2':float(alpha),'robot_arm_inertia_kg_m2':Maa.tolist(),'hybrid_task_mass_matrix_mixed_units':lam.tolist(),'J5_world_transverse_linear_and_angular':J.tolist(),'J5_min_singular_value':float(np.linalg.svd(J,compute_uv=False)[-1]),
            'scheduled_orbital_tool_acceleration_m_s2':orbital.tolist(),'hybrid_acceleration_m_s2_rad_s2':acceleration.tolist(),'candidate_inertia_wrench_world_N_Nm':wff.tolist(),'candidate_joint_torques_Nm':tau.tolist(),'original_command_wrench_N_Nm':original.tolist(),'uncapped_combined_wrench_N_Nm':combined.tolist(),'uncapped_combined_joint_torques_Nm':combined_tau.tolist(),
            'candidate_transverse_force_norm_N':float(np.linalg.norm(fworld)),'candidate_torque_norm_Nm':float(np.linalg.norm(ff[2:])), 'uncapped_combined_force_norm_N':float(np.linalg.norm(combined[:3])),'uncapped_combined_torque_norm_Nm':float(np.linalg.norm(combined[3:])), 'uncapped_combined_motor_fraction_of_cap':(np.abs(combined_tau)/caps).tolist(), 'candidate_axial_force_N':float(np.dot(axis,fworld)),
            'post_state_head_radial_vector_m':head_rel[:2].tolist(),'post_state_bolt_base_radial_vector_m':base_rel[:2].tolist(),'post_state_head_radial_velocity_m_s':(fr[:,:2].T@headrelvel).tolist(),'post_state_tool_linear_velocity_world_m_s':hvel.tolist(),'scheduled_orbital_tool_velocity_world_m_s':target_linear.tolist(),'calibrated_head_tool_position_m':cp.tolist(),'original_same_label_calibration_position_m':r['command_calibration']['head_to_tool_p_m']})
    assert result
    return {'scope':'Prospective finite robot-inertia feedforward capacity at exact archived sampled state; not executed control, native force replay, convergence proof or successful threading', 'method':'Compile archived model; copy original saved qpos/qvel; mj_kinematics/mj_comPos/mj_crb/mj_fullM and analytic Jacobians only. No mj_forward, mj_collision, mj_step or original force reconstruction.', 'timing':'Saved post-q t matches the retained geometry for original command label t+2dt. qvel-based geometry velocities are at saved post-time t; original per-step contact forces have separate pre-state label minusdt timing.', 'robot_inertia_scope':'M_aa is the compiled right6arm joint mass block with actual downstream arm/finger inertias. Hybrid5task rows constrain2transverse translations and3rotations; axial translation row is absent. No bolt/object inertia added or object drive. Bias, damping, Jdot and tiny calibration-derivative feedforward are not reestimated here.', 'task_mass_units':'Lambda5 translational block kg; translation/rotation blocks kg*m; rotational block kg*m^2. a5 first2 m/s^2,last3 rad/s^2; wrench first2N,last3Nm.', 'runtime':runtime,'model_identity':identity,'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'trace_sha256':hashlib.sha256((run/'insertion_trace.npz').read_bytes()).hexdigest(),'original_passed':metadata['passed'],'evaluated_archived_states':len(result),
        'maximum_candidate_force_N':max(x['candidate_transverse_force_norm_N'] for x in result),'maximum_candidate_torque_Nm':max(x['candidate_torque_norm_Nm'] for x in result),'maximum_uncapped_combined_force_N':max(x['uncapped_combined_force_norm_N'] for x in result),'maximum_uncapped_combined_torque_Nm':max(x['uncapped_combined_torque_norm_Nm'] for x in result),'maximum_uncapped_combined_motor_fraction_of_cap':np.max([x['uncapped_combined_motor_fraction_of_cap'] for x in result],axis=0).tolist(),'maximum_candidate_absolute_axial_force_N':max(abs(x['candidate_axial_force_N']) for x in result),'minimum_J5_singular_value':min(x['J5_min_singular_value'] for x in result),'rows':result}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=audit(a.run.resolve());a.output.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:r[k] for k in ('evaluated_archived_states','maximum_candidate_force_N','maximum_candidate_torque_Nm','maximum_uncapped_combined_force_N','maximum_uncapped_combined_torque_Nm','maximum_uncapped_combined_motor_fraction_of_cap','maximum_candidate_absolute_axial_force_N')},indent=2))
