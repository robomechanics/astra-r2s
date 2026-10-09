"""Output-only cold checkpoint experiment; never a canonical/restarted rollout."""
from pathlib import Path
import argparse, hashlib, json, sys, time, shutil
import importlib.util
from dataclasses import asdict
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.audit_m8_insertion_trace import recorded_model
from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine
from yam_twin.m8_scene import YamM8Config
from yam_twin.m8_simulation import YamM8ControlConfig, YamCartesianController, smooth_profile
from yam_twin.m8_supported_scene import SupportedConfig, initial_left_grasp_position, left_grasp_rotation
from yam_twin.m8_supported_simulation import (_table_support_state, _left_hand_contact_state,
    _unexpected_native_contacts, TableLoadWindow)
from yam_twin.m8_insertion_simulation import _relative_axial_velocity, formed_flank_overlap
from types import SimpleNamespace
from yam_twin import m8_supported_start as observer
spec=SimpleNamespace(origin=observer.__file__)
EXPECTED_OBSERVER_SHA='ce9c3c2272b475167108ff2d25e568125d6378e64af7a0eadc1fe2219be656f7'
if hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()!=EXPECTED_OBSERVER_SHA or observer.OBSERVER_SOURCE_SHA256!=EXPECTED_OBSERVER_SHA: raise ValueError('Frozen native helper identity changed')

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def json_write(path,value): Path(path).write_text(json.dumps(value,indent=2,allow_nan=False,default=lambda v:v.item() if isinstance(v,np.generic) else str(v))+'\n')
def run(args):
    parent=Path(args.parent).resolve()
    saved=np.load(parent,allow_pickle=False)
    meta=json.loads(str(saved['metadata_json'].item()))
    rows=json.loads(str(saved['info_json'].item()))
    index=len(saved['time'])-1 if args.checkpoint<0 else int(np.argmin(abs(saved['time']-args.checkpoint)))
    checkpoint=float(saved['time'][index]); old=rows[index]
    # Reuse code only after comparing every bound producer source to its frozen bytes.
    expected={**meta['recorded_source_dependencies_sha256'],
              'yam_twin/m8_supported_simulation.py':meta['controller_module_sha256'],
              'yam_twin/m8_supported_scene.py':meta['scene_source_sha256']}
    source_proof={}
    for name,digest in expected.items():
        path=ROOT/name
        if not path.is_file():
            path=ROOT/'yam_twin'/Path(name).name
        actual=sha(path)
        if actual != digest: raise ValueError(f'Frozen producer source changed: {name}')
        source_proof[str(path.relative_to(ROOT))]=actual
    scene_values=dict(meta['scene_config']); base=dict(scene_values.pop('base'))
    thread=ThreadConfig(**base.pop('thread')); scene=SupportedConfig(base=YamM8Config(thread=thread,**base),**scene_values)
    arm=YamM8ControlConfig(**meta['control_config']['arm'])
    model,identity=recorded_model(parent,meta)
    # Exact model initial female/bolt frames recover original desired grasp clock.
    initial=mujoco.MjData(model); mujoco.mj_forward(model,initial)
    bolt,block,hole=(model.body(n).id for n in ('male_bolt','fixture_block','female_frame'))
    br0=initial.xmat[bolt].reshape(3,3).copy(); hr0=initial.xmat[hole].reshape(3,3).copy()
    hand_rel_R=hr0.T@br0@Rotation.from_euler('z',np.pi/2+meta['control_config'].get('pickup_grasp_face_offset_rad',0.)).as_matrix()
    block_p0=initial.xpos[block].copy(); block_R0=initial.xmat[block].reshape(3,3).copy()
    data=mujoco.MjData(model)
    # The only writes of free-body qpos/qvel are this declared cold initialization.
    data.qpos[:]=saved['qpos'][index]; data.qvel[:]=saved['qvel'][index]
    data.ctrl[:]=saved['controller'][index]; data.time=checkpoint
    mujoco.mj_forward(model,data)
    right=YamCartesianController(model,data,'right',arm,scene.base)
    left=YamCartesianController(model,data,'left',arm,scene.base)
    block_dof=int(model.jnt_dofadr[model.joint('fixture_block_free').id]); bolt_dof=int(model.jnt_dofadr[model.joint('male_bolt_free').id])
    bg,fg,hg=(model.geom(n).id for n in ('bolt_thread','female_thread','bolt_head'))
    table_geoms=[model.geom(n).id for n in meta['table_support_geom_names']]
    left_p,left_R=initial_left_grasp_position(scene),left_grasp_rotation(scene)
    acquisition=meta['left_acquisition']; left_ref_p=np.asarray(acquisition['grasp_relative_block_position_m']); left_ref_R=np.asarray(acquisition['grasp_relative_block_rotation'])
    rg=meta['right_grasp_acquisitions'][-1]
    grip_p=np.asarray(rg['grasp_relative_bolt_head_position_m']); grip_R=np.asarray(rg['grasp_relative_bolt_rotation'])
    hp=data.xpos[hole].copy(); hr=data.xmat[hole].reshape(3,3).copy()
    relative_z=float((hr.T@(right.pose()[0]-hp))[2])
    weight=float(model.body_mass[bolt]*np.linalg.norm(model.opt.gravity))
    block_weight=float(meta['known_block_weight_N']);
    reverse_duration=1.875*args.reverse_limit/args.turn_speed
    pre_duration=args.center+args.ramp+args.hold
    settled_reference_streak_s=0.
    duration=pre_duration+args.maximum_reference_settle+reverse_duration+args.maximum_stop+4.*1.875/args.turn_speed+args.post_turn_hold
    target_net=weight+args.extra_feed
    win=observer.BoltWeightTransferWindow(bolt_weight_N=weight)
    partial_win=observer.BoltWeightTransferWindow(bolt_weight_N=weight)
    initial_rp,initial_rR=right.pose()
    tcp_calibration=initial_rR.T@(data.xpos[bolt]+data.xmat[bolt].reshape(3,3)@np.array([0.,0.,-scene.head_height/2])-initial_rp)
    event=None; event_report=None; stage='transfer_bolt_weight'; stage_start=0.; stop_clock=0.; forward_start=0.; forward_duration=0.; termination=None
    phase_names=['center_measured_grasp','transfer_bolt_weight','reverse_seat_1','stop_reverse_seat_1','start_thread_1','stop_start_1']
    table_win=TableLoadWindow(.1,block_weight)
    dt=float(model.opt.timestep); num=round(duration/dt)
    if args.maximum_native_steps: num=min(num,args.maximum_native_steps)
    out=Path(args.output); out.mkdir(parents=True,exist_ok=False)
    script_sha=sha(__file__); observer_sha=sha(spec.origin)
    (out/'diagnostic_source.py').write_bytes(Path(__file__).read_bytes())
    (out/'observer_source.py').write_bytes(Path(spec.origin).read_bytes())
    for name in ('scene.xml','supported_scene.zip'):
        shutil.copy2(parent.parent/name,out/name)
    for name in source_proof:
        target=out/'recorded_sources'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes((ROOT/name).read_bytes())
    target=out/'recorded_sources/yam_twin/m8_supported_start.py';target.write_bytes(Path(spec.origin).read_bytes())
    np.savez_compressed(out/'declared_cold_initialization.npz',qpos=saved['qpos'][index],qvel=saved['qvel'][index],ctrl=saved['controller'][index],time=checkpoint)
    declaration={'scope':'Fresh cold checkpoint diagnostic only; not a continuous full trajectory, not a warmstarted original native solve, not a splice or qualification result',
        'parent_trace_sha256':sha(parent),'parent_model':identity,'parent_source':source_proof,
        'initial_saved_state_index':index,'initial_saved_state_time_s':checkpoint,'initial_saved_phase':old['phase'],
        'initial_qpos_qvel_sha256':hashlib.sha256(saved['qpos'][index].tobytes()+saved['qvel'][index].tobytes()).hexdigest(),
        'cold_init_note':'Exact post-integration archived qpos/qvel/ctrl once into new MjData; qacc_warmstart and solver state default zero; one mj_forward initializes a new solve',
        'force_timing':'Original new mj_step solved contacts and retained derived kinematics at time-dt; no mj_forward between integration and force recording; saved qpos/qvel post-integration',
        'controller_command_scope':'Finite native arm/finger motors only; transverse position and full orientation stabilization; axial velocity damper and bounded net-feed ramp; no axial position spring or imposed thread lead',
        'initial_left_target_m':left_p.tolist(),'desired_right_relative_rotation':hand_rel_R.tolist(),
        'right_stroke_theta_rad':np.pi if old['phase'].startswith('stop_start_') else 0.,
        'bolt_weight_N':weight,'net_feed_start_N':.05,'net_feed_final_N':target_net,
        'centering_s':args.center,'phase_names':phase_names,'ramp_s':args.ramp,'hold_s':args.hold,'physical_reverse_limit_rad':args.reverse_limit,'forward_workspace_endpoint_clock_rad':args.forward_end,'measured_head_to_tool_offset_m':tcp_calibration.tolist(),'seat_reference_scope':'Measured only after physical finite lateral centering and gravity ramp, minimum closed hold and continuously aligned actual axial speed<=.2mm/s and both actual angular speeds<=.01rad/s for100ms; no ramp displacement counts as a seat event','physical_closed_turn_angle_rad':None,'physical_closed_turn_peak_speed_rad_s':args.turn_speed,'post_turn_hold_s':args.post_turn_hold,'axial_velocity_damping_Ns_per_m':args.damping,
        'argv':sys.argv,'helper_scope':'Canonical native force and seat event helper frozen ce9c; cold observer use only, no canonical controller feedback integration','diagnostic_source_sha256':script_sha,'observer_sha256':observer_sha,'runtime':require_micron_engine()}
    json_write(out/'declaration.json',declaration)
    scalar=[]; states=[]; velocities=[]; samples=[]; abort=None; start=time.perf_counter(); ever_ready=False
    hv=np.zeros(6); sv=np.zeros(6); bv=np.zeros(6); force=np.zeros(6)
    last_yaw=old['bolt_yaw_unwrapped_rad']; wrapped_last=float(np.arctan2((hr.T@data.xmat[bolt].reshape(3,3))[1,0],(hr.T@data.xmat[bolt].reshape(3,3))[0,0]))
    theta0=0.
    events=[]
    for step in range(num):
        hp=data.xpos[hole].copy(); hr=data.xmat[hole].reshape(3,3).copy(); down=hr[:,2]
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_XBODY,hole,hv,0)
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_SITE,right.site_id,sv,0)
        v=_relative_axial_velocity(data.site_xpos[right.site_id],sv[3:],hp,hv,down)
        elapsed=(step+1)*dt
        blend=smooth_profile(min(max((elapsed-args.center)/args.ramp,0.),1.))[0]
        net=.05+blend*(target_net-.05)
        feed=net-float(model.body_mass[bolt]*np.dot(model.opt.gravity,down))-args.damping*v
        phase_elapsed=elapsed-stage_start
        if stage=='transfer_bolt_weight':
            theta,omega=0.,0.
            physical_phase='center_measured_grasp' if elapsed<=args.center else stage
        elif stage=='reverse_seat_1':
            turn_blend,turn_derivative=smooth_profile(min(phase_elapsed/reverse_duration,1.))
            theta=-args.reverse_limit*turn_blend
            omega=-args.reverse_limit*turn_derivative/reverse_duration
            physical_phase=stage
        elif stage=='stop_reverse_seat_1':
            theta,omega=stop_clock,0.; physical_phase=stage
        elif stage=='start_thread_1':
            turn_blend,turn_derivative=smooth_profile(min(phase_elapsed/forward_duration,1.))
            theta=forward_start+(args.forward_end-forward_start)*turn_blend
            omega=(args.forward_end-forward_start)*turn_derivative/forward_duration
            physical_phase=stage
        else:
            theta,omega=args.forward_end,0.;physical_phase=stage
        target_R=hr@Rotation.from_euler('z',theta).as_matrix()@hand_rel_R
        centering_blend=smooth_profile(min(elapsed/args.center,1.))[0]
        goal=hp+down*relative_z-centering_blend*(target_R@tcp_calibration)
        right.command(goal,target_R,arm.closed_aperture,
            linear_velocity=hv[3:]+np.cross(hv[:3],goal-hp),angular_velocity=hv[:3]+down*omega,axial_float=True,axis_world=down,axial_feed_N=feed)
        left.command(left_p,left_R,arm.left_closed_aperture)
        mujoco.mj_step(model,data)
        sample=observer.native_weight_transfer_sample(model,data,right,thread,with_records=step%100==0)
        bp=data.xpos[bolt].copy(); br=data.xmat[bolt].reshape(3,3).copy(); hp=data.xpos[hole].copy(); hr=data.xmat[hole].reshape(3,3).copy()
        rel=hr.T@(bp-hp); rr=hr.T@br
        formed=formed_flank_overlap(rel,rr,thread); overlap=min(rel[2]+thread.bolt_length*rr[2,2],thread.nut_height/2)-max(rel[2],-thread.nut_height/2)
        yaw=float(np.arctan2(rr[1,0],rr[0,0])); last_yaw+=float((yaw-wrapped_last+np.pi)%(2*np.pi)-np.pi); wrapped_last=yaw
        lp,lr=left.pose(); rp,rright=right.pose()
        lrel=lr.T@(data.xpos[block]-lp); lR=lr.T@data.xmat[block].reshape(3,3)
        rrel=rright.T@(bp+br@np.array([0,0,-scene.head_height/2])-rp); rR=rright.T@br
        left_slip=float(np.linalg.norm(lrel-left_ref_p)); left_rot=float(np.linalg.norm(Rotation.from_matrix(lR@left_ref_R.T).as_rotvec()))
        right_slip=float(np.linalg.norm(rrel-grip_p)); right_rot=float(np.linalg.norm(Rotation.from_matrix(rR@grip_R.T).as_rotvec()))
        table=_table_support_state(model,data,block,table_geoms); lh=_left_hand_contact_state(model,data,block,left)
        table_win.observe(table['table_upward_force_N'],lh['wrench_world'][2],dt,True)
        unexpected=_unexpected_native_contacts(model,data,block,bolt,table_geoms,left.pad_geom_ids,right.pad_geom_ids,(bg,fg),allow_bolt_rest=False,head_geom=hg)
        depth=max([-float(c.dist) for c in data.contact if {int(c.geom1),int(c.geom2)}=={bg,fg}],default=0.)
        margin=min(float(np.min(np.minimum(data.qpos[c.qpos_indices]-model.jnt_range[c.joint_ids,0],model.jnt_range[c.joint_ids,1]-data.qpos[c.qpos_indices]))) for c in (left,right))
        tip_z=float((bp+br@np.array([0.,0.,thread.bolt_length]))[2]); translation=float(np.linalg.norm(data.xpos[block]-block_p0)); block_rot=float(np.linalg.norm(Rotation.from_matrix(data.xmat[block].reshape(3,3)@block_R0.T).as_rotvec())); lift=float(data.xpos[block,2]-block_p0[2])
        checks={'radial':sample['radial_offset_m']<=150e-6,'tilt':sample['bolt_tilt_rad']<=np.deg2rad(2),'thread_depth':depth<=10e-6,
            'left_slip':left_slip<=.001,'left_rotation':left_rot<=np.deg2rad(2),'right_slip':right_slip<=.001,'right_rotation':right_rot<=np.deg2rad(2),
            'no_bolt_world_support':sample['bolt_world_support_contact_count']==0,'no_head_seating':sample['nonthread_block_bolt_contact_count']==0,
            'no_external_drive':sample['external_drive_zero'],'no_unexpected_native_contacts':not unexpected,'joint_limits':margin>=-1e-5,
            'no_lift':lift<=.0005,'block_translation':translation<=.001,'block_rotation':block_rot<=np.deg2rad(2),
            'table_original_load':table['table_upward_force_N']>.1*block_weight,'no_unexpected_block_world_support':not table['unexpected_world_contact_candidates'],
            'male_tip_table_clearance':tip_z>=.001,'no_warnings':not sum(w.number for w in data.warning),
            'finite_state':bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
            'bilateral_left_pad_load':bool(np.all(np.asarray(lh['pad_normal_force_N'])>.1)),
            'bilateral_right_pad_load':bool(np.all(np.asarray(sample['right_pad_normal_force_N'])>.1)),
            'active_table_rolling_share':bool((step+1)*dt<.1001 or table_win.ready),
            'finite_motor_caps':all(np.all(abs(c.last_motor_torques)<=c.torque_caps+1e-12) for c in (left,right)),
            'finite_wrench_caps':all(np.linalg.norm(c.last_wrench[:3])<=8.+1e-12 and np.linalg.norm(c.last_wrench[3:])<=2.+1e-12 for c in (left,right))}
        checks={k:bool(v) for k,v in checks.items()}
        wr=win.observe(float(data.time),dt,sample,valid=all(checks.values())); ever_ready|=wr['ready_for_diagnostic_release_attempt']
        partial_report=partial_win.observe(float(data.time),dt,sample,valid=bool(all(checks.values()) and formed>0 and sample['loaded_actual_interior_flank_contact_count']>0))
        if event is not None:
            actual_w=max(sample['relative_bolt_angular_speed_rad_per_s'],sample['relative_hand_angular_speed_rad_per_s'])
            event_report=event.observe(float(data.time),dt,float(rel[2]),sample['relative_bolt_axial_velocity_m_per_s'],sample['thread_summed_normal_force_N'],actual_relative_angular_speed_rad_per_s=actual_w,valid=all(checks.values()),physically_stopped=bool(stage=='stop_reverse_seat_1' and actual_w<=.01))
        sample.update({'settled_reference_streak_s':settled_reference_streak_s,'phase_index':phase_names.index(physical_phase),'seat_direction_event':event_report,'partial_profile_support_window':partial_report,'physical_phase':physical_phase,'desired_turn_theta_rad':theta,'desired_turn_angular_speed_rad_s':omega,'time_s':float(data.time),'elapsed_s':(step+1)*dt,'net_feed_N':net,'axial_motor_feed_N':feed,'relative_hand_axial_velocity_m_per_s':v,
            'base_z_m':float(rel[2]),'yaw_unwrapped_rad':last_yaw,'thread_overlap_m':float(overlap),'formed_flank_overlap_m':formed,
            'native_thread_depth_m':depth,'right_grip_slip_m':right_slip,'right_grip_rotation_slip_rad':right_rot,
            'left_grip_slip_m':left_slip,'left_grip_rotation_slip_rad':left_rot,'table_upward_force_N':table['table_upward_force_N'],
            'left_hand_upward_force_N':lh['wrench_world'][2],'left_pad_normal_force_N':lh['pad_normal_force_N'],
            'block_lift_m':lift,'minimum_joint_margin_rad':margin,'block_translation_m':translation,'block_rotation_rad':block_rot,'table_loaded_ready':table_win.ready,'left_pad_0_N':float(lh['pad_normal_force_N'][0]),'left_pad_1_N':float(lh['pad_normal_force_N'][1]),'right_pad_0_N':float(sample['right_pad_normal_force_N'][0]),'right_pad_1_N':float(sample['right_pad_normal_force_N'][1]),'checks':checks,'weight_window':wr,'table_window':table_win.report(),'all_checks_held':bool(all(checks.values()))})
        scalar.append([float(sample[k]) for k in ('time_s','elapsed_s','net_feed_N','axial_motor_feed_N','thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','relative_bolt_axial_velocity_m_per_s','radial_offset_m','bolt_tilt_rad','thread_overlap_m','formed_flank_overlap_m','interior_flank_gravity_opposing_force_N','interior_flank_summed_normal_force_N','loaded_actual_interior_flank_contact_count','native_thread_depth_m','right_grip_slip_m','table_upward_force_N','left_hand_upward_force_N','external_drive_zero','right_pad_gravity_opposing_force_N','native_thread_contact_count','bolt_world_support_contact_count','nonthread_block_bolt_contact_count','all_checks_held','base_z_m','yaw_unwrapped_rad','desired_turn_theta_rad','desired_turn_angular_speed_rad_s','relative_bolt_angular_speed_rad_per_s','relative_hand_angular_speed_rad_per_s','phase_index','minimum_joint_margin_rad','block_translation_m','block_rotation_rad','block_lift_m','left_pad_0_N','left_pad_1_N','right_pad_0_N','right_pad_1_N','left_grip_slip_m','right_grip_rotation_slip_rad','left_grip_rotation_slip_rad','table_loaded_ready')])
        end_now=False
        if not all(checks.values()):
            abort={'elapsed_s':sample['elapsed_s'],'failed_checks':[k for k,value in checks.items() if not value]};end_now=True
        elif stage=='transfer_bolt_weight':
            actual_w=max(sample['relative_bolt_angular_speed_rad_per_s'],sample['relative_hand_angular_speed_rad_per_s'])
            stable=bool(elapsed>=args.center+args.ramp and all(checks.values()) and abs(sample['relative_bolt_axial_velocity_m_per_s'])<=.0002 and abs(v)<=.0002 and actual_w<=.01)
            settled_reference_streak_s=settled_reference_streak_s+dt if stable else 0.
            if elapsed>=pre_duration and settled_reference_streak_s>=.1-1e-12:
                stage='reverse_seat_1';stage_start=elapsed
                ref=float(rel[2]);event=observer.SeatDropWindow(ref)
                events.append({'event':'Actual continuously settled-load seat reference','time_s':float(data.time),'reference_base_z_m':ref,'continuous_aligned_stopped_streak_s':settled_reference_streak_s,'weight_window':win.report()})
            elif elapsed>=pre_duration+args.maximum_reference_settle:
                termination='No continuous aligned native stop window before seat reference';end_now=True
        elif stage=='reverse_seat_1':
            if event.stop_requested:
                clock_matrix=hr.T@data.site_xmat[right.site_id].reshape(3,3)@hand_rel_R.T
                measured=float(np.arctan2(clock_matrix[1,0],clock_matrix[0,0]))
                stop_clock=theta+float((measured-theta+np.pi)%(2*np.pi)-np.pi)
                events.append({'event':'Measured axial drop requests actual stop','time_s':float(data.time),'actual_stop_tool_clock_rad':stop_clock,'observation':event_report})
                stage='stop_reverse_seat_1';stage_start=elapsed
            elif phase_elapsed>=reverse_duration:
                termination='No measured seat-direction drop within collision-safe reverse bound';end_now=True
        elif stage=='stop_reverse_seat_1':
            if event.ready:
                clock_matrix=hr.T@data.site_xmat[right.site_id].reshape(3,3)@hand_rel_R.T
                measured=float(np.arctan2(clock_matrix[1,0],clock_matrix[0,0]))
                forward_start=stop_clock+float((measured-stop_clock+np.pi)%(2*np.pi)-np.pi)
                forward_duration=1.875*(args.forward_end-forward_start)/args.turn_speed
                events.append({'event':'Confirmed stable direction event; physically turn forward','time_s':float(data.time),'actual_start_tool_clock_rad':forward_start,'observation':event_report})
                stage='start_thread_1';stage_start=elapsed
            elif phase_elapsed>=args.maximum_stop:
                termination='Measured drop did not retain continuously stable loaded stop';end_now=True
        elif stage=='start_thread_1' and phase_elapsed>=forward_duration:
            stage='stop_start_1';stage_start=elapsed
        elif stage=='stop_start_1' and phase_elapsed>=args.post_turn_hold:
            termination='Closed bounded forward turn and native settle completed';end_now=True
        if step%100==0 or step==num-1 or end_now:
            if step%100!=0:
                original_details=observer.native_weight_transfer_sample(model,data,right,thread,with_records=True)
                sample['native_contact_records']=original_details['native_contact_records']
            sample['right_motor_torques_Nm']=right.last_motor_torques.tolist()
            sample['left_motor_torques_Nm']=left.last_motor_torques.tolist()
            sample['right_motor_wrench_N_Nm']=right.last_wrench.tolist()
            samples.append(sample);states.append(data.qpos.copy());velocities.append(data.qvel.copy())
        if step%2000==0 or end_now:
            print(f't={sample["elapsed_s"]:.4f} phase={physical_phase} clock={theta:.6f} thread_up={sample["thread_gravity_opposing_force_N"]:.6f} hand_up={sample["hand_gravity_opposing_force_N"]:.6f} overlap_mm={overlap*1000:.6f} formed_um={formed*1e6:.3f} radial_um={sample["radial_offset_m"]*1e6:.3f}',flush=True)
        if end_now:break
    cols=['time_s','elapsed_s','net_feed_N','axial_motor_feed_N','thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','relative_bolt_axial_velocity_m_per_s','radial_offset_m','bolt_tilt_rad','thread_overlap_m','formed_flank_overlap_m','interior_flank_gravity_opposing_force_N','interior_flank_summed_normal_force_N','loaded_actual_interior_flank_contact_count','native_thread_depth_m','right_grip_slip_m','table_upward_force_N','left_hand_upward_force_N','external_drive_zero','right_pad_gravity_opposing_force_N','native_thread_contact_count','bolt_world_support_contact_count','nonthread_block_bolt_contact_count','all_checks_held','base_z_m','yaw_unwrapped_rad','desired_turn_theta_rad','desired_turn_angular_speed_rad_s','relative_bolt_angular_speed_rad_per_s','relative_hand_angular_speed_rad_per_s','phase_index','minimum_joint_margin_rad','block_translation_m','block_rotation_rad','block_lift_m','left_pad_0_N','left_pad_1_N','right_pad_0_N','right_pad_1_N','left_grip_slip_m','right_grip_rotation_slip_rad','left_grip_rotation_slip_rad','table_loaded_ready']
    arr=np.asarray(scalar); raw={k:arr[:,i] for i,k in enumerate(cols)}
    np.savez_compressed(out/'original_native_force_ledger.npz',**raw)
    np.savez_compressed(out/'checkpoint_trace.npz',qpos=np.asarray(states),qvel=np.asarray(velocities),info_json=json.dumps(samples,allow_nan=False,default=lambda v:v.item() if isinstance(v,np.generic) else str(v)),metadata_json=json.dumps(declaration,allow_nan=False))
    turn_summary={'scope':'Measured native reverse direction search and bounded forward turn; never full capture, qualified lead or unsupported reset proof','events':events,'termination':termination,'final_direction_event':event_report}
    final={'physical_closed_turn':turn_summary,'scope':declaration['scope'],'all_original_guards_held':abort is None,'aborted':abort,'actual_duration_s':len(scalar)*dt,
        'wall_seconds':time.perf_counter()-start,'ever_weight_transfer_ready':bool(ever_ready),'final_weight_window':win.report(),
        'final_table_window':table_win.report(),'final_sample':samples[-1],
        'partial_formed_loaded_release_allowed':bool(partial_win.ready), 'partial_profile_support_window':partial_win.report(), 'phase_names':phase_names,
        'ranges':{k:[float(np.min(raw[k])),float(np.max(raw[k]))] for k in cols},
        'final100ms':{k:float(np.mean(raw[k][-round(.1/dt):])) for k in cols},
        'ledger_sha256':sha(out/'original_native_force_ledger.npz'),'trace_sha256':sha(out/'checkpoint_trace.npz')}
    if sha(spec.origin)!=observer_sha or sha(out/'observer_source.py')!=observer_sha: raise ValueError('Frozen diagnostic observer changed during probe')
    json_write(out/'report.json',final);print(json.dumps({k:final[k] for k in ('all_original_guards_held','aborted','actual_duration_s','wall_seconds','ever_weight_transfer_ready','final_weight_window','partial_formed_loaded_release_allowed','final100ms')},allow_nan=False),flush=True)
    return 0 if abort is None else 1
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--parent',required=True);p.add_argument('--maximum-native-steps',type=int,default=0);p.add_argument('--maximum-reference-settle',type=float,default=1.);p.add_argument('--center',type=float,default=.15);p.add_argument('--reverse-limit',type=float,default=1.9);p.add_argument('--forward-end',type=float,default=1.);p.add_argument('--maximum-stop',type=float,default=.75);p.add_argument('--output',required=True);p.add_argument('--checkpoint',type=float,default=-1.);p.add_argument('--damping',type=float,default=200.);p.add_argument('--ramp',type=float,default=.5);p.add_argument('--hold',type=float,default=.5);p.add_argument('--extra-feed',type=float,default=0.);p.add_argument('--turn-angle',type=float,default=0.);p.add_argument('--turn-speed',type=float,default=2.);p.add_argument('--post-turn-hold',type=float,default=.5);raise SystemExit(run(p.parse_args()))
