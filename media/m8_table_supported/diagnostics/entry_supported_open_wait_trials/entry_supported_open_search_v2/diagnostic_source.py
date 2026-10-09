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
    state_report_path=Path(args.state_source).parent/'report.json';state_report=json.loads(state_report_path.read_text())
    if state_report['trace_sha256']!=sha(args.state_source) or state_report['ledger_sha256']!=sha(Path(args.state_source).parent/'original_native_force_ledger.npz'):
        raise ValueError('Readiness report does not bind the exact loaded native checkpoint/ledger')
    if not state_report['all_original_guards_held'] or not state_report['final_weight_window']['ready_for_diagnostic_release_attempt']:
        raise ValueError('Cold parent lacks actual original100ms native entry-weight readiness')
    parent_last=state_report['final_sample']
    if max(parent_last['relative_bolt_angular_speed_rad_per_s'],parent_last['relative_hand_angular_speed_rad_per_s'])>.01 or abs(parent_last['relative_bolt_axial_velocity_m_per_s'])>.0002:
        raise ValueError('Cold parent is not actually quiet')
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
    state_archive=np.load(args.state_source,allow_pickle=False);state_samples=json.loads(str(state_archive['info_json'].item()));native_last=state_samples[-1]
    data.qpos[:]=state_archive['qpos'][-1];data.qvel[:]=state_archive['qvel'][-1];data.time=float(native_last['time_s'])
    data.ctrl[:]=saved['controller'][index]
    for side in ('left','right'):
        ids=[model.actuator(f'{side}_servo{i}').id for i in range(1,7)]
        data.ctrl[ids]=native_last[f'{side}_motor_torques_Nm']
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
    clock_matrix=hr.T@initial_rR@hand_rel_R.T
    forward_start=float(np.arctan2(clock_matrix[1,0],clock_matrix[0,0]));forward_duration=1.875*(args.forward_end-forward_start)/args.turn_speed
    initial_bolt_R=data.xmat[bolt].reshape(3,3).copy()
    frozen_open_head_tool_R=initial_rR.T@initial_bolt_R
    frozen_open_head_tool_p=tcp_calibration.copy()
    frozen_open_relative_tool_R=hr.T@initial_rR
    frozen_open_relative_tool_p=hr.T@(initial_rp-hp)
    measured_head_yaw=float(np.arctan2((hr.T@initial_bolt_R)[1,0],(hr.T@initial_bolt_R)[0,0]))
    clock_matrix=hr.T@initial_rR@hand_rel_R.T
    open_start_clock=float(np.arctan2(clock_matrix[1,0],clock_matrix[0,0]))
    reset_clock=open_start_clock-np.pi
    forward_start=reset_clock;forward_duration=1.875*np.pi/args.turn_speed
    reset_duration=1.875*np.pi/args.reset_speed
    phase_schedule=[('release_search_2',args.release),('open_settle_search_2',args.open_settle),
        ('reset_open_search_2',reset_duration),('open_hold_search_2',args.open_hold),
        ('regrip_search_2',args.regrip),('settle_regrip_search_2',args.regrip_settle),
        ('start_thread_2',forward_duration),('stop_start_2',args.post_turn_hold)]
    phase_ends=np.cumsum([x[1] for x in phase_schedule]);duration=float(phase_ends[-1])
    phase_names=[x[0] for x in phase_schedule]
    open_phases={'open_settle_search_2','reset_open_search_2','open_hold_search_2'}
    open_window=observer.BoltWeightTransferWindow(bolt_weight_N=weight)
    open_reference=None;open_drift_peak_axial=0.;open_drift_peak_yaw=0.;open_weight_ever_ready=False
    new_acquisition=None;regrasp_streak_s=0.;last_phase=None
    head_yaw_start_closed=None;closed_reference_z=None
    stage='entry_supported_open_search';events=[]
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
    np.savez_compressed(out/'declared_cold_initialization.npz',qpos=state_archive['qpos'][-1],qvel=state_archive['qvel'][-1],ctrl=data.ctrl.copy(),time=float(data.time))
    declaration={'scope':'Fresh cold native entry-supported SEARCH open/reindex/regrasp/nextpi diagnostic only; v3 interior0 means no captured fullflank reward or qualified passive-reset label; no fulltrajectory splice/claim',
        'parent_trace_sha256':sha(parent),'parent_model':identity,'parent_source':source_proof,
        'canonical_parent_saved_state_index':index,'canonical_parent_saved_state_time_s':checkpoint,'canonical_parent_saved_phase':old['phase'],'state_parent_trace_sha256':sha(args.state_source),'state_parent_report_sha256':sha(state_report_path),'original_parent_entry_weight_readiness':state_report['final_weight_window'],'original_parent_loaded_actual_interior_contacts':parent_last['loaded_actual_interior_flank_contact_count'],'initial_saved_state_index':len(state_samples)-1,'initial_saved_state_time_s':float(native_last['time_s']),'initial_saved_phase':native_last['physical_phase'],'state_parent_scope':'Exact cold v3 stopped visualservo native qpos/qvel; actual original100ms99.9966%mg supportedentry, loadedinterior0; not fullcapture/passiveresetqualification','initial_tool_clock_rad':open_start_clock,
        'initial_qpos_qvel_sha256':hashlib.sha256(state_archive['qpos'][-1].tobytes()+state_archive['qvel'][-1].tobytes()).hexdigest(),
        'cold_init_note':'Exact post-integration archived qpos/qvel once into new MjData; initial arm ctrl reconstructed from archived last native motor commands, fixed closed finger ctrl from canonical parent; all ctrl overwritten by bounded controllers before first integration; qacc_warmstart and solver state default zero; one mj_forward initializes a new solve',
        'force_timing':'Original new mj_step solved contacts and retained derived kinematics at time-dt; no mj_forward between integration and force recording; saved qpos/qvel post-integration',
        'controller_command_scope':'Finite native arm/finger motors only. Intentionalrelease/open/regrip: dynamic measured transform is frozen, explicit fullXYZ toolimpedance trajectory uses actual release headanchor and prescribed−pi toolrotation; axialfloatinactive/extraaxialfeedzero. Fullyopen requireszero actual entirerightrobot/bolt contacts, so freehand positionservo cannot support/drive bolt. Afteractual100ms quietbilateral centered regrasp: perfect native headpose closedvisualservo with independentlyprescribedpi yaw, axialprojection removes position/lead feedback; mgnetfeed+B200 actualsite-axis damper through nativecap. Allphases retain8N/2Nm wrenches/nativecaps; scheduled toolvelocity includes correct−cross(axisomega,targetR*offset), no measured-transform derivative compensation or realperceptionclaim.',
        'initial_left_target_m':left_p.tolist(),'left_downward_feed_N':args.left_down_force,'left_axial_velocity_damping_Ns_per_m':args.left_axial_damping,'left_downward_hold_scope':'World+Z axial_float removes toolZposition spring; existing forcefeed−2N is preserved continuously through unchanged8NCartesianwrenchcap/finitearmmotors with no reramp,−BactualsitevZ dissipates motion; actual native pads/table bear support, no objectworldforce; table may exceedmg due physical clampdown','right_closed_aperture_m':args.strong_aperture,'grasp_guard_phase_scope':'Original cumulative right1mm/2deg and bilateral-pad guard validated in cold v3 parent; intentionalrelease/open/regrip suspend that reference; new actual100ms quietbilateral reference restorescumulativeguards immediately on all remaining settle ticks and nextclosedpi. Left cumulativeguard and all hard geometry/depth/drive/collision/table/caps remain everytick. Fullyopen additionally zeroentirerightrobot-contact plus10um/.02rad unassisted drift.', 'open_controller_note':'Dynamic head/tool feedback disabled before opening; measured actual release transform frozen and explicit robot tool−pi clock trajectory commanded; closed feedback restored only after quiet bilateral native regrasp and fresh reference','neutral_head_orientation_note':'Perfect-simulator measured head/tool transform visualservo through finite nativearmmotors eachstep; desired headaxis aligns bore and yaw follows independent relative robot scan schedule seeded by actual initialheadyaw; transverse headcentering only, axial projection removes positionservo/lead feedback; no threadphase input/objectforce/statewrite','desired_right_relative_rotation':hand_rel_R.tolist(),'preserved_actual_head_yaw_rad':measured_head_yaw,
        'right_stroke_theta_rad':np.pi if old['phase'].startswith('stop_start_') else 0.,
        'bolt_weight_N':weight,'net_feed_start_N':target_net,'net_feed_final_N':target_net,
        'centering_s':0.,'left_preload_s':0.,'alignment_s':0.,'closed_pre_turn_hold_s':0.,'left_press_preservation':'Continuous existing −2N worldZ float; no preload reramp','phase_names':phase_names,'ramp_s':0.,'hold_s':args.hold,'scope_warning':'Entry-supported SEARCH opening only; existingv3 geometric formed21um butoriginalinterior0 means no fullcapture/passiveresetqualifiedreward','frozen_open_head_to_tool_rotation':frozen_open_head_tool_R.tolist(),'frozen_open_head_to_tool_offset_m':frozen_open_head_tool_p.tolist(),'frozen_open_tool_rotation_relative_hole':frozen_open_relative_tool_R.tolist(),'frozen_open_tool_position_relative_hole_m':frozen_open_relative_tool_p.tolist(),'actual_open_start_tool_clock_rad':open_start_clock,'actual_reset_clock_rad':reset_clock,'phase_schedule':phase_schedule,'physical_reverse_limit_rad':args.reverse_limit,'forward_workspace_endpoint_clock_rad':args.forward_end,'measured_head_to_tool_offset_m':tcp_calibration.tolist(),'seat_reference_scope':'No seat event selected in this branch. Cold v3 stopped entry support enables research opening only; never fullformed reward','physical_closed_turn_angle_rad':None,'physical_closed_turn_peak_speed_rad_s':args.turn_speed,'post_turn_hold_s':args.post_turn_hold,'axial_velocity_damping_Ns_per_m':args.damping,
        'argv':sys.argv,'helper_scope':'Canonical native force and seat event helper frozen ce9c; cold observer use only, no canonical controller feedback integration','diagnostic_source_sha256':script_sha,'observer_sha256':observer_sha,'runtime':require_micron_engine()}
    json_write(out/'declaration.json',declaration)
    scalar=[]; states=[]; velocities=[]; samples=[]; abort=None; start=time.perf_counter(); ever_ready=False
    hv=np.zeros(6); sv=np.zeros(6); bv=np.zeros(6); force=np.zeros(6);left_site_velocity=np.zeros(6)
    wrapped_last=float(np.arctan2((hr.T@data.xmat[bolt].reshape(3,3))[1,0],(hr.T@data.xmat[bolt].reshape(3,3))[0,0]));last_yaw=wrapped_last
    theta0=0.
    for step in range(num):
        hp=data.xpos[hole].copy(); hr=data.xmat[hole].reshape(3,3).copy(); down=hr[:,2]
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_XBODY,hole,hv,0)
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_SITE,right.site_id,sv,0)
        v=_relative_axial_velocity(data.site_xpos[right.site_id],sv[3:],hp,hv,down)
        elapsed=(step+1)*dt
        phase_index=min(int(np.searchsorted(phase_ends,elapsed,side='left')),len(phase_schedule)-1)
        physical_phase,phase_duration=phase_schedule[phase_index]
        phase_start=0. if phase_index==0 else float(phase_ends[phase_index-1])
        phase_elapsed=elapsed-phase_start
        blend,derivative=smooth_profile(min(phase_elapsed/phase_duration,1.))
        down=hr[:,2]
        net=target_net
        feed=net-float(model.body_mass[bolt]*np.dot(model.opt.gravity,down))-args.damping*v
        if physical_phase=='release_search_2':
            theta,omega=open_start_clock,0.;right_aperture=args.strong_aperture+blend*(arm.open_aperture-args.strong_aperture)
        elif physical_phase=='reset_open_search_2':
            theta=open_start_clock-blend*np.pi;omega=-derivative*np.pi/phase_duration;right_aperture=arm.open_aperture
        elif physical_phase in open_phases:
            theta=open_start_clock if physical_phase=='open_settle_search_2' else reset_clock
            omega=0.;right_aperture=arm.open_aperture
        elif physical_phase=='regrip_search_2':
            theta,omega=reset_clock,0.;right_aperture=arm.open_aperture+blend*(args.strong_aperture-arm.open_aperture)
        elif physical_phase=='settle_regrip_search_2':
            theta,omega=reset_clock,0.;right_aperture=args.strong_aperture
        else:
            if new_acquisition is None:
                abort={'elapsed_s':elapsed,'failed_checks':['actual_quiet_bilateral_regrasp_not_acquired']};break
            theta=reset_clock+blend*np.pi if physical_phase=='start_thread_2' else open_start_clock
            omega=derivative*np.pi/phase_duration if physical_phase=='start_thread_2' else 0.
            right_aperture=args.strong_aperture
        if physical_phase=='reset_open_search_2' and last_phase!='reset_open_search_2' and not open_window.ready:
            abort={'elapsed_s':elapsed,'failed_checks':['fresh_fully_open_unloaded100ms_entry_support_not_ready']};break
        closed_servo=physical_phase in {'start_thread_2','stop_start_2'}
        grasp_guard_active=bool(closed_servo or new_acquisition is not None)
        if closed_servo:
            actual_tool_p,actual_tool_R=right.pose();actual_bolt_R=data.xmat[bolt].reshape(3,3)
            actual_head_p=data.xpos[bolt]+actual_bolt_R@np.array([0.,0.,-scene.head_height/2])
            measured_head_tool_R=actual_tool_R.T@actual_bolt_R
            measured_head_tool_p=actual_tool_R.T@(actual_head_p-actual_tool_p)
            desired_bolt_R=hr@Rotation.from_euler('z',head_yaw_start_closed+theta-reset_clock).as_matrix()
            target_R=desired_bolt_R@measured_head_tool_R.T
            goal=hp+down*closed_reference_z-target_R@measured_head_tool_p
            right.command(goal,target_R,right_aperture,
                linear_velocity=hv[3:]+np.cross(hv[:3],goal-hp)-np.cross(down*omega,target_R@measured_head_tool_p),
                angular_velocity=hv[:3]+down*omega,axial_float=True,axis_world=down,axial_feed_N=feed)
        else:
            measured_head_tool_R=frozen_open_head_tool_R
            measured_head_tool_p=frozen_open_head_tool_p
            target_R=hr@Rotation.from_euler('z',theta-open_start_clock).as_matrix()@frozen_open_relative_tool_R
            head_anchor_relative=frozen_open_relative_tool_p+frozen_open_relative_tool_R@frozen_open_head_tool_p
            goal=hp+hr@head_anchor_relative-target_R@frozen_open_head_tool_p
            right.command(goal,target_R,right_aperture,
                linear_velocity=hv[3:]+np.cross(hv[:3],goal-hp)-np.cross(down*omega,target_R@frozen_open_head_tool_p),
                angular_velocity=hv[:3]+down*omega,axial_float=False,axis_world=down)
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_SITE,left.site_id,left_site_velocity,0)
        left_vz=float(left_site_velocity[5]);left_feed=-args.left_down_force-args.left_axial_damping*left_vz
        left.command(left_p,left_R,arm.left_closed_aperture,axial_float=True,axis_world=np.array([0.,0.,1.]),axial_feed_N=left_feed)
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
        table=_table_support_state(model,data,block,table_geoms,with_contacts=step%100==0); lh=_left_hand_contact_state(model,data,block,left,with_contacts=step%100==0)
        table_win.observe(table['table_upward_force_N'],lh['wrench_world'][2],dt,True)
        unexpected=_unexpected_native_contacts(model,data,block,bolt,table_geoms,left.pad_geom_ids,right.pad_geom_ids,(bg,fg),allow_bolt_rest=False,head_geom=hg)
        depth=max([-float(c.dist) for c in data.contact if {int(c.geom1),int(c.geom2)}=={bg,fg}],default=0.)
        margin=min(float(np.min(np.minimum(data.qpos[c.qpos_indices]-model.jnt_range[c.joint_ids,0],model.jnt_range[c.joint_ids,1]-data.qpos[c.qpos_indices]))) for c in (left,right))
        tip_z=float((bp+br@np.array([0.,0.,thread.bolt_length]))[2]); translation=float(np.linalg.norm(data.xpos[block]-block_p0)); block_rot=float(np.linalg.norm(Rotation.from_matrix(data.xmat[block].reshape(3,3)@block_R0.T).as_rotvec())); lift=float(data.xpos[block,2]-block_p0[2])
        checks={'radial':sample['radial_offset_m']<=150e-6,'tilt':sample['bolt_tilt_rad']<=np.deg2rad(2),'thread_depth':depth<=10e-6,
            'left_slip':left_slip<=.001,'left_rotation':left_rot<=np.deg2rad(2),'right_slip':bool(not grasp_guard_active or right_slip<=.001),'right_rotation':bool(not grasp_guard_active or right_rot<=np.deg2rad(2)),
            'no_bolt_world_support':sample['bolt_world_support_contact_count']==0,'no_head_seating':sample['nonthread_block_bolt_contact_count']==0,
            'no_external_drive':sample['external_drive_zero'],'no_unexpected_native_contacts':not unexpected,'joint_limits':margin>=-1e-5,
            'no_lift':lift<=.0005,'block_translation':translation<=.001,'block_rotation':block_rot<=np.deg2rad(2),
            'table_original_load':table['table_upward_force_N']>.1*block_weight,'no_unexpected_block_world_support':not table['unexpected_world_contact_candidates'],
            'male_tip_table_clearance':tip_z>=.001,'no_warnings':not sum(w.number for w in data.warning),
            'finite_state':bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
            'bilateral_left_pad_load':bool(np.all(np.asarray(lh['pad_normal_force_N'])>.1)),
            'bilateral_right_pad_load':bool(not grasp_guard_active or np.all(np.asarray(sample['right_pad_normal_force_N'])>.1)),
            'active_table_rolling_share':bool((step+1)*dt<.1001 or table_win.ready),
            'finite_motor_caps':all(np.all(abs(c.last_motor_torques)<=c.torque_caps+1e-12) for c in (left,right)),
            'finite_wrench_caps':all(np.linalg.norm(c.last_wrench[:3])<=8.+1e-12 and np.linalg.norm(c.last_wrench[3:])<=2.+1e-12 for c in (left,right))}
        right_bolt_contacts=sum(1 for c in data.contact if any(int(model.body_weldid[int(model.geom_bodyid[int(g)])])==bolt for g in (c.geom1,c.geom2))
            and any(((model.geom(int(g)).name or '').startswith('right_') or (model.body(int(model.geom_bodyid[int(g)])).name or '').startswith('right_')) for g in (c.geom1,c.geom2)))
        fully_open=physical_phase in open_phases
        checks['genuine_no_right_robot_contacts_while_open']=bool(not fully_open or right_bolt_contacts==0)
        if fully_open:
            if open_reference is None and right_bolt_contacts==0:
                open_reference={'time_s':float(data.time),'base_z_m':float(rel[2]),'yaw_unwrapped_rad':last_yaw}
                events.append({'event':'First fully-open zero-right-contact native state','reference':open_reference})
            if open_reference is not None:
                open_drift_peak_axial=max(open_drift_peak_axial,abs(float(rel[2])-open_reference['base_z_m']))
                open_drift_peak_yaw=max(open_drift_peak_yaw,abs(last_yaw-open_reference['yaw_unwrapped_rad']))
                checks['open_unassisted_axial_drift']=open_drift_peak_axial<=10e-6
                checks['open_unassisted_yaw_drift']=open_drift_peak_yaw<=.02
        checks={k:bool(v) for k,v in checks.items()}
        wr=win.observe(float(data.time),dt,sample,valid=all(checks.values()));ever_ready|=wr['ready_for_diagnostic_release_attempt']
        partial_report=partial_win.observe(float(data.time),dt,sample,valid=bool(all(checks.values()) and formed>0 and sample['loaded_actual_interior_flank_contact_count']>0))
        open_wr=open_window.observe(float(data.time),dt,sample,valid=bool(fully_open and right_bolt_contacts==0 and all(checks.values())
            and sample['relative_bolt_angular_speed_rad_per_s']<=.01))
        open_weight_ever_ready|=open_wr['ready_for_diagnostic_release_attempt']
        if physical_phase=='settle_regrip_search_2':
            quiet=bool(all(checks.values()) and np.linalg.norm(rrel)<.001 and np.all(np.asarray(sample['right_pad_normal_force_N'])>.1) and sample['relative_bolt_angular_speed_rad_per_s']<=.01
                and sample['relative_hand_angular_speed_rad_per_s']<=.01 and abs(sample['relative_bolt_axial_velocity_m_per_s'])<=.0002)
            regrasp_streak_s=regrasp_streak_s+dt if quiet else 0.
            if new_acquisition is None and regrasp_streak_s>=.1-1e-12:
                grip_p=rrel.copy();grip_R=rR.copy();head_yaw_start_closed=yaw
                closed_reference_z=float((hr.T@(rp-hp))[2])
                new_acquisition={'time_s':float(data.time),'phase':physical_phase,'continuous_quiet_bilateral_streak_s':regrasp_streak_s,
                    'pad_normal_force_N':sample['right_pad_normal_force_N'],'grasp_relative_bolt_head_position_m':grip_p.tolist(),'grasp_relative_bolt_rotation':grip_R.tolist(),
                    'desired_next_head_yaw_anchor_rad':head_yaw_start_closed,'scope':'Actual native loaded quiet opposite-flat regrasp; new cumulative within-grasp reference starts here'}
                events.append({'event':'Actual quiet bilateral regrasp acquired','reference':new_acquisition})
        sample.update({'right_bolt_contact_count':right_bolt_contacts,'fully_open_unassisted':fully_open,'regrasp_streak_s':regrasp_streak_s,'open_weight_window':open_wr,'open_drift_peak_axial_m':open_drift_peak_axial,'open_drift_peak_yaw_rad':open_drift_peak_yaw,'new_grasp_acquisition':new_acquisition,'phase_index':phase_index,'seat_direction_event':None,'partial_profile_support_window':partial_report,'physical_phase':physical_phase,'desired_turn_theta_rad':theta,'desired_turn_angular_speed_rad_s':omega,'time_s':float(data.time),'elapsed_s':(step+1)*dt,'left_worldZ_velocity_m_per_s':left_vz,'left_axial_forcefeed_N':left_feed,'right_grasp_guard_active':grasp_guard_active,'right_axial_float_active':closed_servo,'right_applied_axial_feed_N':feed if closed_servo else 0.,'right_site_axial_wrench_command_N':float(np.dot(right.last_wrench[:3],down)),'right_commanded_aperture_m':right_aperture,'measured_head_to_tool_rotation':measured_head_tool_R.tolist(),'measured_head_to_tool_offset_m':measured_head_tool_p.tolist(),'net_feed_N':net,'axial_motor_feed_N':feed if closed_servo else 0.,'relative_hand_axial_velocity_m_per_s':v,
            'base_z_m':float(rel[2]),'yaw_unwrapped_rad':last_yaw,'thread_overlap_m':float(overlap),'formed_flank_overlap_m':formed,
            'native_thread_depth_m':depth,'right_grip_slip_m':right_slip,'right_grip_rotation_slip_rad':right_rot,
            'left_grip_slip_m':left_slip,'left_grip_rotation_slip_rad':left_rot,'table_upward_force_N':table['table_upward_force_N'],
            'native_table_contact_records':table['contacts'],'native_left_contact_records':lh['native_contact_records'],'left_hand_wrench_world_N_Nm':lh['wrench_world'],'left_motor_wrench_N_Nm':left.last_wrench.tolist(),'left_hand_upward_force_N':lh['wrench_world'][2],'left_pad_normal_force_N':lh['pad_normal_force_N'],
            'block_lift_m':lift,'minimum_joint_margin_rad':margin,'block_translation_m':translation,'block_rotation_rad':block_rot,'table_loaded_ready':table_win.ready,'left_pad_0_N':float(lh['pad_normal_force_N'][0]),'left_pad_1_N':float(lh['pad_normal_force_N'][1]),'right_pad_0_N':float(sample['right_pad_normal_force_N'][0]),'right_pad_1_N':float(sample['right_pad_normal_force_N'][1]),'checks':checks,'weight_window':wr,'table_window':table_win.report(),'all_checks_held':bool(all(checks.values()))})
        scalar.append([float(sample[k]) for k in ('time_s','elapsed_s','net_feed_N','axial_motor_feed_N','thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','relative_bolt_axial_velocity_m_per_s','radial_offset_m','bolt_tilt_rad','thread_overlap_m','formed_flank_overlap_m','interior_flank_gravity_opposing_force_N','interior_flank_summed_normal_force_N','loaded_actual_interior_flank_contact_count','native_thread_depth_m','right_grip_slip_m','table_upward_force_N','left_hand_upward_force_N','external_drive_zero','right_pad_gravity_opposing_force_N','native_thread_contact_count','bolt_world_support_contact_count','nonthread_block_bolt_contact_count','all_checks_held','base_z_m','yaw_unwrapped_rad','desired_turn_theta_rad','desired_turn_angular_speed_rad_s','relative_bolt_angular_speed_rad_per_s','relative_hand_angular_speed_rad_per_s','phase_index','right_bolt_contact_count','fully_open_unassisted','open_drift_peak_axial_m','open_drift_peak_yaw_rad','left_worldZ_velocity_m_per_s','left_axial_forcefeed_N','right_commanded_aperture_m','right_grasp_guard_active','right_axial_float_active','right_applied_axial_feed_N','right_site_axial_wrench_command_N','thread_summed_normal_force_N','minimum_joint_margin_rad','block_translation_m','block_rotation_rad','block_lift_m','left_pad_0_N','left_pad_1_N','right_pad_0_N','right_pad_1_N','left_grip_slip_m','right_grip_rotation_slip_rad','left_grip_rotation_slip_rad','table_loaded_ready')])
        end_now=False
        if not all(checks.values()):
            abort={'elapsed_s':sample['elapsed_s'],'failed_checks':[k for k,value in checks.items() if not value]};end_now=True
        elif elapsed>=duration-dt/2:
            termination='Entry-supported SEARCH open/reindex/regrasp and next physical pi completed';end_now=True
        if step%100==0 or step==num-1 or end_now:
            if step%100!=0:
                original_details=observer.native_weight_transfer_sample(model,data,right,thread,with_records=True)
                sample['native_contact_records']=original_details['native_contact_records']
                sample['native_table_contact_records']=_table_support_state(model,data,block,table_geoms,with_contacts=True)['contacts']
                sample['native_left_contact_records']=_left_hand_contact_state(model,data,block,left,with_contacts=True)['native_contact_records']
            sample['right_motor_torques_Nm']=right.last_motor_torques.tolist()
            sample['left_motor_torques_Nm']=left.last_motor_torques.tolist()
            sample['right_motor_wrench_N_Nm']=right.last_wrench.tolist()
            samples.append(sample);states.append(data.qpos.copy());velocities.append(data.qvel.copy())
        if step%2000==0 or end_now:
            print(f't={sample["elapsed_s"]:.4f} phase={physical_phase} clock={theta:.6f} thread_up={sample["thread_gravity_opposing_force_N"]:.6f} hand_up={sample["hand_gravity_opposing_force_N"]:.6f} overlap_mm={overlap*1000:.6f} formed_um={formed*1e6:.3f} radial_um={sample["radial_offset_m"]*1e6:.3f}',flush=True)
        last_phase=physical_phase
        if end_now:break
    # A pre-command readiness timeout may occur after an unsampled last native
    # step. Preserve that exact retained solve/state; never rerun or forward it.
    if scalar and (not samples or float(samples[-1]['time_s'])!=float(scalar[-1][0])):
        original_details=observer.native_weight_transfer_sample(model,data,right,thread,with_records=True)
        sample['native_contact_records']=original_details['native_contact_records']
        sample['native_table_contact_records']=_table_support_state(model,data,block,table_geoms,with_contacts=True)['contacts']
        sample['native_left_contact_records']=_left_hand_contact_state(model,data,block,left,with_contacts=True)['native_contact_records']
        sample['right_motor_torques_Nm']=right.last_motor_torques.tolist()
        sample['left_motor_torques_Nm']=left.last_motor_torques.tolist()
        sample['right_motor_wrench_N_Nm']=right.last_wrench.tolist()
        samples.append(sample);states.append(data.qpos.copy());velocities.append(data.qvel.copy())
    cols=['time_s','elapsed_s','net_feed_N','axial_motor_feed_N','thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','relative_bolt_axial_velocity_m_per_s','radial_offset_m','bolt_tilt_rad','thread_overlap_m','formed_flank_overlap_m','interior_flank_gravity_opposing_force_N','interior_flank_summed_normal_force_N','loaded_actual_interior_flank_contact_count','native_thread_depth_m','right_grip_slip_m','table_upward_force_N','left_hand_upward_force_N','external_drive_zero','right_pad_gravity_opposing_force_N','native_thread_contact_count','bolt_world_support_contact_count','nonthread_block_bolt_contact_count','all_checks_held','base_z_m','yaw_unwrapped_rad','desired_turn_theta_rad','desired_turn_angular_speed_rad_s','relative_bolt_angular_speed_rad_per_s','relative_hand_angular_speed_rad_per_s','phase_index','right_bolt_contact_count','fully_open_unassisted','open_drift_peak_axial_m','open_drift_peak_yaw_rad','left_worldZ_velocity_m_per_s','left_axial_forcefeed_N','right_commanded_aperture_m','right_grasp_guard_active','right_axial_float_active','right_applied_axial_feed_N','right_site_axial_wrench_command_N','thread_summed_normal_force_N','minimum_joint_margin_rad','block_translation_m','block_rotation_rad','block_lift_m','left_pad_0_N','left_pad_1_N','right_pad_0_N','right_pad_1_N','left_grip_slip_m','right_grip_rotation_slip_rad','left_grip_rotation_slip_rad','table_loaded_ready']
    arr=np.asarray(scalar); raw={k:arr[:,i] for i,k in enumerate(cols)}
    np.savez_compressed(out/'original_native_force_ledger.npz',**raw)
    np.savez_compressed(out/'checkpoint_trace.npz',qpos=np.asarray(states),qvel=np.asarray(velocities),info_json=json.dumps(samples,allow_nan=False,default=lambda v:v.item() if isinstance(v,np.generic) else str(v)),metadata_json=json.dumps(declaration,allow_nan=False))
    turn_summary={'scope':'Actual native entry-supported SEARCH open/reindex/regrasp and next independent closedpi; never fullcapture or qualified passive-reset reward','events':events,'termination':termination,'entry_supported_open_reference':open_reference,'peak_unsupported_axial_drift_m':open_drift_peak_axial,'peak_unsupported_yaw_drift_rad':open_drift_peak_yaw,'actual_regrasp_acquisition':new_acquisition,'fully_open_weight_ready_observed':open_weight_ever_ready}
    final={'physical_closed_turn':turn_summary,'scope':declaration['scope'],'all_original_guards_held':abort is None,'guard_scope':declaration['grasp_guard_phase_scope'],'is_full_capture_or_qualified_reset':False,'aborted':abort,'actual_duration_s':len(scalar)*dt,
        'wall_seconds':time.perf_counter()-start,'ever_weight_transfer_ready':bool(ever_ready),'final_weight_window':win.report(),
        'final_table_window':table_win.report(),'final_sample':samples[-1],
        'partial_formed_loaded_release_allowed':bool(partial_win.ready), 'partial_profile_support_window':partial_win.report(), 'phase_names':phase_names,
        'ranges':{k:[float(np.min(raw[k])),float(np.max(raw[k]))] for k in cols},
        'final100ms':{k:float(np.mean(raw[k][-round(.1/dt):])) for k in cols},
        'ledger_sha256':sha(out/'original_native_force_ledger.npz'),'trace_sha256':sha(out/'checkpoint_trace.npz')}
    if sha(spec.origin)!=observer_sha or sha(out/'observer_source.py')!=observer_sha: raise ValueError('Frozen diagnostic observer changed during probe')
    if sha(__file__)!=script_sha:raise ValueError('Diagnostic harness changed while loaded')
    if any(sha(ROOT/name)!=digest for name,digest in source_proof.items()):raise ValueError('Canonical producer dependency changed while loaded')
    json_write(out/'report.json',final);print(json.dumps({k:final[k] for k in ('all_original_guards_held','aborted','actual_duration_s','wall_seconds','ever_weight_transfer_ready','final_weight_window','partial_formed_loaded_release_allowed','final100ms')},allow_nan=False),flush=True)
    return 0 if abort is None else 1
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--parent',required=True);p.add_argument('--state-source',required=True);p.add_argument('--release',type=float,default=.25);p.add_argument('--open-settle',type=float,default=.12);p.add_argument('--open-hold',type=float,default=.12);p.add_argument('--regrip',type=float,default=.25);p.add_argument('--regrip-settle',type=float,default=.25);p.add_argument('--reset-speed',type=float,default=4.);p.add_argument('--left-preload',type=float,default=.3);p.add_argument('--left-down-offset',type=float,default=.0001);p.add_argument('--left-down-force',type=float,default=2.);p.add_argument('--left-axial-damping',type=float,default=200.);p.add_argument('--prealign',type=float,default=.3);p.add_argument('--prehold',type=float,default=.3);p.add_argument('--strong-aperture',type=float,default=.0184);p.add_argument('--maximum-native-steps',type=int,default=0);p.add_argument('--maximum-reference-settle',type=float,default=1.);p.add_argument('--center',type=float,default=.15);p.add_argument('--reverse-limit',type=float,default=1.9);p.add_argument('--forward-end',type=float,default=1.);p.add_argument('--maximum-stop',type=float,default=.75);p.add_argument('--output',required=True);p.add_argument('--checkpoint',type=float,default=-1.);p.add_argument('--damping',type=float,default=200.);p.add_argument('--ramp',type=float,default=.5);p.add_argument('--hold',type=float,default=.5);p.add_argument('--extra-feed',type=float,default=0.);p.add_argument('--turn-angle',type=float,default=0.);p.add_argument('--turn-speed',type=float,default=1.);p.add_argument('--post-turn-hold',type=float,default=.5);raise SystemExit(run(p.parse_args()))
