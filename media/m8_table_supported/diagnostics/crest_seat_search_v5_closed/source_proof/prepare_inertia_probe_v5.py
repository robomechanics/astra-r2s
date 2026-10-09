from pathlib import Path
import ast,hashlib,json
p=Path(__file__).resolve().parent
base=p/'crest_seat_brake_probe_v4.py'
assert hashlib.sha256(base.read_bytes()).hexdigest()=='aee1c3d5f9f9b8b6b8359b9da92726130acc606862975ab66a8b9ba9174cf74b'
source=base.read_text(); changes=[]
def replace(old,new,purpose):
 global source
 assert source.count(old)==1,(purpose,source.count(old))
 source=source.replace(old,new);changes.append({'purpose':purpose,'original':old,'diagnostic':new})
tree=ast.parse(source)
names={node.targets[0].id:ast.literal_eval(node.value) for node in ast.walk(tree) if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('ledger_names','feedback_names')}
extra='''
from robot_inertia_feedforward_v5 import (HybridInertiaController, InertiaFeedforwardError,
    orbital_site_acceleration, hybrid5d_feedforward, combine_capped_robot_command,
    _array as _inertia_array, _positive_scalar as _inertia_positive_scalar)

class DiagnosticControlRejection(Exception):
    def __init__(self, report, buffers, checkpoint):
        super().__init__(report['aborted']['reason'])
        self.report, self.buffers, self.checkpoint = report, buffers, checkpoint


def _flatten_inertia_record(record):
    inertia = record['inertia']
    if inertia is None:
        raise InertiaFeedforwardError('This selected CLOSED diagnostic requires active five-dimensional feedforward')
    result = {k:v for k,v in record.items() if k != 'inertia'}
    result.update({'mass_aa':inertia['mass_aa'],'hybrid_jacobian5':inertia['jacobian5'],
        'projected_world_jdot_qdot5':inertia['projected_world_jdot_qdot5'],
        'desired_world_acceleration5':inertia['desired_world_acceleration5'],
        'mobility5':inertia['mobility5'],'transverse_basis_world':inertia['transverse_basis_world'],
        'mass_condition':inertia['mass_condition'],'mobility_condition':inertia['mobility_condition'],
        'minimum_mobility_eigenvalue':inertia['minimum_mobility_eigenvalue']})
    return {k:np.asarray(v).copy() for k,v in result.items()}


def _force_arrays(rows, columns):
    shapes={'table_wrench_world_at_block_origin_N_Nm':(6,),
        'left_hand_wrench_world_at_block_origin_N_Nm':(6,), 'pad_normal_force_N':(2,),
        'block_position_m':(3,), 'block_rotation_matrix':(3,3),
        'thread_wrench_on_bolt_world_N_Nm':(6,), 'hand_wrench_on_bolt_world_N_Nm':(6,),
        'right_pad_normal_force_N':(2,), 'right_command_wrench_N_Nm':(6,),
        'left_command_wrench_N_Nm':(6,), 'right_motor_torques_Nm':(6,),
        'left_motor_torques_Nm':(6,)}
    bools={'external_drive_zero','supported_task_active','fully_open_unassisted',
        'right_grasp_guard_active','right_axial_float_active','weight_window_ready',
        'open_weight_window_ready','all_hard_guards_held','weight_observation_valid',
        'open_observation_valid','cold_table_window_warmup','robot_yaw_brake_active'}
    ints={'phase_index','table_contact_candidates','table_loaded_contacts'}
    return {name:np.asarray([row[i] for row in rows],
        dtype=(bool if name in bools else np.int64 if name in ints or name.endswith('_count') or name.endswith('_contacts') else float))
        .reshape((len(rows),)+shapes.get(name,())) for i,name in enumerate(columns)}


def _write_inertia_ledger(output, records, metadata):
    shapes={'enabled':(), 'retained_native_state_time_s':(), 'retained_arm_velocity_rad_s':(6,),
        'arm_jacobian_position':(3,6),'arm_jacobian_rotation':(3,6),
        'arm_bias_torque_Nm':(6,),'native_drag_torque_Nm':(6,),
        'arm_jacobian_position_derivative':(3,6),'arm_jacobian_rotation_derivative':(3,6),
        'requested_site_acceleration_world_m_s2':(3,), 'requested_angular_acceleration_world_rad_s2':(3,),
        'pd_wrench_world_N_Nm':(6,), 'ff_wrench_world_N_Nm':(6,),
        'combined_uncapped_wrench_world_N_Nm':(6,), 'capped_wrench_world_N_Nm':(6,),
        'motor_uncapped_torques_Nm':(6,), 'motor_torques_Nm':(6,),
        'cartesian_force_clipped':(), 'cartesian_torque_clipped':(), 'motor_clipped':(6,),
        'mass_aa':(6,6), 'hybrid_jacobian5':(5,6), 'projected_world_jdot_qdot5':(5,),
        'desired_world_acceleration5':(5,), 'mobility5':(5,5), 'transverse_basis_world':(3,2),
        'mass_condition':(), 'mobility_condition':(), 'minimum_mobility_eigenvalue':(),
        'scheduled_angular_velocity_world_rad_s':(3,), 'scheduled_lever_world_m':(3,),
        'command_time_s':(), 'time':(), 'phase_index':(), 'postintegration_arm_velocity_rad_s':(6,)}
    if records and set(records[0]) != set(shapes):
        raise ValueError('Executed inertia record columns differ from declared stable schema')
    keys=list(shapes)
    bools={'enabled','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped'}
    values={key:np.asarray([row[key] for row in records],
        dtype=bool if key in bools else np.int64 if key=='phase_index' else float)
        .reshape((len(records),)+shape) for key,shape in shapes.items()}
    path=output/'robot_inertia_command_history.npz'
    np.savez_compressed(path,**values,metadata_json=np.asarray(json.dumps({
        'model_fingerprint':metadata['model_fingerprint'],
        'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
        'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
        'inertia_helper_source_sha256':metadata['robot_inertia_feedforward']['source_sha256'],
        'phase_labels':list(metadata['maximum_phase_plan'][i][0] for i in range(len(metadata['maximum_phase_plan']))),
        'scope':'Actual retained native M/J/Jdot/pre-solve velocity and independently scheduled acceleration used for finite robot control. Command state precedes force state by one native timestep; actual retained timestamps are authoritative. No original contact forces are reconstructed.'})))
    return {'filename':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'executed_native_commands':len(records),'columns':keys}


def _preserve_control_rejection(output, rejection):
    metadata = rejection.report
    for filename, rows, columns in (
        ('table_support_force_history.npz', rejection.buffers['support_rows'], ORIGINAL_SUPPORT_COLUMNS),
        ('native_feedback_force_history.npz', rejection.buffers['feedback_rows'], ORIGINAL_FEEDBACK_COLUMNS)):
        values=_force_arrays(rows,columns)
        np.savez_compressed(output/filename,**values,
            phase_labels_json=np.asarray(json.dumps(rejection.buffers['phase_labels'])),
            metadata_json=np.asarray(json.dumps({'model_fingerprint':metadata['model_fingerprint'],
                'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
                'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
                'force_timing':metadata['native_force_recording_note'],
                'scope':'All original solved native observations before orderly control rejection. No native tick or force sample was invented at rejection.'})))
        metadata[filename.removesuffix('.npz')]={'filename':filename,
            'sha256':hashlib.sha256((output/filename).read_bytes()).hexdigest(),
            'observed_physics_steps':len(rows),'columns':list(columns)}
    pad_path=output/'left_pad_force_history.npz'
    np.savez_compressed(pad_path,time=np.asarray(rejection.buffers['left_pad_times'],dtype=float),
        pad_normal_force_N=np.asarray(rejection.buffers['left_pad_forces'],dtype=float).reshape(-1,2),
        phase_index=np.asarray(rejection.buffers['left_pad_phase_indices'],dtype=np.int16),
        phase_labels_json=np.asarray(json.dumps(rejection.buffers['phase_labels'])),
        metadata_json=np.asarray(json.dumps({'model_fingerprint':metadata['model_fingerprint'],
            'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
            'observer':PadLoadHistory.version,'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
            'minimum_loaded_force_N':rejection.buffers['left_pad_history_threshold'],
            'scope':metadata['left_pad_history_scope'],'timing':metadata['native_force_recording_note']})))
    metadata['left_pad_force_history']={'filename':pad_path.name,
        'sha256':hashlib.sha256(pad_path.read_bytes()).hexdigest(),
        'observed_physics_steps':len(rejection.buffers['left_pad_times'])}
    metadata['all_substep_left_pad_loads']=rejection.buffers['left_pad_history_report']
    metadata['robot_inertia_command_history']=_write_inertia_ledger(output,rejection.buffers['inertia_rows'],metadata)
    np.savez_compressed(output/'insertion_trace.npz',time=np.asarray(rejection.buffers['times'],dtype=float),
        qpos=np.asarray(rejection.buffers['positions'],dtype=float).reshape(-1,len(rejection.checkpoint['qpos'])),
        qvel=np.asarray(rejection.buffers['velocities'],dtype=float).reshape(-1,len(rejection.checkpoint['qvel'])),
        controller=np.asarray(rejection.buffers['controls'],dtype=float).reshape(-1,len(rejection.checkpoint['controller'])),
        info_json=np.asarray(json.dumps(rejection.buffers['rows'])),
        metadata_json=np.asarray(json.dumps(metadata)))
    np.savez_compressed(output/'control_rejection_checkpoint.npz',**rejection.checkpoint,
        metadata_json=np.asarray(json.dumps({'scope':'Exact last native post-state or declared initialization state, with no new integration/force solve at rejection','original_native_steps':len(rejection.buffers['feedback_rows'])})))
    (output/'insertion_validation.json').write_text(json.dumps(metadata,indent=2,allow_nan=False))
    (output/'control_rejection.json').write_text(json.dumps(metadata['aborted'],indent=2,allow_nan=False))
'''
extra+='\nORIGINAL_SUPPORT_COLUMNS = '+repr(names['ledger_names'])+'\nORIGINAL_FEEDBACK_COLUMNS = '+repr(names['feedback_names'])+'\n'
replace('_COLD_CONTEXT = None\n',extra+'\n_COLD_CONTEXT = None\n','Independent source-bound FF controller, ledger and orderly pre-step rejection storage')
replace('controllers = {s: YamCartesianController(model, data, s, control.arm, scene.base)', 'controllers = {s: (HybridInertiaController if s == "right" else YamCartesianController)(model, data, s, control.arm, scene.base)', 'Replace only right controller in ignored diagnostic; shared/left originals unchanged')
replace('    right, left = controllers["right"], controllers["left"]\n','    right, left = controllers["right"], controllers["left"]\n    right.capture_velocity_before_step()\n', 'First coherent cache capture follows exact declared cold forward/state initialization')
replace('C2ReverseBrake, BrakeSample, _brake_finite_scalar,\n','C2ReverseBrake, BrakeSample, _brake_finite_scalar,\n            HybridInertiaController, InertiaFeedforwardError, orbital_site_acceleration,\n            hybrid5d_feedforward, combine_capped_robot_command, _inertia_array, _inertia_positive_scalar,\n            DiagnosticControlRejection, _flatten_inertia_record, _force_arrays, _write_inertia_ledger, _preserve_control_rejection,\n','Bind every executed new definition in selected controller bundle')
replace('ArmIK, CrestSeatDropWindowV3, C2ReverseBrake):\n','ArmIK, CrestSeatDropWindowV3, C2ReverseBrake, HybridInertiaController):\n','Archive full executed FF module and controller class')
replace('    feedback_rows = []\n','    feedback_rows = []\n    inertia_rows = []\n','Separate all-step command input/output ledger without altering original force schema')
replace('    metadata["robot_yaw_brake"] = {','    metadata["robot_inertia_feedforward"] = {"source_sha256":_COLD_CONTEXT["inertia_sha256"],\n        "scope":"Approximate retained native6-arm-DOF M_aa with fingers at actual configuration; downstream finger rigid inertia included, independent finger acceleration/coupled contact dynamics not inverted. Five instantaneous WORLD acceleration projections, no axial inversion/position/lead input. Consistently active through all selected CLOSED phases.",\n        "command_velocity_timing":"Separate qvel copy captured immediately BEFORE previous native mj_step matches retained M/J/Jdot/cvel/bias; used coherently for PD damping, Jdot product and drag. Native data.qvel is never overwritten. First command uses declared initialized cold forward state.",\n        "desired_acceleration_scope":"Independently scheduled head angular alpha and orbital site acceleration -alpha cross lever-omega cross(omega cross lever). Current hole basis treated instantaneously; hole acceleration and measured calibration derivatives omitted, not estimated from groove/pitch.",\n        "caps":"Total PD+FF including existing axial feed capped8N/2Nm BEFORE mapping to unchanged finite native robot motors; native bias/drag once, then motorclip.",\n        "numerical_condition_limits":{"mass":1e12,"hybrid_mobility":1e12,"scope":"Numerical bounds in current SI convention, not a physical singularity certificate; finite/SPD/full-rank failures reject without regularization."}}\n    metadata["robot_yaw_brake"] = {','Truthful mechanics/timing/approximation/totalcap scopes')
old='''            right.command(target_p, target_r, gap0 + blend * (gap1 - gap0),
                          linear_velocity=target_v,
                          angular_velocity=target_w,
                          axial_float=axial_float, axis_world=down, axial_feed_N=feed)
'''
new='''            applied_ctrl_before_command=data.ctrl.copy()
            try:
                ff_angular = down*alpha
                ff_scheduled_omega = down*omega
                ff_lever = target_r@target["measured_head_to_tool_p"]
                ff_site_acceleration = orbital_site_acceleration(ff_lever,ff_scheduled_omega,ff_angular)
                right.command(target_p, target_r, gap0+blend*(gap1-gap0),
                    linear_velocity=target_v, angular_velocity=target_w,
                    axial_float=axial_float, axis_world=down, axial_feed_N=feed,
                    desired_site_acceleration_world=ff_site_acceleration,
                    desired_angular_acceleration_world=ff_angular,
                    transverse_basis_world=hole_r[:,:2], inertia_feedforward_enabled=True)
                command_inertia_record=_flatten_inertia_record(right.last_inertia_record)
                command_inertia_record.update({"scheduled_angular_velocity_world_rad_s":ff_scheduled_omega.copy(),
                    "scheduled_lever_world_m":ff_lever.copy(),"command_time_s":float(data.time)})
            except (ValueError, np.linalg.LinAlgError, FloatingPointError) as error:
                rejected=dict(metadata)
                rejected.update({"passed":False,"partial":True,"diagnostic_completed":False,
                    "full_fresh_trajectory_qualified":False,"capture_or_open_reset_qualified":False,
                    "phases":summaries,"final_experimental_crest_direction_event":seat_event.report(),
                    "aborted":{"phase":label,"time":float(data.time),
                        "reason":"Finite hybrid robot inertia command rejected before any new native step",
                        "validation_error":str(error),"new_native_step_executed":False},
                    "original_native_steps":len(feedback_rows),"wall_seconds":time.perf_counter()-start_wall,
                    "diagnostic_acceptance_scope":"Orderly numerical command rejection; original observations and exact last native state preserved. No force solve/tick/readiness was invented at rejection."})
                raise DiagnosticControlRejection(rejected,
                    {"support_rows":support_rows,"feedback_rows":feedback_rows,"inertia_rows":inertia_rows,
                     "phase_labels":[p[0] for p in selected],"times":times,"positions":positions,
                     "velocities":velocities,"controls":controls,"rows":rows,
                     "left_pad_times":left_pad_times,"left_pad_forces":left_pad_forces,
                     "left_pad_phase_indices":left_pad_phase_indices,
                     "left_pad_history_report":left_pad_history.report(),
                     "left_pad_history_threshold":left_pad_history.threshold},
                    {"time":np.asarray(float(data.time)),"qpos":data.qpos.copy(),
                     "qvel":data.qvel.copy(),"controller":applied_ctrl_before_command,
                     "unapplied_rejected_controller":data.ctrl.copy()}) from error
'''
replace(old,new,'Apply approximate5D feedforward throughout closed stages; preserve last solved state/all original rows on pre-step numerical rejection')
replace('            mujoco.mj_step(model, data)\n','            right.capture_velocity_before_step()\n            mujoco.mj_step(model, data)\n            command_inertia_record.update({"time":float(data.time),"phase_index":phase_index,\n                "postintegration_arm_velocity_rad_s":data.qvel[right.dof_indices].copy()})\n            inertia_rows.append(command_inertia_record)\n','Capture coherent pre-solve velocity immediately before native step; record only commands actually executed by that native solve')
replace('"command_calibration": (','"inertia_control": {k:np.asarray(v).tolist() for k,v in command_inertia_record.items()},\n                    "command_calibration": (','Bind sampled input/output snapshots separately from original native force-time transforms')
replace('    result["passed"] = False\n','    metadata["robot_inertia_command_history"]=_write_inertia_ledger(output,inertia_rows,metadata)\n    result["robot_inertia_command_history"]=metadata["robot_inertia_command_history"]\n    result["passed"] = False\n','Freeze complete executed all-step FF input/output ledger before final trace/report')
replace('"cold-crest-return-c2-brake-diagnostic-v4"','"cold-crest-return-c2-hybrid-inertia-diagnostic-v5"','Separate physical controller experiment version')
replace('    brake_sha = _sha(brake_path)\n','    brake_sha = _sha(brake_path)\n    inertia_path=Path(inspect.getfile(HybridInertiaController))\n    inertia_sha=_sha(inertia_path)\n','Bind exact new dependency at preparation/startup')
replace('"robot_yaw_brake_source_sha256":brake_sha, "robot_yaw_brake_duration_s":args.brake_duration,\n','"robot_yaw_brake_source_sha256":brake_sha, "robot_yaw_brake_duration_s":args.brake_duration,\n        "robot_inertia_source_sha256":inertia_sha,\n','Declare actual FF source dependency')
replace('    (args.output/"robot_yaw_brake_source.py").write_bytes(brake_path.read_bytes())\n','    (args.output/"robot_yaw_brake_source.py").write_bytes(brake_path.read_bytes())\n    (args.output/"robot_inertia_source.py").write_bytes(inertia_path.read_bytes())\n','Preserve full new dependency in closed archive')
replace('"brake_sha256":brake_sha, "brake_duration_s":args.brake_duration}\n','"brake_sha256":brake_sha, "brake_duration_s":args.brake_duration,"inertia_sha256":inertia_sha}\n','Pass only declared FF source identity')
replace('    result = run_supported_demo(args.output, scene_config=scene, control_config=control)\n','    try:\n        result = run_supported_demo(args.output, scene_config=scene, control_config=control)\n    except DiagnosticControlRejection as rejection:\n        _preserve_control_rejection(args.output,rejection)\n        result=rejection.report\n','Orderly close all original ledgers/last checkpoint/report on numerical rejection, without another integration or invented solved force')
replace('"brake_sha256_before":brake_sha,"brake_sha256_after":_sha(brake_path),\n','"brake_sha256_before":brake_sha,"brake_sha256_after":_sha(brake_path),\n        "inertia_sha256_before":inertia_sha,"inertia_sha256_after":_sha(inertia_path),\n','Verify new executing dependency unchanged at closure')
replace('            or brake_sha != _sha(brake_path)\n','            or brake_sha != _sha(brake_path) or inertia_sha != _sha(inertia_path)\n','Reject changed native execution dependencies')
replace('    if args.prepare_only:\n        print(json.dumps({"prepared":True,"declaration":declaration},indent=2,allow_nan=False))\n', '''    if args.prepare_only:
        # Separate nonintegrated initialization/command check; never a native
        # trajectory or a solved-force observation reused by the trial.
        prepared_data=mujoco.MjData(model)
        prepared_data.qpos[:]=qpos
        prepared_data.qvel[:]=qvel
        prepared_data.ctrl[:]=ctrl
        mujoco.mj_forward(model,prepared_data)
        prepared_controller=HybridInertiaController(model,prepared_data,"right",control.arm,scene.base)
        prepared_controller.capture_velocity_before_step()
        prepared_qpos,prepared_qvel=prepared_data.qpos.copy(),prepared_data.qvel.copy()
        prepared_tool_p,prepared_tool_r=prepared_controller.pose()
        prepared_hole=model.body("female_frame").id
        prepared_bolt=model.body("male_bolt").id
        prepared_hole_p=prepared_data.xpos[prepared_hole].copy()
        prepared_hole_r=prepared_data.xmat[prepared_hole].reshape(3,3).copy()
        prepared_bolt_r=prepared_data.xmat[prepared_bolt].reshape(3,3).copy()
        prepared_head=prepared_data.xpos[prepared_bolt]+prepared_bolt_r @ [0.,0.,-scene.head_height/2]
        prepared_hole_v=np.zeros(6)
        mujoco.mj_objectVelocity(model,prepared_data,mujoco.mjtObj.mjOBJ_BODY,prepared_hole,prepared_hole_v,0)
        prepared_z=float((prepared_hole_r.T@(prepared_tool_p-prepared_hole_p))[2])
        prepared_target=closed_head_target(prepared_tool_p,prepared_tool_r,prepared_head,prepared_bolt_r,
            prepared_hole_p,prepared_hole_r,yaw_anchor,prepared_z,0.,prepared_hole_v)
        prepared_down=prepared_hole_r[:,2]
        prepared_site_v=np.zeros(6)
        mujoco.mj_objectVelocity(model,prepared_data,mujoco.mjtObj.mjOBJ_SITE,prepared_controller.site_id,prepared_site_v,0)
        prepared_axial_v=_relative_axial_velocity(prepared_tool_p,prepared_site_v[3:],prepared_hole_p,prepared_hole_v,prepared_down)
        prepared_mass=float(model.body_mass[prepared_bolt])
        prepared_mg=float(prepared_mass*np.linalg.norm(model.opt.gravity))
        prepared_feed=prepared_mg-prepared_mass*float(np.dot(model.opt.gravity,prepared_down))-control.axial_velocity_damping_Ns_per_m*prepared_axial_v
        prepared_controller.command(prepared_target["position_m"],prepared_target["rotation"],control.arm.closed_aperture,
            linear_velocity=prepared_target["linear_velocity_m_per_s"],angular_velocity=prepared_target["angular_velocity_rad_per_s"],
            axial_float=True,axis_world=prepared_down,axial_feed_N=prepared_feed,
            desired_site_acceleration_world=np.zeros(3),desired_angular_acceleration_world=np.zeros(3),
            transverse_basis_world=prepared_hole_r[:,:2],inertia_feedforward_enabled=True)
        prepared_record=_flatten_inertia_record(prepared_controller.last_inertia_record)
        prepared_record.update({"scheduled_angular_velocity_world_rad_s":np.zeros(3),
            "scheduled_lever_world_m":prepared_target["rotation"]@prepared_target["measured_head_to_tool_p"],
            "command_time_s":0.,"time":0.,"phase_index":0,
            "postintegration_arm_velocity_rad_s":prepared_qvel[prepared_controller.dof_indices].copy()})
        # The zero-time validation record is not written into the executed
        # native command ledger. Inspect its fixed schema without a dummy tick.
        if not np.array_equal(prepared_data.qpos,prepared_qpos) or not np.array_equal(prepared_data.qvel,prepared_qvel):
            raise ValueError("Command-only preparation mutated native state")
        prepared_proof={"scope":"Separate command-only cold initialization/M/J/Jdot validation; zero native integrations, no physical trajectory/support proof",
            "native_time_s":float(prepared_data.time),"qpos_qvel_preserved":True,
            "actual_right_closed_aperture_m":control.arm.closed_aperture,
            "all_input_output_finite":all(np.isfinite(value).all() for value in prepared_record.values()),
            "record":{key:np.asarray(value).tolist() for key,value in prepared_record.items()},
            "source_65_unchanged":before=={name:_sha(root/name) for name in before}}
        print(json.dumps({"prepared":True,"declaration":declaration,"command_only_initialization":prepared_proof},indent=2,allow_nan=False))
''','Validate coherent exact cold native mass/Jdot/control initialization with no integration or force-proof reuse')
replace('"physical_change":"Same V3 measured-crest-return observer, original cold input and hard guards. Replace abrupt yaw-stop with explicit C2 CLOSED robot-yaw braking from independently scheduled preceding theta/omega/alpha, then original750ms stop observation bound. Actual persistent return and fresh100ms native impulse/angular/axial quiet PLUS unchanged100ms90/10 native weight are required before first closed forward. No object/pitch/helix drive or state write after initialization.",\n','"physical_change":"Same original13.1949 cold input, actual crest observer, C2 robot yaw brake and native hard guards. Add approximate coherent-retained-arm hybrid5D inertia FF consistently through all closed phases, combined with PD/feed inside original8N/2Nm and jointcaps. Separate retainedpre-step velocity changes right PD/drag timing coherently. Original100ms impulse/quiet/90–10 gates unchanged. No axial inversion, object/pitch/helix drive or state write after initialization.",\n','Truthful V5 changed timing/controller scope')
out=p/'crest_seat_inertia_probe_v5.py';out.write_text(source)
(p/'crest_seat_inertia_probe_v5.changes.json').write_text(json.dumps(changes,indent=2)+'\n')
print(json.dumps({'harness_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'changes':len(changes)}))
