from pathlib import Path
import json,hashlib,sys
import numpy as np
p=Path(sys.argv[1]);trace=p/'insertion_trace.npz'
sha=hashlib.sha256(trace.read_bytes()).hexdigest()
with np.load(trace,allow_pickle=False) as z:
 times=z['time'].copy();rows=json.loads(str(z['info_json']));r=json.loads(str(z['metadata_json']))
with np.load(p/'left_pad_force_history.npz',allow_pickle=False) as z:
 dt=json.loads(str(z['metadata_json']))['timestep_s']
c=r['control_config'];duration=c['entry_support_window_s'];limit=c['entry_support_velocity_limit_m_per_s'];feed=c['net_axial_feed_N']
force_min=.1*feed;impulse_min=.1*feed*duration;loaded_min=.1*duration
errors=[];ready_rows=0
for i,row in enumerate(rows):
 w=row['entry_support']
 expected=(w['observed_window_s']+1e-12>=duration and w['normal_impulse_Ns']>=impulse_min and w['loaded_duration_s']>=loaded_min and w['final_native_normal_force_N']>force_min)
 if w['ready']!=expected:errors.append([i,'ready formula'])
 for key,value in [('required_window_s',duration),('minimum_loaded_normal_force_N',force_min),('minimum_normal_impulse_Ns',impulse_min),('minimum_loaded_duration_s',loaded_min),('relative_bolt_axial_velocity_limit_m_per_s',limit)]:
  if w[key]!=value:errors.append([i,key])
 if w['final_native_normal_force_N']!=row['native_thread_pair_normal_force_N']:errors.append([i,'native pair force input'])
 observed=w['observed_window_s'];duty=w['loaded_duration_s']/observed if observed else 0.
 if abs(w['loaded_substep_duty']-duty)>1e-12:errors.append([i,'duty'])
 if w['ready']:
  ready_rows+=1
  if not w['aligned'] or not w['relative_bolt_axial_velocity_range_m_per_s'] or max(abs(x) for x in w['relative_bolt_axial_velocity_range_m_per_s'])>limit:errors.append([i,'settled velocity range'])
  if row['radial_offset_m']>150e-6 or row['bolt_tilt_rad']>np.deg2rad(2) or row['bolt_world_support_contacts'] or row['block_world_support_contacts'] or not row['external_drive_zero']:errors.append([i,'native alignment inputs'])
events=[]
for event in r['entry_support_events']:
 indices=np.flatnonzero(times==event['time_s']);assert len(indices)==1
 i=int(indices[0]);window=rows[i]['entry_support'];same={k:event[k] for k in window}==window
 if not same:errors.append([i,'event support report'])
 timing=True
 if 'actual_dwell_s' in event:
  start=next(j for j,row in enumerate(rows) if row['phase']==event['phase'])
  observed_time=event['time_s']-(times[start]-dt)
  timing=abs(observed_time-event['actual_dwell_s'])<1e-8 and event['actual_dwell_s']<=event['maximum_dwell_s'] and (not event['phase'].startswith('stop_start_') or event['actual_dwell_s']>=.12-1e-12)
  if not timing:errors.append([i,'bounded dwell timing'])
 events.append({'phase':event['phase'],'time_s':event['time_s'],'row_index':i,'row_phase':rows[i]['phase'],'support_report_exact_match':same,'bounded_dwell_timing_match':timing,'ready':event['ready'],'original_event':event})
entry_time=r['acceptance_checks']['native_cone_entry_acquired_before_rotation']['acquisition_time_s']
first_start=min(row['time'] for row in rows if row['phase'].startswith('start_thread_'))
if not entry_time<first_start:errors.append(['entry did not precede rotation'])
B=c['axial_velocity_damping_Ns_per_m'];commands=[row for row in rows if row['axial_float']]
v=np.asarray([row['right_relative_axial_velocity_m_per_s'] for row in commands]);f=np.asarray([row['axial_velocity_damping_force_N'] for row in commands]);assert np.isfinite(v).all() and np.isfinite(f).all()
res=np.abs(f+B*v)
if not np.all(res<1e-12) or not np.all(f*v<=1e-12):errors.append(['damping command algebra or power sign'])
peaks={}
for row in rows:
 name=row['phase']
 if name.startswith(('start_thread_','turn_')):peaks[name]=max(peaks.get(name,0.),abs(row['right_commanded_stroke_angular_speed_rad_s']))
for name,peak in peaks.items():
 permitted=c['starting_angular_speed_rad_s'] if name.startswith('start_thread_') else c['arm']['angular_speed_rad_s']
 if peak>permitted+1e-12:errors.append([name,'commanded angular speed exceeded profile'])
result={'kind':'saved_entry_window_and_arm_command_consistency','trajectory_sha256':sha,'controller_whole_sha256':hashlib.sha256((p/'controller_source.py').read_bytes()).hexdigest(),'controller_function_bundle_sha256':r['controller_sha256'],'inspection_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'entry_observer_version':'native-settled-entry-support-v1','thresholds':{'window_s':duration,'relative_bolt_axial_velocity_limit_m_per_s':limit,'native_pair_normal_force_threshold_N':force_min,'normal_impulse_threshold_Ns':impulse_min,'loaded_duration_threshold_s':loaded_min},'saved_rows_checked':len(rows),'ready_rows_checked':ready_rows,'events':events,'entry_time_before_first_starting_rotation':entry_time<first_start,'entry_time_s':entry_time,'first_starting_row_time_s':first_start,'axial_damping_commands':{'count':len(commands),'coefficient_Ns_per_m':B,'max_abs_F_plus_B_v_residual_N':float(res.max(initial=0.)),'max_force_times_relative_velocity_W':float((f*v).max(initial=0.))},'sampled_commanded_angular_speed_peaks_rad_s':peaks,'all_consistency_checks_passed':not errors,'discrepancies':errors,'source_review':'Archived EntrySupportWindow.observe/report, pre-opening/reset guards and bounded dwell logic were inspected statically. This remains separate from unchanged formed-flank capture.','timing_scope':'Original native force/derived geometry and entry observer inputs correspond to reported time minus dt; saved qpos/qvel are post-integration. Original commands were computed before integration.','limitations':'Checks original saved observer aggregates/current inputs/event chronology and command algebra. Every-substep native thread-pair force/bolt-speed window samples were not separately archived, so impulse/duty/range cannot be independently reaggregated from sparse poses. Pair-normal impulse and closed-jaw axial settling do not establish axial wrench balance, unsupported holding, or realized actuator passivity. Actual open-reset/contact and formed-capture/lead proofs remain separate.'}
(p/'independent_entry_command_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
assert not errors,errors[:10]
print(json.dumps({'rows':len(rows),'ready_rows':ready_rows,'events':len(events),'all_consistency_checks_passed':not errors,'commanded_speed_peaks':peaks},indent=2))
