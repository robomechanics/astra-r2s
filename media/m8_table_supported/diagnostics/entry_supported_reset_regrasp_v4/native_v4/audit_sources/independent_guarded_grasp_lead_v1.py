"""Original all-step guard/lead audit for cold entry-search branches; no simulation."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def expected_grasp_active(times,labels,acquisition_time):
    closed=np.array([p in {'start_thread_2','stop_start_2'}for p in labels])
    # The producer caches guard-active before acquisition on this solve.
    return closed | (np.asarray(times)>acquisition_time)


def measure_guarded_grasp(raw,active):
    if not np.any(active):raise ValueError('No original guarded-grasp tick was executed.')
    return {'original_guarded_tick_count':int(np.sum(active)),
        'maximum_original_translation_slip_m':float(np.max(raw['right_grip_slip_m'][active])),
        'maximum_original_rotation_slip_rad':float(np.max(raw['right_grip_rotation_slip_rad'][active])),
        'minimum_original_bilateral_pad_force_N':[float(np.min(raw[k][active]))for k in ('right_pad_0_N','right_pad_1_N')],
        'every_original_guarded_tick_retains_1mm_2deg_loaded_pads':bool(np.all(raw['right_grip_slip_m'][active]<=.001)
            and np.all(raw['right_grip_rotation_slip_rad'][active]<=np.deg2rad(2))
            and np.all(raw['right_pad_0_N'][active]>.1) and np.all(raw['right_pad_1_N'][active]>.1))}


def audit(path):
    path=Path(path)
    with np.load(path/'original_native_force_ledger.npz',allow_pickle=False)as f:raw={k:f[k].copy()for k in f.files}
    with np.load(path/'checkpoint_trace.npz',allow_pickle=False)as f:
        saved=json.loads(str(f['info_json'].item()));metadata=json.loads(str(f['metadata_json'].item()))
    report=json.loads((path/'report.json').read_text());decl=json.loads((path/'declaration.json').read_text())
    dt=float(ET.fromstring((path/'scene.xml').read_text()).find('option').get('timestep'))
    pitch=float(ET.fromstring((path/'scene.xml').read_text()).find("extension/plugin/instance[@name='bolt_shape']/config[@key='pitch']").get('value'))
    labels=np.array([metadata['phase_schedule'][int(i)][0]for i in raw['phase_index']])
    acquisition=report['physical_closed_turn']['actual_regrasp_acquisition']
    at=float(acquisition['time_s']);ids=np.flatnonzero(raw['time_s']==at)
    assert len(ids)==1,'Measured original acquisition solve must be in all-step ledger.'
    ai=int(ids[0]);active=raw['right_grasp_guard_active']==1
    assert np.array_equal(active,expected_grasp_active(raw['time_s'],labels,at))
    assert not active[ai] and active[ai+1]
    assert acquisition['continuous_quiet_bilateral_streak_s']>=.1-1e-12
    assert np.linalg.norm(acquisition['grasp_relative_bolt_head_position_m'])<.001
    assert min(acquisition['pad_normal_force_N'])>.1
    assert (path/'diagnostic_source.py').read_text().index('grasp_guard_active=') < (path/'diagnostic_source.py').read_text().index("if new_acquisition is None and regrasp_streak_s")
    grip=measure_guarded_grasp(raw,active);assert grip['every_original_guarded_tick_retains_1mm_2deg_loaded_pads']
    # Acquisition times are source pre-force transforms t-dt; sparse saved qpost
    # normally omits this exact off-grid solve in this historical cold harness.
    exact_saved=[r for r in saved if r['time_s']==at]
    grip.update({'measured_acquisition':acquisition,'original_acquisition_native_index':ai,
        'original_acquisition_elapsed_s':float(raw['elapsed_s'][ai]),
        'acquisition_row_uses_old_reference_and_guard_is_inactive':True,
        'new_reference_guard_starts_next_native_tick':True,
        'exact_acquisition_post_state_archived':bool(exact_saved),
        'global_including_intentionally_open_translation_slip_m':float(np.max(raw['right_grip_slip_m'])),
        'scope':'Only active native closed-grasp ticks use the original per-acquisition reference. Global intentional open/reindex displacement is separately reported. All original slip inputs describe retained solved pose at time-dt, not sampled qpost replay.'})
    opened=raw['fully_open_unassisted']==1
    assert np.any(opened)
    assert np.all(raw['right_bolt_contact_count'][opened]==0)
    assert np.all(raw['hand_gravity_opposing_force_N'][opened]==0)
    assert np.all(raw['right_applied_axial_feed_N'][opened]==0)
    assert np.all(raw['right_axial_float_active'][opened]==0)
    open_report={'original_fully_open_ticks':int(np.sum(opened)),
        'whole_right_robot_to_bolt_contact_ticks':int(np.sum(raw['right_bolt_contact_count'][opened]>0)),
        'maximum_absolute_original_hand_upward_force_N':float(np.max(abs(raw['hand_gravity_opposing_force_N'][opened]))),
        'maximum_unassisted_axial_drift_m':float(np.max(raw['open_drift_peak_axial_m'][opened])),
        'maximum_unassisted_yaw_drift_rad':float(np.max(raw['open_drift_peak_yaw_rad'][opened])),
        'no_extra_axial_feed_or_closed_feedback_while_open':True,
        'scope':'Entry-supported SEARCH only. Unloaded whole-robot contact absence proves the moving robot provides no bolt contact support; this is not full-flank capture or qualified passive-reset reward.'}
    assert open_report['maximum_unassisted_axial_drift_m']<=10e-6 and open_report['maximum_unassisted_yaw_drift_rad']<=.02
    turn=np.flatnonzero(labels=='start_thread_2');assert len(turn)
    ref=int(turn[0])-1;end=int(turn[-1]);z=raw['base_z_m'];yaw=raw['yaw_unwrapped_rad']
    dz=z[turn]-z[ref];angle=yaw[turn]-yaw[ref];residual=dz-pitch*angle/(2*np.pi)
    lead={'original_reference_native_index':ref,'original_reference_elapsed_s':float(raw['elapsed_s'][ref]),
        'original_final_turn_native_index':end,'original_final_turn_elapsed_s':float(raw['elapsed_s'][end]),
        'actual_relative_turn_rad':float(yaw[end]-yaw[ref]),'actual_axial_advance_m':float(z[end]-z[ref]),
        'native_pitch_m':pitch,'pitch_expected_advance_from_actual_angle_m':float(pitch*(yaw[end]-yaw[ref])/(2*np.pi)),
        'final_pitch_residual_m':float(residual[-1]),'maximum_absolute_original_turn_pitch_residual_m':float(np.max(abs(residual))),
        'maximum_original_turn_withdrawal_m':float(max(0.,-np.min(dz))),
        'scope':'Both advance and unwrapped yaw come from simultaneous original retained solved geometry. Reference is the original solve immediately before actual forward-turn phase. The half-turn begins in partial entry; this measured pitch comparison does not satisfy a full-pitch capture/qualified-lead proof.'}
    tail=np.flatnonzero(raw['elapsed_s']>raw['elapsed_s'][-1]-.1+dt/4)
    weight=float(decl['bolt_weight_N'])
    final={'original_native_ticks':len(tail),'duration_s':len(tail)*dt,
        'mean_thread_gravity_opposing_force_in_bolt_weight_units':float(np.mean(raw['thread_gravity_opposing_force_N'][tail])/weight),
        'mean_positive_hand_upward_force_in_bolt_weight_units':float(np.mean(np.maximum(raw['hand_gravity_opposing_force_N'][tail],0.))/weight),
        'loaded_thread_support_duty':float(np.mean(raw['thread_gravity_opposing_force_N'][tail]>.1*weight)),
        'loaded_actual_interior_duty':float(np.mean(raw['loaded_actual_interior_flank_contact_count'][tail]>0)),
        'maximum_actual_bolt_angular_speed_rad_s':float(np.max(raw['relative_bolt_angular_speed_rad_per_s'][tail])),
        'maximum_actual_hand_angular_speed_rad_s':float(np.max(raw['relative_hand_angular_speed_rad_per_s'][tail])),
        'maximum_actual_absolute_axial_speed_m_s':float(np.max(abs(raw['relative_bolt_axial_velocity_m_per_s'][tail]))),
        'final_formed_overlap_m':float(raw['formed_flank_overlap_m'][-1]),
        'final_loaded_actual_interior_contact_count':int(raw['loaded_actual_interior_flank_contact_count'][-1]),
        'maximum_formed_overlap_m':float(np.max(raw['formed_flank_overlap_m'])),
        'formed_overlap_reaches_full_pitch':bool(np.max(raw['formed_flank_overlap_m'])>=pitch),
        'scope':'Partial formed axial interval and actual conservative interior-contact force tags are different observations. Positive interior loading does not imply a full pitch of overlap.'}
    assert report['is_full_capture_or_qualified_reset'] is False
    return {'auditor_source_sha256':sha(__file__),'trace_sha256':sha(path/'checkpoint_trace.npz'),
        'ledger_sha256':sha(path/'original_native_force_ledger.npz'),
        'source_sha256':sha(path/'diagnostic_source.py'),'helper_sha256':sha(path/'observer_source.py'),
        'timestep_s':dt,'raw_native_ticks':len(z),'guarded_grasp':grip,'fully_open_unassisted':open_report,
        'actual_original_forward_half_turn':lead,'final_stopped_100ms':final,
        'all_original_raw_guards_held':bool(np.all(raw['all_checks_held']==1)),
        'external_drive_zero_every_tick':bool(np.all(raw['external_drive_zero']==1)),
        'scope':'Cold diagnostic branch only; no native replay/integration or original evidence alteration. Original pre-force geometry/wrenches and post-q saved state timing remain distinct. No continuous full-trajectory/capture/passive-reset qualification.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory);args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k]for k in ('guarded_grasp','actual_original_forward_half_turn','final_stopped_100ms')},indent=2))
