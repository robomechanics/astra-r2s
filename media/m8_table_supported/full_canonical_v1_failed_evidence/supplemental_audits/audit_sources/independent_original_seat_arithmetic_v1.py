"""Original dense seat-reference/crest arithmetic; no native imports or replay."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def measured_return(base,reference,minimum_return):
    base=np.asarray(base,dtype=float)
    crest=np.minimum.accumulate(base)
    return {'absolute_gain':base-reference,'return_from_observed_running_crest':base-crest,
        'local_direction_candidate':base-crest>=minimum_return}


def audit(run):
    run=Path(run)
    with np.load(run/'insertion_trace.npz',allow_pickle=False)as f:m=json.loads(str(f['metadata_json'].item()))
    with np.load(run/'table_support_force_history.npz',allow_pickle=False)as f:t={k:f[k].copy()for k in f.files}
    with np.load(run/'native_feedback_force_history.npz',allow_pickle=False)as f:r={k:f[k].copy()for k in f.files}
    reference=next(e for e in m['physical_motion_events']if e['event']=='Actual post-ramp settled seat reference')
    assert np.array_equal(t['time'],r['time'])and np.array_equal(t['phase_index'],r['phase_index'])
    labels=json.loads(str(t['phase_labels_json'].item()));names=np.asarray(labels,dtype=object)[t['phase_index'].astype(int)]
    dt=5e-5;indices=np.flatnonzero(np.isin(names,['reverse_seat_1','stop_reverse_seat_1']))
    base=t['bolt_base_insertion_m'][indices];relative=measured_return(base,reference['base_z_m'],50e-6)
    reference_index=int(np.searchsorted(t['time'],reference['time_s']));assert t['bolt_base_insertion_m'][reference_index]==reference['base_z_m']
    def sample(local_index):
        i=int(indices[local_index]);return {'original_native_index':i,'recorded_post_step_time_s':float(t['time'][i]),
            'original_force_geometry_time_s':float(t['time'][i]-dt),'phase':str(names[i]),
            'base_z_m':float(t['bolt_base_insertion_m'][i]),'actual_relative_yaw_rad':float(t['bolt_yaw_unwrapped_rad'][i]),
            'independent_commanded_clock_rad':float(r['desired_independent_clock_rad'][i]),
            'absolute_gain_from_settled_reference_m':float(relative['absolute_gain'][local_index]),
            'return_from_observed_running_crest_m':float(relative['return_from_observed_running_crest'][local_index]),
            'original_thread_normal_force_N':float(r['thread_summed_normal_force_N'][i]),
            'original_thread_upward_force_N':float(r['thread_gravity_opposing_force_N'][i]),
            'original_signed_hand_upward_force_N':float(r['hand_gravity_opposing_force_N'][i]),
            'actual_axial_speed_m_s':float(r['relative_bolt_axial_velocity_m_per_s'][i]),
            'actual_bolt_angular_speed_rad_s':float(r['relative_bolt_angular_speed_rad_per_s'][i]),
            'actual_hand_angular_speed_rad_s':float(r['relative_hand_angular_speed_rad_per_s'][i]),
            'formed_overlap_m':float(t['formed_flank_overlap_m'][i]),
            'loaded_actual_interior_contacts':int(r['loaded_actual_interior_flank_contact_count'][i]),
            'general_weight_window_ready':bool(r['weight_window_ready'][i])}
    crest_i=int(np.argmin(base));first=np.flatnonzero(relative['local_direction_candidate'])
    deeper=int(np.argmax(relative['absolute_gain']));ret_i=int(np.argmax(relative['return_from_observed_running_crest']))
    final=sample(len(indices)-1);original=m['aborted']['seat_direction_event']
    assert np.isclose(final['absolute_gain_from_settled_reference_m'],original['actual_axial_drop_m'],rtol=0.,atol=1e-15)
    assert not np.any(relative['absolute_gain']>=50e-6)
    assert original['stop_requested_from_measured_drop'] is False and original['confirmed_search_direction_event'] is False
    return {'scope':'Independent original dense scalar arithmetic only; transient local return is a possible NEW closed direction heuristic, not the frozen absolute-drop criterion, support/engagement/capture/open proof or continuous trajectory success.',
        'auditor_source_sha256':sha(__file__),'trace_sha256':sha(run/'insertion_trace.npz'),
        'table_ledger_sha256':sha(run/'table_support_force_history.npz'),'feedback_ledger_sha256':sha(run/'native_feedback_force_history.npz'),
        'measured_original_settled_reference':reference,'reference_original_native_index':reference_index,
        'reference_original_force_geometry_time_s':reference['time_s']-dt,
        'first_reverse_or_stop_native_index':int(indices[0]),'original_reverse_and_stop_ticks':len(indices),
        'maximum_withdrawal_from_settled_reference_m':float(-np.min(relative['absolute_gain'])),
        'maximum_true_deeper_gain_from_settled_reference_m':float(np.max(relative['absolute_gain'])),
        'maximum_return_from_observed_running_crest_m':float(np.max(relative['return_from_observed_running_crest'])),
        'original_absolute_50um_drop_ever_observed':False,'original_full_failure_preserved':m['aborted'],
        'measured_withdrawal_crest':sample(crest_i),'first_original_running_crest_50um_return':sample(int(first[0]))if len(first)else None,
        'maximum_true_deeper_gain_sample':sample(deeper),'maximum_running_crest_return_sample':sample(ret_i),
        'final_stopped_sample':final,
        'formed_overlap_maximum_during_reverse_stop_m':float(np.max(t['formed_flank_overlap_m'][indices])),
        'loaded_actual_interior_contact_ticks_during_reverse_stop':int(np.sum(r['loaded_actual_interior_flank_contact_count'][indices]>0)),
        'no_criteria_or_source_or_native_report_changed':True,
        'scientific_next_step_scope':'If a new version tracks an observed axial crest during reverse, local50um return may request braking while jaws stay closed. Confirm a stopped persistent return using unchanged native impulse/duty/endpoint and quiet guards before forward action. Continue to report absolute depth separately; never infer engagement from moving/unloaded first return, or permit opening without the separate physical support/capture criteria.'}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();d=audit(a.run);a.output.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n');print(json.dumps({k:d[k]for k in ['maximum_withdrawal_from_settled_reference_m','maximum_true_deeper_gain_from_settled_reference_m','maximum_return_from_observed_running_crest_m','first_original_running_crest_50um_return','final_stopped_sample']},indent=2))
