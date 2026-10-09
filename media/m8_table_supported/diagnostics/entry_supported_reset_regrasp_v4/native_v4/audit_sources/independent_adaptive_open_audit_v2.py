"""Bind a live original readiness event to actual saved and native phase clocks."""
import argparse
import json
from pathlib import Path

import numpy as np

from independent_open_search_audit_v1 import audit as original_open_audit, sha


def check_event(raw, declaration, metadata, report, saved, dt):
    event = report['physical_closed_turn']['adaptive_open_readiness_event']
    assert event == metadata['actual_adaptive_open_readiness_event']
    assert declaration['adaptive_open_settle'] is True
    i = round(event['elapsed_s']/dt)-1
    assert raw['elapsed_s'][i] == event['elapsed_s']
    assert raw['time_s'][i] == event['time_s']
    assert event['force_time_s'] == event['time_s']-dt
    assert event['first_reset_command_elapsed_s'] == event['elapsed_s']+dt
    release = declaration['phase_schedule'][0][1]
    assert event['actual_open_settle_s'] == event['elapsed_s']-release
    assert event['actual_open_settle_s'] >= declaration['minimum_open_settle_s']-dt/2
    assert event['actual_open_settle_s'] <= declaration['maximum_open_settle_s']+dt/2
    rows = [r for r in saved if r['elapsed_s'] == event['elapsed_s']]
    assert len(rows) == 1, 'Event native sample/post-state must be archived at its actual time.'
    row = rows[0]
    assert row['adaptive_open_ready_event'] == event
    assert row['open_weight_window'] == event['open_weight_window']
    assert row['open_weight_window']['ready_for_diagnostic_release_attempt'] is True
    assert row['physical_phase'] == 'open_settle_search_2'
    assert row['right_bolt_contact_count'] == 0 and row['all_checks_held']
    for key in ('time_s','relative_bolt_angular_speed_rad_per_s','relative_bolt_axial_velocity_m_per_s'):
        assert row[key] == raw[key][i]
    schedule = metadata['actual_shifted_phase_schedule']
    assert schedule == report['physical_closed_turn']['actual_shifted_phase_schedule']
    original = declaration['phase_schedule']
    assert len(original) == len(schedule)
    for j in range(len(original)):
        if j == 1:
            assert schedule[j] == ['open_settle_search_2',event['actual_open_settle_s']]
        else:
            assert schedule[j] == original[j]
    ends = np.cumsum([s[1] for s in schedule])
    phases = np.minimum(np.searchsorted(ends,raw['elapsed_s'],side='left'),len(schedule)-1)
    assert np.array_equal(phases, raw['phase_index']), 'Actual phase rows cannot use the old maximum clock.'
    assert schedule[int(raw['phase_index'][i])][0] == 'open_settle_search_2'
    assert schedule[int(raw['phase_index'][i+1])][0] == 'reset_open_search_2'
    assert raw['elapsed_s'][i+1] == event['first_reset_command_elapsed_s']
    assert metadata['actual_executed_native_duration_s'] == len(raw['time_s'])*dt
    assert metadata['actual_shifted_planned_duration_s'] == float(ends[-1])
    return dict(verified_event=event, event_original_native_index=i,
        event_saved_state_index=saved.index(row),
        event_original_native_force_and_post_state_saved_at_actual_time=True,
        first_reset_native_index=i+1, first_reset_command_elapsed_s=float(raw['elapsed_s'][i+1]),
        declared_maximum_schedule=original, actual_shifted_schedule=schedule,
        all_original_native_phase_indices_match_actual_clock=True,
        actual_executed_native_duration_s=float(raw['elapsed_s'][-1]),
        actual_shifted_planned_duration_s=float(ends[-1]),
        scope='The immutable maximum plan and actual event-shifted plan remain distinct. '
            'The event uses retained original pre-integration solved forces with an actual post-integration saved qpos/qvel. '
            'Reset starts one native tick later; no sample/state is retimestamped.')


def source_equivalent_open_readiness(raw,weight,dt):
    """Original timestamp-window predicate; no hand-speed gate during reset."""
    good=((raw['fully_open_unassisted']==1)&(raw['right_bolt_contact_count']==0)
        &(raw['all_checks_held']==1)&(raw['relative_bolt_angular_speed_rad_per_s']<=.01)
        &(abs(raw['relative_bolt_axial_velocity_m_per_s'])<=.0002)
        &(raw['external_drive_zero']==1)&(raw['bolt_world_support_contact_count']==0)
        &(raw['nonthread_block_bolt_contact_count']==0)&(raw['radial_offset_m']<=.000150)
        &(raw['bolt_tilt_rad']<=np.deg2rad(2)))
    time=raw['time_s'];index=np.arange(len(time))
    previous_bad=np.maximum.accumulate(np.where(good,-1,index))
    left=np.maximum(np.searchsorted(time,time-.1,side='right'),previous_bad+1)
    count=index-left+1
    def sums(values):
        prefix=np.r_[0.,np.cumsum(values,dtype=float)]
        return prefix[index+1]-prefix[left]
    denominator=np.maximum(count,1)*weight
    return (good & ((index-previous_bad)*dt+1e-9>=.1) & (count*dt+1e-9>=.1)
        &(sums(raw['thread_gravity_opposing_force_N'])/denominator>=.9)
        &(sums(np.maximum(raw['hand_gravity_opposing_force_N'],0.))/denominator<=.1)
        &(raw['thread_gravity_opposing_force_N']>.1*weight))


def first_current_ready_after_minimum(ready,elapsed,release,minimum_dwell,dt):
    ids=np.flatnonzero(np.asarray(ready,dtype=bool) &
        (np.asarray(elapsed)-release >= minimum_dwell-dt/2))
    return int(ids[0]) if len(ids) else None


def audit(path):
    path = Path(path)
    result = original_open_audit(path)
    with np.load(path/'original_native_force_ledger.npz',allow_pickle=False) as archive:
        raw = {k:archive[k].copy() for k in archive.files}
    with np.load(path/'checkpoint_trace.npz',allow_pickle=False) as archive:
        saved = json.loads(str(archive['info_json'].item()))
        metadata = json.loads(str(archive['metadata_json'].item()))
    declaration = json.loads((path/'declaration.json').read_text())
    report = json.loads((path/'report.json').read_text())
    dt=result['timestep_s']
    result['base_opening_auditor_sha256']=result.pop('auditor_source_sha256')
    result['auditor_source_sha256']=sha(__file__)
    timing=result['timing']
    failure_time=report['aborted']['elapsed_s'] if report['aborted'] else None
    if failure_time is not None and abs(failure_time-raw['elapsed_s'][-1])<1e-12:
        timing.pop('pre_step_abort_elapsed_s')
        timing['abort_after_original_native_step_elapsed_s']=failure_time
        timing['scope']=('Hard contact guard aborted after the retained original native solve/integration. '
            'Force/geometry belongs to time-dt; saved qpos/qvel is post-integration; no extra native step or replay.')
    result['adaptive_event_and_actual_clock']=check_event(raw,declaration,metadata,report,saved,dt)
    # Wider physical opening can become ready before the required minimum
    # dwell. Consume the first CURRENT source-equivalent ready solve after
    # that minimum, instead of equating it with the first ever ready solve.
    event=report['physical_closed_turn']['adaptive_open_readiness_event']
    ready_mask=source_equivalent_open_readiness(raw,declaration['bolt_weight_N'],dt)
    release=declaration['phase_schedule'][0][1]
    eligible=first_current_ready_after_minimum(ready_mask,raw['elapsed_s'],release,
        declaration['minimum_open_settle_s'],dt)
    assert eligible is not None and raw['elapsed_s'][eligible]==event['elapsed_s']
    result['minimum_eligible_original_event']={
        'first_source_equivalent_ready_elapsed_s':float(raw['elapsed_s'][np.flatnonzero(ready_mask)[0]]),
        'minimum_open_settle_s':declaration['minimum_open_settle_s'],
        'first_current_ready_after_minimum_elapsed_s':float(raw['elapsed_s'][eligible]),
        'event_consumes_that_exact_original_solve':True,
        'historical_v1_auditor_scope_note':'Frozen v1 assumed minimum dwell never excludes an earlier ready solve. New version corrects that event-eligibility assumption; original probe/helper/force bytes and prior failure reports remain unchanged.'}
    bad=np.flatnonzero(raw['right_bolt_contact_count']>0)
    openbad=np.flatnonzero((raw['fully_open_unassisted']==1)&(raw['right_bolt_contact_count']>0))
    result['original_open_robot_contact_observations']=dict(fully_open_robot_contact_ticks=len(openbad),
        first_open_contact_elapsed_s=float(raw['elapsed_s'][openbad[0]]) if len(openbad) else None,
        final_native_whole_right_contact_count=int(raw['right_bolt_contact_count'][-1]),
        final_native_right_pad_normals_N=[float(raw['right_pad_0_N'][-1]),float(raw['right_pad_1_N'][-1])],
        final_original_thread_upward_N=float(raw['thread_gravity_opposing_force_N'][-1]),
        final_original_signed_hand_upward_N=float(raw['hand_gravity_opposing_force_N'][-1]),
        final_commanded_aperture_m=float(raw['right_commanded_aperture_m'][-1]),
        final_commanded_tool_clock_rad=float(raw['desired_turn_theta_rad'][-1]),
        final_original_native_checks=saved[-1]['checks'],
        original_right_hand_wrench_on_bolt_world_N_Nm=saved[-1]['hand_wrench_on_bolt_world_N_Nm'],
        scope='Fully-open contacts are forbidden and reported separately from intentional later closed regrasp contacts. Any actual original abort remains unchanged. '
            'Individual original right pad contact-local solves were not archived; original hand world wrench and pad loads are available. '
            'No post-qpos collision/force reconstruction may replace the original solution.')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k]for k in('adaptive_event_and_actual_clock','original_open_robot_contact_observations','timing')},indent=2))
