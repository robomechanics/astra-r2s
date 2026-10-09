"""Scientific original dense cold-V3 brake boundary; no simulator."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(args):
    run, output = args.run.resolve(), args.output.resolve()
    assert not output.exists()
    report = json.loads((run/'insertion_validation.json').read_text())
    event = next(event for event in report['physical_motion_events']
        if event.get('event') == 'Live physical phase readiness consumed' and event.get('phase') == 'reverse_seat_1')
    request_time = event['time_s']
    request_crest = event['seat_direction_event']['actual_crest_base_z_m']
    assert event['seat_direction_event']['stop_requested_from_measured_return'] is True
    assert report['passed'] is False and report['aborted']['phase'] == 'stop_reverse_seat_1'
    feedback_path, table_path = run/'native_feedback_force_history.npz', run/'table_support_force_history.npz'
    before = {path:sha(path) for path in (feedback_path, table_path, run/'insertion_validation.json')}
    with np.load(feedback_path, allow_pickle=False) as z:
        feedback = {name:z[name].copy() for name in z.files if name not in ('metadata_json','phase_labels_json')}
        phase_labels = json.loads(str(z['phase_labels_json']))
    with np.load(table_path, allow_pickle=False) as z:
        assert np.array_equal(z['time'], feedback['time'])
        base_z = z['bolt_base_insertion_m'].copy()
    times = feedback['time']
    assert len(times) == 32508 == report['native_feedback_force_history']['observed_physics_steps']
    indices = np.flatnonzero(times >= request_time-.012-1e-12)
    request_index = int(np.flatnonzero(abs(times-request_time)<1e-10)[0])
    stop_indices = np.flatnonzero(feedback['phase_index'] == phase_labels.index('stop_reverse_seat_1'))
    first_stop = int(stop_indices[0])
    assert first_stop == request_index+1
    dt = float(times[first_stop]-times[request_index])
    force_cap = report['control_config']['arm']['maximum_cartesian_force']
    torque_cap = report['control_config']['arm']['maximum_cartesian_torque']
    command_wrench = feedback['right_command_wrench_N_Nm']
    force_norm = np.linalg.norm(command_wrench[:,:3], axis=1)
    torque_norm = np.linalg.norm(command_wrench[:,3:], axis=1)
    mg = report['known_bolt_mass_kg']*9.81
    x = (times[indices]-request_time)*1e3
    crest_return = (base_z-request_crest)*1e6
    fig, axes = plt.subplots(5, 1, figsize=(12.8, 12), sharex=True,
        gridspec_kw=dict(height_ratios=[1.15,1.1,1.1,1,1]))
    fig.suptitle('Cold V3 failure: abrupt CLOSED stop → radial guard abort', fontsize=17, x=.5, y=.984)
    fig.text(.5, .949,
        'Original32,508 native ticks · local request1.61915s · first stop command+50µs · abort+6.25ms\n'
        'No quiet-stop confirmation / forward scan / formed capture / opening', ha='center', va='top', fontsize=11)
    for ax in axes:
        ax.axvline(0, color='#555555', lw=1, ls='--')
        ax.axvspan(0, (times[-1]-request_time)*1e3, color='#cf675b', alpha=.075)
        ax.grid(True, alpha=.22)
        ax.spines[['top','right']].set_visible(False)
    axes[0].plot(x, feedback['desired_independent_angular_speed_rad_s'][indices], label='Command ω (signed independent clock)', color='#2a6a9e')
    axes[0].plot(x, feedback['relative_bolt_angular_speed_rad_per_s'][indices], label='Actual |ωbolt−block| (magnitude)', color='#d68b29')
    axes[0].plot(x, feedback['relative_hand_angular_speed_rad_per_s'][indices], label='Actual |ωhand−block| (magnitude)', color='#52855c', ls=':')
    axes[0].set_ylabel('Angular speed\nrad/s')
    axes[0].legend(loc='center left', bbox_to_anchor=(1.01,.5), fontsize=8)
    axes[1].plot(x, feedback['radial_offset_m'][indices]*1e6, color='#a73c34', label='Original solved radial offset')
    axes[1].axhline(150, color='#6b3030', ls='--', lw=1.2, label='Unchanged150µm guard')
    axes[1].scatter([(times[-1]-request_time)*1e3], [feedback['radial_offset_m'][-1]*1e6], color='#a73c34', marker='x', zorder=5)
    axes[1].set_ylabel('Radial offset\nµm')
    axes[1].legend(loc='center left', bbox_to_anchor=(1.01,.5), fontsize=8)
    axes[2].plot(x, force_norm[indices]/force_cap, label=f'Cartesian |force| / {force_cap:g}N cap', color='#5c61a0')
    axes[2].plot(x, torque_norm[indices]/torque_cap, label=f'Cartesian |torque| / {torque_cap:g}N·m cap', color='#52855c')
    axes[2].axhline(1, color='#555555', ls='--', lw=1)
    axes[2].set_ylabel('Command norm\nfraction of cap')
    axes[2].legend(loc='center left', bbox_to_anchor=(1.01,.5), fontsize=8)
    axes[3].plot(x, crest_return[indices], color='#327389', label='Native z − retained REQUEST crest z')
    axes[3].axhline(50, color='#555555', ls='--', label='50µm CLOSED stop-request rule')
    axes[3].set_ylabel('Return from prior\nrequest crest, µm')
    axes[3].legend(loc='center left', bbox_to_anchor=(1.01,.5), fontsize=8)
    axes[4].plot(x, feedback['thread_gravity_opposing_force_N'][indices]/mg, color='#327389', label='Original thread upward / mg')
    axes[4].plot(x, np.maximum(feedback['hand_gravity_opposing_force_N'][indices],0)/mg, color='#d68b29', label='Original positive hand upward / mg')
    axes[4].set_ylabel('Native bolt\nweight fractions')
    axes[4].set_xlabel('Original dense ledger row time minus1.61915s stop-request event, ms')
    axes[4].legend(loc='center left', bbox_to_anchor=(1.01,.5), fontsize=8)
    fig.text(.05, .045,
        'Force / retained geometry: original solve at rowt−50µs. Commands use prior retained kinematics at rowt−100µs;\n'
        'saved qpos and actual aperture: post-step rowt. No force reconstruction / integration / interpolation.\n'
        'Final original observer cleared its epoch after the invalid radial sample; the prior request reference above is\n'
        'retained metadata for diagnosis only. Current request/confirmation flags remain false; no retrospective pass.', fontsize=9)
    fig.subplots_adjust(left=.09, right=.74, top=.91, bottom=.15, hspace=.22)
    output.mkdir(parents=True)
    image_path = output/'native_stop_boundary.png'
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    for path, digest in before.items():
        assert sha(path) == digest, path
    summary = {
        'scope':'Scientific plot from original dense solved native ledgers only; no simulator, integration or sparse-q force inference. Failed physical gate and final cleared observer flags retained.',
        'original_feedback_sha256':before[feedback_path], 'original_table_ledger_sha256':before[table_path],
        'original_report_sha256':before[run/'insertion_validation.json'],
        'original_native_rows':len(times), 'plotted_original_row_indices':indices.tolist(),
        'request_row_index':request_index, 'first_stop_row_index':first_stop,
        'request_time_s':request_time, 'first_stop_command_time_s':float(times[first_stop]),
        'abort_time_s':float(times[-1]), 'command_transition_dt_s':dt,
        'command_omega_before_rad_s':float(feedback['desired_independent_angular_speed_rad_s'][request_index]),
        'command_omega_first_stop_rad_s':float(feedback['desired_independent_angular_speed_rad_s'][first_stop]),
        'radial_at_request_um':float(feedback['radial_offset_m'][request_index]*1e6),
        'radial_at_abort_um':float(feedback['radial_offset_m'][-1]*1e6),
        'original_radial_guard_um':150., 'retained_request_crest_base_z_m':request_crest,
        'request_crest_return_um':event['seat_direction_event']['actual_crest_return_m']*1e6,
        'force_cap_N':force_cap, 'torque_cap_Nm':torque_cap,
        'max_boundary_command_force_N':float(force_norm[indices].max()),
        'max_boundary_command_torque_Nm':float(torque_norm[indices].max()),
        'original_failed_guard_rows':int(np.count_nonzero(~feedback['all_hard_guards_held'])),
        'original_stop_event_metadata':event,
        'original_final_cleared_event':report['final_experimental_crest_direction_event'],
        'source_sha256':sha(__file__), 'plot_sha256':sha(image_path),
        'force_state_command_timing':'Original solve/retained geometry rowt−50µs; post-step qpos/aperture rowt; command calibration prior retained solve rowt−100µs, with first-command initialization exception.'}
    shutil.copyfile(__file__, output/Path(__file__).name)
    (output/'plot_manifest.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key:summary[key] for key in ('request_time_s','abort_time_s',
        'radial_at_request_um','radial_at_abort_um','command_omega_before_rad_s',
        'command_omega_first_stop_rad_s','max_boundary_command_force_N',
        'max_boundary_command_torque_Nm','original_failed_guard_rows','plot_sha256')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
