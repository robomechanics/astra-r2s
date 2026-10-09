"""Original dense native direction-search history; no state/force replay."""
from pathlib import Path
import argparse,hashlib,json,os
os.environ.setdefault('MPLCONFIGDIR','/workspace/.cache/matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(a):
    run=a.run.resolve();output=a.output.resolve();assert not output.exists()
    report=json.loads((run/'insertion_validation.json').read_text());event=next(e for e in report['physical_motion_events'] if e['event']=='Actual post-ramp settled seat reference')
    table=run/report['table_support_force_history']['filename'];feedback=run/report['native_feedback_force_history']['filename']
    assert sha(table)==report['table_support_force_history']['sha256'] and sha(feedback)==report['native_feedback_force_history']['sha256']
    with np.load(table,allow_pickle=False) as z:
        time=z['time'].copy();base=z['bolt_base_insertion_m'].copy();yaw=z['bolt_yaw_unwrapped_rad'].copy();formed=z['formed_flank_overlap_m'].copy()
    with np.load(feedback,allow_pickle=False) as z:
        assert np.array_equal(time,z['time']);thread=z['thread_gravity_opposing_force_N'].copy();hand=z['hand_gravity_opposing_force_N'].copy();interior=z['loaded_actual_interior_flank_contact_count'].copy();weight_metadata=json.loads(str(z['metadata_json']))
    rows=np.flatnonzero(time>=event['time_s']-1e-9);begin=rows[0];drop=(base-event['base_z_m'])*1e6;threshold=report['aborted']['seat_direction_event']['minimum_actual_drop_m']*1e6
    assert abs(drop[-1]-report['aborted']['seat_direction_event']['actual_axial_drop_m']*1e6)<1e-6
    mg=report['known_bolt_mass_kg']*9.81
    fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained');t=time[rows]
    ax[0].plot(t,drop[rows],label='Deeper (+) / withdrawn (−) relative to original settled reference')
    ax[0].axhline(threshold,ls='--',color='#c34e36',label='Unchanged 50 µm direction gate');ax[0].axhline(0,color='gray',ls=':')
    ax[0].plot([t[-1]],[drop[-1]],'o',color='#c34e36');ax[0].set_ylabel('Original axial displacement (µm)');ax[0].legend(loc='lower left')
    ax[0].text(.02,.97,f'Final {drop[-1]:.4f} µm; maximum deeper gain {drop[rows].max():.4f} µm < 50 µm\nWithdrawal/crest return is not redefined as original drop or capture',transform=ax[0].transAxes,va='top',fontsize=10)
    ax[1].plot(t,yaw[rows]-yaw[begin],label='Original unwrapped bolt yaw − original reference-row yaw');ax[1].set_ylabel('Original relative yaw (rad)');ax[1].legend(loc='lower left')
    ax[2].plot(t,thread[rows]/mg,label='Original native thread support / bolt weight',lw=.7)
    ax[2].plot(t,np.maximum(hand[rows],0)/mg,label='Original positive hand support / bolt weight',lw=.7);ax[2].axhline(1,color='gray',ls=':')
    ax[2].set_ylabel('Original solve (mg)');ax[2].set_xlabel('Original logged native time t (s); force and derived geometry at t − 50 µs');ax[2].legend(loc='upper right')
    for panel in ax:panel.grid(alpha=.2)
    fig.suptitle('Fresh continuous native attempt: direction gate FAILED\nOriginal settled reference retained; formed overlap / conservative interior counts remain zero',fontsize=14)
    output.mkdir(parents=True);fig.savefig(output/'native_direction_gate.png',dpi=150);plt.close(fig)
    (output/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    manifest={'scope':'Scientific plot of unchanged original all-step native table/feedback ledgers after actual settled reference. No integration, forward/collision/force solve, sparse-q inference, retiming or changed direction/capture criterion.',
        'original_table_ledger_sha256':sha(table),'original_feedback_ledger_sha256':sha(feedback),'original_report_sha256':sha(run/'insertion_validation.json'),
        'native_dense_rows_total':len(time),'native_dense_rows_after_reference':len(rows),'original_reference_event':event,'original_reference_row':int(begin),
        'original_reference_geometry_time_s':event['time_s']-50e-6,'original_reference_base_z_m':event['base_z_m'],'plotted_axial_measurement':'1000000*(original bolt_base_insertion_m − original settled event base_z_m); positive is deeper gain, negative withdrawal',
        'final_actual_drop_um':float(drop[-1]),'maximum_actual_deeper_gain_um':float(drop[rows].max()),'maximum_withdrawal_below_reference_um':float(-drop[rows].min()),
        'unchanged_minimum_drop_um':threshold,'loaded_interior_count_max_after_reference':int(interior[rows].max()),'formed_overlap_max_after_reference_m':float(formed[rows].max()),
        'force_and_geometry_timing':'Original preintegration force/retained solved geometry at logged t−50us. No saved post-q replay forces. Yaw retains original sign convention.',
        'native_feedback_metadata_original':weight_metadata,'plot_sha256':sha(output/'native_direction_gate.png'),'plot_source_sha256':sha(__file__),
        'capture_or_direction_gate_redefined':False,'capture_qualified':False,'full_trajectory_completed':False}
    (output/'plot_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n');print(json.dumps({k:manifest[k] for k in ['final_actual_drop_um','maximum_actual_deeper_gain_um','maximum_withdrawal_below_reference_um','native_dense_rows_after_reference']},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('output',type=Path);main(p.parse_args())
