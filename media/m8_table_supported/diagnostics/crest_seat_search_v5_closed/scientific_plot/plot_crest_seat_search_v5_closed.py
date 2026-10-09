"""Plot original dense closed native V5 observations; never invoke an engine."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main(args):
    run,out=args.run.resolve(),args.output.resolve()
    if out.exists():raise FileExistsError(out)
    report=json.loads((run/'insertion_validation.json').read_text())
    assert report['diagnostic_completed'] is True and report['aborted'] is None
    assert report['full_fresh_trajectory_qualified'] is False and report['capture_or_open_reset_qualified'] is False
    names=('native_feedback_force_history.npz','table_support_force_history.npz',
        'robot_inertia_command_history.npz','insertion_validation.json')
    before={name:sha(run/name) for name in names}
    with np.load(run/names[0],allow_pickle=False) as f:
        keys=('time','phase_index','desired_independent_clock_rad','desired_independent_angular_speed_rad_s',
            'relative_bolt_angular_speed_rad_per_s','relative_hand_angular_speed_rad_per_s',
            'radial_offset_m','bolt_tilt_rad','relative_bolt_axial_velocity_m_per_s',
            'thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','left_downward_feed_N')
        feedback={k:f[k].copy() for k in keys}
        labels=json.loads(f['phase_labels_json'].item())
    with np.load(run/names[1],allow_pickle=False) as f:
        assert np.array_equal(f['time'],feedback['time'])
        table={k:f[k].copy() for k in ('bolt_base_insertion_m','bolt_yaw_unwrapped_rad',
            'formed_flank_overlap_m','table_wrench_world_at_block_origin_N_Nm',
            'left_hand_wrench_world_at_block_origin_N_Nm')}
    with np.load(run/names[2],allow_pickle=False) as f:
        assert np.array_equal(f['time'],feedback['time'])
        commands={k:f[k].copy() for k in ('pd_wrench_world_N_Nm','ff_wrench_world_N_Nm',
            'capped_wrench_world_N_Nm','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped')}
    times=feedback['time'];n=len(times)
    assert n==147132 and np.all(np.diff(times)>0)
    boundaries=[]
    for i in range(1,len(labels)):
        rows=np.flatnonzero(feedback['phase_index']==i)
        if len(rows):boundaries.append((times[rows[0]],labels[i]))
    mg=report['known_bolt_mass_kg']*9.81
    block_mg=report['known_block_weight_N']
    half=report['scene_config']['base']['block_size'][2]/2
    shaft=report['scene_config']['base']['thread']['bolt_length']
    nominal=table['bolt_base_insertion_m']+shaft+half
    fig,axes=plt.subplots(7,1,figsize=(13,14),sharex=True)
    fig.suptitle('Closed cold V5: real braking, stopped entry support, then first forward scan',fontsize=16,y=.985)
    for ax in axes:
        ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
        for t,label in boundaries:ax.axvline(t,color='#778899',lw=.6,ls=':')
    axes[0].plot(times,feedback['desired_independent_clock_rad'],label='Independent robot clock',lw=1.2)
    axes[0].plot(times,table['bolt_yaw_unwrapped_rad']-table['bolt_yaw_unwrapped_rad'][0],label='Actual bolt yaw change',lw=.9)
    axes[0].set_ylabel('Clock / yaw\nrad');axes[0].legend(fontsize=9)
    axes[1].plot(times,nominal*1e3,label='Actual nominal tip insertion',color='#29768b')
    axes[1].plot(times,table['formed_flank_overlap_m']*1e3,label='Conservative formed overlap',color='#bb7744')
    axes[1].axhline(1.25,color='#884444',ls='--',lw=.8,label='Full-pitch formed capture threshold (1.25 mm)')
    axes[1].set_ylabel('Measured depth\nmm');axes[1].legend(fontsize=8,loc='lower right')
    axes[2].plot(times,feedback['radial_offset_m']*1e6,color='#98574d',label='Actual radial error')
    axes[2].axhline(150,color='#884444',ls='--',lw=.8,label='Unchanged hard guard')
    axes[2].set_ylabel('Radial error\nµm');axes[2].legend(fontsize=8)
    axes[3].plot(times,feedback['relative_bolt_axial_velocity_m_per_s']*1e3,color='#467596')
    axes[3].axhline(.2,color='#888888',ls=':',lw=.7);axes[3].axhline(-.2,color='#888888',ls=':',lw=.7)
    axes[3].set_ylabel('Actual axial\nvelocity, mm/s')
    axes[4].plot(times,feedback['thread_gravity_opposing_force_N']/mg,label='Native thread upward / bolt weight',lw=.55)
    axes[4].plot(times,np.maximum(feedback['hand_gravity_opposing_force_N'],0)/mg,label='Native positive hand upward / bolt weight',lw=.55)
    axes[4].set_ylabel('Original native\nweight fractions');axes[4].legend(fontsize=8)
    axes[5].plot(times,table['table_wrench_world_at_block_origin_N_Nm'][:,2]/block_mg,label='Native table upward / block weight',lw=.8)
    axes[5].plot(times,np.maximum(table['left_hand_wrench_world_at_block_origin_N_Nm'][:,2],0)/block_mg,label='Native positive left upward / block weight',lw=.8)
    axes[5].set_ylabel('Table / left\nweight fractions');axes[5].legend(fontsize=8)
    for key,label in (('pd_wrench_world_N_Nm','PD'),('ff_wrench_world_N_Nm','FF'),('capped_wrench_world_N_Nm','Capped total')):
        axes[6].plot(times,np.linalg.norm(commands[key][:,:3],axis=1),label=label,lw=.8)
    axes[6].axhline(8,color='#888888',ls='--',lw=.8,label='Unchanged 8 N combined cap')
    axes[6].set_ylabel('Executed command\nforce norm, N');axes[6].legend(fontsize=8,ncol=4)
    axes[6].set_xlabel('Local native row time, seconds (force/geometry at row time minus 50 µs)')
    fig.text(.06,.019,'147,132 original native ticks; no interpolation or force reconstruction. Command cache precedes force state by one native step.\nFinal formed overlap 21.615 µm, loaded interior 0: no captured thread, OPEN/reset, full fresh trajectory or policy qualification.',fontsize=10)
    fig.tight_layout(rect=(0,.055,1,.967));out.mkdir(parents=True)
    fig.savefig(out/'native_closed_trajectory.png',dpi=150);plt.close(fig)
    # Exact new-brake phase, including its actual quiet settling; no prediction
    # or substitution of historical V4 states into this trajectory.
    request=next(e for e in report['physical_motion_events'] if e.get('event')=='Measured crest return requests C2 CLOSED robot-yaw braking')
    mask=(times>=request['time_s']-.02)&(times<=request['time_s']+.4)
    x=(times[mask]-request['time_s'])*1e3
    fig,axes=plt.subplots(3,1,figsize=(11,7),sharex=True)
    for ax in axes:
        ax.grid(alpha=.2);ax.axvspan(0,150,alpha=.08,color='#467596');ax.axvline(150,ls=':',lw=.8,color='#666666')
    axes[0].plot(x,feedback['desired_independent_angular_speed_rad_s'][mask],label='Command signed ω')
    axes[0].plot(x,feedback['relative_bolt_angular_speed_rad_per_s'][mask],label='Actual |bolt ω|')
    axes[0].plot(x,feedback['relative_hand_angular_speed_rad_per_s'][mask],label='Actual |hand ω|',ls=':')
    axes[0].set_ylabel('Angular speed\nrad/s');axes[0].legend(fontsize=8)
    axes[1].plot(x,feedback['radial_offset_m'][mask]*1e6,label='Original native radial error')
    axes[1].axhline(150,ls='--',color='#884444',label='Unchanged 150 µm guard')
    axes[1].set_ylabel('Radial error\nµm');axes[1].legend(fontsize=8)
    for key,label in (('pd_wrench_world_N_Nm','PD'),('ff_wrench_world_N_Nm','FF'),('capped_wrench_world_N_Nm','Total')):
        axes[2].plot(x,np.linalg.norm(commands[key][mask,:3],axis=1),label=label)
    axes[2].axhline(8,color='#888888',ls='--');axes[2].set_ylabel('Command force\nN');axes[2].legend(fontsize=8)
    axes[2].set_xlabel('Original row time minus measured stop request, ms')
    fig.suptitle('Actual 150 ms C2 braking + fresh stopped native load/quiet window')
    fig.tight_layout();fig.savefig(out/'native_braking.png',dpi=150);plt.close(fig)
    manifest={'scope':'Original dense native scalar/matrix ledger plots only. No solver invocation, force replay, interpolation, phase splicing or capture inference.',
        'native_ticks':n,'last_native_time_s':float(times[-1]),'source_sha256':sha(__file__),
        'original_input_sha256':before,'originals_unchanged':before=={name:sha(run/name) for name in names},
        'all_cartesian_force_clip_flags':int(commands['cartesian_force_clipped'].sum()),
        'all_cartesian_torque_clip_flags':int(commands['cartesian_torque_clipped'].sum()),
        'all_motor_clip_flags':int(commands['motor_clipped'].sum()),
        'maximum_radial_error_um':float(feedback['radial_offset_m'].max()*1e6),
        'full_fresh_trajectory_qualified':False,'capture_or_open_reset_qualified':False,
        'plot_sha256':{p.name:sha(p) for p in out.glob('*.png')}}
    assert manifest['originals_unchanged']
    (out/'plot_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    shutil.copy2(__file__,out/Path(__file__).name)
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    main(p.parse_args())
