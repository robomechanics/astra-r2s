"""Scientific plot of original closed V4 all-step native ledger, no replay."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(a):
    directory=a.trial_dir.resolve();output=a.output.resolve()
    if output.exists():raise FileExistsError(output)
    raw_path=directory/'original_native_force_ledger.npz';report_path=directory/'report.json'
    report=json.loads(report_path.read_text());declaration=json.loads((directory/'declaration.json').read_text())
    assert digest(raw_path)==report['ledger_sha256']
    with np.load(raw_path,allow_pickle=False) as z:raw={k:z[k].copy() for k in z.files}
    with np.load(directory/'checkpoint_trace.npz',allow_pickle=False) as z:metadata=json.loads(str(z['metadata_json']))
    assert np.all(raw['all_checks_held']==1)
    t=raw['elapsed_s'];mg=declaration['bolt_weight_N'];mask=raw['right_grasp_guard_active'].astype(bool)
    opened=raw['fully_open_unassisted'].astype(bool)
    guarded={'native_ticks':int(mask.sum()),'maximum_translation_slip_m':float(raw['right_grip_slip_m'][mask].max()),'maximum_rotation_slip_rad':float(raw['right_grip_rotation_slip_rad'][mask].max())}
    assert guarded['maximum_translation_slip_m']<=.001 and guarded['maximum_rotation_slip_rad']<=np.deg2rad(2)
    assert np.all(raw['right_bolt_contact_count'][opened]==0)
    fig,ax=plt.subplots(4,1,figsize=(12,10),sharex=True,layout='constrained')
    ax[0].plot(t,raw['formed_flank_overlap_m']*1e3,label='Measured formed overlap',color='#225ea8')
    ax[0].axhline(1.25,color='#c34e36',ls='--',label='One M8 pitch (capture remains unqualified)')
    ax[0].set_ylabel('Formed overlap (mm)');ax[0].legend(loc='upper left')
    ax[1].plot(t,raw['thread_gravity_opposing_force_N']/mg,label='Original thread support / bolt weight',lw=.7)
    ax[1].plot(t,np.maximum(0,raw['hand_gravity_opposing_force_N'])/mg,label='Positive hand support / bolt weight',lw=.7)
    ax[1].axhline(1,color='gray',ls=':');ax[1].set_ylabel('Original native solve (mg)');ax[1].legend(loc='upper left')
    ax[2].plot(t,raw['right_actual_aperture_postintegration_m']*1e3,label='Actual saved post-step jaw aperture')
    ax[2].plot(t,raw['right_commanded_aperture_m']*1e3,label='Bounded physical jaw command',ls='--')
    ax[2].set_ylabel('Jaw aperture (mm)');ax[2].legend(loc='upper right')
    ax[3].plot(t,raw['right_bolt_contact_count'],label='Original whole-right / bolt contacts')
    ax[3].plot(t,raw['loaded_actual_interior_flank_contact_count'],label='Original loaded interior contacts')
    ax[3].fill_between(t,0,1,where=opened,transform=ax[3].get_xaxis_transform(),color='#58a979',alpha=.13,label='Fully open: all whole-right contacts zero')
    ax[3].set_ylabel('Original contact counts');ax[3].set_xlabel('Cold local native elapsed time (s)');ax[3].legend(loc='upper left')
    event=metadata['actual_adaptive_open_readiness_event']
    for panel in ax:
        panel.axvline(event['elapsed_s'],color='#4c8b56',ls=':',lw=1)
        panel.grid(alpha=.2)
    fig.suptitle('Closed cold V4: native open / −π reset / quiet regrasp / next π\nAll original guards held; 646.637 µm formed, full-pitch capture unqualified',fontsize=14)
    output.mkdir(parents=True);fig.savefig(output/'native_open_reset_history.png',dpi=150);plt.close(fig)
    source=Path(__file__);(output/source.name).write_bytes(source.read_bytes())
    manifest={'scope':'All original 177062 native scalar rows of one closed cold V4. No integration, force recomputation, resampling, or splice. Curves preserve original values; only display-unit conversions and positive-hand clipping apply.',
        'original_ledger_sha256':digest(raw_path),'original_report_sha256':digest(report_path),'source_sha256':digest(source),
        'native_rows':len(t),'native_duration_s':report['actual_duration_s'],'first_elapsed_s':float(t[0]),'last_elapsed_s':float(t[-1]),
        'force_time_scope':'Original preintegration contact solves at logged time−50us. Actual aperture, alignment/overlap are saved postintegration state metrics; no causal identity implied.',
        'actual_open_readiness_event':event,'guarded_grasp_slip':guarded,
        'global_unmasked_slip_scope':'Intentional opening suspends cumulative closed-grasp guards. Global3.975mm includes opening; original right_grasp_guard_active mask defines reported guarded maxima.',
        'capture_qualified':False,'continuous_full_trajectory':False,'full_pitch_lead_qualified':False,
        'plot_sha256':digest(output/'native_open_reset_history.png')}
    (output/'plot_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'native_rows':len(t),'guarded_grasp_slip':guarded,'output':str(output)},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trial-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
