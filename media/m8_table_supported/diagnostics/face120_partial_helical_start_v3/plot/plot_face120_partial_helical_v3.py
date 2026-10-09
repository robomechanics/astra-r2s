"""Scientific plot of original all-step cold-v3 loads and geometry; no replay."""
from pathlib import Path
import argparse,hashlib,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(directory,output):
    directory=Path(directory);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    source=Path(__file__).read_bytes();report=json.loads((directory/'report.json').read_text())
    audit=json.loads((directory/'independent_formed_entry_audit.json').read_text())
    ledger=directory/'original_native_force_ledger.npz';assert sha(ledger)==report['ledger_sha256']==audit['input_sha256'][ledger.name]
    with np.load(ledger,allow_pickle=False) as z:data={k:z[k].copy() for k in z.files}
    t=data['elapsed_s'];dt=5e-5;weight=report['final_weight_window']['bolt_weight_N'];n=round(.1/dt)
    assert len(t)==136562 and np.all(data['all_checks_held']==1)
    assert np.all(data['loaded_actual_interior_flank_contact_count']==0)
    def trailing(a):
        sums=np.concatenate(([0.],np.cumsum(a,dtype=np.float64)))
        return (sums[n:]-sums[:-n])/n
    meanthread=trailing(data['thread_gravity_opposing_force_N'])/weight
    meanpositivehand=trailing(np.maximum(0.,data['hand_gravity_opposing_force_N']))/weight
    original=report['final_weight_window']
    assert abs(meanthread[-1]-original['mean_thread_weight_fraction'])<1e-9
    assert abs(meanpositivehand[-1]-original['mean_positive_hand_upward_weight_fraction'])<1e-9
    plt.rcParams.update({'font.size':11,'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'white'})
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    fig.suptitle('Cold v3 · jaws stay closed · partial helical starting load\nCapture, full-pitch lead and unsupported reset remain unqualified',fontsize=15)
    axes[0].plot(t[n-1:],meanthread,label='Original signed thread support / bolt weight',color='#176aa2',lw=1.5)
    axes[0].plot(t[n-1:],meanpositivehand,label='Original positive hand support / bolt weight',color='#d46a18',lw=1.5)
    axes[0].axhline(1,color='gray',ls=':',lw=1);axes[0].set_ylabel('Trailing 100 ms mean\n(weight units)');axes[0].legend(loc='upper right',fontsize=9)
    axes[1].plot(t,1e6*data['formed_flank_overlap_m'],color='#3c8761',lw=1.2,label='Conservative complete-ring geometry overlap')
    axes[1].set_ylabel('Formed geometry (µm)');axes[1].legend(loc='upper left',fontsize=9)
    axes[1].text(.99,.13,'Actual loaded full-interior contacts = 0 on every native tick',ha='right',transform=axes[1].transAxes,fontsize=10)
    axes[2].plot(t,1e6*data['radial_offset_m'],color='#744c9d',lw=1.2,label='Original measured radial offset')
    axes[2].axhline(150,color='gray',ls=':',lw=1,label='Original 150 µm radial guard');axes[2].set_ylabel('Radial offset (µm)');axes[2].set_xlabel('Elapsed native time since declared cold initialization (s)');axes[2].legend(loc='upper right',fontsize=9)
    for ax in axes:ax.axvspan(t[-1]-.1,t[-1],color='#ccd8e6',alpha=.5);ax.set_xlim(0,t[-1])
    fig.text(.06,.045,'Final exact 100 ms: thread 99.996643% of bolt weight; positive hand 0.0151166%; loaded duty 100%.',fontsize=10)
    fig.text(.06,.022,'136,562 original preintegration solves (logged time − 50 µs); privileged 20 kHz pose feedback; no force replay.',fontsize=9)
    fig.subplots_adjust(top=.84,bottom=.12,hspace=.28)
    path=output/'native_load_geometry.png';fig.savefig(path,dpi=150);plt.close(fig)
    (output/'plot_face120_partial_helical_v3.py').write_bytes(source)
    manifest={'scope':'Read-only scientific presentation of complete original all-step native scalar loads/geometry. No qstate force reconstruction or integration. Trailing100ms means are explicitly new presentation derivatives; originals remain unchanged. Jaws closed; no capture/full-pitch/reset qualification.',
        'original_ledger_sha256':sha(ledger),'original_report_sha256':sha(directory/'report.json'),
        'formed_entry_audit_sha256':sha(directory/'independent_formed_entry_audit.json'),
        'source_sha256':hashlib.sha256(source).hexdigest(),'original_native_rows':len(t),
        'force_geometry_timing':'Preintegration original solve at logged absolute time−50µs; elapsed clock starts at declared cold initialization, first row50µs.',
        'rolling_window_native_samples':n,'rolling_window_s':.1,'no_interpolation_or_force_replay':True,
        'final_computed_signed_thread_weight_fraction':float(meanthread[-1]),
        'final_computed_positive_hand_weight_fraction':float(meanpositivehand[-1]),
        'maximum_actual_formed_geometry_m':float(data['formed_flank_overlap_m'].max()),
        'actual_loaded_full_interior_substeps':int(np.count_nonzero(data['loaded_actual_interior_flank_contact_count']>0)),
        'plot_sha256':sha(path),'plot_bytes':path.stat().st_size}
    (output/'plot_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps(manifest,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('output',type=Path);a=p.parse_args();main(a.directory,a.output)
