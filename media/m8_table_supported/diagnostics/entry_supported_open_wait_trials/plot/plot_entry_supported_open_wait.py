"""Original native opening-window diagnostics; no integration or force replay."""
from pathlib import Path
import argparse,hashlib,json,importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/workspace/astra-r2s')
TRIALS=('entry_supported_open_search_v1','entry_supported_open_search_v2')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    base=ROOT/'outputs/m8_table_supported/diagnostics';source=Path(__file__).read_bytes()
    auditor=base/TRIALS[1]/'audit_sources/independent_open_search_audit_v1.py';auditor_bytes=auditor.read_bytes()
    spec=importlib.util.spec_from_file_location('closed_open_plot_auditor',auditor);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    originals={};raws={};readies={}
    for name in TRIALS:
        d=base/name;a=json.loads((d/'independent_open_search_audit.json').read_text());r=json.loads((d/'report.json').read_text())
        assert a['auditor_source_sha256']==sha(auditor)==sha(d/'audit_sources/independent_open_search_audit_v1.py')
        assert a['original_ledger_sha256']==r['ledger_sha256']==sha(d/'original_native_force_ledger.npz')
        with np.load(d/'original_native_force_ledger.npz',allow_pickle=False) as z:raw={k:z[k].copy() for k in z.files}
        good,ready=module.exact_readiness(raw,r['final_weight_window']['bolt_weight_N'],5e-5)
        ids=np.flatnonzero(ready);record=a['strict_open_ready_independent_exact100ms']
        assert len(ids)==record['count']
        assert (float(raw['elapsed_s'][ids[0]]) if len(ids) else None)==record['first_elapsed_s']
        assert (float(raw['elapsed_s'][ids[-1]]) if len(ids) else None)==record['last_elapsed_s']
        assert not ready[-1] and a['all_recorded_native_hard_checks_held']
        assert a['actual_regrasp_acquisition'] is None and not a['qualified_reset'] and not a['full_capture']
        opened=raw['fully_open_unassisted']==1
        assert np.all(raw['right_bolt_contact_count'][opened]==0) and np.all(raw['hand_gravity_opposing_force_N'][opened]==0)
        originals[name]={'original_ledger_sha256':r['ledger_sha256'],'original_report_sha256':sha(d/'report.json'),'original_open_audit_sha256':sha(d/'independent_open_search_audit.json'),'native_steps':len(raw['time_s']),'strict_readiness':record,'tail_gap_s':a['timing']['saved_state_tail_gap_s'],'fully_open':a['fully_open'],'motion':a['fully_open_bolt_motion']}
        raws[name]=raw;readies[name]=ready
    first,second=(raws[n] for n in TRIALS)
    assert set(first)==set(second) and all(np.array_equal(first[k],second[k][:7400]) for k in first)
    plt.rcParams.update({'font.size':11,'axes.grid':True,'grid.alpha':.2,'figure.facecolor':'white'})
    fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True)
    fig.suptitle('Cold entry-supported opening / waiting · no reset or qualified capture\nEarlier strict-ready windows do not pass a later fixed endpoint',fontsize=14)
    raw=second;t=raw['elapsed_s'];opened=raw['fully_open_unassisted']==1
    ax[0].semilogy(t[opened],raw['relative_bolt_angular_speed_rad_per_s'][opened],color='#176aa2',lw=1,label='Original actual bolt angular speed')
    ax[0].axhline(.01,color='#ae4b3f',ls='--',label='Original instantaneous quiet gate: 0.01 rad/s')
    ax[0].set_ylabel('Bolt angular speed\n(rad/s, log scale)');ax[0].legend(loc='upper right',fontsize=9)
    ax[1].step(t,readies[TRIALS[1]].astype(int),where='post',color='#33805a',lw=1.1)
    ax[1].fill_between(t,0,readies[TRIALS[1]].astype(int),step='post',alpha=.2,color='#33805a')
    ax[1].set_ylim(-.05,1.15);ax[1].set_yticks([0,1]);ax[1].set_ylabel('Strict trailing 100 ms\nreadiness (0 / 1)')
    ax[1].set_title('v1 never ready; v2 first ready 0.46205 s, last ready 0.87115 s; intermittent endpoints',loc='left',fontsize=10)
    ax[2].plot(t[opened],1e9*raw['open_drift_peak_axial_m'][opened],color='#176aa2',label='Original cumulative peak axial drift')
    ax[2].set_ylabel('Peak axial drift (nm)',color='#176aa2')
    yaw=ax[2].twinx();yaw.plot(t[opened],1e6*raw['open_drift_peak_yaw_rad'][opened],color='#a86620',ls='--',label='Original cumulative peak yaw drift');yaw.set_ylabel('Peak yaw drift (µrad)',color='#a86620');yaw.grid(False)
    ax[2].set_xlabel('Elapsed native time from each separate identical cold initialization (s)')
    for a in ax:
        a.axvline(.37005,color='#ae4b3f',ls=':',lw=1.2);a.axvline(1.00005,color='#ae4b3f',ls=':',lw=1.2);a.set_xlim(.23,1.015)
    ax[0].text(.37005,.018,'v1 fixed gate',fontsize=9,rotation=90,va='bottom',ha='right',color='#ae4b3f')
    ax[0].text(1.00005,.018,'v2 fixed gate',fontsize=9,rotation=90,va='bottom',ha='right',color='#ae4b3f')
    fig.text(.07,.047,'Fully open: whole-right contacts 0, hand load 0, external drive 0; v2 max drift 141.733 nm /393.647 µrad.',fontsize=10)
    fig.text(.07,.023,'Original 50 µs native solves; 7,400 shared scalar rows identical. v1 saved qstate tail 4.95 ms is absent.',fontsize=9)
    fig.subplots_adjust(top=.84,bottom=.13,hspace=.32,right=.87)
    path=output/'native_strict_window_history.png';fig.savefig(path,dpi=150);plt.close(fig)
    (output/'plot_entry_supported_open_wait.py').write_bytes(source)
    (output/'independent_open_search_audit_v1.py').write_bytes(auditor_bytes)
    manifest={'scope':'Read-only scientific display of original native opening angular speed, drift and exact independent trailing-window arithmetic. No integration/force replay/interpolation. Original fixed-endpoint failures remain failed; no reset/capture qualification.',
        'source_sha256':hashlib.sha256(source).hexdigest(),'archived_arithmetic_dependency_sha256':hashlib.sha256(auditor_bytes).hexdigest(),
        'trials':originals,'source_faithful_exact_arithmetic_scope':'Independent fixed2000-native-tick trailing calculation from frozen auditor, verified against original arithmetic readiness count/first/last endpoints.',
        'force_geometry_timing':'Original preintegration solve at logged absolute time−50µs; elapsed time is branch-local, initialization physical state identical; no warmstarted splice.',
        'shared_first7400_rows_identical':True,'plot_sha256':sha(path),'plot_bytes':path.stat().st_size}
    (output/'plot_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n');print(json.dumps(manifest,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();main(a.output)
