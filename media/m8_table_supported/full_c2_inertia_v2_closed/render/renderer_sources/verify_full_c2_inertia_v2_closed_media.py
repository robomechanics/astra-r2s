"""Read-only CLOSED original saved-state/media preservation review; no physics calls."""
import argparse
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageSequence
from full_c2_closed_bindings import verify_closed

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main(args):
    snapshot,render=args.run.resolve(),args.render.resolve()
    bound=verify_closed(snapshot,args.repository_root,args.proof_archive,args.identity,args.identity_sha256,args.audit_binding,args.audit_binding_sha256)
    m=json.loads((render/'render_manifest.json').read_text())
    assert m['mode']=='closed_native' and m['native_closed_result'] is True
    assert m['identity_report_sha256']==bound['identity_sha256'] and m['independent_audit_binding_sha256']==bound['audit_sha256']
    assert m['original_native_exit_code']==bound['after']['native_exit_code']
    assert m['progress_is_not_closed_rollout_proof'] is False and m['slow_motion']==1 and m['fps']==12
    assert m['source74_unchanged'] is True and len(m['source74_before_after_render'])==74
    trace=snapshot/m['trace_filename'];assert m['trajectory_sha256']==sha(trace)
    with np.load(trace,allow_pickle=False) as a:
        t,q,v=a['time'],a['qpos'],a['qvel'];rows=json.loads(str(a['info_json']))
        for name,item in m['stills'].items():
            i=item['sample_index']
            assert item['time_s']==float(t[i]) and item['phase']==rows[i]['phase'] and item['original_sample']==rows[i]
            assert item['qpos_sha256']==hashlib.sha256(q[i].tobytes()).hexdigest()
            assert item['qvel_sha256']==hashlib.sha256(v[i].tobytes()).hexdigest()
            record=rows[i]['inertia_control'];norms=item['original_executed_command_norms']
            assert norms['inertia_enabled']==record['enabled']
            assert norms['inertia_inputs_present']==record['inertia_inputs_present']
            assert norms['jacobian_derivatives_present']==record['jacobian_derivatives_present']
        assert m['frame_saved_times_s']==[float(t[i]) for i in m['frame_sample_indices']]
        if m['frames']:
            assert m['frame_sample_indices'][-1]==len(t)-1
        else:
            assert m['frame_sample_indices']==[] and m['frame_saved_times_s']==[]
        assert m['native_last_time_s']==float(t[-1])
        original=json.loads(str(a['metadata_json']))
        assert original==json.loads((snapshot/'insertion_validation.json').read_text())
        assert m['original_report_sha256']==sha(snapshot/'insertion_validation.json')
        assert m['original_passed']==original.get('passed') and m['original_partial']==original.get('partial') and m['original_aborted']==original.get('aborted')
        assert m['original_acceptance_checks']==original.get('acceptance_checks')
        for key,source_key in (('frame_original_inertia_enabled','enabled'),('frame_original_inertia_inputs_present','inertia_inputs_present'),('frame_original_jacobian_derivatives_present','jacobian_derivatives_present')):
            assert m[key]==[rows[i]['inertia_control'][source_key] for i in m['frame_sample_indices']]
        assert m['original_run_files_sha256_before_after']==bound['original_run_files_sha256']

        disabled=sum(r['inertia_control']['enabled'] is False for r in rows)
        active=sum(r['inertia_control']['enabled'] is True for r in rows)
    for name,digest in m['media_sha256'].items():assert sha(render/name)==digest,name
    for name,digest in m['original_run_files_sha256_before_after'].items():assert sha(snapshot/name)==digest,name
    meta=None;count=0;durations=[]
    if m['frames']:
        reader=imageio.get_reader(render/'demo.mp4');meta=reader.get_meta_data();count=reader.count_frames();reader.close()
        assert count==m['frames'] and meta['fps']==12
        with Image.open(render/'demo.gif') as image:durations=[f.info['duration'] for f in ImageSequence.Iterator(image)]
    else:
        assert not (render/'demo.mp4').exists() and not (render/'demo.gif').exists()
    assert len(durations)==m['gif_frames'] and sum(durations)==m['gif_duration_ms']
    result=dict(scope='Read-only saved native states/original sampled metadata/media encoding identities only. No native integration, dense-force/inertia validation or new physics/capture qualification; original closed success/failure flags preserved.',passed=True,trajectory_sha256=m['trajectory_sha256'],render_manifest_sha256=sha(render/'render_manifest.json'),original_saved_state_stills=len(m['stills']),all_original_sample_qpos_qvel_hashes_exact=True,all_original_frame_indices_times_exact=True,mp4_frames=count,mp4_fps=meta['fps'] if meta else None,mp4_encoded_duration_s=count/12 if count else None,gif_duration_ms=sum(durations),original_prefix_bytes_unchanged=True,source74_identity_unchanged=True,sampled_inertia_disabled_records=disabled,sampled_inertia_active_records=active,identity_report_sha256=bound['identity_sha256'],independent_audit_binding_sha256=bound['audit_sha256'],original_native_passed=m['original_passed'],original_native_partial=m['original_partial'],original_native_aborted=m['original_aborted'],disabled_inverse_inputs_are_not_measured_mass_or_Jdot=True,verification_source_sha256=sha(__file__))
    out=render/'media_identity_review.json';assert not out.exists();out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (render/'renderer_sources'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,required=True)
    p.add_argument('--proof-archive',type=Path,required=True)
    p.add_argument('--identity',type=Path,required=True)
    p.add_argument('--identity-sha256',required=True)
    p.add_argument('--audit-binding',type=Path,required=True)
    p.add_argument('--audit-binding-sha256',required=True)
    p.add_argument('run',type=Path);p.add_argument('render',type=Path);main(p.parse_args())
