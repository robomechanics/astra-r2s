"""Read-only original saved-state/media preservation review; no physics calls."""
import argparse
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageSequence

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main(args):
    snapshot,render=args.snapshot.resolve(),args.render.resolve()
    m=json.loads((render/'render_manifest.json').read_text())
    assert m['helper_sha256']=='18c83b4d1747ec15988783dddf268b7e637e2cb8057bfa405931f552feacfa3a'
    assert m['replay_plugin_library']['sha256']=='53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18'
    assert m['replay_plugin_library']['plugin_built_during_replay'] is False
    alias=m['portable_renderer_alias']
    assert alias['byte_identical_to_actual_versioned_source'] is True and alias['sha256']==m['helper_sha256']
    assert sha(render/'renderer_sources'/alias['basename'])==alias['sha256']
    assert sha(render/'renderer_sources'/'render_full_reset_speed2_v3_regrasp_progress_v2.py')==m['helper_sha256']
    assert sha(render/'renderer_sources'/'render_full_reset_speed2_v3_regrasp_compiler_origin.py')=='ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8'
    assert m['immutable_snapshot_binding']['schema']=='fresh-full-reset-speed2-v3-phase-prefix-snapshot-v1'
    assert m['immutable_snapshot_binding']['producer_commit']=='9ae1a9fe76968a4013ea6c39e67718026622b9c0'
    assert m['mode']=='progress' and m['native_closed_result'] is False
    assert m['progress_is_not_closed_rollout_proof'] is True and m['slow_motion']==1 and m['fps']==12
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
        assert m['frame_sample_indices'][-1]==len(t)-1
        assert m['native_last_time_s']==float(t[-1])
        disabled=sum(r['inertia_control']['enabled'] is False for r in rows)
        active=sum(r['inertia_control']['enabled'] is True for r in rows)
    for name,digest in m['media_sha256'].items():assert sha(render/name)==digest,name
    for name,digest in m['original_run_files_sha256_before_after'].items():assert sha(snapshot/name)==digest,name
    reader=imageio.get_reader(render/'demo.mp4');meta=reader.get_meta_data();count=reader.count_frames();reader.close()
    assert count==m['frames'] and meta['fps']==12
    with Image.open(render/'demo.gif') as image:durations=[f.info['duration'] for f in ImageSequence.Iterator(image)]
    assert len(durations)==m['gif_frames'] and sum(durations)==m['gif_duration_ms']
    result=dict(existing_replay_plugin_sha256=m['replay_plugin_library']['sha256'],plugin_built_during_replay=False,native_plugin_binary_equivalence_claimed=False,scope='Read-only saved native states/original sampled metadata/media encoding identities only. No native integration, dense-force/inertia validation, closed-rollout or capture qualification.',passed=True,trajectory_sha256=m['trajectory_sha256'],render_manifest_sha256=sha(render/'render_manifest.json'),original_saved_state_stills=len(m['stills']),all_original_sample_qpos_qvel_hashes_exact=True,all_original_frame_indices_times_exact=True,mp4_frames=count,mp4_fps=meta['fps'],mp4_encoded_duration_s=count/12,gif_duration_ms=sum(durations),original_prefix_bytes_unchanged=True,source74_identity_unchanged=True,sampled_inertia_disabled_records=disabled,sampled_inertia_active_records=active,disabled_inverse_inputs_are_not_measured_mass_or_Jdot=True,verification_source_sha256=sha(__file__))
    out=render/'media_identity_review.json';assert not out.exists();out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (render/'renderer_sources'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);p.add_argument('render',type=Path);main(p.parse_args())
