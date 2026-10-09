"""Original saved-state/media identity check only, no simulator calls."""
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageSequence

ROOT=Path('/workspace/astra-r2s')
SNAPSHOT=ROOT/'outputs/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress_snapshot'
RENDER=ROOT/'outputs/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress_render'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    m=json.loads((RENDER/'render_manifest.json').read_text())
    assert m['mode']=='progress' and m['native_closed_result'] is False and m['progress_is_not_closed_rollout_proof'] is True
    assert m['slow_motion']==1 and m['fps']==12 and m['frames']==23
    assert m['trajectory_sha256']==sha(SNAPSHOT/'insertion_trace_partial.npz')
    with np.load(SNAPSHOT/'insertion_trace_partial.npz',allow_pickle=False) as a:
        t,q,v=a['time'],a['qpos'],a['qvel'];rows=json.loads(str(a['info_json']))
        for name,item in m['stills'].items():
            i=item['sample_index']
            assert item['time_s']==float(t[i]) and item['phase']==rows[i]['phase'] and item['original_sample']==rows[i]
            assert item['qpos_sha256']==hashlib.sha256(q[i].tobytes()).hexdigest()
            assert item['qvel_sha256']==hashlib.sha256(v[i].tobytes()).hexdigest()
        assert m['frame_saved_times_s']==[float(t[i]) for i in m['frame_sample_indices']]
        assert m['frame_sample_indices'][-1]==len(t)-1
        assert m['native_last_time_s']==float(t[-1])==1.9062500000018106
    for name,digest in m['media_sha256'].items():assert sha(RENDER/name)==digest,name
    for name,digest in m['original_run_files_sha256_before_after'].items():assert sha(SNAPSHOT/name)==digest,name
    reader=imageio.get_reader(RENDER/'demo.mp4');meta=reader.get_meta_data();count=reader.count_frames();reader.close()
    assert count==23 and meta['fps']==12
    with Image.open(RENDER/'demo.gif') as a:durations=[f.info['duration'] for f in ImageSequence.Iterator(a)]
    assert len(durations)==23 and sum(durations)==m['gif_duration_ms']==1920
    result=dict(scope='Read-only original saved states/sampled metadata/media encoding identities. No native integration, physics audit, dense-force/FF-ledger inference or closed-rollout qualification.',passed=True,trajectory_sha256=m['trajectory_sha256'],render_manifest_sha256=sha(RENDER/'render_manifest.json'),actual_saved_state_stills=len(m['stills']),all_original_sample_qpos_qvel_hashes_exact=True,all23_frame_indices_times_exact=True,mp4_frames=count,mp4_fps=meta['fps'],mp4_encoded_duration_s=23/12,gif_duration_ms=sum(durations),original_snapshot_bytes_unchanged=True,source_sha256=sha(__file__))
    out=RENDER/'media_identity_review.json';assert not out.exists();out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (RENDER/'renderer_sources'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
