"""Verify exact original saved-state/media bindings; no simulator import."""
from pathlib import Path
import ast
import hashlib
import json
import numpy as np
import imageio.v2 as imageio


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    root=Path('/workspace/astra-r2s');base=root/'outputs/m8_table_supported/diagnostics'
    run=base/'crest_seat_search_v5';render=base/'crest_seat_search_v5_closed_render'
    manifest=json.loads((render/'render_manifest.json').read_text())
    with np.load(run/'insertion_trace.npz',allow_pickle=False) as f:
        times,qpos,qvel=f['time'],f['qpos'],f['qvel'];rows=json.loads(f['info_json'].item())
        assert json.loads(f['metadata_json'].item())==json.loads((run/'insertion_validation.json').read_text())
    assert manifest['native_closed_result'] and manifest['mode']=='closed'
    assert manifest['trajectory_sha256']==sha(run/'insertion_trace.npz')
    assert manifest['native_last_time_s']==times[-1] and manifest['frames']==89
    indices=manifest['frame_sample_indices']
    assert len(indices)==89 and all(0<=i<len(times) for i in indices)
    assert np.array_equal(times[indices],manifest['frame_saved_times_s'])
    assert indices[-1]==len(times)-1 and np.all(np.diff(indices)>=0)
    for name,item in manifest['stills'].items():
        i=item['sample_index']
        assert item['qpos_sha256']==hashlib.sha256(qpos[i].tobytes()).hexdigest()
        assert item['qvel_sha256']==hashlib.sha256(qvel[i].tobytes()).hexdigest()
        assert item['original_sample']==rows[i] and item['time_s']==times[i]
    for name,digest in manifest['media_sha256'].items():assert sha(render/name)==digest,name
    script=render/'renderer_sources/render_crest_seat_search_v5_closed.py'
    calls={node.func.attr for node in ast.walk(ast.parse(script.read_text()))
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
    assert not calls.intersection({'mj_step','mj_forward','mj_collision','mj_inverse'})
    reader=imageio.get_reader(render/'demo.mp4');count=reader.count_frames();meta=reader.get_meta_data()
    inspected=[]
    for i in (0,count//2,count-1):
        frame=reader.get_data(i);assert frame.shape==(720,1296,3)
        inspected.append({'video_frame':i,'saved_native_time_s':float(times[indices[i]]),'shape':list(frame.shape)})
    reader.close()
    assert count==89 and meta['fps']==12
    proof={'scope':'Original trace/metadata/still-state checksums and exact frame-index/time map, three decoded video frames, no native replay/solver call or force/capture proof.',
        'passed':True,'render_manifest_sha256':sha(render/'render_manifest.json'),
        'trace_sha256':sha(run/'insertion_trace.npz'),'renderer_source_sha256':sha(script),
        'verify_source_sha256':sha(__file__),'video_frames':count,'fps':meta['fps'],
        'decoded_frame_checks':inspected,'all_original_sample_qpos_qvel_hashes_exact':True,
        'no_renderer_step_forward_collision_inverse_calls':True}
    (render/'media_identity_review.json').write_text(json.dumps(proof,indent=2)+'\n')
    (render/'media_identity_review_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(proof,indent=2))


if __name__=='__main__':main()
