"""Read-only actual media/state identity; no simulator imports."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageSequence

ROOT = Path('/workspace/astra-r2s/outputs/m8_table_supported/diagnostics')
RUN, RENDER = ROOT/'crest_seat_search_v4', ROOT/'crest_seat_search_v4_render'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    manifest = json.loads((RENDER/'render_manifest.json').read_text())
    assert manifest['trajectory_sha256'] == sha(RUN/'insertion_trace.npz')
    with np.load(RUN/'insertion_trace.npz', allow_pickle=False) as archive:
        samples = json.loads(str(archive['info_json']))
        for still in manifest['stills'].values():
            index = still['sample_index']
            assert samples[index] == still['original_sample']
            assert float(archive['time'][index]) == still['time_s']
            for field in ('qpos', 'qvel'):
                assert hashlib.sha256(archive[field][index].tobytes()).hexdigest() == still[field+'_sha256']
        assert list(archive['time'][manifest['frame_sample_indices']]) == manifest['frame_saved_times_s']
    for name, digest in manifest['media_sha256'].items():
        assert sha(RENDER/name) == digest, name
    with Image.open(RENDER/'demo.gif') as image:
        durations = [frame.info.get('duration', 0) for frame in ImageSequence.Iterator(image)]
    assert len(durations) == manifest['frames'] == 21
    assert sum(durations) == manifest['gif_duration_ms'] == 1750
    video = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames',
        '-select_streams', 'v:0', '-show_entries', 'stream=nb_read_frames,r_frame_rate,duration',
        '-of', 'json', str(RENDER/'demo.mp4')]))['streams'][0]
    assert int(video['nb_read_frames']) == 21 and video['r_frame_rate'] == '12/1'
    assert abs(float(video['duration'])-21/12) < 1e-6
    plot = json.loads((RENDER/'scientific_plot/plot_manifest.json').read_text())
    assert sha(RENDER/'scientific_plot/native_stop_boundary.png') == plot['plot_sha256']
    assert sha(RUN/'native_feedback_force_history.npz') == plot['original_feedback_sha256']
    assert sha(RUN/'table_support_force_history.npz') == plot['original_table_ledger_sha256']
    summary = dict(passed=True, scope='Read-only seven-still/native-frame/media serialization verification; no simulator, integration, force solve, native repeat or physical qualification.',
        all7_still_samples_and_qpos_qvel_exact=True, all21_frame_indices_times_exact=True,
        all_media_hashes_exact=True, original_plot_ledger_bindings_exact=True,
        original_trace_sha256=manifest['trajectory_sha256'],
        render_manifest_sha256=sha(RENDER/'render_manifest.json'),
        plot_manifest_sha256=sha(RENDER/'scientific_plot/plot_manifest.json'),
        gif_frames=len(durations), gif_duration_ms=sum(durations), video=video,
        review_source_sha256=sha(__file__))
    assert not (RENDER/'media_identity_review.json').exists()
    (RENDER/'media_identity_review.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    shutil.copyfile(__file__, RENDER/'media_identity_review_source.py')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
