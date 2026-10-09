"""Output-only final media checks and additive frozen-source archive."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageSequence

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/full_canonical_v1'
RENDER = ROOT/'outputs/m8_table_supported/full_canonical_v1_failed_render'
PROGRESS = ROOT/'media/m8_table_supported/entry_transfer_progress'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for raw in iter(lambda: stream.read(1024*1024), b''):
            digest.update(raw)
    return digest.hexdigest()


def main():
    manifest = json.loads((RENDER/'render_manifest.json').read_text())
    before = json.loads((RUN/'run_publication_identity.json').read_text())
    after = json.loads((RUN/'run_publication_identity_after.json').read_text())
    source_hashes = before['source_hashes_before']
    assert source_hashes == after['source_hashes_after']
    assert len(source_hashes) == 65
    assert sha(RUN/'insertion_trace.npz') == manifest['trajectory_sha256']
    with np.load(RUN/'insertion_trace.npz', allow_pickle=False) as trace:
        samples = json.loads(str(trace['info_json']))
        index = manifest['screenshot_sample_index']
        for field in ('qpos', 'qvel'):
            assert hashlib.sha256(trace[field][index].tobytes()).hexdigest() == manifest['screenshot_'+field+'_sha256']
            assert np.isfinite(trace[field]).all()
        assert samples[index] == manifest['original_saved_sample']
        for still in manifest['stills'].values():
            index = still['sample_index']
            for field in ('qpos', 'qvel'):
                assert hashlib.sha256(trace[field][index].tobytes()).hexdigest() == still[field+'_sha256']
            assert samples[index] == still['original_sample']
        assert list(trace['time'][manifest['frame_sample_indices']]) == manifest['frame_saved_times_s']
    for name, digest in manifest['media_sha256'].items():
        assert sha(RENDER/name) == digest
    with Image.open(RENDER/'demo.gif') as image:
        durations = [frame.info.get('duration', 0) for frame in ImageSequence.Iterator(image)]
    assert len(durations) == manifest['frames'] == 198
    assert sum(durations) == manifest['gif_duration_ms'] == 16500
    video = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
        '-show_entries', 'stream=nb_read_frames,r_frame_rate,duration',
        '-of', 'json', str(RENDER/'demo.mp4')]))['streams'][0]
    assert int(video['nb_read_frames']) == 198
    assert video['r_frame_rate'] == '12/1'
    assert float(video['duration']) == 16.5
    archive = RENDER/'producer_sources'
    assert not archive.exists()
    for name, digest in source_hashes.items():
        frozen = PROGRESS/'producer_sources'/name
        assert sha(frozen) == digest
        target = archive/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(frozen, target)
        assert sha(target) == digest
    source_manifest = {
        'producer_commit': before['producer_commit'],
        'source_hash_count': len(source_hashes),
        'source_hashes': source_hashes,
        'byte_origin': 'Immutable historical entry_transfer_progress producer_sources; each byte verified against original whole-run402/65 before/after source map.',
        'original_software_proof_sha256': sha(RUN/'software_tests.json'),
        'original_beginning_sha256': sha(RUN/'run_publication_identity.json'),
        'original_closure_sha256': sha(RUN/'run_publication_identity_after.json'),
        'scope': 'All65 exact original software source files, including tests. This copy neither reruns nor enlarges the original402-test proof and makes no physical acceptance claim.'}
    (RENDER/'producer_source_manifest.json').write_text(json.dumps(source_manifest, indent=2, allow_nan=False)+'\n')
    review = {
        'passed': True,
        'scope': 'Read-only media serialization/state identity check. No simulator imported, integration, force solve, native retry, app test rerun or physical qualification.',
        'trajectory_sha256': manifest['trajectory_sha256'],
        'render_manifest_sha256': sha(RENDER/'render_manifest.json'),
        'plot_manifest_sha256': sha(RENDER/'scientific_plot/plot_manifest.json'),
        'all7_still_original_samples_and_qpos_qvel_exact': True,
        'all198_frame_original_indices_and_times_exact': True,
        'all_media_sha256_exact': True,
        'gif_frames': len(durations), 'gif_duration_ms': sum(durations),
        'video': video, 'native_last_time_s': manifest['native_last_time_s'],
        'all65_original_source_bytes_archived': True,
        'review_source_sha256': sha(__file__)}
    shutil.copyfile(__file__, RENDER/'media_identity_review_source.py')
    (RENDER/'media_identity_review.json').write_text(json.dumps(review, indent=2, allow_nan=False)+'\n')
    print(json.dumps(review, indent=2))


if __name__ == '__main__':
    main()
