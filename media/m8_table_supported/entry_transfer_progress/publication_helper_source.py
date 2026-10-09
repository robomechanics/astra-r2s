"""Publish only immutable original partial-prefix bytes and geometry media."""
from pathlib import Path
import hashlib,json,shutil
ROOT=Path('/workspace/astra-r2s');SNAPSHOT=ROOT/'outputs/m8_table_supported/progress/full_canonical_v1_entry_transfer'
TARGET=ROOT/'media/m8_table_supported/entry_transfer_progress'
def sha(raw):return hashlib.sha256(raw).hexdigest()
def parse(raw):return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))
def main():
    assert not TARGET.exists()
    files={str(p.relative_to(SNAPSHOT)):p.read_bytes() for p in SNAPSHOT.rglob('*') if p.is_file()}
    snapshot=parse(files['progress_snapshot.json']);render=parse(files['render_manifest.json']);proof=parse(files['software_tests.json'])
    assert len(snapshot['source_hashes_before'])==65 and snapshot['source_hashes_before']==proof['source_hashes']
    assert proof['tests_passed']==402 and proof['passed'] and proof['source_hashes_unchanged']
    assert render['trajectory_sha256']==snapshot['snapshot_trajectory_sha256']==sha(files['insertion_trace_partial.npz'])
    assert render['runtime_before']==render['runtime_after']==snapshot['runtime']
    assert render['source_recording']==str(SNAPSHOT/'insertion_trace_partial.npz') and render['producer_commit']==snapshot['producer_commit']
    assert render['frames']==159 and render['fps']==12 and render['slow_motion']==1
    assert render['frame_sample_indices'][-1]==snapshot['saved_samples']-1 and render['frame_saved_times_s'][-1]==snapshot['last_time_s']
    assert render['screenshot_qpos_sha256']==snapshot['final_qpos_sha256'] and render['screenshot_qvel_sha256']==snapshot['final_qvel_sha256']
    assert render['original_saved_sample']==snapshot['final_original_sample']
    for field in ['mj_forward_called','collision_discovery_called','dynamics_or_contact_force_solve_called','integration_called','pose_interpolation_called','additional_physical_body_hiding','geometry_changed']:assert not render[field],field
    for n,digest in snapshot['source_hashes_before'].items():assert sha(files['producer_sources/'+n])==digest,n
    for n,digest in snapshot['copied_original_source_model_proof_files'].items():assert sha(files[n])==digest,n
    for name,identity in render['media'].items():assert sha(files[name])==identity['sha256'] and len(files[name])==identity['bytes']
    for n,raw in files.items():
        if n.endswith('.json'):parse(raw)
    files['publication_helper_source.py']=Path(__file__).read_bytes()
    files['README.md']='''# Native entry and weight-transfer progress\n\nThis is one original fresh continuous native prefix, from separately spawned\nblock/bolt through pickup, alignment, entry and measured weight transfer.\nActual saved native time: **0.00005 to13.19490s**,2652 exact saved states.\n**PHASE-ONLY PROGRESS**: final formed overlap0 and loaded interior contacts0.\nNo captured pitch, qualified turn/reset, full physical audit or completed\nend-to-end trajectory is claimed. The native full run continues separately;\nits every-step raw ledgers are not yet closed and are not invented here.\n\n![Actual native endpoint](entry_transfer_detail.png)\n\n[Normal1x GIF](entry_transfer_progress.gif) · [Normal1x MP4](entry_transfer_progress.mp4) ·\n[Full robot/table view](entry_transfer_progress.png)\n\nOriginal entry dwell6.67495s was within the declared10s maximum; entry\noverlap was1.310219mm, separate from formed overlap. At the endpoint,\noriginal solved table up-force is3.261569N; the left physical pads\nstabilize the free block on the unchanged solid table. Original100.05ms\nload-window measurements: thread100.000001% of boltweight and positive\nright-hand support1.309500%, with zero loaded interior duration.\nThis is starting-geometry support, not captured full-pitch engagement.\n\nEvery frame is an exact original saved post-step qpos/qvel. Geometry refresh\nuses ONLY mj_kinematics, mj_comPos and mj_camlight: no mj_forward, collision\ndiscovery, dynamics/contact-force solve, integration, interpolation, manual\nfreebody pose, or splice with cold diagnostics/earlier progress. Displayed\nforce samples/contact records are original native preintegration solves\nat saved time minus50us. The detail views share the same exact endpoint.\nThe canonical renderer/cameras and its existing geomgroup visibility filter\nare unchanged; no extra body hiding occurs. Real detail cameras are archived.\n\nPlayback is normal1x at12fps:159 frames,13.25000s encoded video and13.250s\nGIF. Finite frame-time quantization and the exact endpoint in the last\ndisplay slot are explicit; original timestamps/states are never retimed.\n\nProducer commit6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1 and ALL65 original\nsource-file bytes are archived under producer_sources. Exact source hashes\nmatch before/after capture and render. The original402-test proof/log/\nverifier are retained separately from physical qualification and historical\nb2/281 cold trials. Runtime native core/plugin identities, original model\nXML/ZIP,65-source map, launch command, contact-local records, snapshot and\nrender source/state bindings are preserved. Matched CPU MuJoCo/GCC setup\nis documented in [docs/m8_setup.md](../../../docs/m8_setup.md); the stock\nwheel is insufficient and another-machine binary equality is not assumed.\n\nVerify from this folder: `sha256sum -c SHA256SUMS`. Every artifact is below\n45MB, with the original NPZ preserved whole without dropped/quantized rows.\nTo repeat this geometry-only replay in a matching producer checkout at\n/workspace/astra-r2s, copy this COMPLETE folder to a new ignored output\nlocation and remove only its copied four generated media/render_manifest\nfiles; run its archived render_progress_source.py with `--snapshot` pointing\nto that copied folder. The helper refuses an existing output video and\nchecks ALL65 source hashes/runtime before/after. Do not use the active\nproducer folder or alter any original native output.\n'''.encode()
    maximum=max(map(len,files.values()));assert maximum<45_000_000
    manifest={'scope':snapshot['scope'],'producer_commit':snapshot['producer_commit'],'software_checks_passed':402,'software_source_files':65,
        'snapshot_trajectory_sha256':snapshot['snapshot_trajectory_sha256'],'partial':True,'passed_full_physics':False,'full_raw_ledgers_closed':False,
        'native_first_time_s':snapshot['first_time_s'],'native_last_time_s':snapshot['last_time_s'],'saved_native_states':snapshot['saved_samples'],
        'original_final_sample_sha256':sha(json.dumps(snapshot['final_original_sample'],sort_keys=True,separators=(',',':'),allow_nan=False).encode()),
        'original_source_model_runtime_identities_preserved':True,'no_splice_no_integration_no_force_solve':True,
        'final_formed_overlap_m':0,'final_loaded_interior_count':0,'full_capture_qualified':False,'full_pitch_lead_qualified':False,'full_trajectory_qualified':False,
        'all_artifacts_below45MB_no_chunks_needed':True,'maximum_artifact_bytes':maximum,
        'files':{n:{'sha256':sha(raw),'bytes':len(raw)} for n,raw in sorted(files.items())}}
    files['package_manifest.json']=(json.dumps(manifest,indent=2,allow_nan=False)+'\n').encode()
    TARGET.mkdir(parents=True)
    for n,raw in files.items():
        p=TARGET/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {n}\n' for n,raw in sorted(files.items())))
    for n,raw in files.items():assert (TARGET/n).read_bytes()==raw,n
    print(json.dumps({'folder':str(TARGET),'files':len(files)+1,'checksum_entries':len(files),'total_bytes':sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),'largest_bytes':maximum,'final_ledger_sha256':sha((TARGET/'SHA256SUMS').read_bytes())},indent=2))
if __name__=='__main__':main()
