"""Losslessly publish one original fresh-run phase prefix, no full qualification."""
import argparse
import hashlib
import importlib.util
import json
import shutil
import uuid
from pathlib import Path


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as stream:
        for raw in iter(lambda:stream.read(1024*1024),b''):h.update(raw)
    return h.hexdigest()

def read(p):return json.loads(Path(p).read_text(),parse_constant=lambda token:(_ for _ in ()).throw(ValueError(token)))

def main(args):
    root=args.repository_root.resolve();snapshot=args.snapshot.resolve();render=args.render.resolve();target=args.output.resolve()
    assert not target.exists()
    shared=root/'outputs/m8_table_supported/publish_supported_feedback_run.py'
    assert sha(shared)=='8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'
    reassembler=root/'outputs/m8_table_supported/reassemble_archives.py'
    assert sha(reassembler)=='0706d299abfc70a0883aa8227ea3d89c7da420e961b4fc542fd696f3b05a3cfb'
    spec=importlib.util.spec_from_file_location('frozen_lossless_writer',shared);writer=importlib.util.module_from_spec(spec);spec.loader.exec_module(writer)
    binding=read(snapshot/'snapshot_binding.json');launch=read(snapshot/'run_publication_identity.json')
    assert binding['immutable_snapshot'] is True and binding['full_native_run_closed'] is False
    assert binding['schema']=='fresh-full-reset-speed2-v3-phase-prefix-snapshot-v1'
    assert binding['producer_commit']==launch['producer_commit']=='9ae1a9fe76968a4013ea6c39e67718026622b9c0'
    source_map=binding['source74_sha256_before_after_copy']
    assert len(source_map)==74 and source_map==launch['source_hashes_before']
    for name,digest in source_map.items():assert sha(root/name)==digest,name
    for name,digest in binding['file_sha256'].items():assert sha(snapshot/name)==digest,name
    for name,digest in binding['runtime_file_sha256_before_after_copy'].items():assert sha(name)==digest,name
    m=read(render/'render_manifest.json');review=read(render/'media_identity_review.json')
    assert m['helper_sha256']=='18c83b4d1747ec15988783dddf268b7e637e2cb8057bfa405931f552feacfa3a'
    assert m['replay_plugin_library']['sha256']=='53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18'
    assert m['replay_plugin_library']['plugin_built_during_replay'] is False
    assert review['native_plugin_binary_equivalence_claimed'] is False
    assert review['existing_replay_plugin_sha256']==m['replay_plugin_library']['sha256']
    assert m['portable_renderer_alias']['byte_identical_to_actual_versioned_source'] is True
    assert sha(render/'renderer_sources'/m['portable_renderer_alias']['basename'])==m['helper_sha256']
    assert m['trajectory_sha256']==binding['trajectory_sha256']==sha(snapshot/'insertion_trace_partial.npz')
    assert m['native_closed_result'] is False and m['progress_is_not_closed_rollout_proof'] is True
    assert m['source74_before_after_render']==source_map and m['source74_unchanged'] is True
    assert m['runtime_before']==m['runtime_after']==binding['runtime']
    assert m['physics_integration'] is False and m['mj_forward_called'] is False
    assert m['force_solve_called'] is False and m['state_interpolation'] is False
    assert review['passed'] is True and review['render_manifest_sha256']==sha(render/'render_manifest.json')
    for name,digest in m['media_sha256'].items():assert sha(render/name)==digest,name
    for name,digest in m['original_run_files_sha256_before_after'].items():assert sha(snapshot/name)==digest,name
    software=root/'media/m8_table_supported/software_proof_reset_speed2_v3'
    proof=read(software/'software_proof.json')
    assert sha(software/'software_proof.json')=='9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
    assert proof['passed'] is True and proof['tests_passed']==672 and proof['source_hashes_unchanged'] is True
    assert proof['source_hashes']==source_map and proof['runtime']==binding['runtime']
    assert read(snapshot/'software_tests.json')==proof
    for line in (software/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1);assert sha(software/name)==digest,name
    for name,digest in source_map.items():assert sha(software/'source_files'/name)==digest,name
    originals={}
    def add(name,path):
        assert name not in originals
        originals[name]=Path(path).resolve()
    for folder,prefix in ((snapshot,'native_snapshot'),(render,'render'),(software,'software_proof')):
        for path in sorted(folder.rglob('*')):
            if path.is_file():add(prefix+'/'+str(path.relative_to(folder)),path)
    add('publication_dependencies/shared_lossless_writer.py',shared)
    stage_execution=[]
    if args.stage_provenance is not None:
        provenance=args.stage_provenance.resolve()
        assert provenance.is_dir()
        for finished in sorted(provenance.rglob('execution_finished.json')):
            record=read(finished);folder=finished.parent
            assert record['schema']=='reset-speed2-v3-media-stage-execution-v1'
            assert sha(folder/'stage_recorder_source.py')==record['recorder_source_sha256']
            assert sha(folder/'stdout.log')==record['stdout_sha256']
            assert sha(folder/'stderr.log')==record['stderr_sha256']
            start=read(folder/'execution_started.json')
            assert all(record[key]==value for key,value in start.items())
            assert isinstance(record['exit_code'],int)
            assert record['stage_command_succeeded']==(record['exit_code']==0)
            stage_execution.append(dict(path=str(finished.relative_to(provenance)),
                execution_record_sha256=sha(finished),original_record=record))
        assert stage_execution, 'No closed media-stage execution records supplied'
        for path in sorted(provenance.rglob('*')):
            if path.is_file():add('stage_execution_provenance/'+str(path.relative_to(provenance)),path)

    identities={name:writer.file_identity(path) for name,path in originals.items()}
    target.parent.mkdir(parents=True,exist_ok=True);stage=target.parent/(target.name+'.packaging-'+uuid.uuid4().hex);stage.mkdir()
    try:
        artifacts={name:writer.write_artifact(path,stage,name,45_000_000) for name,path in originals.items()}
        for name,path in originals.items():assert writer.file_identity(path)==identities[name],name
        for name,digest in source_map.items():assert sha(root/name)==digest,name
        for name,digest in binding['runtime_file_sha256_before_after_copy'].items():assert sha(name)==digest,name
        manifest=dict(status='historical_partial_fresh_native_phase_prefix',full_native_run_closed=False,
            final_dense_force_and_inertia_ledgers_present=False,independent_full_audit_present=False,
            producer_commit=binding['producer_commit'],original_source74=source_map,
            whole672_software_proof_is_separate_from_native_qualification=True,
            software_packet_sha256_ledger=sha(software/'SHA256SUMS'),software_proof_sha256=sha(software/'software_proof.json'),
            original_snapshot_binding_sha256=sha(snapshot/'snapshot_binding.json'),
            original_trajectory_sha256=binding['trajectory_sha256'],original_launch_identity_sha256=sha(snapshot/'run_publication_identity.json'),
            original_endpoint=binding['original_endpoint'],original_npz_array_catalog=binding['original_npz_array_catalog'],
            original_runtime=binding['runtime'],native_frame_timing_and_inertia_presence_review=review,
            supplied_existing_replay_plugin=m['replay_plugin_library'],native_plugin_binary_equivalence_inferred=False,
            additive_read_only_stage_execution_records=stage_execution,
            original_snapshot_file_to_artifact={name:'native_snapshot/'+name for name in binding['file_sha256']},
            original_embedded_metadata_and_acceptance_fields_unmodified=True,
            original_fresh_table_rest_spawns_verified=True,no_cold_checkpoint_or_splice=True,
            all_original_snapshot_and_render_bytes_preserved=True,archives_are_lossless=True,
            dropping_or_quantization=False,maximum_stored_file_bytes=45_000_000,
            scope='One fresh native attempt, exact copied original closed-phase prefix and sparse sampled state/contact/inertia records only. Disabled inverse/Jdot placeholders explicitly distinguished from active inputs. Dense native force/FF ledgers and final independent audit pending at capture. No complete threading/reset/fresh full trajectory/policy qualification inferred. Geometry-only actual state replay.',artifacts=artifacts)
        (stage/'package_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        shutil.copyfile(__file__,stage/'publication_helper_source.py');shutil.copyfile(reassembler,stage/'reassemble_archives.py');shutil.copyfile(args.readme,stage/'README.md')
        paths=sorted(path for path in stage.rglob('*') if path.is_file());assert max(path.stat().st_size for path in paths)<=45_000_000
        (stage/'SHA256SUMS').write_text(''.join(f'{sha(path)}  {path.relative_to(stage)}\n' for path in paths))
        assert not target.exists();stage.rename(target)
        stored=[path for path in target.rglob('*') if path.is_file()]
        print(json.dumps(dict(target=str(target),files=len(stored),checksum_entries=len(paths),artifacts=len(artifacts),total_bytes=sum(path.stat().st_size for path in stored),maximum_stored_bytes=max(path.stat().st_size for path in stored),ledger_sha256=sha(target/'SHA256SUMS')),indent=2))
    except Exception:
        shutil.rmtree(stage)
        raise

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--repository-root',type=Path,default=Path('/workspace/astra-r2s'));p.add_argument('--readme',type=Path,required=True);p.add_argument('--stage-provenance',type=Path);p.add_argument('snapshot',type=Path);p.add_argument('render',type=Path);p.add_argument('output',type=Path);main(p.parse_args())
