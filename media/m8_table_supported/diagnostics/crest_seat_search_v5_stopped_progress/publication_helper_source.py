"""Publish only one immutable V5 phase-end prefix; no closed-run inference."""
import hashlib
import importlib.util
import json
import shutil
import uuid
from pathlib import Path

import numpy as np

ROOT=Path('/workspace/astra-r2s')
BASE=ROOT/'outputs/m8_table_supported/diagnostics'
SNAPSHOT=BASE/'crest_seat_search_v5_stopped_progress_snapshot'
RENDER=BASE/'crest_seat_search_v5_stopped_progress_render'
TARGET=ROOT/'media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress'
PARENT=ROOT/'media/m8_table_supported/full_canonical_v1_failed_evidence'
WRITER=ROOT/'outputs/m8_table_supported/publish_supported_feedback_run.py'
WRITER_SHA='8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as stream:
        for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def read(p):return json.loads(Path(p).read_text(),parse_constant=lambda token:(_ for _ in ()).throw(ValueError(token)))

def main():
    assert not TARGET.exists() and sha(WRITER)==WRITER_SHA
    spec=importlib.util.spec_from_file_location('frozen_byte_writer',WRITER)
    writer=importlib.util.module_from_spec(spec);spec.loader.exec_module(writer)
    before=read(BASE/'crest_seat_search_v5_execution_before.json')
    frozen=read(BASE/'crest_seat_search_v5_frozen_inputs.json')
    assert before['frozen_inputs_manifest_sha256']==sha(BASE/'crest_seat_search_v5_frozen_inputs.json')
    assert before['source_files_before']==frozen['source_files'] and len(before['source_files_before'])==18
    assert before['producer_source_65_before']==frozen['producer_source_65'] and len(frozen['producer_source_65'])==65
    for name,digest in before['source_files_before'].items():assert sha(BASE/name)==digest,name
    for name,digest in before['producer_source_65_before'].items():assert sha(ROOT/name)==digest,name
    progress=read(SNAPSHOT/'progress_manifest.json');binding=read(SNAPSHOT/'snapshot_binding.json')
    assert binding['schema']=='m8-v5-native-media-snapshot-v1' and binding['immutable_snapshot'] is True
    assert binding['snapshot_is_not_closed_rollout_proof'] is True
    assert binding['original_progress_manifest_sha256']==sha(SNAPSHOT/'progress_manifest.json')
    for name,digest in progress['files'].items():assert sha(SNAPSHOT/name)==digest,name
    for name,digest in binding['file_sha256'].items():assert sha(SNAPSHOT/name)==digest,name
    aliases=read(SNAPSHOT/'snapshot_model_aliases.json')
    assert aliases['original_snapshot_binding_sha256']==sha(SNAPSHOT/'snapshot_binding.json')
    for name,item in aliases['aliases'].items():assert sha(SNAPSHOT/name)==sha(SNAPSHOT/item['copied_from'])==item['sha256'],name
    declaration=read(SNAPSHOT/'diagnostic_declaration.json')
    assert declaration['source_65_before']==before['producer_source_65_before']
    assert declaration['runtime']==before['runtime']
    assert declaration['controller_source_sha256']==sha(SNAPSHOT/'controller_source.py')==before['source_files_before']['crest_seat_inertia_probe_v5.py']
    assert declaration['robot_inertia_source_sha256']==sha(SNAPSHOT/'robot_inertia_source.py')==before['source_files_before']['robot_inertia_feedforward_v5.py']
    trace=SNAPSHOT/'insertion_trace_partial.npz'
    assert sha(trace)==progress['partial_trace_sha256']
    with np.load(trace,allow_pickle=False) as a:
        times=a['time'];rows=json.loads(str(a['info_json']));metadata=json.loads(str(a['metadata_json']))
        assert float(times[-1])==progress['snapshot_boundary']['time']==1.9062500000018106
        endpoint=rows[-1]
        assert endpoint['phase']=='stop_reverse_seat_1' and endpoint['all_hard_guards_held'] is True
        assert endpoint['seat_direction_event']['confirmed_search_direction_event'] is True
        assert endpoint['weight_window']['ready_for_diagnostic_release_attempt'] is True
        assert endpoint['formed_flank_overlap_m']==0 and endpoint['native_feedback']['loaded_actual_interior_flank_contact_count']==0
        state_count=len(times)
        array_catalog={key:dict(shape=list(a[key].shape),dtype=str(a[key].dtype)) for key in a.files}
    render=read(RENDER/'render_manifest.json');review=read(RENDER/'media_identity_review.json')
    assert render['mode']=='progress' and render['native_closed_result'] is False
    assert render['trajectory_sha256']==sha(trace) and render['frames']==23 and render['slow_motion']==1
    assert render['source65_unchanged'] is True and render['source65_before_after_render']==before['producer_source_65_before']
    assert render['runtime_before']==render['runtime_after']==before['runtime']
    assert render['physics_integration'] is False and render['mj_forward_called'] is False
    assert review['passed'] is True and review['all_original_sample_qpos_qvel_hashes_exact'] is True
    assert review['render_manifest_sha256']==sha(RENDER/'render_manifest.json')
    for name,digest in render['media_sha256'].items():assert sha(RENDER/name)==digest,name
    for name,digest in render['original_run_files_sha256_before_after'].items():assert sha(SNAPSHOT/name)==digest,name
    proof67=read(BASE/'robot_inertia_feedforward_v5_pure_proof.json')
    proof6=read(BASE/'inertia_rejection_artifacts_v5_pure_proof.json')
    for proof in (proof67,proof6):
        assert proof['exit_code']==0 and proof['frozen_files_unchanged'] is True
        assert proof['source_and_test_sha256_before']==proof['source_and_test_sha256_after']
        for name,digest in proof['source_and_test_sha256_before'].items():assert sha(BASE/name)==digest,name
    assert '67 passed in 0.78s' in proof67['actual_pytest_output']
    assert '6 passed in 0.58s' in proof6['actual_pytest_output']
    sources={}
    def add(name,path):
        assert name not in sources
        sources[name]=Path(path).resolve()
    for folder,prefix in ((SNAPSHOT,'native_snapshot'),(RENDER,'render')):
        for path in sorted(folder.rglob('*')):
            if path.is_file():add(prefix+'/'+str(path.relative_to(folder)),path)
    for name in before['source_files_before']:add('source_proof/'+name,BASE/name)
    for name in ('crest_seat_search_v5_execution_before.json','crest_seat_search_v5_frozen_inputs.json'):
        add('execution_provenance/'+name,BASE/name)
    for name,digest in before['producer_source_65_before'].items():
        path=PARENT/'producer_sources'/name;assert sha(path)==digest,name
        add('producer_sources/'+name,path)
    for name in ('software_tests.json','software_tests.log','manifest.json','verify_sources.py'):
        add('original402_software_proof/'+name,ROOT/'outputs/m8_table_supported/full_canonical_v1'/name)
    add('historical69_observer_contracts/crest_seat_search_v3_execution_before.json',BASE/'crest_seat_search_v3_execution_before.json')
    identities={name:writer.file_identity(path) for name,path in sources.items()}
    TARGET.parent.mkdir(parents=True,exist_ok=True)
    stage=TARGET.parent/(TARGET.name+'.packaging-'+uuid.uuid4().hex);stage.mkdir()
    try:
        artifacts={name:writer.write_artifact(path,stage,name,45_000_000) for name,path in sources.items()}
        for name,path in sources.items():assert writer.file_identity(path)==identities[name],name
        for name,digest in before['source_files_before'].items():assert sha(BASE/name)==digest,name
        for name,digest in before['producer_source_65_before'].items():assert sha(ROOT/name)==digest,name
        manifest=dict(status='historical_partial_phase_end_progress_snapshot',native_closed_result=False,
            full_allstep_force_and_inertia_ledgers_present=False,independent_closed_audit_present=False,
            trajectory_sha256=sha(trace),original_snapshot_boundary=progress['snapshot_boundary'],
            actual_saved_post_states=state_count,original_npz_array_catalog=array_catalog,
            original_progress_manifest_sha256=sha(SNAPSHOT/'progress_manifest.json'),
            snapshot_binding_sha256=sha(SNAPSHOT/'snapshot_binding.json'),
            snapshot_alias_binding_sha256=sha(SNAPSHOT/'snapshot_model_aliases.json'),
            render_manifest_sha256=sha(RENDER/'render_manifest.json'),
            source18_before_and_publication_verified=before['source_files_before'],
            source65_before_and_publication_verified=before['producer_source_65_before'],
            original_before_execution_sha256=sha(BASE/'crest_seat_search_v5_execution_before.json'),
            original_frozen_inputs_sha256=sha(BASE/'crest_seat_search_v5_frozen_inputs.json'),
            original_runtime=before['runtime'],original402_canonical_proof_is_separate=True,
            original_source67_pure_command_proof=proof67,original_source6_rejection_artifact_proof=proof6,
            historical69_and50_scopes_remain_separate=True,original_embedded_metadata_acceptance_flags_unmodified=True,
            parent_package='media/m8_table_supported/full_canonical_v1_failed_evidence',
            parent_package_ledger_sha256=sha(PARENT/'SHA256SUMS'),
            all_original_snapshot_and_media_bytes_preserved=True,archives_are_lossless=True,
            dropping_or_quantization=False,maximum_stored_file_bytes=45_000_000,
            scope='Actual cold V5 phase-end prefix through stopped direction event1.90625s only. Sampled controller window is shallow entry support; formed/interior0. All-step force/FF histories and final audit pending at snapshot capture. No full capture/open/reset/fresh trajectory or policy qualification. No integration/force replay/splice/native acceptance rewriting.',
            artifacts=artifacts)
        (stage/'package_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        shutil.copyfile(__file__,stage/'publication_helper_source.py')
        shutil.copyfile(ROOT/'outputs/m8_table_supported/crest_seat_search_v5_stopped_progress_README.md',stage/'README.md')
        files=sorted(path for path in stage.rglob('*') if path.is_file())
        assert max(path.stat().st_size for path in files)<=45_000_000
        (stage/'SHA256SUMS').write_text(''.join(f'{sha(path)}  {path.relative_to(stage)}\n' for path in files))
        stage.rename(TARGET)
        print(json.dumps(dict(target=str(TARGET.relative_to(ROOT)),files=len(files)+1,checksum_entries=len(files),artifacts=len(artifacts),total_bytes=sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),maximum_stored_bytes=max(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),ledger_sha256=sha(TARGET/'SHA256SUMS')),indent=2))
    except Exception:
        shutil.rmtree(stage)
        raise

if __name__=='__main__':main()
