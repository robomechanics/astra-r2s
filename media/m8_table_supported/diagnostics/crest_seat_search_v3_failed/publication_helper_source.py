"""Preserve one CLOSED failed cold crest-search branch and its derivatives."""
import hashlib
import importlib.util
import json
import shutil
import uuid
from pathlib import Path

import numpy as np

ROOT = Path('/workspace/astra-r2s')
BASE = ROOT/'outputs/m8_table_supported/diagnostics'
RUN = BASE/'crest_seat_search_v3'
AUDITS = BASE/'crest_seat_search_v3_audits'
RENDER = BASE/'crest_seat_search_v3_render'
TARGET = ROOT/'media/m8_table_supported/diagnostics/crest_seat_search_v3_failed'
PARENT_PACKAGE = ROOT/'media/m8_table_supported/full_canonical_v1_failed_evidence'
PUBLISHER_SHA = '8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'
HELPER_SHAS = {
    'reassemble_archives.py':'0706d299abfc70a0883aa8227ea3d89c7da420e961b4fc542fd696f3b05a3cfb',
    'restore_original_run_layout.py':'2a64428a4c1f3a96c9abce16f708576ac310d6e426eac54e38441097ac4cdf8b'}
AUDIT_BINDING_SHA = 'ae870d0ae2f6d53b989b1f22f1d1fc42f3bc36eb1ea6b866595efb9b76fc12ee'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for raw in iter(lambda: stream.read(1024*1024), b''):
            digest.update(raw)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def main():
    assert not TARGET.exists()
    publisher_path = ROOT/'outputs/m8_table_supported/publish_supported_feedback_run.py'
    assert sha(publisher_path) == PUBLISHER_SHA
    for name, digest in HELPER_SHAS.items():
        assert sha(ROOT/'outputs/m8_table_supported'/name) == digest, name
    spec = importlib.util.spec_from_file_location('frozen_byte_publisher', publisher_path)
    publisher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(publisher)
    before_path = BASE/'crest_seat_search_v3_execution_before.json'
    after_path = BASE/'crest_seat_search_v3_execution_after.json'
    before, after = read_json(before_path), read_json(after_path)
    assert after['source_files_unchanged'] is True
    assert after['producer_source_65_unchanged'] is True
    assert after['parent_inputs_unchanged'] is True and after['runtime_files_unchanged'] is True
    assert after['actual_native_exit_code'] == 1
    assert after['before_manifest_sha256'] == sha(before_path)
    report = read_json(RUN/'insertion_validation.json')
    assert report['passed'] is False and report['partial'] is True
    assert report['capture_or_open_reset_qualified'] is False
    assert report['full_fresh_trajectory_qualified'] is False
    assert report['aborted']['phase'] == 'stop_reverse_seat_1'
    for name, digest in after['closed_output_sha256'].items():
        assert sha(RUN/name) == digest, name
    for name, digest in before['source_files_before'].items():
        assert sha(BASE/name) == digest == after['source_files_after'][name], name
    assert sha(BASE/'crest_seat_search_v3.log') == after['original_stdout_stderr_sha256']
    declaration = read_json(RUN/'diagnostic_declaration.json')
    source_map = declaration['source_65_before']
    assert len(source_map) == 65
    assert source_map == before['producer_source_65_before'] == after['producer_source_65_after']
    parent_run = ROOT/'outputs/m8_table_supported/full_canonical_v1'
    for name, digest in declaration['parent_input_sha256'].items():
        assert sha(parent_run/name) == digest, name
    feedback = report['native_feedback_force_history']
    assert feedback['observed_physics_steps'] == 32508 and len(feedback['columns']) == 47
    assert sha(RUN/feedback['filename']) == feedback['sha256']
    with np.load(RUN/feedback['filename'], allow_pickle=False) as archive:
        assert all(name in archive.files for name in feedback['columns'])
        assert all(len(archive[name]) == 32508 for name in feedback['columns'])
        feedback_catalog = {name:dict(shape=list(archive[name].shape), dtype=str(archive[name].dtype))
            for name in archive.files}
    render = read_json(RENDER/'render_manifest.json')
    assert render['trajectory_sha256'] == sha(RUN/'insertion_trace.npz')
    assert render['slow_motion'] == 1 and render['frames'] == 20
    assert render['mj_forward_called'] is False and render['physics_integration'] is False
    for name, digest in render['media_sha256'].items():
        assert sha(RENDER/name) == digest, name
    # Independent audit closure is required, not assumed from native source/exit.
    audit_binding_path = AUDITS/'independent_audit_binding.json'
    audit_binding = read_json(audit_binding_path)
    assert sha(audit_binding_path) == AUDIT_BINDING_SHA
    assert audit_binding['schema'] == 'independent-cold-crest-audit-binding-v1'
    assert audit_binding['closed'] is True and audit_binding['source_frozen'] is True
    assert audit_binding['source65_unchanged'] is True
    assert audit_binding['original_native_passed'] is False
    assert type(audit_binding['original_native_exit_code']) is int
    assert audit_binding['original_native_exit_code'] == after['actual_native_exit_code'] == 1
    assert audit_binding['original_native_trace_sha256'] == sha(RUN/'insertion_trace.npz')
    assert audit_binding['original_native_report_sha256'] == sha(RUN/'insertion_validation.json')
    assert audit_binding['original_native_aborted'] == report['aborted']
    assert audit_binding['original_harness_sha256'] == sha(RUN/'controller_source.py')
    assert audit_binding['original_experimental_observer_sha256'] == sha(RUN/'experimental_observer_source.py')
    assert audit_binding['original_native_steps'] == 32508
    assert audit_binding['original_raw_closed_output_sha256'] == after['closed_output_sha256']
    for item in audit_binding['auditor_artifacts'].values():
        assert sha(AUDITS/item['path']) == item['sha256'], item['path']
    primary = read_json(AUDITS/'independent_cold_crest_audit.json')
    assert sha(AUDITS/'independent_cold_crest_audit.json') == audit_binding['primary_report_sha256']
    assert sha(AUDITS/'independent_cold_crest_audit_v1.py') == audit_binding['reader_source_sha256']
    assert primary['reader_source_sha256'] == audit_binding['reader_source_sha256']
    assert primary['original_passed'] is False and primary['diagnostic_completed'] is False
    assert primary['full_fresh_trajectory_qualified'] is False
    assert primary['capture_or_open_reset_qualified'] is False
    assert primary['closed_original_exit_code'] == 1
    assert primary['original_aborted'] == report['aborted']
    assert primary['source_and_parent_identity_verified'] is True
    assert primary['selected_source_bundle_verified'] is True
    assert primary['every_step_observer_and_saved_event_consistency'] is True
    proof = read_json(AUDITS/'independent_cold_reader_software_proof.json')
    assert proof == audit_binding['dedicated_reader_proof']
    assert proof['passed'] is True and proof['tests_passed'] == 10 and proof['return_code'] == 0
    assert proof['all_sources_unchanged'] is True and proof['source_before'] == proof['source_after']
    assert proof['reader_source_sha256'] == audit_binding['reader_source_sha256']
    assert proof['test_source_sha256'] == sha(AUDITS/'test_independent_cold_crest_audit_v1.py')
    assert proof['test_log_sha256'] == sha(AUDITS/'independent_cold_reader_tests.log')
    sources = {}

    def add(name, path):
        assert name not in sources, name
        sources[name] = Path(path).resolve()

    # All original/native and later frozen independent-audit bytes are retained.
    for path in sorted(RUN.rglob('*')):
        if path.is_file():
            add('native_v3/'+str(path.relative_to(RUN)), path)
    for path in sorted(RENDER.rglob('*')):
        if path.is_file():
            add('render/'+str(path.relative_to(RENDER)), path)
    for path in sorted(AUDITS.rglob('*')):
        if path.is_file():
            add('independent_audits/'+str(path.relative_to(AUDITS)), path)
    for name in before['source_files_before']:
        add('source_proof/'+name, BASE/name)
    for name in ('crest_seat_search_v3_execution_before.json', 'crest_seat_search_v3_execution_after.json',
            'crest_seat_search_v3.log', 'crest_seat_search_v3_boundary_diagnosis.json',
            'crest_seat_search_v3_audit_plan.md'):
        add('execution_provenance/'+name, BASE/name)
    # Exact aliases resolve the preserved binder's original ../ sibling links.
    for path in (before_path, after_path):
        add(path.name, path)
    add('publication_dependencies/publish_supported_feedback_run.py', publisher_path)
    for name in ('publish_supported_feedback_preservation_tests.json',
            'test_publish_supported_feedback_preservation.py'):
        add('publication_dependencies/'+name, ROOT/'outputs/m8_table_supported'/name)
    for name, digest in source_map.items():
        source = PARENT_PACKAGE/'producer_sources'/name
        assert sha(source) == digest, name
        add('producer_sources/'+name, source)
    for name in ('software_tests.json', 'software_tests.log', 'manifest.json', 'verify_sources.py'):
        add('original402_software_proof/'+name, parent_run/name)
    snapshot = {name:publisher.file_identity(path) for name, path in sources.items()}
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    stage = TARGET.parent/(TARGET.name+'.packaging-'+uuid.uuid4().hex)
    stage.mkdir()
    try:
        artifacts = {name:publisher.write_artifact(path, stage, name, 45_000_000)
            for name, path in sources.items()}
        for name, path in sources.items():
            assert publisher.file_identity(path) == snapshot[name], name
        original_identities = {name:publisher.file_identity(RUN/name)
            for name in after['closed_output_sha256']}
        manifest = {
            'status':'closed_failed_cold_diagnostic', 'original_native_passed':False,
            'original_native_partial':True, 'original_native_exit_code':1,
            'original_aborted':report['aborted'],
            'native_duration_s':after['original_native_final_time_s'],
            'original_wall_seconds':after['original_native_wall_seconds'],
            'trajectory_sha256':sha(RUN/'insertion_trace.npz'),
            'original_report_sha256':sha(RUN/'insertion_validation.json'),
            'original_feedback_sha256':feedback['sha256'], 'original_feedback_columns':feedback['columns'],
            'original_feedback_array_catalog':feedback_catalog, 'original_native_rows':32508,
            'all_original_closed_files_preserved':True,
            'original_run_file_to_published_artifact_names':{name:['native_v3/'+name] for name in original_identities},
            'original_run_file_identities':original_identities,
            'whole_trial_files_and_independent_audits_preserved':True,
            'original_independent_binding_sha256':sha(audit_binding_path),
            'independent_reader_original_failed_flags_preserved':True,
            'independent_reader_source_sha256':audit_binding['reader_source_sha256'],
            'independent_reader10_tests_are_separate':proof,
            'binder_sibling_aliases_byte_identical':{
                path.name:sha(path) for path in (before_path, after_path)},
            'original_final_cleared_event':report['final_experimental_crest_direction_event'],
            'original_live_request_event':render['original_stop_request_event'],
            'cold_parent_lineage':declaration,
            'parent_package':'media/m8_table_supported/full_canonical_v1_failed_evidence',
            'parent_package_ledger_sha256':sha(PARENT_PACKAGE/'SHA256SUMS'),
            'original402_software_proof_sha256':sha(parent_run/'software_tests.json'),
            'original_producer_source_count':65, 'original_producer_sources':source_map,
            'observer_contract_proof':before['observer_contract_tests'],
            'observer_contract_standalone_log_saved':False,
            'observer_proof_provenance':'Original execution_before observer_contract_tests + exact frozen source/test bytes; original actual69/.63s tool result was recorded there, no standalone log was saved and none is synthesized.',
            'publisher_shared_lossless_writer_sha256':PUBLISHER_SHA,
            'publisher_shared19_storage_tests_are_separate':'No new native or original402/65 qualification; reused frozen byte writer only.',
            'archives_are_lossless':True, 'dropping_or_quantization':False,
            'maximum_stored_file_bytes':45_000_000,
            'scope':'Separate cold post-state branch, initialized from exact full_v1 checkpoint13.1949s with no solver warmstart/old force windows. A live50µm crestreturn requests CLOSED stop; abrupt stop radial guard fails after6.25ms. No quiet-stop confirmation, forward scan, formed capture, opening/reset or full fresh trajectory. Original final epoch-clear flags and prior request metadata both preserved; no splice or retrospective pass.',
            'artifacts':artifacts}
        (stage/'package_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        shutil.copyfile(__file__, stage/'publication_helper_source.py')
        for name in ('reassemble_archives.py', 'restore_original_run_layout.py'):
            assert sha(ROOT/'outputs/m8_table_supported'/name) == HELPER_SHAS[name]
            shutil.copyfile(ROOT/'outputs/m8_table_supported'/name, stage/name)
        shutil.copyfile(ROOT/'outputs/m8_table_supported/crest_seat_search_v3_README.md', stage/'README.md')
        paths = sorted(path for path in stage.rglob('*') if path.is_file())
        assert max(path.stat().st_size for path in paths) <= 45_000_000
        (stage/'SHA256SUMS').write_text(''.join(f'{sha(path)}  {path.relative_to(stage)}\n' for path in paths))
        assert not TARGET.exists()
        stage.rename(TARGET)
        print(json.dumps(dict(target=str(TARGET.relative_to(ROOT)), files=len(paths)+1,
            artifacts=len(artifacts), checksum_entries=len(paths),
            checksum_ledger_sha256=sha(TARGET/'SHA256SUMS'),
            original_passed=False, original_partial=True), indent=2))
    except Exception:
        shutil.rmtree(stage)
        raise


if __name__ == '__main__':
    main()
