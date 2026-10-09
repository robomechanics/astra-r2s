"""Preserve one CLOSED failed cold crest-search branch and its derivatives."""
import ast
import hashlib
import importlib.util
import json
import shutil
import sys
import uuid
from pathlib import Path

import numpy as np

ROOT = Path('/workspace/astra-r2s')
BASE = ROOT/'outputs/m8_table_supported/diagnostics'
RUN = BASE/'crest_seat_search_v4'
AUDITS = BASE/'crest_seat_search_v4_audits'
AUDITS_V2 = BASE/'crest_seat_search_v4_audits_v2'
RENDER = BASE/'crest_seat_search_v4_render'
TARGET = ROOT/'media/m8_table_supported/diagnostics/crest_seat_search_v4_failed'
PARENT_PACKAGE = ROOT/'media/m8_table_supported/full_canonical_v1_failed_evidence'
PUBLISHER_SHA = '8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'
HELPER_SHAS = {
    'reassemble_archives.py':'0706d299abfc70a0883aa8227ea3d89c7da420e961b4fc542fd696f3b05a3cfb',
    'restore_original_run_layout.py':'2a64428a4c1f3a96c9abce16f708576ac310d6e426eac54e38441097ac4cdf8b'}
AUDIT_BINDING_SHA = 'd524e6f70f004fcce46b43a6de0c005a60ff09553f6f384340587d415a109db4'
AUDIT_V2_BINDING_SHA = '3c45982e821069c9ac1a4f4636764fa267356389f05606bf0af7d20871304ff0'
CORRECTION_SHA = '6f862f0c7f2d7b9ec4eb6ab0ab3fc5d19bef92a65905fd0ee6e37e843060d16a'


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
    before_path = BASE/'crest_seat_search_v4_execution_before.json'
    after_path = BASE/'crest_seat_search_v4_execution_after.json'
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
    assert sha(BASE/'crest_seat_search_v4.log') == after['original_stdout_stderr_sha256']
    declaration = read_json(RUN/'diagnostic_declaration.json')
    historical_before = read_json(BASE/'crest_seat_search_v3_execution_before.json')
    C2_proof = read_json(BASE/'reverse_brake_v4_pure_proof.json')
    assert C2_proof['actual_pytest_output'] == '50 passed in 0.15s'
    assert C2_proof['source_sha256'] == sha(RUN/'robot_yaw_brake_source.py')
    assert C2_proof['test_sha256'] == sha(BASE/'test_reverse_brake_v4.py')
    source_map = declaration['source_65_before']
    assert len(source_map) == 65
    assert source_map == before['producer_source_65_before'] == after['producer_source_65_after']
    parent_run = ROOT/'outputs/m8_table_supported/full_canonical_v1'
    for name, digest in declaration['parent_input_sha256'].items():
        assert sha(parent_run/name) == digest, name
    feedback = report['native_feedback_force_history']
    assert feedback['observed_physics_steps'] == 34182 and len(feedback['columns']) == 50
    assert sha(RUN/feedback['filename']) == feedback['sha256']
    with np.load(RUN/feedback['filename'], allow_pickle=False) as archive:
        assert all(name in archive.files for name in feedback['columns'])
        assert all(len(archive[name]) == 34182 for name in feedback['columns'])
        feedback_catalog = {name:dict(shape=list(archive[name].shape), dtype=str(archive[name].dtype))
            for name in archive.files}
    render = read_json(RENDER/'render_manifest.json')
    assert render['trajectory_sha256'] == sha(RUN/'insertion_trace.npz')
    assert render['slow_motion'] == 1 and render['frames'] == 21
    assert render['mj_forward_called'] is False and render['physics_integration'] is False
    for name, digest in render['media_sha256'].items():
        assert sha(RENDER/name) == digest, name
    # Independent audit closure is required, not assumed from native source/exit.
    audit_binding_path = AUDITS/'independent_audit_binding.json'
    audit_binding = read_json(audit_binding_path)
    assert sha(audit_binding_path) == AUDIT_BINDING_SHA
    assert audit_binding['schema'] == 'independent-cold-crest-c2-audit-binding-v1'
    assert audit_binding['closed'] is True and audit_binding['source_frozen'] is True
    assert audit_binding['producer_source65_unchanged'] is True
    assert audit_binding['original_native_passed'] is False
    assert type(audit_binding['original_native_exit_code']) is int
    assert audit_binding['original_native_exit_code'] == after['actual_native_exit_code'] == 1
    assert audit_binding['original_native_trace_sha256'] == sha(RUN/'insertion_trace.npz')
    assert audit_binding['original_native_report_sha256'] == sha(RUN/'insertion_validation.json')
    assert audit_binding['original_native_aborted'] == report['aborted']
    assert audit_binding['original_harness_sha256'] == sha(RUN/'controller_source.py')
    assert audit_binding['original_CrestV3_observer_sha256'] == sha(RUN/'experimental_observer_source.py')
    assert audit_binding['original_C2_helper_sha256'] == sha(RUN/'robot_yaw_brake_source.py')
    assert audit_binding['original_native_steps'] == 34182
    assert audit_binding['original_raw_closed_output_sha256'] == after['closed_output_sha256']
    for field, path in (('outer_execution_before', before_path), ('outer_execution_after', after_path)):
        assert audit_binding[field]['sha256'] == sha(path)
        assert (AUDITS/audit_binding[field]['path']).resolve() == path.resolve()
    assert audit_binding['no_canonical_or_native_original_mutation'] is True
    for item in audit_binding['auditor_artifacts'].values():
        assert sha(AUDITS/item['path']) == item['sha256'], item['path']
    primary = read_json(AUDITS/'independent_cold_crest_brake_audit.json')
    assert sha(AUDITS/'independent_cold_crest_brake_audit.json') == audit_binding['primary_report_sha256']
    assert sha(AUDITS/'independent_cold_crest_brake_audit_v1.py') == audit_binding['reader_source_sha256']
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
    assert proof['passed'] is True and proof['tests_passed'] == 17 and proof['return_code'] == 0
    assert proof['all_sources_unchanged'] is True and proof['source_before'] == proof['source_after']
    assert proof['reader_source_sha256'] == audit_binding['reader_source_sha256']
    assert proof['test_source_sha256'] == sha(AUDITS/'test_independent_cold_crest_brake_audit_v1.py')
    assert proof['test_log_sha256'] == sha(AUDITS/'independent_cold_reader_tests.log')
    assert proof['inertia_source_sha256'] == sha(AUDITS/'independent_robot_inertia_v1.py')
    inertia = read_json(AUDITS/'independent_robot_inertia_feasibility.json')
    inertia_note = read_json(AUDITS/'robot_inertia_scope_note.json')
    passive_binding = read_json(AUDITS/'equal_model_passive_property_binding.json')
    assert audit_binding['robot_inertia_scope_note_sha256'] == sha(AUDITS/'robot_inertia_scope_note.json')
    assert inertia['source_sha256'] == proof['inertia_source_sha256']
    assert inertia['trace_sha256'] == sha(RUN/'insertion_trace.npz')
    assert inertia['original_passed'] is False
    assert inertia['evaluated_archived_states'] == len(inertia['rows']) == 19
    assert inertia['runtime'] == before['runtime'] == declaration['runtime'] == report['runtime']
    assert inertia['model_identity']['model_xml_sha256'] == sha(RUN/'scene.xml') == report['model_xml_sha256']
    assert inertia['model_identity']['model_fingerprint'] == report['model_fingerprint']
    with np.load(RUN/'insertion_trace.npz', allow_pickle=False) as trace:
        assert all(row['saved_post_state_time_s'] in trace['time'] for row in inertia['rows'])
    assert inertia_note['original_geometry_mass_source_sha256'] == inertia['source_sha256']
    assert inertia_note['original_geometry_mass_report_sha256'] == sha(AUDITS/'independent_robot_inertia_feasibility.json')
    assert inertia_note['evaluated_archived_actual_states'] == 19
    assert inertia_note['original_failure_preserved'] == report['aborted']
    assert inertia_note['no_actual_feedforward_applied'] is True
    assert inertia_note['no_capture_or_full_trajectory_proof'] is True
    assert passive_binding['historical_report_sha256'] == sha(AUDITS/'historical_equal_model_free_joint_properties.json')
    assert passive_binding['current_v4_trace_sha256'] == sha(RUN/'insertion_trace.npz')
    assert passive_binding['scene_xml_sha256'] == sha(RUN/'scene.xml')
    assert passive_binding['scene_zip_sha256'] == sha(RUN/'supported_scene.zip')
    assert passive_binding['runtime'] == declaration['runtime']
    assert passive_binding['model_fingerprint'] == report['model_fingerprint']
    assert passive_binding['no_new_v4_force_solve_or_integration'] is True
    corrected_binding = read_json(AUDITS_V2/'independent_audit_binding.json')
    assert sha(AUDITS_V2/'independent_audit_binding.json') == AUDIT_V2_BINDING_SHA
    assert corrected_binding['schema'] == 'independent-cold-crest-c2-audit-binding-v2-prose-correction'
    assert corrected_binding['closed'] is True and corrected_binding['source_frozen'] is True
    assert corrected_binding['original_v1_tree_unchanged'] is True
    assert corrected_binding['original_v1_binding_sha256'] == AUDIT_BINDING_SHA
    assert corrected_binding['prose_only_correction_binding_sha256'] == CORRECTION_SHA
    for field in ('original_native_trace_sha256', 'original_native_report_sha256',
            'original_native_passed', 'original_native_aborted', 'original_native_exit_code',
            'original_native_steps', 'original_harness_sha256', 'original_CrestV3_observer_sha256',
            'original_C2_helper_sha256', 'original_raw_closed_output_sha256',
            'outer_execution_before', 'outer_execution_after', 'producer_source65_unchanged',
            'robot_inertia_scope_note_sha256', 'no_canonical_or_native_original_mutation'):
        assert corrected_binding[field] == audit_binding[field], field
    for item in corrected_binding['auditor_artifacts'].values():
        assert sha(AUDITS_V2/item['path']) == item['sha256'], item['path']
    correction = read_json(AUDITS_V2/'v1_prose_only_correction_binding.json')
    assert sha(AUDITS_V2/'v1_prose_only_correction_binding.json') == CORRECTION_SHA
    for field in ('original_binding','original_reader','original_report','corrected_reader',
            'corrected_report','narrow_diff','regression_proof'):
        item = correction[field]
        assert sha(AUDITS_V2/item['path']) == item['sha256'], field
    assert correction['all_original_v1_artifact_hashes_unchanged'] is True
    assert correction['all_original_native_fields_checks_criteria_unchanged'] is True
    assert correction['original_passed'] is False and correction['original_aborted'] == report['aborted']
    corrected_primary = read_json(AUDITS_V2/'independent_cold_crest_brake_audit.json')
    allowed_changes = {'scientific_scope','reader_source_sha256'}
    assert set(primary) == set(corrected_primary)
    assert {key for key in primary if primary[key] != corrected_primary[key]} == allowed_changes
    assert corrected_primary['reader_source_sha256'] == corrected_binding['reader_source_sha256']
    assert correction['original_scientific_scope'] == primary['scientific_scope']
    assert correction['corrected_scientific_scope'] == corrected_primary['scientific_scope']
    def normalized_reader(path):
        tree = ast.parse(path.read_text())
        changed = 0
        for node in ast.walk(tree):
            if isinstance(node, ast.Dict):
                for index, key in enumerate(node.keys):
                    if isinstance(key, ast.Constant) and key.value == 'scientific_scope':
                        node.values[index] = ast.Constant(value='V4_PROSE_CORRECTION_SENTINEL')
                        changed += 1
        assert changed == 1
        return ast.dump(tree, include_attributes=False)
    assert normalized_reader(AUDITS/'independent_cold_crest_brake_audit_v1.py') == normalized_reader(AUDITS_V2/'independent_cold_crest_brake_audit_v1.py')
    corrected_proof = read_json(AUDITS_V2/'independent_cold_reader_software_proof.json')
    assert corrected_proof == corrected_binding['dedicated_reader_proof']
    assert corrected_proof['passed'] is True and corrected_proof['tests_passed'] == 17 and corrected_proof['return_code'] == 0
    assert corrected_proof['all_sources_unchanged'] is True and corrected_proof['source_before'] == corrected_proof['source_after']
    assert corrected_proof['reader_source_sha256'] == corrected_binding['reader_source_sha256'] == sha(AUDITS_V2/'independent_cold_crest_brake_audit_v1.py')
    assert corrected_binding['primary_report_sha256'] == sha(AUDITS_V2/'independent_cold_crest_brake_audit.json')
    for field, name in (('test_source_sha256','test_independent_cold_crest_brake_audit_v1.py'),
            ('inertia_source_sha256','independent_robot_inertia_v1.py'),
            ('test_log_sha256','independent_cold_reader_tests.log')):
        assert corrected_proof[field] == sha(AUDITS_V2/name)
    for name in ('robot_inertia_scope_note.json', 'historical_equal_model_free_joint_properties.json',
            'test_independent_cold_crest_brake_audit_v1.py', 'independent_robot_inertia_v1.py',
            'equal_model_passive_property_binding.json', 'independent_robot_inertia_feasibility.json'):
        assert sha(AUDITS_V2/name) == sha(AUDITS/name), name
    if sys.argv[1:] == ['--validate-only']:
        print(json.dumps({'validated_original_v4':True, 'native_rows':34182, 'feedback_columns':50, 'original_audit_binding_sha256':AUDIT_BINDING_SHA, 'corrected_audit_binding_sha256':AUDIT_V2_BINDING_SHA, 'native_passed':False, 'no_package_written':True}))
        return
    assert not sys.argv[1:], sys.argv[1:]
    sources = {}

    def add(name, path):
        assert name not in sources, name
        sources[name] = Path(path).resolve()

    # All original/native and later frozen independent-audit bytes are retained.
    for path in sorted(RUN.rglob('*')):
        if path.is_file():
            add('native_v4/'+str(path.relative_to(RUN)), path)
    for path in sorted(RENDER.rglob('*')):
        if path.is_file():
            add('render/'+str(path.relative_to(RENDER)), path)
    for path in sorted(AUDITS.rglob('*')):
        if path.is_file():
            add('independent_audits/'+str(path.relative_to(AUDITS)), path)
    for path in sorted(AUDITS_V2.rglob('*')):
        if path.is_file():
            add('independent_audits_v2/'+str(path.relative_to(AUDITS_V2)), path)
    # Complete byte-exact original-tree aliases resolve the correction's ../ historical links.
    for path in sorted(AUDITS.rglob('*')):
        if path.is_file():
            add('crest_seat_search_v4_audits/'+str(path.relative_to(AUDITS)), path)
    for name in before['source_files_before']:
        add('source_proof/'+name, BASE/name)
    for name in ('crest_seat_search_v4_execution_before.json', 'crest_seat_search_v4_execution_after.json',
            'crest_seat_search_v4.log', 'crest_seat_search_v4_boundary_diagnosis.json',
            'crest_seat_search_v4_audit_plan.md'):
        add('execution_provenance/'+name, BASE/name)
    # Exact aliases resolve the preserved binder's original ../ sibling links.
    for path in (before_path, after_path):
        add(path.name, path)
    add('publication_dependencies/publish_supported_feedback_run.py', publisher_path)
    for name in ('publish_supported_feedback_preservation_tests.json',
            'test_publish_supported_feedback_preservation.py'):
        add('publication_dependencies/'+name, ROOT/'outputs/m8_table_supported'/name)
    add('historical69_observer_contracts/crest_seat_search_v3_execution_before.json', BASE/'crest_seat_search_v3_execution_before.json')
    add('execution_provenance/crest_seat_search_v4_frozen_inputs.json', BASE/'crest_seat_search_v4_frozen_inputs.json')
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
            'original_feedback_array_catalog':feedback_catalog, 'original_native_rows':34182,
            'all_original_closed_files_preserved':True,
            'original_run_file_to_published_artifact_names':{name:['native_v4/'+name] for name in original_identities},
            'original_run_file_identities':original_identities,
            'whole_trial_files_and_independent_audits_preserved':True,
            'original_independent_binding_sha256':sha(audit_binding_path),
            'independent_reader_original_failed_flags_preserved':True,
            'independent_reader_source_sha256':audit_binding['reader_source_sha256'],
            'independent_reader17_tests_are_separate':proof,
            'corrected_independent_binding_sha256':AUDIT_V2_BINDING_SHA,
            'corrected_independent_reader_source_sha256':corrected_binding['reader_source_sha256'],
            'corrected_independent_reader17_tests_are_separate_rerun':corrected_proof,
            'prose_only_correction_binding_sha256':CORRECTION_SHA,
            'original_and_corrected_audit_trees_preserved':True,
            'prose_correction_native_fields_and_AST_predicates_unchanged':True,
            'historical_audit_tree_aliases_resolve_correction_links':True,
            'prospective_robot_inertia_scope_note':inertia_note,
            'prospective_robot_inertia_source_sha256':inertia['source_sha256'],
            'prospective_robot_inertia_report_sha256':sha(AUDITS/'independent_robot_inertia_feasibility.json'),
            'prospective_inertia_is_19_sample_held_finger_approximation_not_executed_control':True,
            'historical_equal_model_passive_property_binding':passive_binding,
            'right_original_force_frame_limitation':primary['right_local_force_frame_scope'],
            'preserved_primary_scientific_scope_stale_radial_numeral':'Primary report scientific_scope retains historical V3 prose numeral150.848um. Original V4 report/binder/reader original_aborted and raw data give150.0112715108498um. Original audit bytes are preserved unchanged; no retrospective correction to native values or flags.',
            'binder_sibling_aliases_byte_identical':{
                path.name:sha(path) for path in (before_path, after_path)},
            'original_final_cleared_event':report['final_experimental_crest_direction_event'],
            'original_robot_yaw_brake':report['robot_yaw_brake'],
            'actual_native_brake_elapsed_s':report['aborted']['time']-render['original_stop_request_event']['time_s'],
            'original_live_request_event':render['original_stop_request_event'],
            'cold_parent_lineage':declaration,
            'parent_package':'media/m8_table_supported/full_canonical_v1_failed_evidence',
            'parent_package_ledger_sha256':sha(PARENT_PACKAGE/'SHA256SUMS'),
            'original402_software_proof_sha256':sha(parent_run/'software_tests.json'),
            'original_producer_source_count':65, 'original_producer_sources':source_map,
            'historical69_observer_contract_proof':historical_before['observer_contract_tests'],
            'observer_contract_standalone_log_saved':False,
            'observer_proof_provenance':'Historical V3 execution_before observer_contract_tests + unchanged frozen observer/test bytes. Historical69/.63s is not a qualification of new V4 C2 harness. No standalone69 log was saved or synthesized.',
            'C2_robot_command50_contract_proof':C2_proof,
            'publisher_shared_lossless_writer_sha256':PUBLISHER_SHA,
            'publisher_shared19_storage_tests_are_separate':'No new native or original402/65 qualification; reused frozen byte writer only.',
            'archives_are_lossless':True, 'dropping_or_quantization':False,
            'maximum_stored_file_bytes':45_000_000,
            'scope':'Separate cold post-state branch, initialized from exact full_v1 checkpoint13.1949s with no solver warmstart/old force windows. A live50µm crestreturn requests CLOSED stop; planned150ms C2 command continuation radial guard fails89.95ms into braking. No quiet-stop confirmation, forward scan, formed capture, opening/reset or full fresh trajectory. Original final epoch-clear flags and prior request metadata both preserved; no splice or retrospective pass.',
            'artifacts':artifacts}
        (stage/'package_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        shutil.copyfile(__file__, stage/'publication_helper_source.py')
        for name in ('reassemble_archives.py', 'restore_original_run_layout.py'):
            assert sha(ROOT/'outputs/m8_table_supported'/name) == HELPER_SHAS[name]
            shutil.copyfile(ROOT/'outputs/m8_table_supported'/name, stage/name)
        shutil.copyfile(ROOT/'outputs/m8_table_supported/crest_seat_search_v4_README.md', stage/'README.md')
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
