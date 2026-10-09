"""Freeze complete closed native and derivative inventories; no native calls."""
import hashlib
import json
from pathlib import Path

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3'
OUT = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3_audits'
BINDER = OUT/'independent_audit_binding.json'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inventory(directory, excluded=None):
    return {str(p.relative_to(directory)):sha(p) for p in sorted(directory.rglob('*'))
            if p.is_file() and p != excluded}


assert not BINDER.exists()
after_path = RUN/'run_publication_identity_after.json'
assert sha(after_path) == 'f91f91beaaf7a8ee493fc124b068d469c72fee06be38f9920dce955e53aa2792'
after = json.loads(after_path.read_text())
execution = json.loads((OUT/'supplemental_fresh_reset_speed2_execution.json').read_text())
serial = json.loads((OUT/'serial_audit_result.json').read_text())
original = json.loads((RUN/'insertion_validation.json').read_text())
official = json.loads((OUT/'independent_supported_audit.json').read_text())
supplemental = json.loads((OUT/'supplemental_fresh_reset_speed2_audit.json').read_text())
identity = json.loads((OUT/'source_identity_after.json').read_text())
assert execution['reader_exit_code'] == 0 and execution['original_bytes_unchanged'] is True
assert execution['reader_workflow_and_reused_source_unchanged'] is True
assert serial['reader_execution_completed'] is True and serial['identity_and_original_bytes_unchanged'] is True
assert supplemental['reader_source_sha256'] == 'a9c2b710f9f840677402dbc290e263bb29d17fb502408ce2072d4589a59acaea'
assert sha(OUT/'supplemental_fresh_reset_speed2_audit.json') == execution['report_sha256']
assert supplemental['original_report_passed'] == original['passed'] == official['original_report_passed']
assert supplemental['original_acceptance_checks'] == original['acceptance_checks'] == official['original_acceptance_checks']
assert supplemental['independent_official_passed'] == official['passed']
assert identity['producer_commit'] == after['producer_commit'] == '9ae1a9fe76968a4013ea6c39e67718026622b9c0'
assert identity['software_proof_sha256'] == '9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
assert identity['source_files_unchanged'] == 74 and identity['software_tests_passed'] == 672
original_files = inventory(RUN)
assert original_files == execution['original_run_files_sha256_before'] == execution['original_run_files_sha256_after']
assert original_files == serial['original_run_files_sha256_before'] == serial['original_run_files_sha256_after']
audit_files = inventory(OUT, BINDER)
binder = {
    'schema':'closed-supported-complete-audit-binding-v3', 'closed':True,
    'original_run_relative_path':str(RUN.relative_to(ROOT)),
    'producer_commit':after['producer_commit'], 'native_child_exit_code':after['native_exit_code'],
    'original_run_files_sha256':original_files, 'audit_files_sha256':audit_files,
    'original_file_count':len(original_files), 'derivative_file_count_excluding_binder':len(audit_files),
    'original_native_report_passed':original['passed'],
    'original_native_acceptance_checks_passed':sum(v['passed'] for v in original['acceptance_checks'].values()),
    'original_native_acceptance_check_count':len(original['acceptance_checks']),
    'official_supported_report_passed':official['passed'],
    'official_independent_acceptance_checks_passed':sum(v['passed'] for v in official['independent_acceptance_checks'].values()),
    'official_independent_acceptance_check_count':len(official['independent_acceptance_checks']),
    'software_proof_sha256':identity['software_proof_sha256'],
    'software_tests_passed':identity['software_tests_passed'], 'software_source_files':identity['source_files_unchanged'],
    'supplemental_reader_source_sha256':supplemental['reader_source_sha256'],
    'supplemental_report_sha256':execution['report_sha256'],
    'scope':'Complete original native and derivative file inventories. closed:true denotes closure, not a physics qualification. Original native27/27, independent19/19, software672/74 and supplemental execution remain separate evidence scopes. No added proof counts, force replay, native integration, original-file rewrite or acceptance change.'}
with BINDER.open('x') as stream:
    json.dump(binder, stream, indent=2, allow_nan=False)
    stream.write('\n')
assert inventory(RUN) == original_files and inventory(OUT, BINDER) == audit_files
print(json.dumps({'binding_sha256':sha(BINDER), 'original_files':len(original_files),
                  'audit_files_excluding_binder':len(audit_files),
                  'original_passed':original['passed'], 'official_passed':official['passed']}))
