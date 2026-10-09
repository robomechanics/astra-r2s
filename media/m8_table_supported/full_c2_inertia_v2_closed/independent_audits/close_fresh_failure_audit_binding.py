"""Freeze complete old-da69 failure provenance, without touching a native run."""
import hashlib
import json
from pathlib import Path


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()


audit=Path(__file__).resolve().parent
repo=audit.parents[2]
run=audit.with_name('full_c2_inertia_v2')
old_clone=Path('/workspace/astra-r2s-supported-v5-development')
native=json.loads((run/'insertion_validation.json').read_text())
closure=json.loads((run/'run_publication_identity_after.json').read_text())
official=json.loads((audit/'independent_supported_audit.json').read_text())
report=json.loads((audit/'supplemental_open_reset_contact_audit.json').read_text())
proof=json.loads((audit/'supplemental_reader_pure_proof.json').read_text())
identity=json.loads((audit/'replay_source_identity_old_clone.json').read_text())
execution=json.loads((audit/'replay_source_identity_old_clone_execution.json').read_text())
producer='da69a9cd44a8312cc7b97365faf5e09c27a646e2'
assert native['passed'] is False and native['partial'] is False and native['aborted'] is not None
assert closure['producer_commit']==identity['producer_commit']==execution['original_native_producer']==producer
assert closure['native_exit_code']==identity['closure_native_exit_code']==1 and execution['exit_code']==0
assert identity['source_files_unchanged']==74 and identity['complete_current_and_archived_tested_sources_match_original'] is True
assert execution['repository_root_checked']==str(old_clone)
assert execution['identity_sha256']==digest(audit/'replay_source_identity_old_clone.json')
for relative,sha in closure['source_hashes_before'].items():
    assert digest(old_clone/relative)==sha==closure['source_hashes_after'][relative]
assert len(closure['source_hashes_before'])==74
assert report['original_report_passed']==native['passed'] and report['source_original_acceptance_checks']==native['acceptance_checks']==official['original_acceptance_checks']
assert report['original_native_steps']==441657 and report['native_child_exit_code']==1
assert report['reader_source_sha256']==digest(audit/'supplemental_open_reset_contact_audit.py')
assert proof['passed'] is True and proof['tests_passed']==10 and proof['all_sources_unchanged'] is True
for relative,sha in proof['source_sha256_before'].items():
    assert digest(audit/relative)==sha==proof['source_sha256_after'][relative]
assert proof['log_sha256']==digest(audit/'supplemental_reader_pure_proof.log')
assert digest(audit/'geometry_review/manifest.json')=='0b05df823cb5f25c500bd5eca1f4ac5e88931040b706a5a7770e1d4da75c8709'
assert digest(audit/'geometry_review/reset_reachability_report.json')=='79ec32373c688a279b238fd6f0d9715e990ad64bf85d6f2db106f8aea6bd8ee6'
plan=repo/'outputs/m8_table_supported/fresh_da69_open_reset_contact_audit_plan.md'
(audit/plan.name).write_bytes(plan.read_bytes())
raw_files={str(p.relative_to(run)):digest(p) for p in run.rglob('*') if p.is_file()}
audit_files={str(p.relative_to(audit)):digest(p) for p in audit.rglob('*') if p.is_file() and p!=audit/'independent_audit_binding.json'}
binding={'kind':'Closed fresh da69 native failure and independent audits binding','closed':True,
 'original_run_relative_path':str(run.relative_to(repo)),'producer_commit':producer,'native_child_exit_code':1,
 'original_run_files_sha256':raw_files,'audit_files_sha256':audit_files,
 'original_native_passed':False,'original_native_partial':False,'original_native_steps':441657,
 'original_native_final_time_s':native['aborted']['time'],'source_original_acceptance_checks':native['acceptance_checks'],
 'source_original_passed_checks':sum(v['passed'] for v in native['acceptance_checks'].values()),
 'source_original_total_checks':len(native['acceptance_checks']),
 'official_independent_passed':official['passed'],'official_independent_passed_checks':sum(v['passed'] for v in official['independent_acceptance_checks'].values()),
 'official_independent_total_checks':len(official['independent_acceptance_checks']),
 'supplemental_reader_source_sha256':report['reader_source_sha256'],'supplemental_pure_tests_passed':10,
 'official_auditor_source_sha256':report['official_auditor_source_sha256'],
 'replay_repository_root':str(old_clone),'replay_clone_git_head':execution['clone_git_head'],
 'replay_source_identity_sha256':digest(audit/'replay_source_identity_old_clone.json'),
 'current_repository_scope':'Current root may advance its separately tested CLI. All74 original-da69 producer bytes remain exact in the declared old source clone; clone Git HEAD is explicitly distinct from native producer commit. Existing creation-time official before/after identities remain unchanged.',
 'geometry_review_scope':'Separate archived12-artifact geometry-only review of exact saved states and three65-point sampled complete-orbit target branches. No all-clock reachability, native dynamic tracking, collision/force solve or actuation/capture qualification.',
 'closure_scope':'closed means original native process and audits have finished, not that the trajectory succeeded. Original nonzero exit,19/27 native and13/19 independent acceptance outcome, final zero-hand-contact guard failure and unexecuted regrasp/full-capture/reset qualification are retained.',
 'physics_review_scope':'Official exact-source audits verify all-step loads/guards/native FF/cache/control arithmetic and passive properties. Supplemental10 pure regressions and selected original-array analysis quantify the sole actual fully-open pad recontact and finite PD tracking/saturation. No fresh run data was read and no native integration or force replay was performed by the supplemental reader.',
 'timing_scope':'Saved post-q/v at t; original solved contact forces/geometry t-dt; original retained command pose/velocity t-2dt after startup. Original retained6.82512mm error and independently checked saved-post6.82718mm target discrepancy are distinct.'}
(audit/'independent_audit_binding.json').write_text(json.dumps(binding,indent=2,allow_nan=False)+'\n')
print(json.dumps({'closed':True,'binding_sha256':digest(audit/'independent_audit_binding.json'),'original_files':len(raw_files),'audit_files':len(audit_files),'producer_commit':producer,'native_child_exit_code':1,'reader_source_sha256':report['reader_source_sha256'],'supplemental_pure_tests_passed':10}))
