"""Freeze original cold V4 inputs and separate pure-auditor evidence identities."""
import hashlib,json,shutil
from pathlib import Path

p=Path(__file__).resolve().parent
trial=p/'entry_supported_open_search_v4'
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
load=lambda f:json.loads(Path(f).read_text())
before_path=p/'entry_supported_open_search_v4_execution_before.json'
after_path=p/'entry_supported_open_search_v4_execution_after.json'
before,after=load(before_path),load(after_path)
assert before['git_HEAD']==after['git_HEAD']=='b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'
assert before['source_hashes']==after['source_hashes'] and len(before['source_hashes'])==63
assert before['harness_sha256']==after['harness_sha256']==sha(trial/'diagnostic_source.py')
assert after['before_manifest_sha256']==sha(before_path)
assert after['native_run_closed'] is True and after['aborted'] is None
for name,digest in after['closed_output_hashes'].items():assert sha(trial/name)==digest,name
report=load(trial/'report.json');decl=load(trial/'declaration.json')
assert report['all_original_guards_held'] is True and report['is_full_capture_or_qualified_reset'] is False
assert after['runtime']==decl['runtime']
assert decl['execution_producer_commit']==before['git_HEAD']
assert sha(trial/'observer_source.py')==decl['observer_sha256']
assert decl['parent_trace_sha256']==before['cold_prefix_trace_sha256']
assert decl['state_parent_trace_sha256']==before['cold_visual_v3_trace_sha256']

reports={name:sha(trial/name)for name in (
'independent_native_force_audit.json','independent_phase_depth_audit.json',
'independent_table_press_audit.json','independent_adaptive_open_audit.json','independent_guarded_grasp_lead_audit.json')}
source_map={
'independent_native_force_audit.json':('independent_archived_native_force_audit_v3.py','auditor_source_sha256'),
'independent_phase_depth_audit.json':('independent_archived_phase_depth_audit_v3.py','source_sha256'),
'independent_table_press_audit.json':('independent_table_press_audit.py','auditor_source_sha256'),
'independent_adaptive_open_audit.json':('independent_adaptive_open_audit_v2.py','auditor_source_sha256'),
'independent_guarded_grasp_lead_audit.json':('independent_guarded_grasp_lead_v1.py','auditor_source_sha256')}
for name,(source,key)in source_map.items():assert load(trial/name)[key]==sha(p/source)
force=load(trial/'independent_native_force_audit.json')
assert all(v for k,v in force['identity'].items()if k!='archived_source_checks')
assert all(force['identity']['archived_source_checks'].values())

snapshots=trial/'audit_sources';snapshots.mkdir(exist_ok=True)
# Freeze inherited dependencies as well as the separately versioned correction.
old=p/'entry_supported_open_search_v3/audit_sources'
needed=[
'independent_archived_native_force_audit.py','independent_archived_native_force_audit_v3.py',
'independent_archived_phase_depth_audit_v3.py','independent_cold_chain_audit.py',
'independent_table_press_audit.py','independent_yaw_counter_origin.py',
'independent_open_search_audit_v1.py','test_independent_open_search_audit_v1.py',
'independent_adaptive_open_audit_v1.py','test_independent_adaptive_open_audit_v1.py',
'archived_diagnostic_auditors_software_proof.json','cold_chain_auditors_software_proof.json',
'independent_phase_depth_software_proof.json','yaw_origin_and_press_auditors_software_proof.json',
'independent_open_search_software_proof_v1.json','independent_adaptive_open_software_proof_v1.json',
'canonical281_software_proof.json']
for name in needed:
 origin=old/name if (old/name).exists() else p/name
 shutil.copyfile(origin,snapshots/name)
for name in [
'independent_adaptive_open_audit_v2.py','test_independent_adaptive_open_audit_v2.py',
'independent_guarded_grasp_lead_v1.py','test_independent_guarded_grasp_lead_v1.py',
'independent_adaptive_and_grasp_software_proof_v2.json','freeze_entry_supported_v4_audits.py']:
 shutil.copyfile(p/name,snapshots/name)
for name in ['test_independent_archived_native_force_audit.py','test_independent_cold_chain_audit.py',
 'test_independent_yaw_counter_origin.py','test_independent_table_press_audit.py']:
 shutil.copyfile(old/name if (old/name).exists()else p/name,snapshots/name)
for path in [before_path,after_path]:shutil.copyfile(path,trial/path.name)
proof=load(snapshots/'independent_adaptive_and_grasp_software_proof_v2.json')
assert proof['sources_unchanged'] and proof['passed']==12 and proof['failed']==0
for name,digest in proof['sources_before'].items():assert sha(snapshots/name)==digest
assert sha(snapshots/'canonical281_software_proof.json')==before['canonical281_proof_sha256']

adaptive=load(trial/'independent_adaptive_open_audit.json')
grasp=load(trial/'independent_guarded_grasp_lead_audit.json')
binding={
'scope':'Closed cold native entry-supported SEARCH opening/reindex/regrasp and one independent physical half-turn. Original source/runtime/forces/states remain immutable. All original hard guards held; actual partial interior loading observed. No full-pitch capture, qualified passive-reset reward or continuous full-trajectory claim.',
'canonical_parent_code_commit':before['git_HEAD'],
'isolated_execution_root':before['isolated_execution_root'],
'exact63_parent_sources_unchanged_before_after':True,
'parent_source_hashes':before['source_hashes'],
'harness_source_sha256':sha(trial/'diagnostic_source.py'),
'helper_sha256':sha(trial/'observer_source.py'),
'canonical_model_reference_parent_sha256':decl['parent_trace_sha256'],
'actual_cold_state_parent_sha256':decl['state_parent_trace_sha256'],
'original_files':after['closed_output_hashes'],
'execution_identity_sidecars':{before_path.name:sha(before_path),after_path.name:sha(after_path)},
'closed_independent_reports':reports,
'source_snapshots':{f.name:sha(f)for f in sorted(snapshots.iterdir())if f.is_file()},
'software_scope':'Canonical281 proof covers the immutable b2 parent application/helper. Output-only physical 30mm/open event harness is a separately archived cold diagnostic. New pure auditor12-test proof is separate from original producer physics; later6e7 canonical402 proof was not the V4 producer.',
'native_runtime':decl['runtime'],
'native_duration_s':report['actual_duration_s'],
'original_native_ticks':grasp['raw_native_ticks'],
'live_event_minimum_eligible_timing':adaptive['minimum_eligible_original_event'],
'adaptive_live_readiness_elapsed_s':adaptive['adaptive_event_and_actual_clock']['verified_event']['elapsed_s'],
'first_reset_command_elapsed_s':adaptive['adaptive_event_and_actual_clock']['first_reset_command_elapsed_s'],
'per_grasp_original_raw_scope':grasp['guarded_grasp'],
'actual_forward_half_turn':grasp['actual_original_forward_half_turn'],
'final_stopped_native_support':grasp['final_stopped_100ms'],
'fully_open_native_zero_robot_contact':grasp['fully_open_unassisted'],
'original_overall_guards_held':True,'original_producer_failure':None,
'full_capture':False,'qualified_reset':False,'full_trajectory_qualified':False,
'force_state_timing':'Original mj_step retained solved forces/derived geometry at time-dt. Saved qpos/qvel post-integration at time. Cold state lacks original warmstart/solver history. Exact readiness event post-state was saved; historical regrasp acquisition is in all-step native ledger and source reference but lacks an exact sparse post-state row.',
'correction_scope':'Frozen adaptive-v1 auditor assumed the first-ever ready solve was always eligible. V4 became ready at.35000 before min-dwell eligibility. Additive adaptive-v2 correctly binds the first CURRENT ready solve after min.12s, at.37000. No producer/helper/criterion or original failure was altered; current derivative also labels intentional CLOSED regrasp contacts separately from forbidden OPEN contacts.'}
(trial/'independent_audit_binding.json').write_text(json.dumps(binding,indent=2,allow_nan=False)+'\n')
for name,digest in after['closed_output_hashes'].items():assert sha(trial/name)==digest
print(json.dumps({'binding_sha256':sha(trial/'independent_audit_binding.json'),'reports':reports,'all_closed_original_files_unchanged':True},indent=2))
