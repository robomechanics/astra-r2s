"""Append separate layout/runtime review; preserve initial reviewed identities."""
from pathlib import Path
import hashlib,json
ROOT=Path('/workspace/astra-r2s')
PACKAGE=ROOT/'media/m8_table_supported/diagnostics/entry_supported_reset_regrasp_v4'
SUMMARY=ROOT/'outputs/m8_table_supported/reset_v4_recipe_review_summary.json'
def sha(raw):return hashlib.sha256(raw).hexdigest()
def parse(raw):return json.loads(raw,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def encode(v):return (json.dumps(v,indent=2,allow_nan=False)+'\n').encode()
def main():
    target=PACKAGE/'recipe_validation';assert not target.exists()
    old_ledger=(PACKAGE/'SHA256SUMS').read_bytes();old_manifest=(PACKAGE/'package_manifest.json').read_bytes();manifest=parse(old_manifest)
    summary_bytes=SUMMARY.read_bytes();review=parse(summary_bytes)
    assert review['reviewed_prepublication_ledger_sha256']==sha(old_ledger)
    assert review['complete_input_package_checksum_counts']['reset_trials_prepublication']==150
    assert review['native_steps']==1 and review['native_duration_s']==5e-5 and review['exit_code']==0 and review['aborted'] is None
    proof=parse((PACKAGE/'canonical281_software_proof.json').read_bytes())
    assert review['producer_commit']==manifest['canonical_producer_commit']
    assert review['producer_source_hashes_before']==review['producer_source_hashes_after']==proof['source_hashes']
    assert review['runtime_libraries_after_match_recorded_original'] and review['source_hashes_unchanged']
    declaration=parse((PACKAGE/'native_v4/declaration.json').read_bytes())
    for key,declared in [('frozen_harness_sha256','diagnostic_source_sha256'),('actual_closed_v3_state_sha256','state_parent_trace_sha256'),('actual_closed_v3_report_sha256','state_parent_report_sha256'),('initial_qpos_qvel_sha256','initial_qpos_qvel_sha256')]:assert review[key]==declaration[declared],key
    assert review['runtime']==declaration['runtime'] and review['right_actual_open_aperture_configuration_m']==.03
    for line in old_ledger.decode().splitlines():
        digest,name=line.split('  ',1);assert sha((PACKAGE/name).read_bytes())==digest,name
    added={'reviewed150_SHA256SUMS.original.txt':old_ledger,'reviewed150_package_manifest.original.json':old_manifest,
        'summary_original.json':summary_bytes,'append_recipe_evidence_source.py':Path(__file__).read_bytes()}
    for path,digest in review['small_original_artifacts'].items():
        p=Path(path);raw=p.read_bytes();assert sha(raw)==digest
        added['one_step_'+('stdout_stderr_original.log' if p.suffix=='.log' else p.name)]=raw
    binding={'scope':'Reviewed initial150 checksum ledger and original package_manifest retained separately. Final manifest adds only this independent one50us native layout/runtime evidence. Original report/state/force/model/source/media/README bytes remain unchanged. No combined281/402 or fulltrial/capture/reset claim.',
        'reviewed_prepublication_ledger_sha256':sha(old_ledger),'reviewed_prepublication_manifest_sha256':sha(old_manifest),
        'reviewed_original_file_identities':manifest['files'],'summary_original_sha256':sha(summary_bytes),
        'reviewed150_manifest_is_sole_original_file_replaced_by_additive_final_manifest':True}
    added['reviewed_prepublication_binding.json']=encode(binding)
    added['README.md']=b'''# Separate one-step recipe validation\n\nA fresh inactive b2 checkout copied the complete initial150-entry package and\ncomplete canonical prefix, earlier trials and closed-v3 parents. All63 original\nproducer sources, native core/plugin, model, exact source/state/report/ledger\nparents and 30mm physical command configuration matched before/after.\nExactly ONE50us first-release native step exited0/noabort.\n\nThis validates recipe layout/runtime only. It does not repeat the8.8531s native\nbranch, qualify opening/reset/regrasp/capture/lead, or add to281/402 software\nchecks. Initial reviewed SHA ledger and package_manifest are retained under\nexplicit original names because the final manifest adds only this evidence.\nAll original data/source/media/README artifacts remain byte-identical.\n'''
    target.mkdir()
    for name,raw in added.items():(target/name).write_bytes(raw)
    manifest['separate_recipe_validation']={'native_steps':1,'native_duration_s':5e-5,'passed_layout_runtime_only':True,
        'summary_original_sha256':sha(summary_bytes),'initial_reviewed150_ledger_sha256':sha(old_ledger),
        'full_native_trial_repeated':False,'capture_or_reset_qualified':False,'combined_with281_or402':False}
    for name,raw in added.items():manifest['files']['recipe_validation/'+name]={'sha256':sha(raw),'bytes':len(raw)}
    manifest['additional_frozen_execution_sidecar_hash_checks']=2
    (PACKAGE/'package_manifest.json').write_bytes(encode(manifest))
    all_files={str(p.relative_to(PACKAGE)):p.read_bytes() for p in PACKAGE.rglob('*') if p.is_file() and p.name!='SHA256SUMS'}
    for name,raw in all_files.items():
        if name.endswith('.json'):parse(raw)
        assert len(raw)<45_000_000
    for name,digest in declaration['parent_source'].items():assert sha((PACKAGE/'native_v4/recorded_sources'/name).read_bytes())==digest
    (PACKAGE/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(all_files.items())))
    print(json.dumps({'files':len(all_files)+1,'checksum_entries':len(all_files),'manifest_artifacts':len(manifest['files']),
        'largest_bytes':max(map(len,all_files.values())),'total_bytes_including_ledger':sum(p.stat().st_size for p in PACKAGE.rglob('*') if p.is_file()),
        'final_ledger_sha256':sha((PACKAGE/'SHA256SUMS').read_bytes()),'original_reviewed150_bytes_preserved_except_additive_manifest_and_ledger':True},indent=2))
if __name__=='__main__':main()
