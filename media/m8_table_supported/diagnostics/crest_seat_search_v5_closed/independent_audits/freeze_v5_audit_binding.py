"""Close output-only V5 audit provenance, without changing any native input."""
import hashlib
import json
from pathlib import Path


def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


here=Path(__file__).resolve().parent
run=here.with_name('crest_seat_search_v5')
v4=here.with_name('crest_seat_search_v4_audits_v2')
report=json.loads((here/'independent_cold_crest_inertia_audit_v5.json').read_text())
proof=json.loads((here/'reader_pure_proof.json').read_text())
native=json.loads((run/'insertion_validation.json').read_text())
before=run.with_name(run.name+'_execution_before.json')
after=run.with_name(run.name+'_execution_after.json')
closure=json.loads(after.read_text())
assert closure['native_exit_code']==0 and closure['all_bindings_unchanged'] is True
assert native['diagnostic_completed'] is True and native['aborted'] is None
assert report['original_passed']==native['passed'] and report['diagnostic_completed'] is True
assert report['reader_source_sha256']==digest(here/'independent_cold_crest_inertia_audit_v5.py')
assert report['independent_inertia_commands']['reader_source_sha256']==digest(here/'independent_inertia_command_math_v5.py')
assert proof['passed'] is True and proof['tests_passed']==31 and proof['all_sources_unchanged'] is True
for name,sha in proof['source_sha256_before'].items():
    assert digest(here/name)==sha==proof['source_sha256_after'][name]
assert digest(here/'reader_pure_proof.log')==proof['log_sha256']
assert report['source_original_acceptance_checks']==native['acceptance_checks']
old_bind=json.loads((v4/'equal_model_passive_property_binding.json').read_text())
assert digest(run/'scene.xml')==old_bind['scene_xml_sha256']
assert digest(run/'supported_scene.zip')==old_bind['scene_zip_sha256']
assert native['runtime']==old_bind['runtime'] and native['model_fingerprint']==old_bind['model_fingerprint']
historical=v4/'historical_equal_model_free_joint_properties.json'
assert digest(historical)==old_bind['historical_report_sha256']
(here/historical.name).write_bytes(historical.read_bytes())
passive={'scope':'Reuse only the original V3 compiled-model passive-properties bytes, retaining their original trace reference. Exact V5 scene XML, mesh ZIP, runtime and fingerprint equal the historical audited model. No new native compile, force solve, integration or V5 force qualification is claimed by this reuse.',
 'historical_report_sha256':digest(historical),'historical_v4_equality_binding_sha256':digest(v4/'equal_model_passive_property_binding.json'),
 'current_v5_trace_sha256':digest(run/'insertion_trace.npz'),'scene_xml_sha256':digest(run/'scene.xml'),
 'scene_zip_sha256':digest(run/'supported_scene.zip'),'runtime':native['runtime'],'model_fingerprint':native['model_fingerprint'],
 'original_v3_passive_auditor_sha256':old_bind['original_v3_passive_auditor_sha256'],'free_object_passive_terms_all_zero':True,'no_new_native_operation':True}
(here/'equal_model_passive_property_binding.json').write_text(json.dumps(passive,indent=2)+'\n')
audit_files={p.name:digest(p) for p in here.iterdir() if p.is_file() and p.name!='independent_audit_binding.json'}
raw_files={str(p.relative_to(run)):digest(p) for p in run.rglob('*') if p.is_file()}
siblings={p.name:{'path':str(p),'sha256':digest(p)} for p in (before,after,run.with_name(run.name+'_frozen_inputs.json'),run.with_name(run.name+'.log'),run.with_name(run.name+'_audit_plan.md'))}
binding={'observer':'closed-independent-v5-cold-inertia-audit-binding-v1','closed':True,
 'original_run_directory':str(run),'original_run_files_sha256':raw_files,'audit_files_sha256':audit_files,
 'original_sibling_provenance':siblings,'original_exit_code':0,'original_native_passed':native['passed'],
 'diagnostic_completed':True,'full_fresh_trajectory_qualified':False,'capture_or_open_reset_qualified':False,
 'all_original_acceptance_rows_preserved':True,'original_native_steps':report['original_native_steps'],
 'original_native_final_time_s':report['closed_braking_boundary'][-1]['post_step_time_s'],
 'reader_regression_tests_passed':31,'reader_source_sha256':report['reader_source_sha256'],
 'command_math_source_sha256':report['independent_inertia_commands']['reader_source_sha256'],
 'native_source_scope':'Original experimental harness a846/inertia0913/Crest1d37/C2-4bc/source65 producer6e7 were unchanged before/after. Cold exact checkpoint q/qdot/control only; native warmstarts and preceding force windows were not copied. Inherited references are not fresh pickup proof.',
 'software_proof_scope':'Independent31 pure reader regressions; separate producer67 inertia/cache plus6 rejection, historical69 Crest,50 C2 and402 canonical tests. None alone is native policy/thread qualification.',
 'method':'Read original closed arrays and hash-bound pure observer ASTs only. No MuJoCo import, compile, force reconstruction or integration in this reader. Historical model-property bytes reused only after exact XML/ZIP/runtime/fingerprint comparison.',
 'force_timing':'Original native contact forces and their geometry at post-label minusdt; saved q/qdot at post-label; executed command at post-label minusdt using retained matrices/velocity at post-label minus2dt after startup. Two retained velocity rows initialize from the exact cold checkpoint.',
 'scope':'Executed closed direction confirmation and shallow first forward motion with native guards; original actual interior count0 and nominal21.6um formed overlap. No OPEN, full-pitch capture/reset or continuous fresh full trajectory result.'}
(here/'independent_audit_binding.json').write_text(json.dumps(binding,indent=2)+'\n')
print(json.dumps({'closed':True,'binding':str(here/'independent_audit_binding.json'),'sha256':digest(here/'independent_audit_binding.json'),'reader_source_sha256':report['reader_source_sha256'],'original_native_passed':native['passed'],'diagnostic_completed':True,'tests_passed':31}))
