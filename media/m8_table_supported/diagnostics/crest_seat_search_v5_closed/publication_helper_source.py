"""Publish exact CLOSED cold V5 data, media, sources and independently frozen audits."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil
import uuid

ROOT=Path('/workspace/astra-r2s')
BASE=ROOT/'outputs/m8_table_supported/diagnostics'
RUN=BASE/'crest_seat_search_v5'
TARGET=ROOT/'media/m8_table_supported/diagnostics/crest_seat_search_v5_closed'
WRITER=ROOT/'outputs/m8_table_supported/publish_supported_feedback_run.py'
WRITER_SHA='8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'


def sha(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))


def main(args):
    assert not TARGET.exists() and sha(WRITER)==WRITER_SHA
    spec=importlib.util.spec_from_file_location('verified_lossless_writer',WRITER)
    writer=importlib.util.module_from_spec(spec);spec.loader.exec_module(writer)
    before=read(BASE/'crest_seat_search_v5_execution_before.json')
    after=read(BASE/'crest_seat_search_v5_execution_after.json')
    report=read(RUN/'insertion_validation.json')
    closure=read(BASE/'crest_seat_search_v5_closure_summary.json')
    assert after['native_exit_code']==0 and after['all_bindings_unchanged'] is True
    assert after['execution_before_sha256']==sha(BASE/'crest_seat_search_v5_execution_before.json')
    assert report['diagnostic_completed'] is True and report['aborted'] is None
    assert report['passed'] is False and report['partial'] is True
    assert report['full_fresh_trajectory_qualified'] is False and report['capture_or_open_reset_qualified'] is False
    assert closure['native_steps']==147132 and abs(closure['final_native_time_s']-7.3566)<1e-9
    assert closure['validation_sha256']==sha(RUN/'insertion_validation.json')
    assert before['source_files_before']==after['source_files_after'] and len(before['source_files_before'])==18
    assert before['producer_source_65_before']==after['source_65_after'] and len(before['producer_source_65_before'])==65
    assert before['parent_input_sha256_before']==after['parent_input_sha256_after']
    assert before['runtime_file_sha256_before']==after['runtime_file_sha256_after']
    for name,digest in before['source_files_before'].items():assert sha(BASE/name)==digest,name
    producer=BASE/'crest_seat_search_v5_closed_producer_sources'
    for name,digest in before['producer_source_65_before'].items():assert sha(producer/name)==digest,name
    parent=ROOT/'media/m8_table_supported/full_canonical_v1_failed_evidence'
    assert (parent/'SHA256SUMS').is_file()
    original_parent=ROOT/'outputs/m8_table_supported/full_canonical_v1'
    for name,digest in before['parent_input_sha256_before'].items():assert sha(original_parent/name)==digest,name
    render=BASE/'crest_seat_search_v5_closed_render'
    render_manifest=read(render/'render_manifest.json')
    assert render_manifest['native_closed_result'] is True and render_manifest['frames']==89
    assert render_manifest['trajectory_sha256']==sha(RUN/'insertion_trace.npz')
    assert render_manifest['physics_integration'] is False and render_manifest['mj_forward_called'] is False
    assert render_manifest['source65_before_after_render']==before['producer_source_65_before']
    for name,digest in render_manifest['media_sha256'].items():assert sha(render/name)==digest,name
    plots=BASE/'crest_seat_search_v5_closed_plots';plot=read(plots/'plot_manifest.json')
    assert plot['native_ticks']==147132 and plot['originals_unchanged'] is True
    for name,digest in plot['plot_sha256'].items():assert sha(plots/name)==digest,name
    audit_path=args.audit_binding.resolve()
    assert audit_path.is_file()
    audit=read(audit_path)
    assert args.audit_sha256==sha(audit_path),'Audit binding must be supplied after independent source/report freeze'
    # Audit meanings/flags remain verbatim. This packager checks immutable
    # identity closure without changing an independent acceptance criterion.
    assert audit.get('closed') is True
    assert audit['original_exit_code']==0 and audit['diagnostic_completed'] is True
    assert audit['original_native_passed'] is False and audit['reader_regression_tests_passed']==31
    for name,digest in audit['audit_files_sha256'].items():assert sha(audit_path.parent/name)==digest,name
    for name,digest in audit['original_run_files_sha256'].items():assert sha(RUN/name)==digest,name
    for name,entry in audit['original_sibling_provenance'].items():assert sha(BASE/name)==entry['sha256'],name
    sources={}
    def add(name,path):
        assert name not in sources
        sources[name]=Path(path).resolve()
    for folder,prefix in ((RUN,'native_v5'),(render,'render'),(plots,'scientific_plot'),
            (audit_path.parent,'independent_audits'),(audit_path.parent,audit_path.parent.name)):
        for path in sorted(folder.rglob('*')):
            if path.is_file():add(prefix+'/'+str(path.relative_to(folder)),path)
    for name in before['source_files_before']:add('source_proof/'+name,BASE/name)
    for name in ('crest_seat_search_v5_execution_before.json','crest_seat_search_v5_execution_after.json',
            'crest_seat_search_v5_frozen_inputs.json','crest_seat_search_v5.log',
            'crest_seat_search_v5_closure_summary.json','crest_seat_search_v5_audit_plan.md'):
        add('execution_provenance/'+name,BASE/name)
    # Exact sibling aliases preserve the frozen auditor's ../ bindings.
    for name in audit['original_sibling_provenance']:
        add(name,BASE/name)
    for name in before['producer_source_65_before']:add('producer_sources/'+name,producer/name)
    for name in ('software_tests.json','software_tests.log','manifest.json','verify_sources.py'):
        add('original402_software_proof/'+name,original_parent/name)
    add('historical69_observer_contracts/crest_seat_search_v3_execution_before.json',BASE/'crest_seat_search_v3_execution_before.json')
    add('publication_dependencies/publish_supported_feedback_run.py',WRITER)
    native_names={str(p.relative_to(RUN)):writer.file_identity(p) for p in RUN.rglob('*') if p.is_file()}
    identity={name:writer.file_identity(path) for name,path in sources.items()}
    TARGET.parent.mkdir(parents=True,exist_ok=True)
    stage=TARGET.parent/(TARGET.name+'.packaging-'+uuid.uuid4().hex);stage.mkdir()
    try:
        artifacts={name:writer.write_artifact(path,stage,name,45_000_000) for name,path in sources.items()}
        for name,path in sources.items():assert writer.file_identity(path)==identity[name],name
        manifest={'status':'closed_completed_cold_diagnostic','original_native_exit_code':0,
            'original_native_passed':False,'original_native_partial':True,'diagnostic_completed':True,
            'original_aborted':None,'native_duration_s':closure['final_native_time_s'],
            'original_wall_seconds':after['wall_seconds'],'native_ticks':147132,
            'full_fresh_trajectory_qualified':False,'capture_or_open_reset_qualified':False,
            'final_formed_flank_overlap_m':report['phases'][-1]['final_formed_flank_overlap_m'],
            'loaded_actual_interior_flank_duration_s':report['diagnostic_final_native_weight_support']['loaded_actual_interior_flank_duration_s'],
            'original_closed_files_preserved':True,'original_run_file_identities':native_names,
            'original_run_file_to_published_artifact_names':{name:['native_v5/'+name] for name in native_names},
            'source18_before_after':before['source_files_before'],'source65_before_after':before['producer_source_65_before'],
            'parent_input_sha256':before['parent_input_sha256_before'],
            'parent_package':'media/m8_table_supported/full_canonical_v1_failed_evidence',
            'parent_package_ledger_sha256':sha(parent/'SHA256SUMS'),
            'independent_audit_binding_sha256':sha(audit_path),'independent_audit_binding':audit,
            'source_and_native_parent_flags_unmodified':True,
            'original65_402_proof_is_separate':True,'ignored67_and6_69_50_proofs_are_separate':True,
            'runtime':before['runtime'],'archived_parent_native_checkpoint_time_s':13.194899999975918,
            'model_and_grasp_reference_lineage':read(RUN/'diagnostic_declaration.json'),
            'render_manifest_sha256':sha(render/'render_manifest.json'),
            'plot_manifest_sha256':sha(plots/'plot_manifest.json'),
            'archives_are_lossless':True,'dropping_or_quantization':False,'maximum_stored_file_bytes':45_000_000,
            'scope':'Separate new cold branch, initialized once from original full-v1 t13.1949 post-state without warmstart/old force windows. Actual C2 braking/quiet direction support and first closed forward scan complete under finite robot controls. Formed overlap21.615um/interior0; no OPEN/reset/captured thread/full fresh trajectory/policy qualification. Whole original force/FF histories and actual independent acceptance outcomes remain verbatim.',
            'artifacts':artifacts}
        (stage/'package_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        for name in ('reassemble_archives.py','restore_original_run_layout.py'):
            shutil.copy2(ROOT/'outputs/m8_table_supported'/name,stage/name)
        shutil.copy2(__file__,stage/'publication_helper_source.py')
        shutil.copy2(ROOT/'outputs/m8_table_supported/crest_seat_search_v5_closed_README.md',stage/'README.md')
        files=sorted(p for p in stage.rglob('*') if p.is_file())
        assert max(p.stat().st_size for p in files)<=45_000_000
        (stage/'SHA256SUMS').write_text(''.join(f'{sha(p)}  {p.relative_to(stage)}\n' for p in files))
        stage.rename(TARGET)
        print(json.dumps({'target':str(TARGET.relative_to(ROOT)),'files':len(files)+1,'artifacts':len(artifacts),
            'ledger_sha256':sha(TARGET/'SHA256SUMS'),'total_bytes':sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file())},indent=2))
    except Exception:
        shutil.rmtree(stage);raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-binding',type=Path,required=True)
    parser.add_argument('--audit-sha256',required=True)
    main(parser.parse_args())
