"""Freeze original native partial bytes; never modify the live producer."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import numpy as np
ROOT=Path('/workspace/astra-r2s');RUN=ROOT/'outputs/m8_table_supported/full_canonical_v1'
OUT=ROOT/'outputs/m8_table_supported/progress/full_canonical_v1_entry_transfer'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def main():
    assert not OUT.exists()
    identity=read(RUN/'run_publication_identity.json');proof=read(RUN/'software_tests.json')
    assert identity['producer_commit']=='6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1'
    assert identity['source_hashes_before']==proof['source_hashes'] and len(proof['source_hashes'])==65
    assert proof['tests_passed']==402 and proof['passed'] and proof['source_hashes_unchanged']
    for n,d in identity['source_hashes_before'].items():assert sha(ROOT/n)==d,n
    source=RUN/'insertion_trace_partial.npz';before=sha(source)
    OUT.mkdir(parents=True);shutil.copyfile(source,OUT/source.name);after=sha(source)
    assert before==after==sha(OUT/source.name),'Live checkpoint changed during exact copy'
    for name in ('scene.xml','scene_source.py','controller_source.py','renderer_source.py','engagement_observer_source.py','supported_scene.zip','run_publication_identity.json','software_tests.json','software_tests.log','manifest.json','verify_sources.py','launch_source.py'):
        shutil.copyfile(RUN/name,OUT/name)
    for name in ('recorded_sources','frozen_audit_sources'):shutil.copytree(RUN/name,OUT/name)
    for n,d in identity['source_hashes_before'].items():
        p=OUT/'producer_sources'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/n,p);assert sha(p)==d
    for name in ('yam_twin/m8_insertion_demo.py','yam_twin/m8_demo.py'):
        p=OUT/'render_dependencies'/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,p)
    with np.load(OUT/source.name,allow_pickle=False) as z:
        t=z['time'].copy();qpos=z['qpos'].copy();qvel=z['qvel'].copy();metadata=json.loads(str(z['metadata_json']));samples=json.loads(str(z['info_json']))
    assert len(t)==len(samples)==len(qpos)==len(qvel) and np.isfinite(qpos).all() and np.isfinite(qvel).all()
    assert samples[-1]['phase']=='transfer_bolt_weight' and abs(float(t[-1])-13.1949)<1e-7
    assert metadata['partial'] and not metadata.get('passed',False) and not samples[-1]['thread_engaged']
    assert samples[-1]['formed_flank_overlap_m']==0 and samples[-1]['native_feedback']['loaded_actual_interior_flank_contact_count']==0
    for n,d in metadata['recorded_source_dependencies_sha256'].items():assert sha(OUT/n)==d,n
    for name,field in [('controller_source.py','controller_module_sha256'),('scene_source.py','scene_source_sha256'),('scene.xml','model_xml_sha256')]:assert sha(OUT/name)==metadata[field]
    for n,d in identity['source_hashes_before'].items():assert sha(ROOT/n)==d,n
    copied={str(p.relative_to(OUT)):sha(p) for p in OUT.rglob('*') if p.is_file()}
    feed=[(i,s) for i,s in enumerate(samples) if s['phase']=='feed_to_entry'][-1]
    final={'scope':'PHASE-ONLY PROGRESS. One fresh native continuous pickup/align/entry/weight-transfer prefix. Full every-step native ledgers and full physical qualification are not closed. Final formed0/interior0; no captured pitch, qualified turn, opening/reset, task-success or continuous full-completion claim.',
        'producer_commit':identity['producer_commit'],'snapshot_utc':datetime.now(timezone.utc).isoformat(),'parent_directory':str(RUN),
        'original_partial_sha256_before_copy':before,'original_partial_sha256_after_copy':after,'snapshot_trajectory_sha256':before,
        'all_65_producer_source_bytes_match_before_after_snapshot':True,'source_hashes_before':identity['source_hashes_before'],
        'software_proof_tests_passed':402,'software_proof_sha256':sha(OUT/'software_tests.json'),'software_scope':'402 checks qualify this producer software only, not a full physical trial or historicalb2/281 cold branch.',
        'first_time_s':float(t[0]),'last_time_s':float(t[-1]),'saved_samples':len(t),'last_recorded_phase':samples[-1]['phase'],
        'final_original_sample':samples[-1],'final_qpos_sha256':hashlib.sha256(qpos[-1].tobytes()).hexdigest(),'final_qvel_sha256':hashlib.sha256(qvel[-1].tobytes()).hexdigest(),
        'entry_phase_endpoint_original_sample':feed[1],'entry_phase_endpoint_saved_index':feed[0],
        'original_entry_support_events':metadata['entry_support_events'],'original_physical_motion_events':metadata['physical_motion_events'],
        'model_xml_sha256':metadata['model_xml_sha256'],'model_fingerprint':metadata['model_fingerprint'],'controller_sha256':metadata['controller_sha256'],
        'controller_module_sha256':metadata['controller_module_sha256'],'runtime':metadata['runtime'],'copied_original_source_model_proof_files':copied,
        'physics_integration_for_media':False,'force_reconstruction_for_media':False,'full_raw_ledgers_closed':False,'full_qualification_audited':False,
        'no_splice_with_cold_diagnostics_or_earlier_alignment_prefix':True}
    (OUT/'progress_snapshot.json').write_text(json.dumps(final,indent=2,allow_nan=False)+'\n')
    shutil.copyfile(__file__,OUT/'prepare_snapshot_source.py')
    print(json.dumps({'folder':str(OUT),'last_time_s':float(t[-1]),'saved_states':len(t),'trace_sha256':before,'entry_overlap_m':feed[1]['thread_overlap_m'],'final_formed_m':samples[-1]['formed_flank_overlap_m'],'final_weight_window':samples[-1]['weight_window']},indent=2))
if __name__=='__main__':main()
