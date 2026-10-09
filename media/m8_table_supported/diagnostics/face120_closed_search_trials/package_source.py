"""Copy three closed cold experiments; preserve originals and correction lineage."""
from pathlib import Path
import hashlib
import json

ROOT=Path('/workspace/astra-r2s')
BASE=ROOT/'outputs/m8_table_supported/diagnostics'
RENDER=BASE/'face120_closed_search_media'
PARENT=ROOT/'media/m8_table_supported/face120_pickup_entry_v1'
TARGET=ROOT/'media/m8_table_supported/diagnostics/face120_closed_search_trials'
TRIALS=('face120_seat_search_v1','face120_closed_forward_v1','face120_closed_forward_hold_v2')
COMMIT='b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'
FROZEN_README=ROOT/'outputs/m8_table_supported/face120_cold_trials_README.md'


def sha(raw):return hashlib.sha256(raw).hexdigest()


def strict(raw):
    return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))


def encoded(value):return (json.dumps(value,indent=2,allow_nan=False)+'\n').encode()


def main():
    if TARGET.exists():raise FileExistsError(TARGET)
    files={'publication_readme_source.md':FROZEN_README.read_bytes()}
    parent_validation=(PARENT/'validation_original.json.txt').read_bytes()
    parent_report=strict(parent_validation)
    parent_manifest=strict((PARENT/'package_manifest.json').read_bytes())
    parent_binding=strict((PARENT/'published_producer_binding.json').read_bytes())
    proof_bytes=(PARENT/'software_proof/software_proof.json').read_bytes()
    proof=strict(proof_bytes)
    assert parent_binding['published_producer_commit']==COMMIT
    assert proof['passed'] and proof['tests_passed']==281 and proof['source_hashes_unchanged']
    assert sha((PARENT/'insertion_trace.npz').read_bytes())==parent_manifest['trajectory_sha256']
    files['canonical_prefix_validation_original.json']=parent_validation
    files['canonical281_software_proof.json']=proof_bytes
    files['canonical_prefix_published_producer_binding.json']=(PARENT/'published_producer_binding.json').read_bytes()
    trials={}
    binding_checks=0
    for name in TRIALS:
        directory=BASE/name
        for path in sorted(directory.rglob('*')):
            if path.is_file():files[name+'/'+str(path.relative_to(directory))]=path.read_bytes()
        files[name+'/original_stdout_stderr.log']=(BASE/(name+'.log')).read_bytes()
        for path in sorted((RENDER/name).rglob('*')):
            if path.is_file():files[name+'/render/'+str(path.relative_to(RENDER/name))]=path.read_bytes()
        rel=lambda field:files[name+'/'+field]
        report=strict(rel('report.json'));declaration=strict(rel('declaration.json'))
        assert declaration['parent_trace_sha256']==parent_manifest['trajectory_sha256']
        assert declaration['parent_model']['model_xml_sha256']==sha(rel('scene.xml'))==parent_report['model_xml_sha256']
        assert declaration['diagnostic_source_sha256']==sha(rel('diagnostic_source.py'))
        assert declaration['observer_sha256']==sha(rel('observer_source.py'))==proof['source_hashes']['yam_twin/m8_supported_start.py']
        assert report['ledger_sha256']==sha(rel('original_native_force_ledger.npz'))
        assert report['trace_sha256']==sha(rel('checkpoint_trace.npz'))
        for source,digest in declaration['parent_source'].items():
            assert sha(rel('recorded_sources/'+source))==digest==proof['source_hashes'][source],source
        binding=strict(rel('independent_audit_binding.json'));assert binding['frozen'] is True
        for key in ['original_files','independent_reports','closed_independent_reports']:
            for artifact,digest in binding.get(key,{}).items():
                assert sha(rel(artifact))==digest,(name,artifact)
                binding_checks+=1
        for key in ['auditor_source_snapshots','source_snapshots']:
            for artifact,digest in binding.get(key,{}).items():
                assert sha(rel('audit_sources/'+artifact))==digest,(name,artifact)
                binding_checks+=1
        assert binding.get('canonical281_source_proof_sha256',binding.get('canonical281_software_proof_sha256'))==sha(proof_bytes)
        phase_name='independent_phase_depth_audit_corrected_yaw.json' if name=='face120_closed_forward_v1' else 'independent_phase_depth_audit.json'
        phase=strict(rel(phase_name))
        assert phase['capture_qualified'] is False and phase['actual_positive_full_ring_overlap_substeps']==0
        assert phase['actual_loaded_interior_contact_substeps']==0
        if name=='face120_closed_forward_v1':
            superseded=binding['superseded_initial_counter_label_report']
            assert sha(rel(superseded['path']))==superseded['sha256']
            assert (directory/'audit_draft_sources').is_dir()
        render=strict(rel('render/render_manifest.json'))
        assert render['trace_sha256']==report['trace_sha256']
        assert render['original_report_sha256']==sha(rel('report.json'))
        assert render['declaration_sha256']==sha(rel('declaration.json'))
        assert render['parent_canonical_trace_sha256']==parent_manifest['trajectory_sha256']
        assert render['runtime']==declaration['runtime']
        assert render['exact_saved_state_replay'] and not render['physics_integration'] and not render['state_interpolation']
        assert not render['physical_bodies_hidden_or_geometry_changed'] and render['playback_speed']==1
        for artifact,identity in render['media'].items():assert sha(rel('render/'+artifact))==identity['sha256']
        for artifact,digest in render['source_dependencies_sha256'].items():assert sha(rel('render/'+artifact))==digest
        feed_range=report['ranges']['net_feed_N']
        first_feed=render['screenshot_states']['entry_detail']['original_sample']['net_feed_N']
        forward=name!='face120_seat_search_v1'
        if forward:
            assert feed_range[0]==feed_range[1]==first_feed==declaration['net_feed_final_N']
            assert 'net=target_net' in rel('diagnostic_source.py').decode()
        feed_scope={
            'scope':'Executed native net-feed schedule derived from original ledger range/first sample and frozen source. Original declaration fields remain untouched.',
            'first_original_native_net_feed_N':first_feed,'original_allstep_net_feed_range_N':feed_range,
            'constant_bolt_weight_feed_from_first_cold_step':forward,
            'executed_feed_schedule':('Constant bolt-weight net feed from native step1; no forward ramp.' if forward else '0.05N during0.15s centering, then0.5s ramp to bolt-weight feed followed by0.5s hold.'),
            'legacy_forward_start_ramp_hold_fields_not_executed':forward,
            'original_ledger_sha256':report['ledger_sha256'],'frozen_harness_sha256':declaration['diagnostic_source_sha256']}
        files[name+'/executed_feed_schedule.json']=encoded(feed_scope)
        trials[name]={
            'scope':declaration['scope'],'closed':True,'continuous_full_trajectory':False,
            'canonical_model_reference_trace_sha256':declaration['parent_trace_sha256'],
            'actual_state_parent_trace_sha256':declaration.get('state_parent_trace_sha256',declaration['parent_trace_sha256']),
            'initial_saved_state_time_s':declaration['initial_saved_state_time_s'],
            'initial_saved_state_index':declaration['initial_saved_state_index'],
            'initial_qpos_qvel_sha256':declaration['initial_qpos_qvel_sha256'],
            'cold_init_note':declaration['cold_init_note'],
            'warmstarts_or_original_solver_state_restored':False,
            'native_duration_s':report['actual_duration_s'],'native_wall_seconds':report['wall_seconds'],
            'all_original_guards_held':report['all_original_guards_held'],'original_abort':report['aborted'],
            'original_termination':report['physical_closed_turn'].get('termination'),
            'original_final_phase':report['final_sample']['physical_phase'],
            'trace_sha256':report['trace_sha256'],'original_force_ledger_sha256':report['ledger_sha256'],
            'harness_source_sha256':declaration['diagnostic_source_sha256'],
            'helper_source_sha256':declaration['observer_sha256'],
            'canonical281_proof_covers_output_only_harness':False,
            'original_grip_references_preserved':True,
            'formed_capture_qualified':False,'actual_positive_formed_overlap_steps':0,
            'actual_loaded_interior_contact_steps':0,'opening_or_passive_reset_executed':False,
            'nominal_overlap_range_m':phase['nominal_overlap_range_m'],
            'common_unchamfered_axial_span_overlap_threshold_m':phase['common_unchamfered_axial_span_overlap_threshold_m'],
            'native_actual_yaw_range_rad_from_independent_audit':phase['actual_yaw_range_rad'],
            'nominal_profile_scope':'Coaxial/centroid profile context only; not a full tilted bore-fit proof, actual loaded interior engagement, prescribed helix or causal isolation.',
            'original_argv':declaration['argv'],
            'renderer_sha256':render['source_sha256'],'video_frames':render['video_frames'],
            'video_fps':render['fps'],'playback_speed':1,
            'original_yaw_counter_issue':('Original constant yaw-origin instrumentation offset retained unchanged; corrected independent reports and original_yaw_origin_note remain separate.' if name=='face120_closed_forward_v1' else 'No correction applied to original counter; independently verified origin.'),
            'left_downward_position_offset_m':declaration.get('left_downward_position_offset_m',0),
            'executed_net_feed_schedule':feed_scope,
            'render_manifest_sha256':sha(rel('render/render_manifest.json')),
            'independent_audit_binding_sha256':sha(rel('independent_audit_binding.json'))}
        files[name+'/README.md']=trial_readme(name,trials[name]).encode()
    reverse=trials['face120_seat_search_v1']
    for name in TRIALS[1:]:assert trials[name]['actual_state_parent_trace_sha256']==reverse['trace_sha256']
    prefix_refs={
        'scope':'Exact canonical native prefix model and original loaded-grasp references. Both forward trials instead receive physical state from the reverse-seat diagnostic endpoint; model/reference parent and state parent remain distinct.',
        'published_parent_package':'media/m8_table_supported/face120_pickup_entry_v1',
        'published_parent_trace_sha256':parent_manifest['trajectory_sha256'],
        'published_parent_code_commit':COMMIT,
        'original_parent_validation_sha256':sha(parent_validation),
        'original_left_acquisition':parent_report['left_acquisition'],
        'original_right_grasp_acquisitions':parent_report['right_grasp_acquisitions'],
        'original_grasp_references_replaced_by_cold_input_refs':False,
        'source_proof281_sha256':sha(proof_bytes),
        'source_proof_scope':'281 source checks cover canonical app/helper source only. Output-only harnesses are different executed experimental sources; no completed native trajectory claim.'}
    files['canonical_prefix_reference_binding.json']=encoded(prefix_refs)
    files['package_source.py']=Path(__file__).read_bytes()
    for name,raw in files.items():
        if name.endswith('.json'):strict(raw)
    max_bytes=max(map(len,files.values()));assert max_bytes<45_000_000
    manifest={
        'scope':'Three separate closed cold local trials, not a warmstarted chain or completed assembly. All original bytes/logs/failed guards and forward-v1 counter error remain unchanged; independent corrections retain their source identities. Every clip replays exact saved state rows only.',
        'canonical_producer_commit':COMMIT,'canonical_source_tests_passed':281,
        'canonical_source_proof_sha256':sha(proof_bytes),
        'proof_counts_not_combined':True,'output_only_arithmetic_audit_proofs_remain_separate':True,
        'parent_package':'media/m8_table_supported/face120_pickup_entry_v1',
        'parent_trace_sha256':parent_manifest['trajectory_sha256'],
        'trials':trials,'native_pipeline_complete':False,'formed_capture_qualified':False,
        'unspliced_full_trajectory_claim':False,'live_v3_outputs_or_sources_touched':False,
        'complete_original_force_and_state_archives_preserved':True,
        'original_logs_preserved':True,'all_artifacts_below45MB_no_chunks_needed':True,
        'maximum_artifact_bytes':max_bytes,'independent_binding_hash_checks':binding_checks,
        'files':{name:{'sha256':sha(raw),'bytes':len(raw)} for name,raw in sorted(files.items())}}
    files['package_manifest.json']=encoded(manifest)
    files['README.md']=readme(trials).encode()
    TARGET.mkdir(parents=True)
    for name,raw in files.items():
        path=TARGET/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(files.items())))
    for name,raw in files.items():assert (TARGET/name).read_bytes()==raw,name
    print(json.dumps({'folder':str(TARGET.relative_to(ROOT)),'files':len(files)+1,
        'bytes':sum(map(len,files.values())),'maximum_artifact_bytes':max_bytes,
        'independent_binding_hash_checks':binding_checks,'capture_qualified':False},indent=2))


def trial_readme(name,data):
    return f'''# {name}

Separate **cold local trial**, native duration {data['native_duration_s']:.5f} s.
Original guards held: {data['all_original_guards_held']}. Original abort:
`{json.dumps(data['original_abort'])}`. Original termination:
`{data['original_termination']}`. **Zero formed overlap or loaded interior
contact; no opening/reset/full trajectory qualification.**

![Actual cold entry state](render/entry_detail.png)

[Normal 1x MP4](render/demo.mp4) · [Normal 1x GIF](render/demo.gif) ·
[Actual endpoint state](render/endpoint_detail.png)

Media uses exact saved qpos/qvel and two fixed real cameras, with `mj_forward`
only. Caption forces are original native solve records at logged time minus dt;
saved poses are postintegration. Physical occlusion remains. No body hiding,
interpolation, geometry change, free-body posing or physics integration occurs.

Original reports, raw all-step ledger, state archive, declared cold
initialization, stdout/stderr log, harness/helper sources and audits are
retained. {data['original_yaw_counter_issue']}

Use the [package instructions](../README.md) for exact cold dependencies,
canonical model/grip parent, actual state parent, commands and proof scopes.
'''


def readme(trials):
    return FROZEN_README.read_text()


if __name__=='__main__':main()
