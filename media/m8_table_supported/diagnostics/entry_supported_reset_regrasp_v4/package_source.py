"""Additive exact archive of closed V4 and distinct earlier diagnostics."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path('/workspace/astra-r2s');BASE=ROOT/'outputs/m8_table_supported/diagnostics'
RENDER=BASE/'entry_supported_reset_regrasp_media'
PARENT=ROOT/'media/m8_table_supported/face120_pickup_entry_v1'
STATE_PARENT=ROOT/'media/m8_table_supported/diagnostics/face120_partial_helical_start_v3'
TARGET=ROOT/'media/m8_table_supported/diagnostics/entry_supported_reset_regrasp_v4'
README=ROOT/'outputs/m8_table_supported/entry_supported_reset_regrasp_README.md'
COMMIT='b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'
def sha(raw):return hashlib.sha256(raw).hexdigest()
def strict(raw):return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))
def encoded(v):return (json.dumps(v,indent=2,allow_nan=False)+'\n').encode()
def fd(p):return sha(Path(p).read_bytes())
def main():
    if TARGET.exists():raise FileExistsError(TARGET)
    # Refuse to freeze an incomplete independently reviewed native trial.
    for name in ('entry_supported_open_search_v4','entry_supported_open_search_v3'):
        assert (BASE/name/'independent_audit_binding.json').is_file(),name
    files={'README.md':README.read_bytes(),'publication_readme_source.md':README.read_bytes()}
    def tree(source,prefix):
        for p in sorted(source.rglob('*')):
            if p.is_file():files[prefix+'/'+str(p.relative_to(source))]=p.read_bytes()
    parent_validation=(PARENT/'validation_original.json.txt').read_bytes();parent_report=strict(parent_validation)
    proof_bytes=(PARENT/'software_proof/software_proof.json').read_bytes();proof=strict(proof_bytes)
    assert proof['passed'] and proof['tests_passed']==281 and proof['source_hashes_unchanged'] and len(proof['source_hashes'])==63
    published=strict((PARENT/'published_producer_binding.json').read_bytes());assert published['published_producer_commit']==COMMIT
    state_report_bytes=(STATE_PARENT/'report.json').read_bytes();state_report=strict(state_report_bytes)
    assert state_report['trace_sha256']==fd(STATE_PARENT/'checkpoint_trace.npz') and state_report['ledger_sha256']==fd(STATE_PARENT/'original_native_force_ledger.npz')
    files['canonical281_software_proof.json']=proof_bytes
    files['canonical_prefix_validation_original.json']=parent_validation
    files['canonical_prefix_published_producer_binding.json']=(PARENT/'published_producer_binding.json').read_bytes()
    files['closed_v3_state_parent_report_original.json']=state_report_bytes
    with np.load(STATE_PARENT/'checkpoint_trace.npz',allow_pickle=False) as z:
        state_qpos=z['qpos'][-1].copy();state_qvel=z['qvel'][-1].copy();state_sample=strict(str(z['info_json']))[-1]
    cold_seed=sha(state_qpos.tobytes()+state_qvel.tobytes());trials={};checks=0
    for short,name in [('native_v4','entry_supported_open_search_v4'),('earlier_adaptive_v3_failure','entry_supported_open_search_v3')]:
        directory=BASE/name;tree(directory,short);tree(RENDER/(short+'_final'),short+'/render')
        files[short+'/original_stdout_stderr.log']=(BASE/(name+'.log')).read_bytes()
        rel=lambda n:files[short+'/'+n]
        declaration=strict(rel('declaration.json'));report=strict(rel('report.json'));binding=strict(rel('independent_audit_binding.json'));render=strict(rel('render/render_manifest.json'))
        assert declaration['parent_trace_sha256']==fd(PARENT/'insertion_trace.npz')
        assert declaration['state_parent_trace_sha256']==state_report['trace_sha256'] and declaration['state_parent_report_sha256']==sha(state_report_bytes)
        assert declaration['initial_qpos_qvel_sha256']==cold_seed and declaration['initial_saved_state_time_s']==state_sample['time_s']
        assert declaration['diagnostic_source_sha256']==sha(rel('diagnostic_source.py'))
        assert declaration['observer_sha256']==sha(rel('observer_source.py'))==proof['source_hashes']['yam_twin/m8_supported_start.py']
        assert declaration['parent_model']['model_xml_sha256']==sha(rel('scene.xml'))==parent_report['model_xml_sha256']
        assert report['trace_sha256']==sha(rel('checkpoint_trace.npz')) and report['ledger_sha256']==sha(rel('original_native_force_ledger.npz'))
        assert not report['is_full_capture_or_qualified_reset']
        for source,digest in declaration['parent_source'].items():assert sha(rel('recorded_sources/'+source))==digest==proof['source_hashes'][source]
        for key in ('original_files','closed_independent_reports'):
            for artifact,digest in binding[key].items():assert sha(rel(artifact))==digest,(short,artifact);checks+=1
        for artifact,digest in binding['source_snapshots'].items():assert sha(rel('audit_sources/'+artifact))==digest,(short,artifact);checks+=1
        assert render['original_trace_sha256']==report['trace_sha256'] and render['original_ledger_sha256']==report['ledger_sha256']
        assert render['original_declaration_sha256']==sha(rel('declaration.json')) and render['original_report_sha256']==sha(rel('report.json'))
        assert render['runtime_before']==render['runtime_after']==declaration['runtime']
        assert render['exact_saved_state_replay'] and not render['physics_integration'] and not render['state_interpolation']
        assert not render['physical_bodies_hidden_or_geometry_changed'] and not render['model_runtime_ranges_or_geometry_overrides_applied']
        assert render['playback_speed']==1 and render['fps']==12 and render['last_video_frame_is_original_exact_terminal_saved_state']
        for n,v in render['media'].items():assert sha(rel('render/'+n))==v['sha256'] and len(rel('render/'+n))==v['bytes']
        for n,digest in render['dependency_sha256'].items():assert sha(rel('render/renderer_sources/'+n))==digest
        with np.load(directory/'declared_cold_initialization.npz',allow_pickle=False) as z:
            assert np.array_equal(z['qpos'],state_qpos) and np.array_equal(z['qvel'],state_qvel) and float(z['time'])==state_sample['time_s']
        with np.load(directory/'checkpoint_trace.npz',allow_pickle=False) as z:
            qpos=z['qpos'].copy();qvel=z['qvel'].copy();samples=strict(str(z['info_json']));metadata_raw=str(z['metadata_json']);metadata=strict(metadata_raw)
        assert len(qpos)==len(qvel)==len(samples)==render['saved_state_rows']
        assert all(metadata.get(k)==v for k,v in declaration.items())
        assert sha(metadata_raw.encode())==render['original_trace_metadata_utf8_sha256']
        assert {k:v for k,v in metadata.items() if k not in declaration}==render['trace_only_finalized_observed_fields']
        for label,s in render['stills'].items():
            i=s['state_index'];assert s['original_sample']==samples[i] and sha(qpos[i].tobytes())==s['qpos_sha256'] and sha(qvel[i].tobytes())==s['qvel_sha256']
        for i,t in zip(render['video_exact_saved_indices'],render['video_exact_saved_times_s']):assert samples[i]['time_s']==t
        assert render['video_exact_saved_indices'][-1]==len(samples)-1 and samples[-1]['elapsed_s']==report['actual_duration_s']
        with np.load(directory/'original_native_force_ledger.npz',allow_pickle=False) as z:raw={k:z[k].copy() for k in z.files}
        assert np.all(raw['external_drive_zero']==1) and np.all(raw['bolt_world_support_contact_count']==0) and np.all(raw['nonthread_block_bolt_contact_count']==0)
        mask=raw['right_grasp_guard_active'].astype(bool);opened=raw['fully_open_unassisted'].astype(bool)
        guarded=None if not mask.any() else {'native_ticks':int(mask.sum()),'maximum_translation_slip_m':float(raw['right_grip_slip_m'][mask].max()),'maximum_rotation_slip_rad':float(raw['right_grip_rotation_slip_rad'][mask].max())}
        if short=='native_v4':
            assert report['all_original_guards_held'] and report['aborted'] is None and np.all(raw['all_checks_held']==1)
            assert declaration['execution_producer_commit']==COMMIT and declaration['right_open_aperture_m']==.03
            assert declaration['parent_control_open_aperture_m']==.024 and declaration['effective_native_arm_config']['open_aperture']==.03
            for key in ('right_bolt_contact_count','hand_gravity_opposing_force_N','right_applied_axial_feed_N'):assert np.all(raw[key][opened]==0),key
            assert guarded['maximum_translation_slip_m']<=.001 and guarded['maximum_rotation_slip_rad']<=np.deg2rad(2)
            assert report['final_sample']['loaded_actual_interior_flank_contact_count']==4
            assert report['final_sample']['formed_flank_overlap_m']<.00125
            assert report['physical_closed_turn']['actual_regrasp_acquisition'] is not None
            before_bytes=(BASE/(name+'_execution_before.json')).read_bytes();after_bytes=(BASE/(name+'_execution_after.json')).read_bytes();before=strict(before_bytes);after=strict(after_bytes)
            assert before['source_hashes']==after['source_hashes']==proof['source_hashes'] and before['git_HEAD']==after['git_HEAD']==COMMIT
            assert before['harness_sha256']==after['harness_sha256']==sha(rel('diagnostic_source.py'))
            assert before['canonical281_proof_sha256']==after['canonical281_proof_sha256']==sha(proof_bytes)
            assert after['before_manifest_sha256']==sha(before_bytes) and after['runtime']==declaration['runtime'] and after['native_run_closed']
            for artifact,digest in after['closed_output_hashes'].items():assert sha(rel(artifact))==digest;checks+=1
            files[short+'/execution_before_original.json']=before_bytes;files[short+'/execution_after_original.json']=after_bytes
        else:
            assert not report['all_original_guards_held'] and report['aborted']['failed_checks']==['genuine_no_right_robot_contacts_while_open']
            assert np.all(raw['all_checks_held'][:-1]==1) and raw['all_checks_held'][-1]==0
            assert raw['right_bolt_contact_count'][-1]==1 and raw['loaded_actual_interior_flank_contact_count'].max()==0
        trials[short]={'original_native_trial_name':name,'native_duration_s':report['actual_duration_s'],'native_wall_seconds':report['wall_seconds'],
            'native_scalar_rows':len(raw['time_s']),'original_overall_guards_held':report['all_original_guards_held'],'original_abort':report['aborted'],
            'original_trace_sha256':report['trace_sha256'],'original_ledger_sha256':report['ledger_sha256'],'original_report_sha256':sha(rel('report.json')),
            'original_harness_sha256':declaration['diagnostic_source_sha256'],'original_helper_sha256':declaration['observer_sha256'],
            'final_formed_overlap_m':report['final_sample']['formed_flank_overlap_m'],'final_loaded_interior_count':report['final_sample']['loaded_actual_interior_flank_contact_count'],
            'final_native_weight_window':report['final_weight_window'],'observed_native_open_reset_and_regrasp':report['physical_closed_turn'],
            'actual_guarded_closed_grasp_slip':guarded,'global_unmasked_slip_includes_intentional_opening':True,
            'model_or_runtime_range_override':False,'physical_command_open_aperture_m':declaration.get('right_open_aperture_m',.024),
            'canonical281_covers_output_only_harness':False,'capture_qualified':False,'qualified_passive_reset':False,'full_pitch_lead_qualified':False,'continuous_full_trajectory':False,
            'native_duration_s':report['actual_duration_s'],'media_frames':render['video_frames'],'media_fps':render['fps'],
            'encoded_video_duration_s':render['video_frames']/render['fps'],'gif_duration_ms':render['gif_duration_ms'],
            'missing_original_saved_state_tail_s':render['missing_saved_state_tail_s'],'tail_synthesized':False,
            'render_manifest_sha256':sha(rel('render/render_manifest.json')),'independent_audit_binding_sha256':sha(rel('independent_audit_binding.json'))}
    tree(BASE/'open30_static_preflight_v1','static30mm_preflight');study=strict(files['static30mm_preflight/study.json'])
    assert study['sample_count']==416 and study['all_sampled_states_passed'] and study['opening_m']==.03
    assert study['coordinates_sha256']==sha(files['static30mm_preflight/sampled_kinematic_coordinates.npz'])
    assert study['diagnostic_source_sha256']==sha(files['static30mm_preflight/diagnostic_source.py'])
    for n,digest in study['input_source_sha256'].items():assert sha(files['static30mm_preflight/inputs/'+n])==digest
    assert study['v3_checkpoint_sha256']==trials['earlier_adaptive_v3_failure']['original_trace_sha256']
    assert study['v3_native_force_ledger_sha256']==trials['earlier_adaptive_v3_failure']['original_ledger_sha256']
    tree(RENDER/'plot','plot');plot=strict(files['plot/plot_manifest.json'])
    assert plot['plot_sha256']==sha(files['plot/native_open_reset_history.png']) and plot['source_sha256']==sha(files['plot/plot_entry_supported_reset_regrasp.py'])
    assert plot['original_ledger_sha256']==trials['native_v4']['original_ledger_sha256'] and plot['guarded_grasp_slip']==trials['native_v4']['actual_guarded_closed_grasp_slip']
    lineage={'scope':'Exact original cold qpos/qvel state from closed partial-helicalV3, with missing warmstart/runtime solver state; canonical prefix supplies original unchanged geometry/grip references. Two native branches and static study remain separate, no splice.',
        'canonical_producer_commit':COMMIT,'canonical281_proof_sha256':sha(proof_bytes),'canonical_model_reference_package':'media/m8_table_supported/face120_pickup_entry_v1',
        'canonical_model_reference_trace_sha256':fd(PARENT/'insertion_trace.npz'),'actual_cold_state_package':'media/m8_table_supported/diagnostics/face120_partial_helical_start_v3',
        'actual_state_parent_trace_sha256':state_report['trace_sha256'],'actual_state_parent_report_sha256':sha(state_report_bytes),'actual_state_parent_ledger_sha256':state_report['ledger_sha256'],
        'actual_cold_initial_qpos_qvel_sha256':cold_seed,'actual_cold_state_time_s':state_sample['time_s'],'warmstart_solver_state_restored':False,
        'upstream_reverse_state_package':'media/m8_table_supported/diagnostics/face120_closed_search_trials/face120_seat_search_v1',
        'upstream_reverse_state_trace_sha256':strict((STATE_PARENT/'declaration.json').read_bytes())['state_parent_trace_sha256'],
        'original_left_acquisition':parent_report['left_acquisition'],'original_right_grasp_acquisitions':parent_report['right_grasp_acquisitions'],
        'new_native_v4_actual_quiet_bilateral_acquisition':trials['native_v4']['observed_native_open_reset_and_regrasp']['actual_regrasp_acquisition'],
        'original_canonical_references_changed':False,'native_v4_acquired_new_guard_reference_from_actual_grasp_event':True}
    files['lineage.json']=encoded(lineage);files['package_source.py']=Path(__file__).read_bytes()
    for name,raw in files.items():
        if name.endswith('.json'):strict(raw)
    maximum=max(map(len,files.values()));assert maximum<45_000_000
    manifest={'scope':'Separate closed cold native30mm V4 open/reset/regrasp/nextpi with all original guards; earlier failed24mm V3 realpad recontact; separate static30mm sampled preflight NO ROLLOUT. Partial formed engagement below onepitch; capture/passive-reset/full-pitch/fulltrajectory qualification remains false.',
        'canonical_producer_commit':COMMIT,'canonical_software_tests_passed':281,'canonical281_covers_output_only_harnesses':False,
        'newer_canonical_producer_proof_not_relabelled_as_historical_native_trial':True,'audit_arithmetic_tests_remain_separate':True,
        'trials':trials,'static_preflight_scope':study['scope'],'static_preflight_sample_count':416,
        'exact_original_archives_preserved':True,'no_splice_no_interpolation_no_integration_no_geometry_edit':True,
        'native_v4_source_runtime_before_after_hashes_unchanged':True,'original_audit_provenance_hash_checks':checks,
        'all_artifacts_below45MB_no_chunks_needed':True,'maximum_artifact_bytes':maximum,
        'full_capture_qualified':False,'qualified_passive_reset':False,'full_pitch_lead_qualified':False,'continuous_full_trajectory':False,
        'files':{n:{'sha256':sha(raw),'bytes':len(raw)} for n,raw in sorted(files.items())}}
    files['package_manifest.json']=encoded(manifest);TARGET.mkdir(parents=True)
    for n,raw in files.items():
        p=TARGET/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {n}\n' for n,raw in sorted(files.items())))
    for n,raw in files.items():assert (TARGET/n).read_bytes()==raw,n
    print(json.dumps({'folder':str(TARGET.relative_to(ROOT)),'files':len(files)+1,'checksum_entries':len(files),'total_bytes':sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),'largest_bytes':maximum,'native_audit_hash_checks':checks},indent=2))
if __name__=='__main__':main()
