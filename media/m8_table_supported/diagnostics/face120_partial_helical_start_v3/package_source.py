"""Publish one closed cold v3 experiment with immutable native/audit lineage."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path('/workspace/astra-r2s')
BASE=ROOT/'outputs/m8_table_supported/diagnostics'
ORIGINAL=BASE/'face120_closed_forward_visual_v3'
MEDIA=BASE/'face120_partial_helical_media_v3'
PARENT=ROOT/'media/m8_table_supported/face120_pickup_entry_v1'
STATE_PARENT=ROOT/'media/m8_table_supported/diagnostics/face120_closed_search_trials/face120_seat_search_v1'
TARGET=ROOT/'media/m8_table_supported/diagnostics/face120_partial_helical_start_v3'
README=ROOT/'outputs/m8_table_supported/face120_partial_helical_v3_README.md'
RECIPE_SUMMARY=ROOT/'outputs/m8_table_supported/v3_recipe_review_summary.json'
PREPUBLICATION_BINDING=ROOT/'outputs/m8_table_supported/v3_prepublication_recipe_ledger_binding.json'
COMMIT='b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'

def sha(raw):return hashlib.sha256(raw).hexdigest()
def strict(raw):return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))
def encoded(value):return (json.dumps(value,indent=2,allow_nan=False)+'\n').encode()
def file_digest(path):return sha(Path(path).read_bytes())
def main():
    if TARGET.exists():raise FileExistsError(TARGET)
    files={str(p.relative_to(ORIGINAL)):p.read_bytes() for p in sorted(ORIGINAL.rglob('*')) if p.is_file()}
    files['original_stdout_stderr.log']=(BASE/'face120_closed_forward_visual_v3.log').read_bytes()
    for kind in ('render','plot'):
        for p in sorted((MEDIA/kind).rglob('*')):
            if p.is_file():files[kind+'/'+str(p.relative_to(MEDIA/kind))]=p.read_bytes()
    parent_report=strict((PARENT/'validation_original.json.txt').read_bytes())
    parent_manifest=strict((PARENT/'package_manifest.json').read_bytes())
    producer_binding=strict((PARENT/'published_producer_binding.json').read_bytes())
    proof_bytes=(PARENT/'software_proof/software_proof.json').read_bytes();proof=strict(proof_bytes)
    assert producer_binding['published_producer_commit']==COMMIT
    assert proof['passed'] and proof['tests_passed']==281 and proof['source_hashes_unchanged']
    assert file_digest(PARENT/'insertion_trace.npz')==parent_manifest['trajectory_sha256']
    files['canonical_prefix_validation_original.json']=(PARENT/'validation_original.json.txt').read_bytes()
    files['canonical281_software_proof.json']=proof_bytes
    files['canonical_prefix_published_producer_binding.json']=(PARENT/'published_producer_binding.json').read_bytes()
    declaration=strict(files['declaration.json']);report=strict(files['report.json'])
    binding=strict(files['independent_audit_binding.json']);render=strict(files['render/render_manifest.json'])
    plot=strict(files['plot/plot_manifest.json']);phase=strict(files['independent_phase_depth_audit.json'])
    formed=strict(files['independent_formed_entry_audit.json']);force=strict(files['independent_native_force_audit.json'])
    assert declaration['parent_trace_sha256']==parent_manifest['trajectory_sha256']
    assert declaration['parent_model']['model_xml_sha256']==parent_report['model_xml_sha256']==sha(files['scene.xml'])
    assert declaration['diagnostic_source_sha256']==sha(files['diagnostic_source.py'])
    assert declaration['observer_sha256']==sha(files['observer_source.py'])==proof['source_hashes']['yam_twin/m8_supported_start.py']
    assert report['ledger_sha256']==sha(files['original_native_force_ledger.npz'])
    assert report['trace_sha256']==sha(files['checkpoint_trace.npz'])
    assert report['all_original_guards_held'] is True and report['aborted'] is None
    for source,digest in declaration['parent_source'].items():
        assert sha(files['recorded_sources/'+source])==digest==proof['source_hashes'][source],source
    assert binding['frozen'] is True
    checks=0
    for key in ('original_files','closed_independent_reports'):
        for name,digest in binding[key].items():assert sha(files[name])==digest,(key,name);checks+=1
    for name,digest in binding['source_snapshots'].items():assert sha(files['audit_sources/'+name])==digest,name;checks+=1
    assert binding['canonical281_software_proof_sha256']==sha(proof_bytes)
    assert all(binding['canonical281_app_and_loaded_helper_subset_checks'].values())
    assert formed['capture_qualified'] is False and formed['unsupported_reset_qualified'] is False
    assert formed['full_trajectory_qualified'] is False and phase['capture_qualified'] is False
    assert formed['original_native_steps']==136562 and formed['actual_loaded_full_interior_substeps']==0
    assert formed['formed_geometry_substeps']==27583
    assert render['trace_sha256']==report['trace_sha256']
    assert render['original_report_sha256']==sha(files['report.json'])
    assert render['declaration_sha256']==sha(files['declaration.json'])
    assert render['runtime']==declaration['runtime']
    assert render['exact_saved_state_replay'] and not render['physics_integration'] and not render['state_interpolation']
    assert not render['physical_bodies_hidden_or_geometry_changed'] and render['playback_speed']==1
    for name,identity in render['media'].items():assert sha(files['render/'+name])==identity['sha256'],name
    for name,digest in render['source_dependencies_sha256'].items():assert sha(files['render/'+name])==digest,name
    assert plot['original_ledger_sha256']==report['ledger_sha256']
    assert plot['source_sha256']==sha(files['plot/plot_face120_partial_helical_v3.py'])
    assert plot['plot_sha256']==sha(files['plot/native_load_geometry.png'])
    assert plot['formed_entry_audit_sha256']==sha(files['independent_formed_entry_audit.json'])
    assert file_digest(STATE_PARENT/'checkpoint_trace.npz')==declaration['state_parent_trace_sha256']
    with np.load(STATE_PARENT/'checkpoint_trace.npz',allow_pickle=False) as z:
        state_p=z['qpos'][-1].copy();state_v=z['qvel'][-1].copy();state_sample=strict(str(z['info_json']))[-1]
    assert declaration['initial_qpos_qvel_sha256']==sha(state_p.tobytes()+state_v.tobytes())
    with np.load(ORIGINAL/'declared_cold_initialization.npz',allow_pickle=False) as z:
        assert np.array_equal(z['qpos'],state_p) and np.array_equal(z['qvel'],state_v)
        assert float(z['time'])==declaration['initial_saved_state_time_s']==state_sample['time_s']
    with np.load(ORIGINAL/'checkpoint_trace.npz',allow_pickle=False) as z:
        samples=strict(str(z['info_json']));qpos=z['qpos'].copy();qvel=z['qvel'].copy()
        assert strict(str(z['metadata_json']))==declaration
    assert len(samples)==len(qpos)==len(qvel)==render['recorded_saved_state_rows']==1367
    assert all(s['all_checks_held'] and len(s['checks'])==25 and all(s['checks'].values()) for s in samples)
    for name,state in render['screenshot_states'].items():
        i=state['sample_index'];assert state['qpos_sha256']==sha(qpos[i].tobytes()) and state['qvel_sha256']==sha(qvel[i].tobytes())
        assert state['state_time_s']==samples[i]['time_s'] and state['original_sample']==samples[i]
    for i,t in zip(render['video_exact_saved_state_indices'],render['video_exact_saved_state_times_s']):assert samples[i]['time_s']==t
    with np.load(ORIGINAL/'original_native_force_ledger.npz',allow_pickle=False) as z:
        assert len(z['time_s'])==136562 and np.all(z['all_checks_held']==1)
        assert np.all(z['loaded_actual_interior_flank_contact_count']==0)
        assert int(np.count_nonzero(z['formed_flank_overlap_m']>0))==27583
        assert np.all(z['net_feed_N']==declaration['bolt_weight_N'])
        first_feed=float(z['net_feed_N'][0]);lastwindow_start=float(z['time_s'][-2000]);maxformed=float(z['formed_flank_overlap_m'].max())
    final_normals=[c for s in samples if s['time_s']>=lastwindow_start-1e-9 for c in s['native_contact_records'] if c['local_force_N_Nm'][0]>1e-5]
    assert len(final_normals)==formed['original_loaded_entry_normals']['original_sampled_positive_normal_contacts']==82
    assert all(not c['is_actual_interior_flank_contact'] for c in final_normals)
    assert force['force_frame']['maximum_wrench_error_N_Nm']==0
    source=files['diagnostic_source.py'].decode()
    assert 'net=target_net' in source and 'left_feed=-args.left_down_force*left_blend-args.left_axial_damping*left_vz' in source
    feed={
        'scope':'Executed schedules from exact frozen native controller and original scalar ledger; no prescribed axial lead or free-body drive.',
        'constant_right_net_feed_N_from_first_cold_step':first_feed,
        'right_axial_damping_Ns_per_m':declaration['axial_velocity_damping_Ns_per_m'],
        'left_downward_forcefeed_N':declaration['left_downward_feed_N'],
        'left_force_preload_ramp_s':declaration['left_preload_s'],
        'left_worldZ_velocity_damping_Ns_per_m':declaration['left_axial_velocity_damping_Ns_per_m'],
        'alignment_s':declaration['alignment_s'],'closed_pre_turn_hold_s':declaration['closed_pre_turn_hold_s'],
        'post_turn_hold_s':declaration['post_turn_hold_s'],'strong_right_aperture_m':declaration['right_closed_aperture_m'],
        'unused_legacy_left_down_offset_cli_option':True,
        'privileged_perfect_native_pose_measurement':True,'measurement_control_rate_Hz':20000,
        'native_arm_cartesian_force_cap_N':8,'native_arm_cartesian_torque_cap_Nm':2,
        'original_harness_sha256':declaration['diagnostic_source_sha256'],'original_ledger_sha256':report['ledger_sha256']}
    files['executed_control_schedule.json']=encoded(feed)
    references={
        'scope':'Model and unchanged original grasp references come from the canonical prefix. Physical cold input separately comes from failed reverse-seat-v1 endpoint; it does not confirm that failed direction criterion or redefine grasp references.',
        'canonical_parent_package':'media/m8_table_supported/face120_pickup_entry_v1',
        'canonical_parent_trace_sha256':declaration['parent_trace_sha256'],
        'canonical_parent_code_commit':COMMIT,
        'actual_state_parent_package':'media/m8_table_supported/diagnostics/face120_closed_search_trials/face120_seat_search_v1',
        'actual_state_parent_trace_sha256':declaration['state_parent_trace_sha256'],
        'actual_cold_qpos_qvel_sha256':declaration['initial_qpos_qvel_sha256'],
        'actual_state_parent_time_s':declaration['initial_saved_state_time_s'],
        'original_parent_validation_sha256':sha(files['canonical_prefix_validation_original.json']),
        'original_left_acquisition':parent_report['left_acquisition'],
        'original_right_grasp_acquisitions':parent_report['right_grasp_acquisitions'],
        'original_grasp_references_replaced':False,'original_warmstart_solver_state_restored':False,
        'canonical281_proof_sha256':sha(proof_bytes),'canonical281_proof_covers_this_output_only_harness':False}
    files['canonical_prefix_reference_binding.json']=encoded(references)
    recipe_summary=RECIPE_SUMMARY.read_bytes();recipe=strict(recipe_summary)
    assert recipe['harness_sha256']==declaration['diagnostic_source_sha256']
    assert recipe['canonical_parent_trace_sha256']==declaration['parent_trace_sha256']
    assert recipe['actual_state_parent_sha256']==declaration['state_parent_trace_sha256']
    assert recipe['producer_commit']==COMMIT and recipe['native_steps']==1 and recipe['native_duration_s']==5e-5 and recipe['exit_code']==0
    files['recipe_validation/summary_original.json']=recipe_summary
    files['recipe_validation/reviewed_prepublication_binding.json']=PREPUBLICATION_BINDING.read_bytes()
    for original_path,digest in recipe['files'].items():
        raw=Path(original_path).read_bytes();assert sha(raw)==digest
        name=Path(original_path).name
        files['recipe_validation/'+('one_step_stdout_stderr_original.log' if name.endswith('.log') else 'one_step_'+name)]=raw
    files['publication_readme_source.md']=README.read_bytes();files['README.md']=README.read_bytes()
    files['package_source.py']=Path(__file__).read_bytes()
    for name,raw in files.items():
        if name.endswith('.json'):strict(raw)
    maximum=max(len(raw) for raw in files.values());assert maximum<45_000_000
    manifest={
        'scope':'Closed separate cold local v3 partial-helical starting-load trial. All25 original guards hold, jaws remain closed, geometry formed overlap is positive and original entry normals show helical slope, conservative full-interior count is zero. No qualified capture/full-pitch lead/opening/unsupported reset/continuous full trajectory. Perfect native pose feedback is privileged.',
        'original_directory':'outputs/m8_table_supported/diagnostics/face120_closed_forward_visual_v3',
        'native_duration_s':report['actual_duration_s'],'native_wall_seconds':report['wall_seconds'],
        'original_native_rows':136562,'original_saved_state_rows':1367,
        'all25_original_guards_held':True,'original_abort':None,
        'actual_positive_formed_geometry_substeps':27583,'maximum_formed_geometry_m':maxformed,
        'actual_loaded_full_interior_substeps':0,
        'original_sampled_final100ms_positive_normal_contacts':82,
        'original_loaded_entry_normal_inference':formed['original_loaded_entry_normals'],
        'geometric_short_entry_turn':formed['geometric_entry_turn'],
        'final_exact100ms':formed['final_exact100ms'],
        'capture_qualified':False,'full_pitch_lead_qualified':False,'unsupported_reset_qualified':False,
        'opening_executed':False,'continuous_full_trajectory_qualified':False,
        'perfect_native_pose_visual_servo':True,'executed_control_schedule':feed,
        'canonical_producer_commit':COMMIT,'canonical_source_tests_passed':281,
        'canonical_source_proof_sha256':sha(proof_bytes),'canonical281_proof_covers_output_only_harness':False,
        'additional_auditor_proofs_not_combined_with281':True,
        'canonical_model_reference_parent_sha256':declaration['parent_trace_sha256'],
        'actual_state_parent_sha256':declaration['state_parent_trace_sha256'],
        'exact_cold_initialization_state_sha256':declaration['initial_qpos_qvel_sha256'],
        'original_harness_sha256':declaration['diagnostic_source_sha256'],
        'original_helper_sha256':declaration['observer_sha256'],'runtime':declaration['runtime'],
        'original_force_ledger_sha256':report['ledger_sha256'],'original_trace_sha256':report['trace_sha256'],
        'original_report_sha256':sha(files['report.json']),'independent_audit_binding_sha256':sha(files['independent_audit_binding.json']),
        'independent_original_and_source_binding_hash_checks':checks,
        'complete_original_scalar_force_ledger_preserved':True,'complete_original_saved_states_preserved':True,
        'all_original_sampled_contact_normal_records_preserved':True,
        'contact_vector_archive_scope':'Original complete checkpoint contact-local force/normal records every100 native steps (5ms) plus final endpoint; scalar ledger retains every50us native step. No missing per-step vectors are invented.',
        'media_physics_integration':False,'media_state_interpolation':False,'media_physical_body_hiding':False,
        'media_normal1x_frames':render['video_frames'],'media_fps':render['fps'],
        'render_manifest_sha256':sha(files['render/render_manifest.json']),
        'scientific_plot_manifest_sha256':sha(files['plot/plot_manifest.json']),
        'all_original_logs_sources_and_audits_preserved':True,'upcoming_open_v4_files_touched':False,
        'separate_recipe_validation_native_steps':1,'separate_recipe_validation_duration_s':5e-5,
        'separate_recipe_validation_summary_sha256':sha(recipe_summary),
        'recipe_validation_scope':'Fresh isolated matching-source/model/input one-step layout/runtime smoke only. Not a repeated full native trial or physics qualification; not combined with281. Reviewed prepublication ledger identity retained separately from final publication ledger.',
        'all_artifacts_below45MB_no_chunks_needed':True,'maximum_artifact_bytes':maximum,
        'files':{name:{'sha256':sha(raw),'bytes':len(raw)} for name,raw in sorted(files.items())}}
    files['package_manifest.json']=encoded(manifest)
    TARGET.mkdir(parents=True)
    for name,raw in files.items():
        path=TARGET/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(files.items())))
    for name,raw in files.items():assert (TARGET/name).read_bytes()==raw,name
    print(json.dumps({'folder':str(TARGET.relative_to(ROOT)),'files':len(files)+1,'checksum_entries':len(files),
        'total_bytes_including_ledger':sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),
        'maximum_artifact_bytes':maximum,'independent_binding_hash_checks':checks,
        'native_duration_s':report['actual_duration_s'],'native_guards_hold':True,'capture_qualified':False},indent=2))
if __name__=='__main__':main()
