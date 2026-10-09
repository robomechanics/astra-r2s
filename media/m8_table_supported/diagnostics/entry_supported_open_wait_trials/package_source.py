"""Preserve two closed failed fixed-time opening/wait diagnostics and media."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path('/workspace/astra-r2s')
BASE=ROOT/'outputs/m8_table_supported/diagnostics'
RENDER=BASE/'entry_supported_open_wait_media'
PARENT=ROOT/'media/m8_table_supported/face120_pickup_entry_v1'
STATE_PARENT=ROOT/'media/m8_table_supported/diagnostics/face120_partial_helical_start_v3'
TARGET=ROOT/'media/m8_table_supported/diagnostics/entry_supported_open_wait_trials'
TRIALS=('entry_supported_open_search_v1','entry_supported_open_search_v2')
COMMIT='b2b13ff39cd47c48afd19b38f83e9a405c9d6e32'
README=ROOT/'outputs/m8_table_supported/entry_supported_open_wait_README.md'
RECIPE_SUMMARY=ROOT/'outputs/m8_table_supported/open_wait_recipe_review_summary.json'
PREPUBLICATION_BINDING=ROOT/'outputs/m8_table_supported/open_wait_prepublication_recipe_binding.json'

def sha(raw):return hashlib.sha256(raw).hexdigest()
def strict(raw):return json.loads(raw,parse_constant=lambda v:(_ for _ in ()).throw(ValueError(v)))
def encoded(value):return (json.dumps(value,indent=2,allow_nan=False)+'\n').encode()
def fd(p):return sha(Path(p).read_bytes())
def main():
    if TARGET.exists():raise FileExistsError(TARGET)
    files={'README.md':README.read_bytes(),'publication_readme_source.md':README.read_bytes()}
    parent_validation=(PARENT/'validation_original.json.txt').read_bytes();parent_report=strict(parent_validation)
    parent_manifest=strict((PARENT/'package_manifest.json').read_bytes());parent_binding=strict((PARENT/'published_producer_binding.json').read_bytes())
    proof_bytes=(PARENT/'software_proof/software_proof.json').read_bytes();proof=strict(proof_bytes)
    assert parent_binding['published_producer_commit']==COMMIT and proof['passed'] and proof['tests_passed']==281 and proof['source_hashes_unchanged']
    assert fd(PARENT/'insertion_trace.npz')==parent_manifest['trajectory_sha256']
    state_report_bytes=(STATE_PARENT/'report.json').read_bytes();state_report=strict(state_report_bytes)
    assert state_report['all_original_guards_held'] and state_report['final_weight_window']['ready_for_diagnostic_release_attempt']
    assert state_report['trace_sha256']==fd(STATE_PARENT/'checkpoint_trace.npz')
    assert state_report['ledger_sha256']==fd(STATE_PARENT/'original_native_force_ledger.npz')
    files['canonical_prefix_validation_original.json']=parent_validation
    files['canonical281_software_proof.json']=proof_bytes
    files['canonical_prefix_published_producer_binding.json']=(PARENT/'published_producer_binding.json').read_bytes()
    files['closed_v3_state_parent_report_original.json']=state_report_bytes
    files['original_shared_first7400_rows_identity.json']=(BASE/'entry_supported_open_trials_prefix_identity.json').read_bytes()
    shared=strict(files['original_shared_first7400_rows_identity.json']);assert shared['all_original_column_prefixes_equal'] and all(shared['per_original_column_exact_prefix_equal'].values())
    with np.load(STATE_PARENT/'checkpoint_trace.npz',allow_pickle=False) as z:
        state_qpos=z['qpos'][-1].copy();state_qvel=z['qvel'][-1].copy();state_sample=strict(str(z['info_json']))[-1]
    seed=sha(state_qpos.tobytes()+state_qvel.tobytes())
    trials={};raws={};checks=0
    for name in TRIALS:
        directory=BASE/name
        for p in sorted(directory.rglob('*')):
            if p.is_file():files[name+'/'+str(p.relative_to(directory))]=p.read_bytes()
        files[name+'/original_stdout_stderr.log']=(BASE/(name+'.log')).read_bytes()
        for p in sorted((RENDER/name).rglob('*')):
            if p.is_file():files[name+'/render/'+str(p.relative_to(RENDER/name))]=p.read_bytes()
        rel=lambda field:files[name+'/'+field]
        declaration=strict(rel('declaration.json'));report=strict(rel('report.json'));binding=strict(rel('independent_audit_binding.json'))
        opened=strict(rel('independent_open_search_audit.json'));render=strict(rel('render/render_manifest.json'))
        assert declaration['parent_trace_sha256']==parent_manifest['trajectory_sha256']
        assert declaration['state_parent_trace_sha256']==state_report['trace_sha256']
        assert declaration['state_parent_report_sha256']==sha(state_report_bytes)
        assert declaration['initial_qpos_qvel_sha256']==seed
        assert declaration['initial_saved_state_time_s']==state_sample['time_s']
        assert declaration['parent_model']['model_xml_sha256']==sha(rel('scene.xml'))==parent_report['model_xml_sha256']
        assert declaration['diagnostic_source_sha256']==sha(rel('diagnostic_source.py'))
        assert declaration['observer_sha256']==sha(rel('observer_source.py'))==proof['source_hashes']['yam_twin/m8_supported_start.py']
        for source,digest in declaration['parent_source'].items():assert sha(rel('recorded_sources/'+source))==digest==proof['source_hashes'][source],source
        assert report['ledger_sha256']==sha(rel('original_native_force_ledger.npz'))
        assert report['trace_sha256']==sha(rel('checkpoint_trace.npz'))
        assert report['all_original_guards_held'] is False and report['is_full_capture_or_qualified_reset'] is False
        assert report['aborted']['failed_checks']==['fresh_fully_open_unloaded100ms_entry_support_not_ready']
        assert opened['all_recorded_native_hard_checks_held'] and not opened['original_producer_overall_guards_held']
        assert not opened['full_capture'] and not opened['qualified_reset'] and not opened['full_trajectory_qualified']
        assert opened['actual_regrasp_acquisition'] is None
        assert opened['observed_phase_names']==['release_search_2','open_settle_search_2']
        assert not opened['strict_open_ready_original_arithmetic']['final_fully_open_tick_ready']
        for key in ('original_files','closed_independent_reports'):
            for artifact,digest in binding[key].items():assert sha(rel(artifact))==digest,(name,artifact);checks+=1
        for artifact,digest in binding['source_snapshots'].items():assert sha(rel('audit_sources/'+artifact))==digest,(name,artifact);checks+=1
        assert binding['canonical281_software_proof_sha256']==sha(proof_bytes)
        assert render['trace_sha256']==report['trace_sha256'] and render['original_report_sha256']==sha(rel('report.json'))
        assert render['declaration_sha256']==sha(rel('declaration.json'))
        assert render['state_parent_trace_sha256']==state_report['trace_sha256']
        assert render['runtime']==declaration['runtime']
        assert render['exact_saved_state_replay'] and not render['physics_integration'] and not render['state_interpolation']
        assert not render['physical_bodies_hidden_or_geometry_changed'] and render['playback_speed']==1 and render['fps']==40
        assert render['missing_saved_state_tail_s']==opened['timing']['saved_state_tail_gap_s']
        assert not render['missing_state_tail_synthesized_or_integrated']
        for artifact,identity in render['media'].items():assert sha(rel('render/'+artifact))==identity['sha256']
        for artifact,digest in render['source_dependencies_sha256'].items():assert sha(rel('render/'+artifact))==digest
        with np.load(directory/'declared_cold_initialization.npz',allow_pickle=False) as z:
            assert np.array_equal(z['qpos'],state_qpos) and np.array_equal(z['qvel'],state_qvel)
            assert float(z['time'])==state_sample['time_s']
        with np.load(directory/'checkpoint_trace.npz',allow_pickle=False) as z:
            samples=strict(str(z['info_json']));qpos=z['qpos'].copy();qvel=z['qvel'].copy()
            assert strict(str(z['metadata_json']))==declaration
        assert len(samples)==len(qpos)==len(qvel)==render['recorded_saved_state_rows']
        assert all(s['all_checks_held'] and all(s['checks'].values()) for s in samples)
        for label,s in render['screenshot_states'].items():
            i=s['sample_index'];assert sha(qpos[i].tobytes())==s['qpos_sha256'] and sha(qvel[i].tobytes())==s['qvel_sha256']
            assert s['original_sample']==samples[i] and s['state_time_s']==samples[i]['time_s']
        for i,t in zip(render['video_exact_saved_state_indices'],render['video_exact_saved_state_times_s']):assert samples[i]['time_s']==t
        with np.load(directory/'original_native_force_ledger.npz',allow_pickle=False) as z:raw={k:z[k].copy() for k in z.files}
        assert len(raw['time_s'])==opened['native_steps'] and np.all(raw['all_checks_held']==1)
        openmask=raw['fully_open_unassisted']==1
        for key in ['right_bolt_contact_count','hand_gravity_opposing_force_N','right_applied_axial_feed_N','bolt_world_support_contact_count','nonthread_block_bolt_contact_count']:
            assert np.all(raw[key][openmask]==0),(name,key)
        assert np.all(raw['external_drive_zero']==1) and np.all(raw['loaded_actual_interior_flank_contact_count']==0)
        assert np.all(raw['right_grasp_guard_active']==0) and np.all(raw['right_axial_float_active']==0)
        assert len(raw['time_s'][openmask])==opened['fully_open']['native_ticks']
        raws[name]=raw
        trials[name]={
            'scope':declaration['scope'],'closed':True,'continuous_full_native_trajectory':False,
            'native_duration_s':report['actual_duration_s'],'native_wall_seconds':report['wall_seconds'],'native_steps':opened['native_steps'],
            'original_recorded_hard_checks_held':True,'original_overall_guards_held':False,'original_abort':report['aborted'],
            'actual_observed_phases':opened['observed_phase_names'],'reset_or_regrasp_or_next_turn_reached':False,
            'canonical_model_reference_parent_sha256':declaration['parent_trace_sha256'],
            'actual_cold_state_parent_sha256':declaration['state_parent_trace_sha256'],
            'actual_cold_state_parent_report_sha256':declaration['state_parent_report_sha256'],
            'exact_cold_qpos_qvel_sha256':seed,'original_warmstart_solver_state_restored':False,
            'trace_sha256':report['trace_sha256'],'force_ledger_sha256':report['ledger_sha256'],
            'source_harness_sha256':declaration['diagnostic_source_sha256'],'source_helper_sha256':declaration['observer_sha256'],
            'canonical281_tests_cover_this_output_only_harness':False,'additional_arithmetic_audits_not_combined_with281':True,
            'strict_readiness':opened['strict_open_ready_original_arithmetic'],
            'strict_instantaneous_valid_streaks':opened['strict_valid_streaks'],
            'observed_fully_open_support':opened['fully_open'],'unsupported_original_motion':opened['fully_open_bolt_motion'],
            'original_prepost_and_missing_state_timing':opened['timing'],
            'saved_state_tail_gap_s':opened['timing']['saved_state_tail_gap_s'],'tail_synthesized':False,
            'capture_qualified':False,'passive_reset_qualified':False,'opening_counts_as_capture_reward':False,
            'actual_loaded_conservative_full_interior_ticks':0,
            'original_argv':declaration['argv'],'original_phase_schedule':declaration['phase_schedule'],
            'frozen_feedback_during_open':'Dynamic head/tool feedback disabled; measured release transform frozen; finite XYZ robot impedance with zero whole-right/bolt contacts cannot support or drive the bolt.',
            'original_guard_scope':report['guard_scope'],
            'render_manifest_sha256':sha(rel('render/render_manifest.json')),
            'independent_audit_binding_sha256':sha(rel('independent_audit_binding.json'))}
        files[name+'/README.md']=trial_readme(name,trials[name]).encode()
    first,second=(raws[n] for n in TRIALS)
    assert set(first)==set(second) and all(np.array_equal(first[k],second[k][:7400]) for k in first)
    assert shared['first_original_sha256']==trials[TRIALS[0]]['force_ledger_sha256']
    assert shared['longer_original_sha256']==trials[TRIALS[1]]['force_ledger_sha256']
    assert trials[TRIALS[0]]['strict_readiness']['count']==0
    assert trials[TRIALS[1]]['strict_readiness']['count']==5671
    for p in sorted((RENDER/'plot').rglob('*')):
        if p.is_file():files['plot/'+str(p.relative_to(RENDER/'plot'))]=p.read_bytes()
    plot=strict(files['plot/plot_manifest.json']);assert plot['plot_sha256']==sha(files['plot/native_strict_window_history.png'])
    assert plot['source_sha256']==sha(files['plot/plot_entry_supported_open_wait.py'])
    assert plot['archived_arithmetic_dependency_sha256']==sha(files['plot/independent_open_search_audit_v1.py'])
    for name in TRIALS:
        assert plot['trials'][name]['original_ledger_sha256']==trials[name]['force_ledger_sha256']
        assert plot['trials'][name]['strict_readiness']==trials[name]['strict_readiness']
    references={
        'scope':'Exact canonical prefix model and original grip references remain unchanged. Both branches physical cold state comes from separately cold-initialized closed-v3 endpoint; no original warmstart/solver restoration or splice. Intentional opening suspends old right-grip guard as declared; no new regrasp reference was acquired.',
        'canonical_parent_package':'media/m8_table_supported/face120_pickup_entry_v1',
        'canonical_parent_trace_sha256':parent_manifest['trajectory_sha256'],'canonical_source_commit':COMMIT,
        'actual_state_parent_package':'media/m8_table_supported/diagnostics/face120_partial_helical_start_v3',
        'actual_state_parent_trace_sha256':state_report['trace_sha256'],'actual_state_parent_report_sha256':sha(state_report_bytes),
        'actual_state_parent_ledger_sha256':state_report['ledger_sha256'],
        'actual_cold_qpos_qvel_sha256':seed,'actual_state_parent_time_s':state_sample['time_s'],
        'upstream_reverse_state_package':'media/m8_table_supported/diagnostics/face120_closed_search_trials/face120_seat_search_v1',
        'upstream_reverse_state_trace_sha256':strict((STATE_PARENT/'declaration.json').read_bytes())['state_parent_trace_sha256'],
        'original_left_acquisition':parent_report['left_acquisition'],
        'original_right_grasp_acquisitions':parent_report['right_grasp_acquisitions'],
        'new_actual_regrasp_acquisition':None,'original_grip_references_redefined':False,
        'original_warmstart_solver_state_restored':False,'canonical281_source_proof_sha256':sha(proof_bytes)}
    files['canonical_model_grip_and_cold_state_lineage.json']=encoded(references)
    recipe_bytes=RECIPE_SUMMARY.read_bytes();recipe=strict(recipe_bytes)
    assert recipe['producer_commit']==COMMIT and recipe['proof_source_files_verified']==63
    reviewed=strict(PREPUBLICATION_BINDING.read_bytes())
    assert recipe['reviewed_open_package_ledger_sha256']==reviewed['reviewed_prepublication_ledger_sha256']
    assert recipe['actual_state_parent_files_sha256']['checkpoint_trace.npz']==state_report['trace_sha256']
    assert recipe['actual_state_parent_files_sha256']['report.json']==sha(state_report_bytes)
    assert recipe['actual_state_parent_files_sha256']['original_native_force_ledger.npz']==state_report['ledger_sha256']
    files['recipe_validation/summary_original.json']=recipe_bytes
    files['recipe_validation/reviewed_prepublication_binding.json']=PREPUBLICATION_BINDING.read_bytes()
    for short,name in zip(('v1','v2'),TRIALS):
        smoke=recipe['smokes'][short]
        assert smoke['harness_sha256']==trials[name]['source_harness_sha256']
        assert smoke['native_steps']==1 and smoke['native_duration_s']==5e-5 and smoke['exit_code']==0 and smoke['aborted'] is None
        assert smoke['actual_state_parent_trace_sha256']==state_report['trace_sha256'] and smoke['initial_qpos_qvel_sha256']==seed
        for original_path,digest in smoke['files'].items():
            raw=Path(original_path).read_bytes();assert sha(raw)==digest
            basename=Path(original_path).name
            files['recipe_validation/'+short+'/'+('one_step_stdout_stderr_original.log' if basename.endswith('.log') else 'one_step_'+basename)]=raw
    files['package_source.py']=Path(__file__).read_bytes()
    for name,raw in files.items():
        if name.endswith('.json'):strict(raw)
    maximum=max(map(len,files.values()));assert maximum<45_000_000
    manifest={
        'scope':'Two closed cold entry-supported SEARCH opening/wait trials. True whole-right contact-free open support is observed, original hard recorded guards hold, but both fixed strict-readiness gates fail. V2 earlier readiness is retained as observed evidence only; no reset/regrasp/nextturn/fullcapture/passive-reset/fulltrajectory qualification. V1 missing saved-state tail remains absent.',
        'canonical_source_commit':COMMIT,'canonical_app_helper_software_tests_passed':281,'canonical_source_proof_sha256':sha(proof_bytes),
        'canonical281_proof_covers_output_only_harnesses':False,'additional_six_opening_arithmetic_tests_remain_separate':True,
        'canonical_model_reference_parent_trace_sha256':parent_manifest['trajectory_sha256'],
        'actual_state_parent_trace_sha256':state_report['trace_sha256'],'actual_state_parent_report_sha256':sha(state_report_bytes),
        'actual_state_parent_ledger_sha256':state_report['ledger_sha256'],'trials':trials,
        'original_shared_first7400_native_rows_identical':True,'original_shared_identity_sha256':sha(files['original_shared_first7400_rows_identity.json']),
        'complete_original_native_force_state_source_model_audit_archives_preserved':True,
        'normal1x_exact_saved_state_media':True,'physics_integration_or_state_interpolation':False,
        'physical_bodies_hidden_or_model_changed':False,
        'original_contact_vector_scope':'Contact-local records retained at original5ms checkpoints and actual captured endpoints; full50us scalar ledger preserved. V1 absent4.95ms state/contact tail is not reconstructed.',
        'capture_qualified':False,'qualified_passive_reset':False,'continuous_full_trajectory':False,
        'separate_recipe_validation_smokes':{'v1':{'native_steps':1,'native_duration_s':5e-5},'v2':{'native_steps':1,'native_duration_s':5e-5}},
        'recipe_validation_summary_sha256':sha(recipe_bytes),
        'recipe_validation_scope':'Two separate initial-release50us layout/runtime checks with all63 source files and complete exact parent packages; not repeated full native branches, capture/reset/trajectory proof, or combined with281. Initial reviewed publication-ledger binding retained separately from final ledger.',
        'independent_original_audit_source_hash_checks':checks,'live_adaptive_v3_files_touched':False,
        'all_artifacts_below45MB_no_chunks_needed':True,'maximum_artifact_bytes':maximum,
        'scientific_plot_manifest_sha256':sha(files['plot/plot_manifest.json']),
        'files':{name:{'sha256':sha(raw),'bytes':len(raw)} for name,raw in sorted(files.items())}}
    files['package_manifest.json']=encoded(manifest)
    TARGET.mkdir(parents=True)
    for name,raw in files.items():
        p=TARGET/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name,raw in sorted(files.items())))
    for name,raw in files.items():assert (TARGET/name).read_bytes()==raw,name
    print(json.dumps({'folder':str(TARGET.relative_to(ROOT)),'files':len(files)+1,'checksum_entries':len(files),
        'total_bytes_including_ledger':sum(p.stat().st_size for p in TARGET.rglob('*') if p.is_file()),'largest_bytes':maximum,
        'independent_binding_hash_checks':checks,'reset_or_full_capture_qualified':False},indent=2))

def trial_readme(name,data):
    return f'''# {name}

Separate closed cold entry-supported SEARCH opening/wait trial.
Original native duration **{data['native_duration_s']:.5f} s**; original fixed
readiness abort `{json.dumps(data['original_abort'])}` stays failed.
Recorded native hard checks held; producer overall guards held is **false**.
No reset/regrasp/next turn/full capture/passive-reset qualification.

![Exact saved open-jaw state](render/open_jaw_gap.png)

[Normal 1× MP4](render/demo.mp4) · [Normal 1× GIF](render/demo.gif) ·
[Actual first state](render/entry_detail.png)

Saved-state tail gap: **{data['saved_state_tail_gap_s']*1000:.5f} ms**.
Missing state is never reconstructed. Exact qpos/qvel/mj_forward geometry
replay only; no integration, interpolation or body hiding. Caption forces
are original preintegration native solves at saved time minus50 µs.
Strict-ready endpoints: **{data['strict_readiness']['count']}**, first
`{data['strict_readiness']['first_elapsed_s']}`, last
`{data['strict_readiness']['last_elapsed_s']}`; final strict readiness is false.
The complete original reports/native/model/source/audits remain unchanged.

See [package instructions](../README.md) for original cold parents, precise
strict-window scope, complete dependencies and runnable reproduction commands.
'''
if __name__=='__main__':main()
