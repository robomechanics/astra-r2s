"""Losslessly package a CLOSED two-PNG media stage of an immutable native PREFIX.

Standard library only; never reads the live native folder or imports arrays,
model, engine, renderer, or a closed-run gate. Native full closure/acceptance is
pending. Actual stage exit0 permits byte publication only, not physics reward.
Source-only preparation until root explicitly approves publication execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import types
import uuid

CHUNK_BYTES = 45_000_000
SNAPSHOT_SHA = '9c3deb00a19066c7c8e4d0be99d652257a4da0e98bac4fa451c4758edf9b32f7'
TRACE_SHA = 'c0f1ed6f05001cb7724af308fddbeb9c48109a8b1970d2a41b8dc7553a645829'
RENDERER_SHA = '7999ff1a3c97fa8981fcacafce15e50840b0f3df11d4631fd08abc166d4085de'
PRIOR_RENDERER_SHA = '2c34b87a512bc931965bba4c9d556677fa22c609ede4496ea906e39048b055d7'
PRIOR_MANIFEST_SHA = '29934a3d22722a64f1bb265be740ca4f59c06e88ecb485ec66aa97316864cf9e'
MANIFEST_SHA = 'ba2c3ec1ba29b333189aa1f85dfae1100ec7041fdec11db929a64bbf3cd608b8'
RECORDER_SHA = '34277d23eee1fb884965ef816356c6d99942e4f5caedbe86a68d6055c34ae6c3'
WRITER_SHA = '8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'
REASSEMBLER_SHA = '0706d299abfc70a0883aa8227ea3d89c7da420e961b4fc542fd696f3b05a3cfb'
RESTORER_SHA = '2a64428a4c1f3a96c9abce16f708576ac310d6e426eac54e38441097ac4cdf8b'
ORIGINS = {
    'render_full_reset_speed2_v3_captured_open_still_v3.py': RENDERER_SHA,
    'render_full_reset_speed2_v3_regrasp_progress_v2.py': '18c83b4d1747ec15988783dddf268b7e637e2cb8057bfa405931f552feacfa3a',
    'render_full_reset_speed2_v3_closed.py': 'ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8',
    'snapshot_helper_source.py': '91db4f94c295585b8f0756c816dfe1179f65ee6529d0a260a24d55f5b9135f56',
}


def identity(path):
    h, count = hashlib.sha256(), 0
    with Path(path).open('rb') as stream:
        for raw in iter(lambda: stream.read(1024*1024), b''):
            h.update(raw); count += len(raw)
    return dict(bytes=count,sha256=h.hexdigest())


def sha(path): return identity(path)['sha256']


def read(path):
    def pairs(items):
        result = {}
        for key,value in items:
            if key in result: raise ValueError('Duplicate JSON key: '+key)
            result[key] = value
        return result
    return json.loads(Path(path).read_text(),object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def rooted(root,value):
    value = Path(value)
    return (value if value.is_absolute() else root/value).resolve()


def relative(name):
    p = Path(name)
    if (not isinstance(name,str) or not name or p.is_absolute() or '..' in p.parts
            or p.as_posix() != name or any(c in name for c in ('\\','\n','\r','\0'))):
        raise ValueError('Unsafe artifact name: '+repr(name))
    return p


def tree(folder):
    if not folder.is_dir(): raise ValueError('Missing input directory: '+str(folder))
    result = {}
    for p in sorted(folder.rglob('*')):
        if p.is_symlink(): raise ValueError('Symlink input refused: '+str(p))
        if p.is_dir(): continue
        if not p.is_file() or not p.resolve().is_relative_to(folder): raise ValueError('Escaped/nonregular input')
        name = p.relative_to(folder).as_posix();relative(name);result[name] = p
    if not result: raise ValueError('Empty immutable input directory')
    return result


def same(a,b):
    return json.dumps(a,sort_keys=True,separators=(',',':'),allow_nan=False) == json.dumps(b,sort_keys=True,separators=(',',':'),allow_nan=False)


def stage_record(folder):
    start,finish = read(folder/'execution_started.json'),read(folder/'execution_finished.json')
    if (start['schema'] != 'reset-speed2-v3-media-stage-execution-v1'
            or start['recorder_source_sha256'] != RECORDER_SHA
            or sha(folder/'stage_recorder_source.py') != RECORDER_SHA
            or any(not same(v,finish.get(k)) for k,v in start.items())
            or type(finish['exit_code']) is not int
            or finish['stage_command_succeeded'] is not (finish['exit_code'] == 0)
            or finish['stdout_sha256'] != sha(folder/'stdout.log')
            or finish['stderr_sha256'] != sha(folder/'stderr.log')):
        raise ValueError('Recorded media stage facts/source/log bytes differ')
    return finish


def main(args):
    root = args.repository_root.resolve();base = root/'outputs/m8_table_supported'
    snapshot,render,stages = (rooted(root,v) for v in (args.snapshot,args.render,args.stage_provenance))
    prior_render = rooted(root,args.prior_render)
    selected_stage = rooted(root,args.render_stage);target = rooted(root,args.output)
    readme = rooted(root,args.readme)
    if (target.exists() or not target.is_relative_to(root/'media/m8_table_supported/full_reset_speed2_v3_progress')
            or any(target.is_relative_to(p) or p.is_relative_to(target) for p in (snapshot,render,prior_render,stages))):
        raise ValueError('A fresh distinct progress-publication target is required')
    if not selected_stage.is_relative_to(stages): raise ValueError('Selected renderer stage must be archived in full stage provenance')
    # Media-stage AFTER is mandatory FIRST. This is not a native full-run gate.
    if not (selected_stage/'execution_finished.json').is_file(): raise ValueError('Renderer stage is not CLOSED')
    executed = stage_record(selected_stage)
    if executed['exit_code'] != 0: raise ValueError('Actual renderer stage failed; no successful still packet')
    command = executed['command']
    if (not isinstance(command,list) or not all(isinstance(v,str) for v in command)
            or not any(Path(v).name == 'render_full_reset_speed2_v3_captured_open_still_v3.py' for v in command)
            or '--expected-phase' not in command or command[command.index('--expected-phase')+1] != 'reset_open_1'
            or rooted(root,command[-2]) != snapshot or rooted(root,command[-1]) != render):
        raise ValueError('Actual recorded renderer command differs from requested immutable inputs')
    if sha(snapshot/'snapshot_binding.json') != SNAPSHOT_SHA: raise ValueError('Snapshot anchor differs')
    binding = read(snapshot/'snapshot_binding.json')
    if sha(snapshot/'snapshot_helper_source.py') != ORIGINS['snapshot_helper_source.py']: raise ValueError('Original snapshot helper lineage differs')
    if (binding['immutable_snapshot'] is not True or binding['full_native_run_closed'] is not False
            or binding['trajectory_sha256'] != TRACE_SHA or binding['original_endpoint']['sample_index'] != 8846
            or binding['original_endpoint']['phase'] != 'settle_regrip_1'):
        raise ValueError('Only the approved complete historical prefix is accepted')
    original, rendered, prior_files, provenance = tree(snapshot),tree(render),tree(prior_render),tree(stages)
    if set(original) != set(binding['file_sha256'])|{'snapshot_binding.json','snapshot_helper_source.py'}:
        raise ValueError('Complete original snapshot inventory differs')
    for name,digest in binding['file_sha256'].items():
        relative(name)
        if sha(original[name]) != digest: raise ValueError('Original snapshot byte differs: '+name)
    if sha(snapshot/binding['trace_filename']) != TRACE_SHA: raise ValueError('Complete original trace byte differs')
    if sha(render/'still_manifest.json') != MANIFEST_SHA: raise ValueError('Root-observed actual v3 manifest differs')
    manifest = read(render/'still_manifest.json')
    if (manifest['mode'] != 'progress_single_saved_state' or manifest['full_native_run_closed'] is not False
            or manifest['independent_full_audit_pending'] is not True or manifest['helper_sha256'] != RENDERER_SHA
            or manifest['snapshot_binding_sha256'] != SNAPSHOT_SHA
            or not same(manifest['immutable_snapshot_binding'],binding)
            or not same(manifest['true_prefix_original_endpoint'],binding['original_endpoint'])
            or manifest['complete_original_prefix_saved_state_count'] != 8847
            or manifest['actual_saved_state_count'] != 1 or manifest['media_file_count'] != 2 or manifest['video_frames'] != 0):
        raise ValueError('Original still manifest identities/scope differ')
    selected = manifest['selected_state'];row = selected['original_sample']
    if (type(selected['sample_index']) is not int or not 0 <= selected['sample_index'] < 8846
            or selected['phase'] != 'reset_open_1' or row['phase'] != 'reset_open_1'
            or selected['time_s'] != row['time'] or manifest['selected_is_true_prefix_endpoint'] is not False
            or row['thread_engaged'] is not True or row['fully_open_unassisted'] is not True
            or row['all_hard_guards_held'] is not True or row['external_drive_zero'] is not True
            or type(row['right_robot_bolt_contact_count']) is not int or row['right_robot_bolt_contact_count'] != 0):
        raise ValueError('Selected reset row/true prefix endpoint distinction differs')
    summaries = [s for s in binding['original_closed_phase_summaries'] if s['phase'] == 'reset_open_1']
    if len(summaries) != 1 or not same(manifest['original_selected_closed_phase_summary'],summaries[0]):
        raise ValueError('Original selected closed phase summary differs')
    for key in ('mj_forward_called','collision_discovery_called','force_solve_called','physics_integration',
            'state_interpolation','model_regeneration','body_hiding_or_geometry_change'):
        if manifest[key] is not False: raise ValueError('Still replay scope changed: '+key)
    expected_render = {'still_manifest.json','captured_open.png','captured_open_detail.png'}|{'renderer_sources/'+n for n in ORIGINS}
    if set(rendered) != expected_render: raise ValueError('Complete two-PNG renderer inventory differs')
    for name,digest in ORIGINS.items():
        if sha(rendered['renderer_sources/'+name]) != digest: raise ValueError('Executing/helper origin differs: '+name)
    for name,digest in manifest['media_sha256'].items():
        if name not in ('captured_open.png','captured_open_detail.png') or sha(render/name) != digest:
            raise ValueError('Actual PNG byte differs')
    if set(manifest['media_sha256']) != {'captured_open.png','captured_open_detail.png'}: raise ValueError('PNG coverage differs')
    # Preserve the already successful v2 attempt and its overlapped caption exactly.
    # This hard anchor is the root-observed original manifest, never rewritten.
    if sha(prior_render/'still_manifest.json') != PRIOR_MANIFEST_SHA:
        raise ValueError('Original successful v2 manifest differs')
    prior = read(prior_render/'still_manifest.json')
    prior_origins = {n:d for n,d in ORIGINS.items() if n != 'render_full_reset_speed2_v3_captured_open_still_v3.py'}
    prior_origins['render_full_reset_speed2_v3_captured_open_still_v2.py'] = PRIOR_RENDERER_SHA
    if (not same(prior['selected_state'],selected) or not same(prior['true_prefix_original_endpoint'],binding['original_endpoint'])
            or set(prior_files) != {'still_manifest.json','captured_open.png','captured_open_detail.png'}|{'renderer_sources/'+n for n in prior_origins}
            or any(sha(prior_files['renderer_sources/'+n]) != d for n,d in prior_origins.items())
            or any(sha(prior_render/n) != d for n,d in prior['media_sha256'].items())):
        raise ValueError('Prior successful v2 state/source/media inventory differs')
    records = {p.parent.relative_to(stages).as_posix():stage_record(p.parent) for p in stages.rglob('execution_started.json')}
    if selected_stage.relative_to(stages).as_posix() not in records: raise ValueError('Selected successful stage absent from full provenance')
    sources = manifest['source74_before_after'];runtime = manifest['runtime_file_sha256_before_after']
    if len(sources) != 74 or not same(sources,binding['source74_sha256_before_after_copy']): raise ValueError('Producer source proof differs')
    for name,digest in sources.items():
        relative(name)
        if sha(root/name) != digest: raise ValueError('Current frozen producer source differs: '+name)
    for name,digest in runtime.items():
        if sha(name) != digest: raise ValueError('Current frozen native runtime differs')
    inputs = {}
    for prefix,files in (('native_prefix',original),('render',rendered),('render_attempts/selected_reset_render_v2',prior_files),('execution_provenance',provenance)):
        for name,path in files.items(): inputs[prefix+'/'+name] = path
    for name in sources: inputs['producer_sources/'+name] = root/name
    dependencies = {'publish_supported_feedback_run.py':WRITER_SHA,'reassemble_archives.py':REASSEMBLER_SHA,'restore_original_run_layout.py':RESTORER_SHA}
    for name,digest in dependencies.items():
        if sha(base/name) != digest: raise ValueError('Frozen byte-only publication dependency differs: '+name)
        inputs['publication_dependencies/'+name] = base/name
    if sha(base/'record_full_reset_speed2_v3_progress_stage.py') != RECORDER_SHA: raise ValueError('Current stage recorder source differs')
    inputs['publication_dependencies/record_full_reset_speed2_v3_progress_stage.py'] = base/'record_full_reset_speed2_v3_progress_stage.py'
    inputs['publication_dependencies/'+Path(__file__).name] = Path(__file__).resolve()
    inputs['publication_dependencies/render_full_reset_speed2_v3_captured_open_still_v1.py'] = base/'render_full_reset_speed2_v3_captured_open_still_v1.py'
    if sha(inputs['publication_dependencies/render_full_reset_speed2_v3_captured_open_still_v1.py']) != '17208496da1bfdd72ebe5415abd503503052cbeec4002527b99eccc7f61fdc7b': raise ValueError('Unexecuted v1 preparation differs')
    original_id = {n:identity(p) for n,p in original.items()};before = {n:identity(p) for n,p in inputs.items()}
    standalone = {'README.md':readme,'reassemble_archives.py':base/'reassemble_archives.py','restore_original_run_layout.py':base/'restore_original_run_layout.py','publication_helper_source.py':Path(__file__).resolve()}
    if readme.is_symlink() or not readme.is_file(): raise ValueError('Regular standalone README required')
    standalone_before = {n:identity(p) for n,p in standalone.items()}
    if any(v['bytes']>CHUNK_BYTES for v in standalone_before.values()): raise ValueError('Standalone artifact exceeds45MB')
    writer = types.ModuleType('frozen_progress_byte_writer');writer.__file__ = str(base/'publish_supported_feedback_run.py')
    exec(compile((base/'publish_supported_feedback_run.py').read_bytes(),writer.__file__,'exec'),writer.__dict__)
    target.parent.mkdir(parents=True,exist_ok=True)
    stage = target.parent/(target.name+'.packaging-'+uuid.uuid4().hex);stage.mkdir()
    try:
        artifacts = {n:writer.write_artifact(p,stage,n,CHUNK_BYTES) for n,p in inputs.items()}
        for n,p in standalone.items():
            shutil.copyfile(p,stage/n)
            if identity(stage/n) != standalone_before[n]: raise ValueError('Copied standalone byte differs')
        if (tree(snapshot) != original or tree(render) != rendered or tree(prior_render) != prior_files or tree(stages) != provenance
                or any(identity(p) != before[n] for n,p in inputs.items())
                or any(identity(p) != standalone_before[n] for n,p in standalone.items())
                or any(sha(p) != digest for p,digest in ((Path(n),d) for n,d in runtime.items()))):
            raise ValueError('Frozen original/source/runtime bytes changed during byte publication')
        packet = dict(schema='captured-open-single-state-progress-byte-packet-v1',status='immutable_prefix_progress',
            full_native_run_closed=False,independent_full_audit_pending=True,native_result_inferred=False,
            physics_acceptance_inferred=False,closed_renderer_stage_exit_code=executed['exit_code'],
            original_complete_saved_prefix_states=8847,selected_saved_states_rendered=1,selected_png_files=2,total_preserved_png_files=4,video_frames=0,
            snapshot_binding_sha256=SNAPSHOT_SHA,original_complete_trace_sha256=TRACE_SHA,
            original_true_prefix_endpoint=binding['original_endpoint'],actual_selected_state=selected,
            original_selected_closed_phase_summary=summaries[0],original_still_manifest_sha256=sha(render/'still_manifest.json'),
            prior_successful_render_manifest_sha256=PRIOR_MANIFEST_SHA,prior_successful_render_source_sha256=PRIOR_RENDERER_SHA,
            prior_v2_caption_overlap_preserved=True,selected_v3_caption_reposition_only=True,
            original_command_retained_native_state_time_s=manifest['original_command_retained_native_state_time_s'],
            original_force_state_timing=manifest['original_force_state_timing'],producer_commit=binding['producer_commit'],
            source74_sha256=sources,runtime_file_sha256=runtime,replay_plugin_library=manifest['replay_plugin_library'],
            original_metadata_and_flags_unmodified=True,full_trace_trimmed=False,stage_execution_facts=records,
            original_run_file_identities=original_id,
            original_run_file_to_published_artifact_names={n:['native_prefix/'+n] for n in original},
            original_snapshot_layout_restoration_only=True,not_a_native_after_gate_waiver=True,
            unexecuted_v1_preparation=dict(sha256='17208496da1bfdd72ebe5415abd503503052cbeec4002527b99eccc7f61fdc7b',executed=False,selection_defect='Endpoint-only v1 cannot select earlier reset phase from complete settle_regrip_1 prefix'),
            chunk_bytes=CHUNK_BYTES,artifacts=artifacts,logical_artifacts=len(artifacts),
            original_snapshot_files=len(original),render_files=len(rendered),prior_render_files=len(prior_files),stage_provenance_files=len(provenance),
            all_original_snapshot_bytes_preserved=True,all_original_relative_names_preserved=True,
            scope='Byte completeness/restoration and actual saved-state rendering only; original captured/reset progress flags retained, full native closure and independent physics acceptance pending.')
        (stage/'package_manifest.json').write_text(json.dumps(packet,indent=2,allow_nan=False)+'\n')
        stored = tree(stage)
        if any(p.stat().st_size>CHUNK_BYTES for p in stored.values()): raise ValueError('Stored byte artifact exceeds45MB')
        (stage/'SHA256SUMS').write_text(''.join(sha(p)+'  '+n+'\n' for n,p in stored.items()))
        stage.rename(target)
    except Exception:
        shutil.rmtree(stage,ignore_errors=True);raise
    print(json.dumps(dict(output=str(target),logical_artifacts=len(artifacts),stored_files=len(stored)+1,
        original_snapshot_files=len(original),selected_phase=selected['phase'],selected_time_s=selected['time_s'],
        package_manifest_sha256=sha(target/'package_manifest.json'),checksum_ledger_sha256=sha(target/'SHA256SUMS'),
        full_native_run_closed=False,full_physics_acceptance_inferred=False),indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,required=True)
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--render',type=Path,required=True)
    p.add_argument('--prior-render',type=Path,required=True,help='Archive original successful v2 tree unchanged alongside selected v3 render')
    p.add_argument('--stage-provenance',type=Path,required=True);p.add_argument('--render-stage',type=Path,required=True)
    p.add_argument('--readme',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
