"""Copy one byte-stable actual phase-end prefix, never step/rebuild physics."""
import argparse
import hashlib
import json
import shutil
import uuid
from datetime import datetime,timezone
from pathlib import Path

import numpy as np


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for raw in iter(lambda:stream.read(1024*1024),b''):h.update(raw)
    return h.hexdigest()

def read(path):return json.loads(Path(path).read_text(),parse_constant=lambda token:(_ for _ in ()).throw(ValueError(token)))

def main(args):
    root,run,target=args.repository_root.resolve(),args.run.resolve(),args.output.resolve()
    assert not target.exists() and run!=target
    start=read(run/'run_publication_identity.json');proof=read(run/'software_tests.json')
    sources=start['source_hashes_before']
    assert start['producer_commit']=='9ae1a9fe76968a4013ea6c39e67718026622b9c0'
    assert sha(run/'run_publication_identity.json')=='1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
    assert sha(run/'software_tests.json')=='9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
    assert '--reset-speed' in start['command'] and start['command'][start['command'].index('--reset-speed')+1]=='2'
    assert len(sources)==74 and proof['source_hashes']==sources
    assert proof['passed'] is True and proof['tests_passed']==672 and proof['source_hashes_unchanged'] is True
    assert proof['runtime']==start['runtime']
    for name,digest in sources.items():assert sha(root/name)==digest,name
    for name,digest in start['runtime_file_sha256_before'].items():assert sha(name)==digest,name
    trace=run/'insertion_trace_partial.npz'
    original_trace_sha=sha(trace)
    static_names=('scene.xml','supported_scene.zip','scene_source.py','controller_source.py',
        'renderer_source.py','engagement_observer_source.py','run_publication_identity.json',
        'software_tests.json','software_tests.log','manifest.json','verify_sources.py','launch_source.py','native_stdout_stderr.log')
    originals={name:sha(run/name) for name in static_names}
    for path in sorted((run/'recorded_sources').rglob('*')):
        if path.is_file():originals[str(path.relative_to(run))]=sha(path)
    for path in sorted((run/'frozen_audit_sources').rglob('*')):
        if path.is_file():originals[str(path.relative_to(run))]=sha(path)
    originals['insertion_trace_partial.npz']=original_trace_sha
    target.parent.mkdir(parents=True,exist_ok=True)
    stage=target.parent/(target.name+'.copying-'+uuid.uuid4().hex);stage.mkdir()
    try:
        for name,digest in originals.items():
            dest=stage/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(run/name,dest)
            assert sha(dest)==digest==sha(run/name),name
        with np.load(stage/'insertion_trace_partial.npz',allow_pickle=False) as a:
            times,qpos,qvel=a['time'],a['qpos'],a['qvel'];rows=json.loads(str(a['info_json']));metadata=json.loads(str(a['metadata_json']))
            assert len(times)==len(qpos)==len(qvel)==len(rows)>0
            assert np.isfinite(qpos).all() and np.isfinite(qvel).all() and np.all(np.diff(times)>0)
            endpoint=rows[-1]
            if args.expected_last_phase is not None:assert endpoint['phase']==args.expected_last_phase,endpoint['phase']
            assert endpoint['all_hard_guards_held'] is True
            assert metadata['runtime']==start['runtime']
            assert metadata['control_config']['arm']['reset_speed_rad_s']==2.
            summaries=[]
            for line in (stage/'native_stdout_stderr.log').read_text().splitlines():
                try:item=json.loads(line)
                except json.JSONDecodeError:continue
                if isinstance(item,dict) and 'phase' in item:summaries.append(item)
            assert summaries and summaries[-1]['phase']==endpoint['phase'], 'stdout/prefix phase-end mismatch'
            regrasp_event_states=[]
            for event in metadata['physical_motion_events']:
                if event.get('event')!='Actual quiet bilateral regrasp acquired':continue
                event_time=event['reference']['time_s']
                matching=[i for i,row in enumerate(rows) if row['phase']==event['phase'] and row['time']==event_time]
                assert len(matching)==1, 'An actual event without its exact saved row is not a renderable state'
                i=matching[0]
                regrasp_event_states.append(dict(original_event=event,sample_index=i,time_s=float(times[i]),
                    qpos_sha256=hashlib.sha256(qpos[i].tobytes()).hexdigest(),
                    qvel_sha256=hashlib.sha256(qvel[i].tobytes()).hexdigest()))
            if endpoint['phase'].startswith('settle_regrip_'):
                assert endpoint['live_physical_phase_gate']=='complete'
                assert endpoint['actual_quiet_regrasp_streak_s']+1e-12>=metadata['control_config']['regrasp_window_s']
                assert any(event['original_event']['phase']==endpoint['phase'] and event['sample_index']==len(rows)-1 for event in regrasp_event_states)
            assert metadata['controller_module_sha256']==sha(stage/'controller_source.py')==sources['yam_twin/m8_supported_simulation.py']
            assert metadata['scene_source_sha256']==sha(stage/'scene_source.py')
            assert metadata['model_xml_sha256']==sha(stage/'scene.xml')
            assert metadata['starts_preengaged'] is False and metadata['starts_grasp_ready'] is False
            assert metadata['block_starts_on_table'] is True and metadata['bolt_starts_on_declared_fixed_rest'] is True
            catalog={name:dict(shape=list(a[name].shape),dtype=str(a[name].dtype)) for name in a.files}
            endpoint_state=dict(sample_index=len(times)-1,time_s=float(times[-1]),phase=endpoint['phase'],
                qpos_sha256=hashlib.sha256(qpos[-1].tobytes()).hexdigest(),qvel_sha256=hashlib.sha256(qvel[-1].tobytes()).hexdigest(),original_sample=endpoint)
        for name,digest in sources.items():assert sha(root/name)==digest,name
        for name,digest in start['runtime_file_sha256_before'].items():assert sha(name)==digest,name
        assert sha(trace)==original_trace_sha
        assert sha(run/'native_stdout_stderr.log')==originals['native_stdout_stderr.log']
        binding=dict(schema='fresh-full-reset-speed2-v3-phase-prefix-snapshot-v1',immutable_snapshot=True,
            historical_phase_end_progress=True,full_native_run_closed=False,
            dense_force_and_inertia_ledgers_pending_at_capture=True,independent_full_audit_pending_at_capture=True,
            copied_utc=datetime.now(timezone.utc).isoformat(),milestone=args.label,
            original_run=str(run),original_source_path=str(trace),trace_filename='insertion_trace_partial.npz',
            trajectory_sha256=original_trace_sha,file_sha256=originals,
            producer_commit=start['producer_commit'],source74_sha256_before_after_copy=sources,
            runtime=start['runtime'],runtime_file_sha256_before_after_copy=start['runtime_file_sha256_before'],
            original_publication_start_sha256=sha(stage/'run_publication_identity.json'),
            original672_software_proof_sha256=sha(stage/'software_tests.json'),
            original_endpoint=endpoint_state,original_npz_array_catalog=catalog,
            original_closed_phase_summaries=summaries,
            original_quiet_regrasp_event_saved_states=regrasp_event_states,
            explicit_original_reset_speed_rad_s=2.,
            fresh_initial_spawns_original_metadata_verified=True,
            scope='Exact original fullfresh native saved prefix through a closed phase, independent table/rest spawns and original event metadata. No cold checkpoint, splice, integration, force replay, body pose edits or native acceptance rewriting. Sparse original inertia_control records may be disabled/presencefalse; complete force/FF ledgers and independent full audit pending at capture.',
            helper_sha256=sha(__file__))
        (stage/'snapshot_binding.json').write_text(json.dumps(binding,indent=2,allow_nan=False)+'\n')
        shutil.copyfile(__file__,stage/'snapshot_helper_source.py')
        stage.rename(target)
        print(json.dumps(dict(snapshot=str(target),milestone=args.label,endpoint_time_s=endpoint_state['time_s'],endpoint_phase=endpoint_state['phase'],original_trace_sha256=original_trace_sha,saved_states=catalog['time']['shape'][0],files=len(originals)+2,snapshot_binding_sha256=sha(target/'snapshot_binding.json')),indent=2))
    except Exception:
        shutil.rmtree(stage)
        raise

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,default=Path('/workspace/astra-r2s'))
    p.add_argument('--label',required=True)
    p.add_argument('--expected-last-phase')
    p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    main(p.parse_args())
