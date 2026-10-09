"""Bind prepared ignored V5 sources to the immutable original native parent."""
from pathlib import Path
import datetime
import hashlib
import json


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    folder=Path(__file__).resolve().parent
    root=folder.parents[2]
    prepare=json.loads((folder/'inertia_v5_prepare_only.json').read_text())
    declaration=prepare['declaration']
    if not (prepare['prepared'] and prepare['command_only_initialization']['qpos_qvel_preserved']
            and prepare['command_only_initialization']['source_65_unchanged']
            and prepare['command_only_initialization']['all_input_output_finite']):
        raise ValueError('Exact native initialization/command-only preparation did not pass')
    names=['crest_seat_inertia_probe_v5.py','prepare_inertia_probe_v5.py',
        'crest_seat_inertia_probe_v5.changes.json','inertia_v5_prepare_only.json',
        'robot_inertia_feedforward_v5.py','test_robot_inertia_feedforward_v5.py',
        'robot_inertia_feedforward_v5_pure_proof.json','robot_inertia_feedforward_v5_pure_proof.log',
        'test_inertia_rejection_artifacts_v5.py','inertia_rejection_artifacts_v5_pure_proof.json',
        'inertia_rejection_artifacts_v5_pure_proof.log',
        'reverse_brake_v4.py','test_reverse_brake_v4.py','reverse_brake_v4_pure_proof.json',
        'crest_seat_observer_v3.py','test_crest_seat_observer_v3.py',
        'launch_crest_inertia_v5.py','freeze_crest_inertia_v5.py']
    sources={name:sha(folder/name) for name in names}
    if sources['crest_seat_inertia_probe_v5.py']!=declaration['controller_source_sha256']:
        raise ValueError('Prepared harness changed')
    if sources['robot_inertia_feedforward_v5.py']!='0913f964c940d2fdf0bcbc92a0734c70c4868499439a4e5915e359a186dc17ff':
        raise ValueError('Tested helper changed')
    producer={name:sha(root/name) for name in declaration['source_65_before']}
    if len(producer)!=65 or producer!=declaration['source_65_before']:
        raise ValueError('Original65 source proof changed')
    parent=root/'outputs/m8_table_supported/full_canonical_v1'
    inputs={name:sha(parent/name) for name in declaration['parent_input_sha256']}
    inputs['controller_source.py']=sha(parent/'controller_source.py')
    if any(inputs[name]!=expected for name,expected in declaration['parent_input_sha256'].items()):
        raise ValueError('Original cold input changed')
    output='outputs/m8_table_supported/diagnostics/crest_seat_search_v5'
    if (root/output).exists():
        raise FileExistsError('V5 native output already exists')
    manifest={'frozen_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'scope':'Prepared source-only V5 experiment; native execution still requires parent authorization. No native physical success/capture/full qualification implied.',
        'producer_commit':declaration['producer_commit'],'producer_source_65':producer,
        'source_files':sources,'parent_input_sha256':inputs,'runtime':declaration['runtime'],
        'runtime_file_sha256':{entry['path']:sha(entry['path']) for entry in declaration['runtime']['libraries']},
        'checkpoint_time_s':declaration['parent_native_checkpoint_time_s'],
        'checkpoint_row':declaration['parent_native_checkpoint_index'],
        'output':output,
        'command':['scripts/run_m8.sh','outputs/m8_table_supported/diagnostics/crest_seat_inertia_probe_v5.py',
            '--parent','outputs/m8_table_supported/full_canonical_v1/insertion_trace.npz',
            '--checkpoint-time','13.1949','--brake-duration','.15','--output',output],
        'command_changes':'Approximate hybrid5D robot inertia FF through every closed phase and coherent retained native arm velocity for right damping/drag. Original C2/crest/coldwarm/physical guard criteria unchanged.',
        'limits':'No native object force, helix or free-body state write after declared cold initialization; separate cold diagnostic never splices a full trajectory. Canonical402/65 and previous67/69/50 proofs remain distinct.'}
    path=folder/'crest_seat_search_v5_frozen_inputs.json'
    if path.exists():
        raise FileExistsError('Never overwrite frozen V5 execution inputs')
    path.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'manifest_sha256':sha(path),'frozen_sources':len(sources),
        'original_sources':len(producer),'parents':len(inputs),'command':manifest['command']},indent=2))


if __name__=='__main__':
    main()
