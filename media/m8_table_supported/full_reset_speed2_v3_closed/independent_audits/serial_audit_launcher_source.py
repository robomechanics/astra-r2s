"""Prepared serial audit launcher for a separately confirmed CLOSED v3 run.

No execution during LIVE preparation. This standard-library wrapper performs
no model/trace decoding itself; official source-bound readers run only after
closure and identity checks. Reader completion is never native physics success.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

PRODUCER = '9ae1a9fe76968a4013ea6c39e67718026622b9c0'
BEFORE_SHA = '1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
PROOF_SHA = '9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
VERIFIER_SHA = '7ceed9d55285b4be3c804db9f5443efdb2d2786adcdd100b9faf114050400f3a'
OFFICIAL = (
    ('supported', 'scripts/audit_m8_supported_trace.py', 'independent_supported_audit.json'),
    ('left_pad', 'scripts/audit_m8_left_pad_force_history.py', 'independent_left_pad_force_history_audit.json'),
    ('free_joint', 'scripts/audit_m8_free_joint_properties.py', 'independent_free_joint_properties.json'),
)

def stamp():
    return datetime.now(timezone.utc).isoformat()

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()

def new_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')

def snapshot_files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob('*')) if p.is_file()}

def inspected_sha(path, exceptions, label):
    try:
        return sha(path)
    except Exception as error:
        exceptions.append({'artifact':label,'path':str(path),
            'type':type(error).__name__,'message':str(error),
            'scope':'Inspection failed; no SHA is invented for unavailable bytes.'})
        return None

def inspected_files(root, exceptions):
    try:
        paths=[p for p in sorted(root.rglob('*')) if p.is_file()]
    except Exception as error:
        exceptions.append({'artifact':'original_run_directory_scan','path':str(root),
            'type':type(error).__name__,'message':str(error)})
        return None
    return {str(p.relative_to(root)):inspected_sha(p,exceptions,str(p.relative_to(root))) for p in paths}

def run_serial(args):
    root, run, archive, output = (p.resolve() for p in
        (args.repository_root, args.run, args.proof_archive, args.output))
    # FIRST: no logs, trace, runtime, source or proof bytes are read while LIVE.
    if not (run/'run_publication_identity_after.json').is_file():
        raise ValueError('Missing original AFTER: refuse LIVE/nonclosed run before reading evidence.')
    if not args.confirmed_closed:
        raise ValueError('Run closure must first be confirmed by the parent; supply --confirmed-closed afterwards.')
    if (output.exists() or output.is_relative_to(run)
            or not output.is_relative_to(root/'outputs/m8_table_supported')):
        raise ValueError('Derivative audit directory must be fresh and outside the immutable original run.')
    verifier_path=root/'outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py'
    if sha(verifier_path)!=VERIFIER_SHA:
        raise ValueError('Frozen generic identity verifier differs from its explicit anchor.')
    spec=importlib.util.spec_from_file_location('frozen_closed_identity_v2',verifier_path)
    verifier=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(verifier)
    runtime_paths=verifier.strict_json(args.runtime_paths_json) if args.runtime_paths_json else None
    identity_kw=dict(expected_before_sha256=BEFORE_SHA, expected_producer=PRODUCER,
        expected_proof_sha256=PROOF_SHA, expected_after_sha256=args.expected_after_sha256,
        runtime_paths=runtime_paths)
    identity_before=verifier.verify(run,root,archive,**identity_kw)
    after_sha=identity_before['after_identity_sha256']
    after=verifier.strict_json(run/'run_publication_identity_after.json')
    if sha(run/'run_publication_identity_after.json')!=after_sha:
        raise ValueError('Original AFTER changed after generic identity verification.')
    original_source_map=dict(after['source_hashes_before'])
    native_exit=after['native_exit_code']
    if type(native_exit) is not int:
        raise ValueError('Original native exit must remain an integer; never a reader success flag.')
    if identity_before['software_tests_passed']!=672 or identity_before['source_files_unchanged']!=74:
        raise ValueError('This v3 recipe requires its completed 672-test/74-file proof.')
    # Bind the actual shell/ABI verification chain in addition to tested *.py.
    # Preserve the venv invocation path: resolving its Python symlink can lose
    # the virtual environment even when executable bytes are identical.
    auditor_python=args.auditor_python.absolute()
    bindings_sdk=args.bindings_sdk.resolve()
    bindings_record=bindings_sdk/'bindings_verification.json'
    if not auditor_python.is_file() or not os.access(auditor_python,os.X_OK):
        raise ValueError('Explicit matched auditor Python is absent or not executable.')
    checked_core_paths=identity_before['runtime_file_paths_checked']
    if len(checked_core_paths)!=1:
        raise ValueError('This matched auditor chain requires exactly one original checked core.')
    original_core_name,current_core_name=next(iter(checked_core_paths.items()))
    bindings=verifier.strict_json(bindings_record)
    if bindings.get('library_sha256')!=identity_before['runtime_file_sha256'][original_core_name]:
        raise ValueError('Explicit matched ABI record differs from the original checked core SHA.')
    effective_child_environment={'ASTRA_PYTHON':str(auditor_python),
        'M8_BINDINGS_SDK':str(bindings_sdk),'ASTRA_MUJOCO_LIB':current_core_name,
        'PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':'removed','PYTHONHOME':'removed',
        'LD_PRELOAD':'unset by the archived run_m8.sh'}
    trace=run/'insertion_trace.npz'
    if not trace.is_file():
        raise ValueError('Final original trajectory is absent; preserve raw-only evidence separately, never substitute a partial trace.')
    trace_sha=sha(trace)
    originals_before=snapshot_files(run)
    if (originals_before['run_publication_identity_after.json']!=after_sha
            or originals_before['insertion_trace.npz']!=trace_sha
            or native_exit!=identity_before['closure_native_exit_code']):
        raise ValueError('Original closure or trace changed before the derivative evidence baseline.')
    original_report=None
    original_report_exception=None
    report_path=run/'insertion_validation.json'
    if report_path.is_file():
        try:
            original_report=verifier.strict_json(report_path)
        except Exception as error:
            original_report_exception={'type':type(error).__name__,'message':str(error)}
    # Exact sources are already bound to the producer/proof by generic verify.
    tracked_modules={**original_source_map,
        'scripts/run_m8.sh':sha(root/'scripts/run_m8.sh'),
        'outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py':VERIFIER_SHA,
        str(Path(__file__).resolve()):sha(__file__),
        str(auditor_python):sha(auditor_python),str(bindings_record):sha(bindings_record),
        str(Path(current_core_name)):sha(current_core_name)}
    def module_snapshot(exceptions):
        return {name:inspected_sha(Path(name) if Path(name).is_absolute() else root/name,exceptions,name)
            for name in tracked_modules}
    initial_inspection_exceptions=[]
    modules_before=module_snapshot(initial_inspection_exceptions)
    if modules_before!=tracked_modules or initial_inspection_exceptions:
        raise ValueError('Producer/module bytes changed before audit launch.')
    output.mkdir(parents=True,exist_ok=False)
    new_json(output/'source_identity_before.json',identity_before)
    shutil.copyfile(__file__,output/'serial_audit_launcher_source.py')
    shutil.copyfile(verifier_path,output/'identity_verifier_source.py')
    shutil.copyfile(root/'scripts/run_m8.sh',output/'run_m8_launcher_source.sh')
    shutil.copyfile(bindings_record,output/'auditor_bindings_verification.json')
    records=[{'reader':label,'script':script,'output':name,'status':'unexecuted',
        'command':None,'started_utc':None,'finished_utc':None,'exit_code':None,
        'exception':None,'log_sha256':None,'output_sha256':None,
        'report_passed_value':None,'report_passed_field_present':False}
        for label,script,name in OFFICIAL]
    declaration={'schema':'closed-supported-serial-audit-v3-declaration',
        'producer':PRODUCER,'original_before_sha256':BEFORE_SHA,'software_proof_sha256':PROOF_SHA,
        'original_after_sha256':after_sha,'original_trace_sha256':trace_sha,
        'original_native_exit_code':native_exit,
        'original_native_report_present':report_path.is_file(),
        'original_report_json_exception':original_report_exception,
        'original_report_passed_field_present':isinstance(original_report,dict) and 'passed' in original_report,
        'original_report_passed_value':original_report.get('passed') if isinstance(original_report,dict) else None,
        'original_report_partial_value':original_report.get('partial') if isinstance(original_report,dict) else None,
        'original_run_files_sha256':originals_before,'source_module_sha256_before':modules_before,
        'wrapper_sha256':sha(__file__),'generic_identity_verifier_sha256':VERIFIER_SHA,
        'effective_reader_environment':effective_child_environment,
        'auditor_python_invocation_path':str(auditor_python),
        'auditor_python_resolved_binary_path':str(auditor_python.resolve()),
        'auditor_bindings_record_path':str(bindings_record),
        'execution_chain_extra_scope':'Closed-time observed shell/interpreter/ABI-record/core bytes are additional reader-execution provenance; they do not change or extend the original 74-file software proof.',
        'run':str(run),'repository_root':str(root),'proof_archive':str(archive),
        'created_utc':stamp(),'planned_readers':records,
        'scope':'Source-bound official independent readers execute serially after separately confirmed closure. Native exit/report and all reader outputs/exceptions remain distinct. No native success is inferred from CLI completion.'}
    new_json(output/'serial_audit_declaration.json',declaration)
    stopped_reason=None
    for record in records:
        if stopped_reason:
            record['unexecuted_reason']=stopped_reason
            continue
        script=root/record['script']
        target=output/record['output']
        log=output/(record['reader']+'_stdout_stderr.log')
        record['inspection_exceptions']=[]
        record['source_module_sha256_before']=module_snapshot(record['inspection_exceptions'])
        record['original_after_sha256_before']=inspected_sha(run/'run_publication_identity_after.json',record['inspection_exceptions'],'original_AFTER')
        record['original_trace_sha256_before']=inspected_sha(trace,record['inspection_exceptions'],'original_trace')
        if (record['source_module_sha256_before']!=modules_before
                or record['original_after_sha256_before']!=after_sha
                or record['original_trace_sha256_before']!=trace_sha):
            record['status']='unexecuted_identity_changed'
            stopped_reason='Source, original AFTER or trace identity changed before next reader.'
            record['exception']={'type':'IdentityChanged','message':stopped_reason}
            new_json(output/(record['reader']+'_execution.json'),record)
            continue
        command=[str(root/'scripts/run_m8.sh'),str(script),str(trace),'--output',str(target)]
        record['command']=command
        record['started_utc']=stamp()
        try:
            env=os.environ.copy()
            for key in ('ASTRA_PYTHON','M8_BINDINGS_SDK','ASTRA_MUJOCO_LIB','PYTHONDONTWRITEBYTECODE'):
                env[key]=effective_child_environment[key]
            env.pop('PYTHONPATH',None)
            env.pop('PYTHONHOME',None)
            with log.open('xb') as stream:
                result=subprocess.run(command,cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,check=False)
            record['exit_code']=result.returncode
            record['status']='exited'
        except Exception as error:
            record['status']='launcher_exception'
            record['exception']={'type':type(error).__name__,'message':str(error)}
        finally:
            record['finished_utc']=stamp()
            record['source_module_sha256_after']=module_snapshot(record['inspection_exceptions'])
            record['original_after_sha256_after']=inspected_sha(run/'run_publication_identity_after.json',record['inspection_exceptions'],'original_AFTER')
            record['original_trace_sha256_after']=inspected_sha(trace,record['inspection_exceptions'],'original_trace')
            record['source_modules_unchanged']=record['source_module_sha256_before']==record['source_module_sha256_after']
            record['original_after_unchanged']=record['original_after_sha256_after']==after_sha
            record['original_trace_unchanged']=record['original_trace_sha256_after']==trace_sha
            if log.is_file(): record['log_sha256']=inspected_sha(log,record['inspection_exceptions'],'reader_stdout_stderr')
            if target.is_file():
                record['output_sha256']=inspected_sha(target,record['inspection_exceptions'],'reader_output')
                try:
                    reader_report=verifier.strict_json(target)
                    record['report_passed_field_present']=isinstance(reader_report,dict) and 'passed' in reader_report
                    record['report_passed_value']=reader_report.get('passed') if isinstance(reader_report,dict) else None
                except Exception as error:
                    record['report_json_exception']={'type':type(error).__name__,'message':str(error)}
            if (record['inspection_exceptions'] or not all(record[k] for k in
                    ('source_modules_unchanged','original_after_unchanged','original_trace_unchanged'))):
                stopped_reason='Source, original AFTER or trace identity changed during reader; later readers remain unexecuted.'
            new_json(output/(record['reader']+'_execution.json'),record)
    identity_after=None
    identity_after_exception=None
    try:
        identity_after=verifier.verify(run,root,archive,**{**identity_kw,'expected_after_sha256':after_sha})
        new_json(output/'source_identity_after.json',identity_after)
    except Exception as error:
        identity_after_exception={'type':type(error).__name__,'message':str(error)}
    final_inspection_exceptions=[]
    originals_after=inspected_files(run,final_inspection_exceptions)
    modules_after=module_snapshot(final_inspection_exceptions)
    execution_completed=all(r['status']=='exited' and type(r['exit_code']) is int
        and r['exit_code']==0 and r['output_sha256'] is not None and r['log_sha256'] is not None
        and 'report_json_exception' not in r and not r.get('inspection_exceptions')
        for r in records)
    stable=(originals_before==originals_after and modules_before==modules_after
        and identity_after_exception is None and not final_inspection_exceptions)
    result={'schema':'closed-supported-serial-audit-v3-result','created_utc':stamp(),
        'original_native_exit_code':native_exit,'original_report_passed_value':declaration['original_report_passed_value'],
        'original_report_passed_field_present':declaration['original_report_passed_field_present'],
        'original_after_sha256':after_sha,'original_trace_sha256':trace_sha,
        'reader_execution_completed':execution_completed,'identity_and_original_bytes_unchanged':stable,
        'readers':records,'remaining_readers_stopped_reason':stopped_reason,
        'identity_after_exception':identity_after_exception,
        'final_inspection_exceptions':final_inspection_exceptions,
        'source_module_sha256_before':modules_before,'source_module_sha256_after':modules_after,
        'original_run_files_sha256_before':originals_before,'original_run_files_sha256_after':originals_after,
        'scope':'Reader completion/exit codes are execution evidence only. This wrapper supplies no physics/task passed field, does not alter native report, and preserves reader rejection or exception without waiver.'}
    new_json(output/'serial_audit_result.json',result)
    catalog_exceptions=[]
    new_json(output/'serial_audit_files.json',{'files_sha256':inspected_files(output,catalog_exceptions),
        'inspection_exceptions':catalog_exceptions,
        'scope':'Derivative files produced by this serial wrapper; original run remains separate and unchanged.'})
    return 0 if execution_completed and stable and not catalog_exceptions else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository-root',type=Path,required=True)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--proof-archive',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--expected-after-sha256')
    parser.add_argument('--runtime-paths-json',type=Path)
    parser.add_argument('--auditor-python',type=Path,default=Path('/workspace/.venvs/m8-contact/bin/python'))
    parser.add_argument('--bindings-sdk',type=Path,default=Path('/workspace/research/m8-contact-sdk'))
    parser.add_argument('--confirmed-closed',action='store_true')
    args=parser.parse_args()
    sys.exit(run_serial(args))
