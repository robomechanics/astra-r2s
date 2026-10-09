"""Outcome-neutral byte gates for future CLOSED slower-reset v3 media.

Prepared while native run is LIVE. No simulator imports or state decoding.
Missing original AFTER is refused before report, trace, force, log or source
reads. Audited closure remains separate from native/thread task success.
"""
from pathlib import Path
import hashlib
import importlib.util
import json
import re

PRODUCER='9ae1a9fe76968a4013ea6c39e67718026622b9c0'
BEFORE_SHA='1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
PROOF_SHA='9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
VERIFIER_SHA='7ceed9d55285b4be3c804db9f5443efdb2d2786adcdd100b9faf114050400f3a'
SERIAL_SHA='e532c407cd35b51ad3a55bcbbdf98fc30676e8e994f427895c6a813875081e2c'
OFFICIAL=(('supported','independent_supported_audit.json'),
    ('left_pad','independent_left_pad_force_history_audit.json'),
    ('free_joint','independent_free_joint_properties.json'))

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for value in iter(lambda:stream.read(1024*1024),b''):digest.update(value)
    return digest.hexdigest()

def strict_json(path):
    def pairs(values):
        result={}
        for key,value in values:
            if key in result:raise ValueError('Duplicate JSON key: '+key)
            result[key]=value
        return result
    return json.loads(Path(path).read_text(),object_pairs_hook=pairs,
        parse_constant=lambda value:(_ for _ in ()).throw(ValueError(value)))

def require(condition,message):
    if not condition:raise ValueError(message)

def same_json(left,right):
    # Python equality equates bool with 0/1; preserved evidence must retain
    # JSON types as well as values, including nested acceptance flags.
    return json.dumps(left,sort_keys=True,separators=(',',':'),allow_nan=False)==json.dumps(right,sort_keys=True,separators=(',',':'),allow_nan=False)

def relative_file(root,name):
    root,name=Path(root).resolve(),Path(name)
    require(not name.is_absolute() and '..' not in name.parts and bool(name.parts),'Unsafe relative artifact name: '+str(name))
    result=root/name
    require(result.resolve().is_relative_to(root) and result.is_file(),'Missing/escaping artifact: '+str(name))
    return result

def verify_map(root,values,label,*,complete=False,except_names=()):
    require(isinstance(values,dict) and bool(values),'Missing '+label+' hash map')
    for name,digest in values.items():
        require(isinstance(digest,str) and re.fullmatch('[0-9a-f]{64}',digest) is not None,'Invalid '+label+' SHA: '+str(name))
        require(sha(relative_file(root,name))==digest,'Changed '+label+' artifact: '+str(name))
    if complete:
        actual={str(p.relative_to(root)) for p in Path(root).rglob('*') if p.is_file()}
        require(actual==set(values)|set(except_names),'Incomplete '+label+' inventory')

def verify_closed(run,root,proof_archive,identity_path,identity_sha256,audit_path,audit_sha256,*,runtime_paths=None):
    run,root,archive=map(lambda p:Path(p).resolve(),(run,root,proof_archive))
    after_path=run/'run_publication_identity_after.json'
    # FIRST, including when every supplied derivative/proof argument is invalid.
    require(after_path.is_file(),'Missing original AFTER: refuse LIVE/nonclosed run')
    identity_path,audit_path=Path(identity_path).resolve(),Path(audit_path).resolve()
    require(bool(identity_sha256) and sha(identity_path)==identity_sha256,'Frozen derivative identity SHA differs')
    require(bool(audit_sha256) and sha(audit_path)==audit_sha256,'Frozen complete audit binder SHA differs')
    identity,audit=strict_json(identity_path),strict_json(audit_path)
    require(audit.get('closed') is True and audit.get('producer_commit')==PRODUCER,'Binder must declare original closure and exact v3 producer')
    verifier_path=root/'outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py'
    require(sha(verifier_path)==VERIFIER_SHA,'Exact frozen generic identity verifier differs')
    spec=importlib.util.spec_from_file_location('media_frozen_generic_closed_identity',verifier_path)
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    checked_paths=identity.get('runtime_file_paths_checked')
    require(isinstance(checked_paths,dict),'Closed derivative lacks original runtime path coverage')
    actual_identity=verifier.verify(run,root,archive,expected_before_sha256=BEFORE_SHA,
        expected_producer=PRODUCER,expected_proof_sha256=PROOF_SHA,
        expected_after_sha256=identity.get('after_identity_sha256'),
        runtime_paths=checked_paths if runtime_paths is None else runtime_paths)
    require(set(actual_identity)==set(identity) and same_json(
        {k:v for k,v in actual_identity.items() if k!='runtime_file_paths_checked'},
        {k:v for k,v in identity.items() if k!='runtime_file_paths_checked'}),'Frozen saved identity differs from current exact generic verification')
    require(set(checked_paths)==set(actual_identity['runtime_file_paths_checked']),'Relocated native runtime path coverage differs')
    require(identity['software_tests_passed']==672 and identity['source_files_unchanged']==74,'Wrong original completed v3 proof count')
    before,after,proof=map(strict_json,(run/'run_publication_identity.json',after_path,run/'software_tests.json'))
    native_exit=after['native_exit_code']
    require(type(native_exit) is int and type(audit.get('native_child_exit_code')) is int
        and audit['native_child_exit_code']==native_exit,'Binder changed original integer native exit')
    require(isinstance(audit.get('original_run_relative_path'),str) and bool(audit['original_run_relative_path']),'Binder lacks declared original run location')
    original,audit_files=audit['original_run_files_sha256'],audit['audit_files_sha256']
    # These byte reads happen only AFTER original closure and anchored complete
    # derivative binder identity. No raw file is decoded by this gate.
    verify_map(run,original,'complete original closed native run',complete=True)
    verify_map(audit_path.parent,audit_files,'complete independent audit',complete=True,except_names=(audit_path.name,))
    checksums={}
    for line in (archive/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        require(name not in checksums,'Duplicate proof checksum name')
        checksums[name]=digest
    verify_map(archive,checksums,'complete 672-test software archive',complete=True,except_names=('SHA256SUMS',))
    directory=audit_path.parent
    for name in ('source_identity_before.json','source_identity_after.json'):
        saved=strict_json(relative_file(directory,name))
        require(same_json(saved,identity),'Official identity before/after does not bind the same closed original')
    declaration=strict_json(relative_file(directory,'serial_audit_declaration.json'))
    result=strict_json(relative_file(directory,'serial_audit_result.json'))
    require(declaration['schema']=='closed-supported-serial-audit-v3-declaration'
        and result['schema']=='closed-supported-serial-audit-v3-result','Wrong actual official serial record schemas')
    require(declaration['producer']==PRODUCER and declaration['original_before_sha256']==BEFORE_SHA
        and declaration['software_proof_sha256']==PROOF_SHA and declaration['wrapper_sha256']==SERIAL_SHA
        and declaration['generic_identity_verifier_sha256']==VERIFIER_SHA,'Actual official execution does not bind this frozen recipe')
    require(sha(relative_file(directory,'serial_audit_launcher_source.py'))==SERIAL_SHA
        and sha(relative_file(directory,'identity_verifier_source.py'))==VERIFIER_SHA,'Archived official execution helper bytes differ')
    require(result['reader_execution_completed'] is True and result['identity_and_original_bytes_unchanged'] is True
        and result['identity_after_exception'] is None and not result['final_inspection_exceptions']
        and result['remaining_readers_stopped_reason'] is None,'Official serial readers are not intact/completely executed')
    for record in (declaration,result):
        require(record['original_after_sha256']==identity['after_identity_sha256']
            and record['original_trace_sha256']==original['insertion_trace.npz']
            and type(record['original_native_exit_code']) is int and record['original_native_exit_code']==native_exit,'Official execution changed original closure/trace/outcome')
    require(declaration['original_run_files_sha256']==original
        and result['original_run_files_sha256_before']==original and result['original_run_files_sha256_after']==original,'Official execution original inventory differs')
    require(result['source_module_sha256_before']==result['source_module_sha256_after']==declaration['source_module_sha256_before'],'Official tested/reader chain bytes changed')
    # Original execution-chain extras may be relocated or absent in a media-
    # only replay. Their original before/after values and archived copies stay
    # bound; current tested producer/core bytes were independently checked above.
    module_map=result['source_module_sha256_after']
    for name,digest in before['source_hashes_before'].items():
        require(module_map.get(name)==digest,'Official module map omitted/changed tested producer: '+name)
    require(sha(relative_file(directory,'run_m8_launcher_source.sh'))==module_map.get('scripts/run_m8.sh'),'Archived original reader shell differs')
    abi_record=relative_file(directory,'auditor_bindings_verification.json')
    abi_path=declaration['auditor_bindings_record_path']
    require(sha(abi_record)==module_map.get(abi_path),'Archived original reader ABI record differs')
    executions=result['readers']
    require(len(executions)==3 and [r['reader'] for r in executions]==[name for name,_ in OFFICIAL],'Official reader order/coverage differs')
    official_reports={}
    for (label,report_name),summary in zip(OFFICIAL,executions):
        record=strict_json(relative_file(directory,label+'_execution.json'))
        require(same_json(record,summary),'Individual official execution differs from serial result: '+label)
        require(record['source_module_sha256_before']==record['source_module_sha256_after']==module_map,'Official per-reader module binding differs: '+label)
        require(record['status']=='exited' and type(record['exit_code']) is int and record['exit_code']==0
            and not record['inspection_exceptions'] and 'report_json_exception' not in record,'Official reader exception/not completed: '+label)
        require(record['source_modules_unchanged'] is True and record['original_after_unchanged'] is True
            and record['original_trace_unchanged'] is True,'Official reader changed source/original bytes: '+label)
        require(record['original_after_sha256_before']==record['original_after_sha256_after']==identity['after_identity_sha256']
            and record['original_trace_sha256_before']==record['original_trace_sha256_after']==original['insertion_trace.npz'],'Official per-reader original binding differs: '+label)
        require(record['output']==report_name and record['output_sha256']==sha(relative_file(directory,report_name))
            and record['log_sha256']==sha(relative_file(directory,label+'_stdout_stderr.log')),'Official report/log bytes differ: '+label)
        official_reports[label]=strict_json(relative_file(directory,report_name))
    # Outputs may legitimately report false; completion is not acceptance.
    require(official_reports['supported']['auditor_source_sha256']==before['frozen_audit_sources']['scripts/audit_m8_supported_trace.py'],'Official supported report source differs')
    require(official_reports['supported']['trajectory_sha256']==original['insertion_trace.npz'],'Official supported report audited a different original trajectory')
    require(official_reports['left_pad']['auditor_source_sha256']==before['frozen_audit_sources']['scripts/audit_m8_left_pad_force_history.py'],'Official left-pad report source differs')
    require(official_reports['free_joint']['auditor_sha256']==before['frozen_audit_sources']['scripts/audit_m8_free_joint_properties.py'],'Official free-joint report source differs')
    require(all(official_reports[label]['trajectory_sha256']==original['insertion_trace.npz'] for label in ('left_pad','free_joint')),'Official left/free reports audited a different original trajectory')
    native_report=strict_json(relative_file(run,'insertion_validation.json'))
    require(same_json(official_reports['supported']['original_report_passed'],native_report.get('passed'))
        and same_json(official_reports['supported']['original_acceptance_checks'],native_report.get('acceptance_checks')),'Official report changed original acceptance facts')
    require(same_json(declaration['original_report_passed_value'],native_report.get('passed'))
        and same_json(result['original_report_passed_value'],native_report.get('passed')),'Serial report changed native outcome')
    return dict(launch=before,after=after,proof=proof,identity=identity,audit=audit,
        sources=before['source_hashes_before'],original_run_files_sha256=original,audit_files_sha256=audit_files,
        identity_sha256=identity_sha256,audit_sha256=audit_sha256,
        root_identity_verifier_sha256=VERIFIER_SHA,official_serial_source_sha256=SERIAL_SHA,
        current_runtime_file_paths_checked=actual_identity['runtime_file_paths_checked'],
        original_native_report=native_report,official_reports=official_reports,
        scope='Anchored closed original and completed source-bound reader execution only; original/independent false acceptance is preserved, never inferred from CLI completion.')
