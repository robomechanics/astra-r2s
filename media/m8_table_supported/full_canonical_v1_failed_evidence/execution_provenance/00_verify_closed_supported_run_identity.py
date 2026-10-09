"""Read-only post-closure binding checks; never imports MuJoCo or application code."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
PRODUCER='6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1'
AUDITOR='e46808601d7974ffdea1e5e20e07e0f28eb8e0a74a394545b276621a7e856702'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify(run):
    run=Path(run).resolve()
    after_path=run/'run_publication_identity_after.json'
    if not after_path.is_file():raise ValueError('Native launch has no closure identity. Do not audit a live run.')
    before=json.loads((run/'run_publication_identity.json').read_text())
    after=json.loads(after_path.read_text())
    proof=json.loads((run/'software_tests.json').read_text())
    if before['producer_commit']!=PRODUCER or after['producer_commit']!=PRODUCER:
        raise ValueError('Run is not the declared fresh6e7 producer.')
    if proof['tests_passed']!=402 or proof['passed'] is not True or proof['source_hashes_unchanged'] is not True:
        raise ValueError('Expected exact402-test source-stable proof is absent.')
    source=before['source_hashes_before']
    if len(source)!=65 or source!=after['source_hashes_before'] or source!=after['source_hashes_after'] or source!=proof['source_hashes']:
        raise ValueError('Original before/after/proof source identities disagree.')
    current={name:sha(ROOT/name)for name in source}
    if current!=source or after['source_hashes_unchanged'] is not True:
        raise ValueError('Complete tested source bytes changed. Use exact producer checkout before auditing; do not patch around mismatch.')
    frozen=before['frozen_audit_sources']
    if frozen!=after['frozen_audit_sources']:
        raise ValueError('Frozen auditor declaration changed during native execution.')
    for name,digest in frozen.items():
        if sha(run/'frozen_audit_sources'/name)!=digest or sha(ROOT/name)!=digest:
            raise ValueError('Current/frozen audit dependency differs: '+name)
    if frozen['scripts/audit_m8_supported_trace.py']!=AUDITOR:
        raise ValueError('Supported auditor is not the frozen402-bound e468 source.')
    manifest=json.loads((run/'manifest.json').read_text())
    for name,item in manifest['files'].items():
        if sha(run/name)!=item['sha256']or (run/name).stat().st_size!=item['bytes']:
            raise ValueError('Immutable launch/software proof artifact changed: '+name)
    trace=run/'insertion_trace.npz'
    return {'kind':'Read-only post-closure native producer/auditor source identity',
        'verifier_source_sha256':sha(__file__),'producer_commit':PRODUCER,
        'software_tests_passed':402,'source_files_unchanged':65,
        'before_identity_sha256':sha(run/'run_publication_identity.json'),
        'after_identity_sha256':sha(after_path),'complete_current_source_matches_producer':True,
        'frozen_audit_sources':frozen,'closure_native_exit_code':after['native_exit_code'],
        'final_trace_present':trace.is_file(),'final_trace_sha256':sha(trace)if trace.is_file()else None,
        'scope':'Closure identity and exact source bytes only. A native nonzero exit or physics failure is retained. No imports, native compile, collision, force solve, dynamics, report alteration or task success inference.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path);args=parser.parse_args();result=verify(args.run)
    value=json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.output:args.output.write_text(value)
    print(value,end='')
