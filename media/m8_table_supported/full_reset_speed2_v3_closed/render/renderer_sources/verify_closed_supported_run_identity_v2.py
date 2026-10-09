"""Read-only closure/provenance verification, with no simulator imports.

Counts, commits and hashes come from pinned original artifacts or explicit CLI
anchors. Native exit/failure is retained; identity does not imply task success.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def strict_json(path):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('Duplicate original JSON field: '+key)
            result[key] = value
        return result
    def nonfinite(value):
        raise ValueError('Nonfinite original JSON value: '+value)
    return json.loads(Path(path).read_text(), object_pairs_hook=pairs, parse_constant=nonfinite)


def digest_map(value, label):
    if not isinstance(value, dict) or not value:
        raise ValueError(label+' must be a nonempty original hash map')
    if any(not isinstance(name, str) or not name or not isinstance(digest, str)
           or re.fullmatch('[0-9a-f]{64}', digest) is None for name, digest in value.items()):
        raise ValueError(label+' has invalid original names or SHA256 values')
    return value


def relative_file(root, name):
    relative = Path(name)
    root = Path(root).resolve()
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('Invalid original relative artifact name: '+str(name))
    result = root/relative
    if not result.resolve().is_relative_to(root) or not result.is_file():
        raise ValueError('Original artifact is absent or escapes its root: '+str(name))
    return result


def anchored(path, expected, label):
    actual = sha(path)
    if expected is not None and actual != expected:
        raise ValueError(label+' differs from its supplied immutable SHA256 anchor')
    return actual


def verify(run, repository_root, proof_archive, *, expected_before_sha256=None,
           expected_producer=None, expected_proof_sha256=None,
           expected_after_sha256=None, runtime_paths=None):
    run, root, archive = map(lambda p: Path(p).resolve(), (run, repository_root, proof_archive))
    after_path = run/'run_publication_identity_after.json'
    # FIRST: do not examine partial states, force histories or an active log.
    if not after_path.is_file():
        raise ValueError('Native launch has no closure identity. Refuse LIVE/nonclosed run.')
    before_path, proof_path = run/'run_publication_identity.json', run/'software_tests.json'
    before_sha = anchored(before_path, expected_before_sha256, 'Original BEFORE identity')
    after_sha = anchored(after_path, expected_after_sha256, 'Original AFTER identity')
    proof_sha = anchored(proof_path, expected_proof_sha256, 'Original completed software proof')
    before, after, proof = map(strict_json, (before_path, after_path, proof_path))
    producer = before['producer_commit']
    if (not isinstance(producer, str) or not producer or after['producer_commit'] != producer
            or (expected_producer is not None and producer != expected_producer)):
        raise ValueError('Original producer commit differs from its declared binding')
    if (proof['passed'] is not True or proof['source_hashes_unchanged'] is not True
            or type(proof['tests_passed']) is not int or proof['tests_passed'] <= 0):
        raise ValueError('Completed positive source-stable software proof is absent')
    source = digest_map(before['source_hashes_before'], 'Original tested source map')
    if (source != after['source_hashes_before'] or source != after['source_hashes_after']
            or source != proof['source_hashes'] or after['source_hashes_unchanged'] is not True):
        raise ValueError('Original BEFORE/AFTER/software source identities disagree')
    for name, digest in source.items():
        if sha(relative_file(root, name)) != digest:
            raise ValueError('Current tested source differs from original producer: '+name)
        if sha(relative_file(archive/'source_files', name)) != digest:
            raise ValueError('Archived software source differs from original producer: '+name)
    archived_proof = relative_file(archive, 'software_proof.json')
    if sha(archived_proof) != proof_sha:
        raise ValueError('Original run proof differs from the complete archived software proof')
    # The immutable launch manifest binds actual child stdout and verifier bytes.
    manifest_path = run/'manifest.json'
    manifest = strict_json(manifest_path)
    if not isinstance(manifest.get('files'), dict) or not manifest['files']:
        raise ValueError('Original launch/software manifest is absent')
    required = {'software_tests.json', 'software_tests.log', 'verify_sources.py', 'launch_source.py'}
    if not required.issubset(manifest['files']):
        raise ValueError('Original manifest omits a launch/software child artifact')
    for name, declaration in manifest['files'].items():
        path = relative_file(run, name)
        if (type(declaration.get('bytes')) is not int or declaration['bytes'] < 0
                or sha(path) != declaration['sha256'] or path.stat().st_size != declaration['bytes']):
            raise ValueError('Original launch/software manifest artifact changed: '+name)
    if manifest['files']['software_tests.json']['sha256'] != proof_sha:
        raise ValueError('Manifest proof identity differs from original software proof')
    if sha(relative_file(archive, 'pytest_all.log')) != sha(run/'software_tests.log'):
        raise ValueError('Original software child log differs from archived completed proof')
    if (sha(relative_file(archive, 'verification_source.py')) != sha(run/'verify_sources.py')
            or sha(run/'verify_sources.py') != proof['wrapper_sha256']):
        raise ValueError('Original software-verifier source differs from its proof/archive')
    frozen = digest_map(before['frozen_audit_sources'], 'Original frozen audit source map')
    if frozen != after['frozen_audit_sources']:
        raise ValueError('Original frozen auditor declaration changed during execution')
    for name, digest in frozen.items():
        if source.get(name) != digest:
            raise ValueError('Frozen auditor lacks exact completed source-proof binding: '+name)
        if sha(relative_file(run/'frozen_audit_sources', name)) != digest:
            raise ValueError('Original frozen auditor bytes changed: '+name)
        if sha(relative_file(root, name)) != digest:
            raise ValueError('Current auditor bytes differ from exact frozen original: '+name)
    runtime_before = digest_map(before['runtime_file_sha256_before'], 'Original native runtime file map')
    if (runtime_before != after['runtime_file_sha256_before']
            or runtime_before != after['runtime_file_sha256_after']
            or runtime_before != proof['runtime_file_sha256_before']
            or runtime_before != proof['runtime_file_sha256_after']
            or after['runtime_files_unchanged'] is not True or proof['runtime_files_unchanged'] is not True
            or before['runtime'] != after['runtime'] or before['runtime'] != proof['runtime']):
        raise ValueError('Original BEFORE/AFTER/software native runtime identities disagree')
    libraries = before['runtime']['libraries']
    if {entry['path']:entry['sha256'] for entry in libraries} != runtime_before or len(libraries) != len(runtime_before):
        raise ValueError('Original mapped-library descriptor differs from runtime file hashes')
    runtime_paths = {} if runtime_paths is None else dict(runtime_paths)
    if set(runtime_paths)-set(runtime_before):
        raise ValueError('Runtime relocation supplied an undeclared original library')
    for name, digest in runtime_before.items():
        path = Path(runtime_paths.get(name, name))
        if not path.is_file() or sha(path) != digest:
            raise ValueError('Current native runtime file differs from original proof: '+name)
    # Original inherited launch fields must remain byte-equivalent values.
    for key, value in before.items():
        if after.get(key) != value:
            raise ValueError('Original AFTER identity altered inherited BEFORE field: '+key)
    exit_code = after['native_exit_code']
    if type(exit_code) is not int:
        raise ValueError('Original native exit code must be an integer, not a success flag')
    started, finished = datetime.fromisoformat(before['started_utc']), datetime.fromisoformat(after['finished_utc'])
    if started.tzinfo is None or finished.tzinfo is None or finished < started:
        raise ValueError('Original closure lacks ordered timezone-aware start/finish clocks')
    native_log = relative_file(run, 'native_stdout_stderr.log')
    if sha(native_log) != after['native_stdout_stderr_sha256']:
        raise ValueError('Original closed native child log differs from its recorded SHA')
    return {'kind':'Read-only closed supported-run producer/software/auditor identity',
        'verifier_source_sha256':sha(__file__), 'producer_commit':producer,
        'software_tests_passed':proof['tests_passed'], 'source_files_unchanged':len(source),
        'before_identity_sha256':before_sha, 'after_identity_sha256':after_sha,
        'software_proof_sha256':proof_sha, 'launch_manifest_sha256':sha(manifest_path),
        'software_child_log_sha256':sha(run/'software_tests.log'),
        'native_child_log_sha256':sha(native_log), 'closure_native_exit_code':exit_code,
        'original_started_utc':before['started_utc'], 'original_finished_utc':after['finished_utc'],
        'complete_current_and_archived_tested_sources_match_original':True,
        'frozen_audit_sources':frozen, 'runtime_file_sha256':runtime_before,
        'runtime_file_paths_checked':{name:str(Path(runtime_paths.get(name,name)).resolve()) for name in runtime_before},
        'original_repository_head_after':after.get('repository_head_after'),
        'scope':'Identity only. Repository HEAD may advance while exact tested current/archive bytes stay fixed. Original nonzero native exit/failure is retained. No trace/force-state decoding, model import, native compile, contact/force solve, dynamics integration, report alteration or physics-success inference. Runtime files are checked on disk against the original mapped-library descriptor; this verifier does not map a library.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--repository-root', type=Path, required=True)
    parser.add_argument('--proof-archive', type=Path, required=True)
    parser.add_argument('--expected-before-sha256', required=True)
    parser.add_argument('--expected-producer', required=True)
    parser.add_argument('--expected-proof-sha256', required=True)
    parser.add_argument('--expected-after-sha256')
    parser.add_argument('--runtime-paths-json', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify(args.run, args.repository_root, args.proof_archive,
        expected_before_sha256=args.expected_before_sha256, expected_producer=args.expected_producer,
        expected_proof_sha256=args.expected_proof_sha256, expected_after_sha256=args.expected_after_sha256,
        runtime_paths=strict_json(args.runtime_paths_json) if args.runtime_paths_json else None)
    value = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.output:
        if args.output.resolve().is_relative_to(args.run.resolve()):
            raise ValueError('Derivative identity report must stay outside immutable original run')
        with args.output.open('x') as stream:
            stream.write(value)
    print(value, end='')
