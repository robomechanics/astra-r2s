"""Pure byte gates for CLOSED fresh C2/inertia media and evidence publication.

No simulator imports. A missing original AFTER is rejected before any report,
trace, log or raw-force file is read. Closure is independent of physics success.
"""
from pathlib import Path
import hashlib
import json
import re

PRODUCER = 'da69a9cd44a8312cc7b97365faf5e09c27a646e2'
BEFORE_SHA = 'f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3'
PROOF_SHA = '71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for value in iter(lambda: stream.read(1024*1024), b''):
            digest.update(value)
    return digest.hexdigest()


def strict_json(path):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('Duplicate original JSON key: '+key)
            result[key] = value
        return result
    return json.loads(Path(path).read_text(), object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def relative_file(root, name):
    root, name = Path(root).resolve(), Path(name)
    if name.is_absolute() or '..' in name.parts or not name.parts:
        raise ValueError('Unsafe relative file: '+str(name))
    result = root/name
    if not result.resolve().is_relative_to(root) or not result.is_file():
        raise ValueError('Missing/escaping original file: '+str(name))
    return result


def verify_map(root, values, label, *, complete=False, except_names=()):
    if not isinstance(values, dict) or not values:
        raise ValueError('Missing '+label+' hash map')
    for name, digest in values.items():
        if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
            raise ValueError('Invalid '+label+' hash: '+str(name))
        if sha(relative_file(root, name)) != digest:
            raise ValueError('Changed '+label+' file: '+str(name))
    if complete:
        actual = {str(p.relative_to(root)) for p in Path(root).rglob('*') if p.is_file()}
        if actual != set(values) | set(except_names):
            raise ValueError('Incomplete '+label+' inventory')


def verify_closed(run, root, proof_archive, identity_path, identity_sha256,
                  audit_path, audit_sha256):
    run, root, archive = map(lambda x: Path(x).resolve(), (run, root, proof_archive))
    # FIRST: no mutable state/report/force/log inspection on a LIVE run.
    after_path = run/'run_publication_identity_after.json'
    if not after_path.is_file():
        raise ValueError('Missing original AFTER: refuse LIVE/nonclosed run')
    identity_path, audit_path = Path(identity_path).resolve(), Path(audit_path).resolve()
    if (not identity_sha256 or sha(identity_path) != identity_sha256
            or not audit_sha256 or sha(audit_path) != audit_sha256):
        raise ValueError('Closed derivative identity/audit SHA anchor differs')
    identity, audit = strict_json(identity_path), strict_json(audit_path)
    if audit.get('closed') is not True:
        raise ValueError('Frozen derivative binder must declare native closure')
    before_path, proof_path = run/'run_publication_identity.json', run/'software_tests.json'
    before, after, proof = map(strict_json, (before_path, after_path, proof_path))
    if sha(before_path) != BEFORE_SHA or sha(proof_path) != PROOF_SHA:
        raise ValueError('Original producer/software anchors differ')
    if before['producer_commit'] != PRODUCER or after['producer_commit'] != PRODUCER:
        raise ValueError('Original native producer differs')
    sources = before['source_hashes_before']
    if (len(sources) != 74 or sources != after['source_hashes_before']
            or sources != after['source_hashes_after'] or sources != proof['source_hashes']
            or after['source_hashes_unchanged'] is not True
            or proof['source_hashes_unchanged'] is not True or proof['passed'] is not True
            or proof['tests_passed'] != 672 or type(after['native_exit_code']) is not int):
        raise ValueError('Original source/software/exit declarations differ')
    for key, value in before.items():
        if after.get(key) != value:
            raise ValueError('Original AFTER changed inherited BEFORE field: '+key)
    expected_identity = {
        'producer_commit': PRODUCER, 'software_tests_passed': 672,
        'source_files_unchanged': 74, 'before_identity_sha256': sha(before_path),
        'after_identity_sha256': sha(after_path), 'software_proof_sha256': sha(proof_path),
        'launch_manifest_sha256': sha(run/'manifest.json'),
        'closure_native_exit_code': after['native_exit_code'],
        'complete_current_and_archived_tested_sources_match_original': True,
        'native_child_log_sha256': after['native_stdout_stderr_sha256'],
        'frozen_audit_sources': before['frozen_audit_sources'],
        'runtime_file_sha256': before['runtime_file_sha256_before']}
    for key, value in expected_identity.items():
        if identity.get(key) != value:
            raise ValueError('Derivative identity differs from original: '+key)
    verifier = root/'outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py'
    if sha(verifier) != identity['verifier_source_sha256']:
        raise ValueError('Root identity verifier source differs')
    verify_map(root, sources, 'current tested producer')
    verify_map(archive/'source_files', sources, 'archived tested producer')
    if sha(archive/'software_proof.json') != PROOF_SHA:
        raise ValueError('Archived672 proof differs from original')
    checksums = {}
    for line in (archive/'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        if name in checksums:
            raise ValueError('Duplicate archived proof checksum name')
        checksums[name] = digest
    verify_map(archive, checksums, 'complete software archive', complete=True,
        except_names=('SHA256SUMS',))
    runtime = before['runtime_file_sha256_before']
    if (runtime != after['runtime_file_sha256_before']
            or runtime != after['runtime_file_sha256_after']
            or runtime != proof['runtime_file_sha256_before']
            or runtime != proof['runtime_file_sha256_after']
            or after['runtime_files_unchanged'] is not True
            or proof['runtime_files_unchanged'] is not True
            or before['runtime'] != after['runtime'] or before['runtime'] != proof['runtime']):
        raise ValueError('Original native runtime before/after/proof differs')
    checked_paths = identity['runtime_file_paths_checked']
    if set(checked_paths) != set(runtime):
        raise ValueError('Derivative identity runtime-path coverage differs')
    for name, digest in runtime.items():
        if sha(checked_paths[name]) != digest:
            raise ValueError('Current native library differs from original')
    if len(before['frozen_audit_sources']) != 5:
        raise ValueError('Incomplete frozen five-auditor provenance')
    verify_map(run/'frozen_audit_sources', before['frozen_audit_sources'], 'frozen auditor')
    original = audit['original_run_files_sha256']
    audit_files = audit['audit_files_sha256']
    verify_map(run, original, 'complete closed native run', complete=True)
    verify_map(audit_path.parent, audit_files, 'complete frozen derivative audit',
        complete=True, except_names=(audit_path.name,))
    if sha(run/'native_stdout_stderr.log') != after['native_stdout_stderr_sha256']:
        raise ValueError('Original native child log differs')
    if 'producer_commit' in audit and audit['producer_commit'] != PRODUCER:
        raise ValueError('Derivative audit producer differs')
    if 'native_child_exit_code' in audit and audit['native_child_exit_code'] != after['native_exit_code']:
        raise ValueError('Derivative audit changed original exit')
    return dict(launch=before, after=after, proof=proof, identity=identity, audit=audit,
        sources=sources, original_run_files_sha256=original, audit_files_sha256=audit_files,
        identity_sha256=identity_sha256, audit_sha256=audit_sha256,
        root_identity_verifier_sha256=sha(verifier))
