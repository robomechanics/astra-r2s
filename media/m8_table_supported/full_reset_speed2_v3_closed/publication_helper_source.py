"""Losslessly publish one outcome-neutral CLOSED fresh reset-speed2 v3 native attempt.

Standard library only. The adjacent closure gate runs before any report, trace,
log, or raw-ledger inspection. Closure and software identity never imply native
physics success. Every original file and failed independent audit is preserved.
Use --raw-only with render argument '-' when media is absent. The frozen gate
still requires original final report/trace and completed serial official readers;
missing final evidence is refused, never replaced by partial trajectory bytes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import types
import uuid


CHUNK_BYTES = 45_000_000
GATE_SHA256 = '921055be53910e9293daa15881efc16e54e096e6943ff3a48e34c98e253b05ea'
RENDERER_SHA256 = 'ab16fd64ba4b14a13abfed9b412fea8c44e66e790773fecd066b6590095888b8'
MEDIA_VERIFIER_SHA256 = '8d58268df1ca851f9bda89f0899349bc0f33cce0c1ec0cf4bd74bea8789dbd4e'
IDENTITY_VERIFIER_SHA256 = '7ceed9d55285b4be3c804db9f5443efdb2d2786adcdd100b9faf114050400f3a'
SERIAL_AUDIT_SHA256 = 'e532c407cd35b51ad3a55bcbbdf98fc30676e8e994f427895c6a813875081e2c'
WRITER_SHA256 = '8678d079a8e5bb756776cf6ea365559c5407e121f75bf816865b046bc28c41bf'
REASSEMBLER_SHA256 = '0706d299abfc70a0883aa8227ea3d89c7da420e961b4fc542fd696f3b05a3cfb'
RESTORER_SHA256 = '2a64428a4c1f3a96c9abce16f708576ac310d6e426eac54e38441097ac4cdf8b'
FINAL_FILES = ('insertion_validation.json', 'insertion_trace.npz',
    'native_feedback_force_history.npz', 'table_support_force_history.npz',
    'left_pad_force_history.npz', 'robot_inertia_command_history.npz')


def identity(path):
    digest, count = hashlib.sha256(), 0
    with Path(path).open('rb') as stream:
        for raw in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(raw)
            count += len(raw)
    return {'bytes': count, 'sha256': digest.hexdigest()}


def sha(path):
    return identity(path)['sha256']


def strict_json(path):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('Duplicate JSON field: ' + key)
            result[key] = value
        return result

    def nonfinite(value):
        raise ValueError('Nonfinite JSON value: ' + value)

    return json.loads(Path(path).read_text(), object_pairs_hook=pairs,
        parse_constant=nonfinite)


def relative_name(value):
    if (not isinstance(value, str) or not value or '\\' in value
            or any(token in value for token in ('\n', '\r', '\x00'))):
        raise ValueError('Invalid relative artifact name: ' + repr(value))
    path = Path(value)
    if (path.is_absolute() or not path.parts or '..' in path.parts
            or path.as_posix() != value):
        raise ValueError('Unsafe relative artifact name: ' + value)
    return path


def tree_files(folder):
    folder = Path(folder).resolve()
    if not folder.is_dir():
        raise ValueError('Artifact directory is absent: ' + str(folder))
    result = {}
    for path in sorted(folder.rglob('*')):
        if path.is_symlink():
            raise ValueError('Refusing a symlink in immutable artifacts: ' + str(path))
        if path.is_dir():
            continue
        if not path.is_file() or not path.resolve().is_relative_to(folder):
            raise ValueError('Nonregular/escaped immutable artifact: ' + str(path))
        name = path.relative_to(folder).as_posix()
        relative_name(name)
        result[name] = path
    if not result:
        raise ValueError('Artifact directory has no files: ' + str(folder))
    return result


def verify_inventory(files, expected, label, *, extra_names=()):
    if set(files) != set(expected) | set(extra_names):
        raise ValueError('Complete ' + label + ' inventory differs')
    for name, digest in expected.items():
        relative_name(name)
        if (not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None
                or sha(files[name]) != digest):
            raise ValueError('Frozen ' + label + ' SHA differs: ' + name)


def source_module(path, name, expected_sha256=None):
    """Execute exact trusted helper source; never call its native reader helpers."""
    path = Path(path).resolve()
    source = path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError('Frozen publication dependency differs: ' + str(path))
    module = types.ModuleType(name)
    module.__file__ = str(path)
    exec(compile(source, str(path), 'exec'), module.__dict__)
    module.__verified_source_sha256__ = digest
    if sha(path) != digest:
        raise ValueError('Publication helper changed while loading: ' + str(path))
    return module


def rooted(root, path):
    path = Path(path)
    return (path if path.is_absolute() else root / path).resolve()


def validate_render(render, closed, identity_sha256, audit_sha256, report):
    manifest_path = render / 'render_manifest.json'
    manifest = strict_json(manifest_path)
    expected = {
        'mode': 'closed_native', 'native_closed_result': True,
        'identity_report_sha256': identity_sha256,
        'independent_audit_binding_sha256': audit_sha256,
        'original_after_identity_sha256': closed['identity']['after_identity_sha256'],
        'trace_filename': 'insertion_trace.npz',
        'trajectory_sha256': closed['original_run_files_sha256'].get('insertion_trace.npz'),
        'original_report_sha256': closed['original_run_files_sha256'].get('insertion_validation.json'),
        'source74_before_after_render': closed['sources'], 'source74_unchanged': True,
        'original_runtime': closed['launch']['runtime'],
        'recorded_runtime_content_matches_before_after': True,
        'original_run_files_sha256_before_after': closed['original_run_files_sha256'],
        'closed_media_gate_source_sha256': GATE_SHA256,
        'generic_identity_verifier_source_sha256': IDENTITY_VERIFIER_SHA256,
        'official_reader_execution_source_sha256': SERIAL_AUDIT_SHA256,
        'reset_speed_rad_s': 2.,
    }
    if manifest.get('runtime_before') != manifest.get('runtime_after'):
        raise ValueError('Actual replay runtime changed during rendering')
    if report is None or expected['trajectory_sha256'] is None:
        raise ValueError('Final report/trace is absent; frozen closed gate cannot be bypassed')
    same_json = closed_same_json
    for key, value in expected.items():
        if not same_json(manifest.get(key), value):
            raise ValueError('Closed render binding differs: ' + key)
    for key in ('physics_integration', 'mj_forward_called', 'collision_discovery_called',
            'force_solve_called', 'state_interpolation', 'body_hiding_or_geometry_change'):
        if manifest.get(key) is not False:
            raise ValueError('Closed media must use only saved-state geometry: ' + key)
    for key in ('passed', 'partial', 'aborted'):
        if not same_json(manifest.get('original_' + key), report.get(key)):
            raise ValueError('Closed media changed an original report field: ' + key)
    if type(manifest.get('original_native_exit_code')) is not int or manifest['original_native_exit_code'] != closed['after']['native_exit_code']:
        raise ValueError('Closed media changed the original integer native exit')
    for manifest_key, report_key in (('original_acceptance_checks','acceptance_checks'),
            ('original_observed_phase_summaries','phases'),('original_physical_motion_events','physical_motion_events'),
            ('original_grasp_acquisitions','right_grasp_acquisitions')):
        if not same_json(manifest.get(manifest_key), report.get(report_key)):
            raise ValueError('Closed media changed original observed facts: ' + manifest_key)
    for key, original_key in (('independent_official_passed','passed'),
            ('independent_official_acceptance_checks','independent_acceptance_checks')):
        if not same_json(manifest.get(key), closed['official_reports']['supported'].get(original_key)):
            raise ValueError('Closed media changed original independent acceptance: ' + key)
    review_path = render/'media_identity_review.json'
    review = strict_json(review_path)
    for key, value in (('passed',True), ('render_manifest_sha256',sha(manifest_path)),
            ('trajectory_sha256',expected['trajectory_sha256']),
            ('identity_report_sha256',identity_sha256),
            ('independent_audit_binding_sha256',audit_sha256),
            ('original_native_exit_code',closed['after']['native_exit_code'])):
        if not same_json(review.get(key), value):
            raise ValueError('Saved-state/media encoding review differs: '+key)
    for key in ('passed','partial','aborted'):
        if not same_json(review.get('original_native_'+key), report.get(key)):
            raise ValueError('Saved-state/media review changed original flag: '+key)
    renderer_source = render/'renderer_sources/render_full_reset_speed2_v3_closed.py'
    verifier_source = render/'renderer_sources/verify_full_reset_speed2_v3_closed_media.py'
    gate_source = render/'renderer_sources/full_reset_speed2_v3_closed_bindings.py'
    identity_source = render/'renderer_sources/verify_closed_supported_run_identity_v2.py'
    if (manifest['helper_sha256'] != sha(renderer_source) or sha(renderer_source) != RENDERER_SHA256
            or review['verification_source_sha256'] != sha(verifier_source) or sha(verifier_source) != MEDIA_VERIFIER_SHA256
            or sha(gate_source) != GATE_SHA256 or sha(identity_source) != IDENTITY_VERIFIER_SHA256):
        raise ValueError('Media helper source archives differ from original execution')
    media = manifest.get('media_sha256')
    if not isinstance(media, dict) or not media:
        raise ValueError('Closed renderer has no hash-bound media')
    for name, digest in media.items():
        path = render / relative_name(name)
        if not path.resolve().is_relative_to(render) or sha(path) != digest:
            raise ValueError('Closed rendered-media SHA differs: ' + name)
    return manifest


def closed_same_json(left, right):
    # Preserve original JSON values and types; True/1 and False/0 are distinct.
    return json.dumps(left, sort_keys=True, separators=(',', ':'), allow_nan=False) == json.dumps(right, sort_keys=True, separators=(',', ':'), allow_nan=False)


def main(args):
    root = args.repository_root.resolve()
    run, target = rooted(root, args.run), rooted(root, args.output)
    proof_archive = rooted(root, args.proof_archive)
    identity_path = rooted(root, args.identity)
    audit_path = rooted(root, args.audit_binding)
    readme = rooted(root, args.readme)
    render = None if args.raw_only else rooted(root, args.render)
    side_artifacts = rooted(root,args.side_artifacts) if args.side_artifacts else None
    base = root / 'outputs/m8_table_supported'
    gate_path = base / 'full_reset_speed2_v3_closed_bindings.py'

    # FIRST native-artifact operation refuses absent original AFTER, even when
    # an optional runtime-path map is invalid or points into mutable evidence.
    if not (run/'run_publication_identity_after.json').is_file():
        raise ValueError('Missing original AFTER: refuse LIVE before optional map/data reads')
    runtime_paths_path = rooted(root, args.runtime_paths_json) if args.runtime_paths_json else None
    runtime_paths = strict_json(runtime_paths_path) if runtime_paths_path is not None else None
    gate = source_module(gate_path, 'closed_reset_speed2_v3_publication_gate', GATE_SHA256)
    closed = gate.verify_closed(run, root, proof_archive,
        identity_path, args.identity_sha256, audit_path, args.audit_binding_sha256, runtime_paths=runtime_paths)
    if sha(gate_path) != gate.__verified_source_sha256__:
        raise ValueError('Closure gate source changed during verification')
    if target.exists():
        raise FileExistsError('Refusing existing publication target: ' + str(target))
    immutable_roots = [run, proof_archive, audit_path.parent]
    if side_artifacts is not None:
        immutable_roots.append(side_artifacts)
    if render is not None:
        immutable_roots.append(render)
    for folder in immutable_roots:
        if target.is_relative_to(folder) or folder.is_relative_to(target):
            raise ValueError('Publication target overlaps immutable inputs: ' + str(folder))
    if readme.is_symlink() or not readme.is_file():
        raise ValueError('A regular standalone README source is required')

    writer_path = base / 'publish_supported_feedback_run.py'
    reassembler_path = base / 'reassemble_archives.py'
    restorer_path = base / 'restore_original_run_layout.py'
    identity_verifier = base / 'verify_closed_supported_run_identity_v2.py'
    workflow = base / 'post_closure_audit_workflow_full_reset_speed2_v3.md'
    writer = source_module(writer_path, 'frozen_closed_reset_speed2_lossless_writer', WRITER_SHA256)
    for path, expected in ((reassembler_path, REASSEMBLER_SHA256),
            (restorer_path, RESTORER_SHA256),
            (identity_verifier, closed['identity']['verifier_source_sha256'])):
        if sha(path) != expected:
            raise ValueError('Frozen publication/identity source differs: ' + str(path))

    original_files = tree_files(run)
    audit_files = tree_files(audit_path.parent)
    proof_files = tree_files(proof_archive)
    side_files = tree_files(side_artifacts) if side_artifacts is not None else {}
    verify_inventory(original_files, closed['original_run_files_sha256'], 'original native run')
    verify_inventory(audit_files, closed['audit_files_sha256'], 'independent audit',
        extra_names=(audit_path.name,))
    original_identities = {name: identity(path) for name, path in original_files.items()}
    report_path = original_files.get('insertion_validation.json')
    report = strict_json(report_path) if report_path is not None else None
    if report is not None and not isinstance(report, dict):
        raise ValueError('Original validation report is not a JSON object')
    if not closed_same_json(report, closed['original_native_report']):
        raise ValueError('Original report differs from anchored official closed gate')
    render_manifest = None
    render_files = {}
    if render is not None:
        render_manifest = validate_render(render, closed, args.identity_sha256,
            args.audit_binding_sha256, report)
        render_files = tree_files(render)

    sources = {}

    def add(name, path):
        relative_name(name)
        path = Path(path).resolve()
        if name in sources:
            raise ValueError('Publication artifact name collision: ' + name)
        if not path.is_file():
            raise ValueError('Publication artifact is absent: ' + str(path))
        sources[name] = path

    for prefix, files in (('native_full', original_files),
            ('independent_audits', audit_files), ('software_proof', proof_files),
            ('render', render_files), ('execution_provenance/media_execution',side_files)):
        for name, path in files.items():
            add(prefix + '/' + name, path)
    add('execution_provenance/root_identity_report.json', identity_path)
    if runtime_paths_path is not None:
        add('execution_provenance/replay_runtime_paths.json', runtime_paths_path)
    add('execution_provenance/post_closure_audit_workflow_full_reset_speed2_v3.md', workflow)
    add('execution_provenance/verify_closed_supported_run_identity_v2.py', identity_verifier)
    add('publication_dependencies/full_reset_speed2_v3_closed_bindings.py', gate_path)
    add('publication_dependencies/publish_supported_feedback_run.py', writer_path)
    add('publication_dependencies/reassemble_archives.py', reassembler_path)
    add('publication_dependencies/restore_original_run_layout.py', restorer_path)
    add('publication_dependencies/' + Path(__file__).name, Path(__file__))
    source_identities = {name: identity(path) for name, path in sources.items()}
    standalone = {'README.md': readme, 'reassemble_archives.py': reassembler_path,
        'restore_original_run_layout.py': restorer_path,
        'publication_helper_source.py': Path(__file__).resolve()}
    standalone_identities = {name: identity(path) for name, path in standalone.items()}
    for name, record in standalone_identities.items():
        if record['bytes'] > CHUNK_BYTES:
            raise ValueError('Standalone helper/README exceeds storage limit: ' + name)

    target.parent.mkdir(parents=True, exist_ok=True)
    stage = target.parent / (target.name + '.packaging-' + uuid.uuid4().hex)
    stage.mkdir()
    try:
        artifacts = {name: writer.write_artifact(path, stage, name, CHUNK_BYTES)
            for name, path in sources.items()}
        for name, path in sources.items():
            if identity(path) != source_identities[name]:
                raise ValueError('Immutable source changed while packaging: ' + name)
        for name, path in standalone.items():
            shutil.copyfile(path, stage / name)
            if identity(stage / name) != standalone_identities[name] or identity(path) != standalone_identities[name]:
                raise ValueError('Standalone source changed while packaging: ' + name)
        # Recheck source/runtime/closure/frozen inventories after all copying.
        repeated = gate.verify_closed(run, root, proof_archive,
            identity_path, args.identity_sha256, audit_path, args.audit_binding_sha256, runtime_paths=runtime_paths)
        if (sha(gate_path) != gate.__verified_source_sha256__ or repeated != closed
                or tree_files(run) != original_files or tree_files(audit_path.parent) != audit_files):
            raise ValueError('Closed inputs changed during publication')
        if side_artifacts is not None and tree_files(side_artifacts) != side_files:
            raise ValueError('Closed media-execution provenance inventory changed during publication')
        if tree_files(proof_archive) != proof_files:
            raise ValueError('Complete proof archive inventory changed during publication')
        if render is not None and tree_files(render) != render_files:
            raise ValueError('Closed renderer inventory changed during publication')

        manifest = {
            'schema': 'fresh-full-reset-speed2-v3-closed-lossless-packet-v1',
            'status': 'closed_raw_evidence_only' if args.raw_only else 'closed_fresh_native_evidence',
            'raw_only': args.raw_only, 'full_native_run_closed': True,
            'raw_only_scope': 'Media absent only; complete final original report/trace and intact serial official readers remain mandatory',
            'media_absent_mode_does_not_bypass_complete_serial_readers': True,
            'gate_requires_complete_final_trace_report_and_official_readers': True,
            'original_physics_outcome_not_inferred': True,
            'explicit_replay_runtime_paths': runtime_paths,
            'current_runtime_file_paths_checked': closed['current_runtime_file_paths_checked'],
            'executed_publication_helper_filename':Path(__file__).name,
            'additional_closed_media_execution_files_sha256':{name:sha(path) for name,path in side_files.items()},
            'producer_commit': closed['launch']['producer_commit'],
            'original_native_exit_code': closed['after']['native_exit_code'],
            'original_native_passed': report.get('passed') if report is not None else None,
            'original_native_partial': report.get('partial') if report is not None else None,
            'original_aborted': report.get('aborted') if report is not None else None,
            'original_report': report,
            'original_final_artifact_presence': {name: name in original_files for name in FINAL_FILES},
            'absent_expected_final_artifacts': [name for name in FINAL_FILES if name not in original_files],
            'partial_trace_is_not_promoted_or_spliced': True,
            'original_run_file_identities': original_identities,
            'original_run_file_to_published_artifact_names': {
                name: ['native_full/' + name] for name in original_files},
            'original_run_files_sha256': closed['original_run_files_sha256'],
            'audit_files_sha256': closed['audit_files_sha256'],
            'original_launch_identity': closed['launch'], 'original_after_identity': closed['after'],
            'original_software_proof': closed['proof'], 'original_source74': closed['sources'],
            'original_software_tests_passed': closed['proof']['tests_passed'],
            'software_proof_archive_sha256_ledger': sha(proof_archive / 'SHA256SUMS'),
            'identity_report_sha256': args.identity_sha256, 'root_identity_report': closed['identity'],
            'identity_verifier_source_sha256': sha(identity_verifier),
            'official_serial_audit_source_sha256': closed['official_serial_source_sha256'],
            'original_official_reports': closed['official_reports'],
            'reset_speed_rad_s': 2.,
            'independent_audit_binding_sha256': args.audit_binding_sha256,
            'independent_audit_binding': closed['audit'],
            'independent_audit_binding_artifact': 'independent_audits/' + audit_path.name,
            'closed_gate_source_sha256': sha(gate_path), 'workflow_source_sha256': sha(workflow),
            'render_manifest_sha256': sha(render / 'render_manifest.json') if render is not None else None,
            'render_manifest': render_manifest,
            'original_reports_acceptance_flags_and_audit_failures_unmodified': True,
            'whole_software_proof_is_separate_from_native_qualification': True,
            'identity_and_exit_zero_do_not_imply_physics_success': True,
            'archives_are_lossless': True, 'dropping_or_quantization': False,
            'maximum_stored_file_bytes': CHUNK_BYTES,
            'publication_source_file_identities': source_identities,
            'standalone_source_file_identities': standalone_identities,
            'scope': 'One original fresh native attempt, closed with its actual integer exit code. '
                'Complete original run, 74-source software proof archive, frozen five-auditor snapshots, '
                'all derivative independent audit results/logs, identity/workflow sources and publisher '
                'dependencies are byte-preserved. Failed or unexecuted original acceptance gates remain '
                'unchanged. Media, when present, refreshes exact final saved-state geometry only. '
                'No integration, contact/force reconstruction, cold checkpoint splice, rewritten '
                'report, implicit full threading/capture/reset/trajectory or policy qualification.',
            'artifacts': artifacts,
        }
        (stage / 'package_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
        files = tree_files(stage)
        if max(path.stat().st_size for path in files.values()) > CHUNK_BYTES:
            raise ValueError('Stored packet file exceeds the 45 MB limit')
        ledger = stage / 'SHA256SUMS'
        ledger.write_text(''.join(sha(path) + '  ' + name + '\n' for name, path in files.items()))
        if ledger.stat().st_size > CHUNK_BYTES:
            raise ValueError('Packet checksum ledger exceeds the 45 MB limit')
        if target.exists():
            raise FileExistsError('Publication target appeared during packaging: ' + str(target))
        stage.rename(target)
        stored = tree_files(target)
        print(json.dumps({'target': str(target), 'artifacts': len(artifacts),
            'original_files_preserved': len(original_files), 'audit_files_preserved': len(audit_files),
            'software_proof_files_preserved': len(proof_files), 'files': len(stored),
            'checksum_entries': len(files), 'total_bytes': sum(path.stat().st_size for path in stored.values()),
            'maximum_stored_bytes': max(path.stat().st_size for path in stored.values()),
            'ledger_sha256': sha(target / 'SHA256SUMS'), 'raw_only': args.raw_only,
            'original_native_exit_code': closed['after']['native_exit_code']}, indent=2))
    except Exception:
        if stage.exists():
            shutil.rmtree(stage)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository-root', type=Path, required=True)
    parser.add_argument('--identity', type=Path, required=True)
    parser.add_argument('--identity-sha256', required=True)
    parser.add_argument('--audit-binding', type=Path, required=True)
    parser.add_argument('--audit-binding-sha256', required=True)
    parser.add_argument('--runtime-paths-json', type=Path)
    parser.add_argument('--proof-archive', type=Path, required=True)
    parser.add_argument('--readme', type=Path, required=True)
    parser.add_argument('--side-artifacts',type=Path,help='Frozen media-execution provenance outside original native/audit trees')
    parser.add_argument('--raw-only', action='store_true',
        help="Media absent only; final trace/report and complete serial readers still required; use '-' for render")
    parser.add_argument('run', type=Path)
    parser.add_argument('render', type=Path)
    parser.add_argument('output', type=Path)
    main(parser.parse_args())
