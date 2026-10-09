"""Publish new-feedback closed native supported evidence without dropping bytes.

This output-only helper never changes physics/controller/auditor sources. Large
files use fixed-size binary chunks, each <=45 MB, plus whole-file SHA256 hashes.
"""
from __future__ import annotations

import argparse
import ast
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import uuid
import zipfile

ROOT = Path("/workspace/astra-r2s")
DEFAULT_CHUNK_BYTES = 45_000_000
ORIGINAL_SUPPORTED_AUDITOR_SHA256 = 'e46808601d7974ffdea1e5e20e07e0f28eb8e0a74a394545b276621a7e856702'
INITIAL_BOUNDARY_CORRECTED_AUDITOR_SHA256 = 'abb4e724de91eeec9dcff9c481d1a89674fba5ea3e3c5c2d9a9b76a074f9cf1e'


def file_identity(path):
    h = hashlib.sha256()
    size = 0
    with Path(path).open("rb") as stream:
        for raw in iter(lambda: stream.read(1024 * 1024), b""):
            size += len(raw)
            h.update(raw)
    return {"bytes": size, "sha256": h.hexdigest()}


def strict_json(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda token:
        (_ for _ in ()).throw(ValueError(f"Nonfinite JSON constant: {token}")))


def add_source(sources, name, path):
    """Never replace an original artifact with a same-named derivative."""
    relative = Path(name)
    if relative.is_absolute() or not relative.parts or '..' in relative.parts:
        raise ValueError(f"Unsafe artifact name: {name}")
    name, path = str(relative), Path(path).resolve()
    if name in sources and Path(sources[name]).resolve() != path:
        raise ValueError(f"Artifact name collision: {name}")
    sources[name] = path


def closed_identity(run):
    """Require an actual producer exit record, without changing its outcome."""
    before_path = run / 'run_publication_identity.json'
    after_path = run / 'run_publication_identity_after.json'
    if not before_path.is_file() or not after_path.is_file():
        raise ValueError('Refusing LIVE/nonclosed evidence: native beginning/end identity is absent')
    before, after = strict_json(before_path), strict_json(after_path)
    producer = before.get('producer_commit')
    if (not isinstance(producer, str) or not producer
            or after.get('producer_commit') != producer
            or type(after.get('native_exit_code')) is not int):
        raise ValueError('Native closure identity has no matching producer/integer exit code')
    if (not isinstance(before.get('source_hashes_before'), dict)
            or not isinstance(after.get('source_hashes_after'), dict)
            or before['source_hashes_before'] != after.get('source_hashes_before')):
        raise ValueError('Native closure beginning source declaration differs from original beginning identity')
    return before, after


def write_artifact(source, destination, name, chunk_bytes):
    """Stream exact bytes to either one file or contiguous fixed-size chunks."""
    source = Path(source)
    size = source.stat().st_size
    original_stat = source.stat()
    whole = hashlib.sha256()
    offset = 0
    chunks = []
    with source.open("rb") as reader:
        if size <= chunk_bytes:
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open("xb") as writer:
                for raw in iter(lambda: reader.read(1024 * 1024), b""):
                    whole.update(raw)
                    writer.write(raw)
                    offset += len(raw)
            artifact = {"storage": "single_file", "path": name,
                        "bytes": offset, "sha256": whole.hexdigest()}
        else:
            while offset < size:
                index = len(chunks)
                chunk_name = f"binary_chunks/{name}.part{index:05d}"
                target = destination / chunk_name
                target.parent.mkdir(parents=True, exist_ok=True)
                chunk_hash = hashlib.sha256()
                count = 0
                start = offset
                with target.open("xb") as writer:
                    while count < chunk_bytes:
                        raw = reader.read(min(1024 * 1024, chunk_bytes-count))
                        if not raw:
                            break
                        writer.write(raw)
                        whole.update(raw)
                        chunk_hash.update(raw)
                        count += len(raw)
                if not count:
                    raise ValueError(f"Source truncated during chunking: {source}")
                offset += count
                chunks.append({"path": chunk_name, "index": index, "offset": start,
                    "bytes": count, "sha256": chunk_hash.hexdigest()})
            artifact = {"storage": "lossless_binary_chunks", "bytes": offset,
                "sha256": whole.hexdigest(), "chunk_bytes": chunk_bytes, "chunks": chunks,
                "reassembly": "Concatenate exact chunks in listed order; verify all chunk and whole-file SHA256 hashes"}
        if reader.read(1):
            raise ValueError(f"Source grew while copying: {source}")
    after = source.stat()
    if offset != size or (original_stat.st_ino, original_stat.st_size, original_stat.st_mtime_ns) != (
            after.st_ino, after.st_size, after.st_mtime_ns):
        raise ValueError(f"Source changed while copying: {source}")
    return artifact


def closed_report(run):
    import numpy as np
    report = strict_json(run / "insertion_validation.json")
    with np.load(run / "insertion_trace.npz", allow_pickle=False) as saved:
        metadata = json.loads(str(saved["metadata_json"]), parse_constant=lambda token:
            (_ for _ in ()).throw(ValueError(token)))
        times = saved["time"].copy()
        qpos, qvel = saved["qpos"].copy(), saved["qvel"].copy()
        rows = json.loads(str(saved["info_json"]))
    if metadata != report or not len(times) or len(rows) != len(times):
        raise ValueError("Final exact trace metadata/states do not match the closed report")
    if not np.isfinite(times).all() or not np.isfinite(qpos).all() or not np.isfinite(qvel).all():
        raise ValueError("Native trace contains nonfinite states")
    return report, times, qpos, qvel, rows


def compatibility_artifact(sidecar_path, declaration, key):
    record = declaration[key]
    relative = Path(record['path'])
    if relative.is_absolute() or not relative.parts or '..' in relative.parts:
        raise ValueError('Unsafe compatibility artifact path: '+key)
    path = (sidecar_path.parent/relative).resolve()
    if not path.is_relative_to(sidecar_path.parent) or not path.is_file():
        raise ValueError('Missing or escaped compatibility artifact: '+key)
    if file_identity(path)['sha256'] != record['sha256']:
        raise ValueError('Compatibility artifact SHA differs: '+key)
    return path


def validate_auditor_compatibility(sidecar_path, run, audit_path, audit, report, closure):
    """Bind only the reviewed output-only initial-grasp reader correction."""
    sidecar_path, run, audit_path = map(lambda p:Path(p).resolve(),
                                      (sidecar_path, run, audit_path))
    declaration = strict_json(sidecar_path)
    if (declaration.get('schema') != 'supported-auditor-compatibility-v1'
            or declaration.get('correction_kind') != 'initial-settle-bolt-same-row-grasp-flag'
            or declaration.get('canonical_source_or_original_report_modified') is not False):
        raise ValueError('Unknown or unsafe auditor compatibility declaration')
    keys = ('original_auditor', 'corrected_auditor', 'corrected_report',
            'original_exception', 'narrow_diff', 'boundary_diagnosis',
            'regression_proof', 'test_source')
    paths = {key:compatibility_artifact(sidecar_path,declaration,key) for key in keys}
    frozen = run/'frozen_audit_sources/scripts/audit_m8_supported_trace.py'
    original_sha, corrected_sha = (file_identity(paths[key])['sha256']
                                  for key in ('original_auditor','corrected_auditor'))
    if (original_sha != ORIGINAL_SUPPORTED_AUDITOR_SHA256
            or original_sha != file_identity(frozen)['sha256']
            or declaration.get('original_auditor_sha256') != original_sha
            or corrected_sha != INITIAL_BOUNDARY_CORRECTED_AUDITOR_SHA256
            or declaration.get('corrected_auditor_sha256') != corrected_sha):
        raise ValueError('Compatibility is not the reviewed original/initial-boundary reader pair')
    old, new = (paths[key].read_text() for key in ('original_auditor','corrected_auditor'))
    trees = [ast.parse(source) for source in (old,new)]
    removed = []
    for tree in trees:
        functions = [node for node in tree.body if isinstance(node,ast.FunctionDef)
                     and node.name == 'original_grasp_flag_audit']
        if len(functions) != 1:
            raise ValueError('Compatibility source has no unique original grasp-boundary function')
        removed.append(ast.dump(functions[0],include_attributes=False))
        tree.body.remove(functions[0])
    if (removed[0] == removed[1] or ast.dump(trees[0],include_attributes=False)
            != ast.dump(trees[1],include_attributes=False)):
        raise ValueError('Compatibility modifies code outside the reviewed acquisition boundary')
    actual_diff = paths['narrow_diff'].read_text().splitlines(keepends=True)
    expected_diff = list(difflib.unified_diff(old.splitlines(keepends=True),
        new.splitlines(keepends=True),fromfile='original',tofile='corrected'))
    if (len(actual_diff)<3 or not actual_diff[0].startswith('--- ')
            or not actual_diff[1].startswith('+++ ') or actual_diff[2:] != expected_diff[2:]):
        raise ValueError('Compatibility narrow diff does not reproduce the actual reader change')
    if 'Acquisition row incorrectly claims the new grasp was already guarded' not in paths['original_exception'].read_text():
        raise ValueError('Original frozen-reader acquisition exception is not preserved')
    beginning = strict_json(run/'run_publication_identity.json')
    if (declaration.get('original_producer_commit') != beginning['producer_commit']
            or declaration.get('original_trace_sha256') != file_identity(run/'insertion_trace.npz')['sha256']
            or type(declaration.get('original_native_exit_code')) is not int
            or type(closure.get('native_exit_code')) is not int
            or declaration.get('original_native_exit_code') != closure['native_exit_code']
            or declaration.get('original_report_passed') is not report['passed']
            or paths['corrected_report'] != audit_path
            or strict_json(paths['corrected_report']) != audit
            or audit.get('auditor_source_sha256') != corrected_sha):
        raise ValueError('Compatibility reader/report/native trial bindings differ')
    if (audit.get('original_report_passed') is not report['passed']
            or json.dumps(audit.get('original_acceptance_checks'),sort_keys=True,allow_nan=False)
            != json.dumps(report['acceptance_checks'],sort_keys=True,allow_nan=False)):
        raise ValueError('Compatibility derivative relabels original acceptance results')
    proof = strict_json(paths['regression_proof'])
    if (proof.get('passed') is not True or type(proof.get('tests_passed')) is not int
            or proof['tests_passed'] <= 0 or proof.get('sources_unchanged') is not True
            or proof.get('corrected_auditor_sha256') != corrected_sha
            or proof.get('test_source_sha256') != file_identity(paths['test_source'])['sha256']):
        raise ValueError('Compatibility correction has no exact source-bound passing regression proof')
    summary = {'schema':declaration['schema'],'correction_kind':declaration['correction_kind'],
        'sidecar_sha256':file_identity(sidecar_path)['sha256'],
        'original_frozen_auditor_sha256':original_sha,
        'corrected_output_only_reader_sha256':corrected_sha,
        'corrected_report_sha256':file_identity(audit_path)['sha256'],
        'original_exception_sha256':file_identity(paths['original_exception'])['sha256'],
        'correction_scope':declaration.get('correction_scope'),
        'regression_tests_passed':proof['tests_passed'],
        'original_source_report_acceptance_and_proof_preserved':True,
        'scope':'Explicit output-only reader compatibility; original frozen reader exception/source and native402/65 proof remain preserved. No native physical acceptance gate is waived.'}
    paths['sidecar'] = sidecar_path
    return summary, paths


def compatibility_publication_sources(sidecar_path, paths):
    """Keep the exact relative companion paths referenced by the sidecar."""
    sidecar_path = Path(sidecar_path).resolve()
    declaration = strict_json(sidecar_path)
    sources = {}
    for key,path in paths.items():
        relative = sidecar_path.name if key == 'sidecar' else declaration[key]['path']
        add_source(sources,'auditor_compatibility/'+relative,path)
    return sources


def package(run, render, target, *, evidence_only=False, require_complete=False,
            chunk_bytes=DEFAULT_CHUNK_BYTES, audit_dir=None, provenance=(),
            auditor_compatibility=None):
    run, render, target = map(lambda p: Path(p).resolve(), (run, render, target))
    if target.is_relative_to(run) or target.is_relative_to(render):
        raise ValueError('Publication target must be outside original run and render evidence')
    if target.exists():
        raise FileExistsError(target)
    if not 1 <= chunk_bytes <= DEFAULT_CHUNK_BYTES:
        raise ValueError("Chunk size must be 1..45,000,000 bytes")
    beginning, closure = closed_identity(run)
    if (closure.get('source_hashes_unchanged') is not True
            or beginning.get('source_hashes_before') != closure.get('source_hashes_before')
            or beginning.get('source_hashes_before') != closure.get('source_hashes_after')):
        raise ValueError('Final audited publication requires source-stable original closure; preserve exceptions with --raw-evidence-bundle')
    report, times, qpos, qvel, rows = closed_report(run)
    if not report["passed"] and not evidence_only:
        raise ValueError("Unsuccessful reports require explicit --evidence-only")
    turns = [p for p in report["phases"] if p["phase"].startswith("turn_")]
    complete = (not report["partial"] and report["aborted"] is None and
                len(turns) == report["control_config"]["qualifying_turns"] and
                all(p["started_engaged"] for p in turns))
    if require_complete and not complete:
        raise ValueError("Canonical complete-sequence publication requires actual unaborted qualified phases")
    rendered = strict_json(render / "render_manifest.json")
    trace_identity = file_identity(run / "insertion_trace.npz")
    if rendered["trajectory_sha256"] != trace_identity["sha256"]:
        raise ValueError("Media belongs to a different native trace")
    if rendered["slow_motion"] != 1:
        raise ValueError("Final supported demo requires normal 1x playback")
    still = rendered["screenshot_sample_index"]
    for key, value in (("screenshot_qpos_sha256", qpos[still].tobytes()),
                       ("screenshot_qvel_sha256", qvel[still].tobytes())):
        if rendered[key] != hashlib.sha256(value).hexdigest():
            raise ValueError("Screenshot native state identity differs")
    if rendered["original_saved_sample"] != rows[still]:
        raise ValueError("Screenshot original solved diagnostic differs")
    audit_directory = run if audit_dir is None else Path(audit_dir).resolve()
    audit_path = audit_directory / "independent_supported_audit.json"
    if auditor_compatibility is not None:
        sidecar_path = Path(auditor_compatibility).resolve()
        audit_path = compatibility_artifact(sidecar_path,strict_json(sidecar_path),'corrected_report')
    audit = strict_json(audit_path)
    if audit["trajectory_sha256"] != trace_identity["sha256"]:
        raise ValueError("Independent audit belongs to another native trace")
    frozen_auditor = run / "frozen_audit_sources/scripts/audit_m8_supported_trace.py"
    compatibility, compatibility_paths = None, {}
    if auditor_compatibility is not None:
        compatibility, compatibility_paths = validate_auditor_compatibility(
            auditor_compatibility,run,audit_path,audit,report,closure)
    elif file_identity(frozen_auditor)["sha256"] != audit["auditor_source_sha256"]:
        raise ValueError("Final audit source differs from frozen reader; exact --auditor-compatibility is required for an approved derivative")
    if audit["archived_model_identity"]["model_xml_sha256"] != report["model_xml_sha256"]:
        raise ValueError("Audited XML is not this original native model")
    identity = audit["archive_and_actuation"]
    if (not identity["all_archived_whole_dependencies_match_recorded"]
            or not identity["runtime_core_and_plugin_match_recorded"]
            or identity["controller_identity"]["controller_sha256"] != report["controller_sha256"]):
        raise ValueError("Original source/runtime identity could not be independently verified")
    if file_identity(run/"scene.xml")["sha256"] != report["model_xml_sha256"]:
        raise ValueError("Archived native XML differs")
    if file_identity(run/"scene_source.py")["sha256"] != report["scene_source_sha256"]:
        raise ValueError("Archived native scene source differs")
    if file_identity(run/"controller_source.py")["sha256"] != report["controller_module_sha256"]:
        raise ValueError("Archived controller module differs")
    for field in ("left_pad_force_history", "table_support_force_history", "native_feedback_force_history"):
        ledger = report[field]
        if file_identity(run/ledger["filename"])["sha256"] != ledger["sha256"]:
            raise ValueError(f"Original solved force ledger differs: {field}")
    feedback = report["native_feedback_force_history"]
    feedback_path = run/feedback["filename"]
    # Read only ZIP's original array-name catalog; preserve the complete NPZ
    # verbatim rather than decoding/re-encoding large vectors or contact rows.
    with zipfile.ZipFile(feedback_path) as archive:
        original_feedback_arrays = sorted(p[:-4] for p in archive.namelist() if p.endswith('.npy'))
    if set(feedback["columns"]) - set(original_feedback_arrays):
        raise ValueError("An original feedback declaration column is absent")
    if feedback["helper_source_sha256"] != report["feedback_source_sha256"]:
        raise ValueError("Original feedback helper binding differs")
    proof = strict_json(run/'software_tests.json')
    if not proof['passed'] or not proof['source_hashes_unchanged']:
        raise ValueError('Original software proof is not closed with unchanged sources')
    if proof['source_hashes'] != beginning['source_hashes_before']:
        raise ValueError('Original whole software proof differs from native beginning/end source declaration')
    producer_hashes = dict(report['recorded_source_dependencies_sha256'])
    producer_hashes['controller_source.py'] = report['controller_module_sha256']
    producer_hashes['scene_source.py'] = report['scene_source_sha256']
    proof_paths={'controller_source.py':'yam_twin/m8_supported_simulation.py',
        'scene_source.py':'yam_twin/m8_supported_scene.py'}
    for archive_path, digest in producer_hashes.items():
        proof_path=proof_paths.get(archive_path,archive_path.removeprefix('recorded_sources/'))
        if file_identity(run/archive_path)['sha256'] != digest or proof['source_hashes'].get(proof_path) != digest:
            raise ValueError(f'Archived native producer differs from this exact original proof: {archive_path}')
    initial_sources = {
        "trace.npz": run/"insertion_trace.npz", "validation.json": run/"insertion_validation.json",
        "scene.xml": run/"scene.xml", "scene_source.py": run/"scene_source.py",
        "controller_source.py": run/"controller_source.py",
        "recorded_app_renderer_source.py": run/"renderer_source.py",
        "engagement_observer_source.py": run/"engagement_observer_source.py",
        "supported_scene.zip": run/"supported_scene.zip",
        "left_pad_force_history.npz": run/"left_pad_force_history.npz",
        "table_support_force_history.npz": run/"table_support_force_history.npz",
        "native_feedback_force_history.npz": feedback_path,
        "independent_supported_audit.json": audit_path,
        "software_tests.json": run/"software_tests.json",
        "software_tests.log": run/"software_tests.log",
        "software_manifest.json": run/"manifest.json",
        "verify_software_sources.py": run/"verify_sources.py",
        "run_publication_identity.json": run/"run_publication_identity.json"}
    sources = {}
    for name, path in initial_sources.items():
        add_source(sources, name, path)
    for subtree in ("recorded_sources", "frozen_audit_sources"):
        for path in sorted((run/subtree).rglob("*")):
            if path.is_file():
                add_source(sources, str(path.relative_to(run)), path)
    # New producers add native command/event/proof/begin/end artifacts. Keep
    # every original closed output instead of maintaining a lossy allow-list.
    for path in sorted(run.rglob('*')):
        if path.is_file() and path not in sources.values():
            original_name = str(path.relative_to(run))
            add_source(sources, original_name, path)
    for index,path in enumerate(provenance):
        path=Path(path).resolve()
        if not path.is_file():raise FileNotFoundError(path)
        add_source(sources, f'execution_provenance/{index:02d}_{path.name}', path)
    if compatibility_paths:
        for name,path in compatibility_publication_sources(auditor_compatibility,compatibility_paths).items():
            add_source(sources,name,path)
    for path in sorted(render.rglob("*")):
        if path.is_file() and path.name != "SHA256SUMS":
            add_source(sources, str(path.relative_to(render)), path)
    for name in ("independent_left_pad_audit.json", "independent_free_joint_audit.json",
                 "independent_reset_audit.json", "source_identity.json"):
        if (run/name).exists():
            add_source(sources, name, run/name)
    if audit_directory != run:
        for path in sorted(audit_directory.rglob("*")):
            if path.is_file():
                add_source(sources, "supplemental_audits/" + str(path.relative_to(audit_directory)), path)
    reserved = {'package_manifest.json', 'reassemble_archives.py', 'restore_original_run_layout.py',
                'publication_helper_source.py', 'README.md', 'SHA256SUMS'}
    if reserved & set(sources):
        raise ValueError('Original artifact collides with generated package metadata: '
                         + ', '.join(sorted(reserved & set(sources))))
    original_mapping = {}
    for path in sorted(run.rglob('*')):
        if path.is_file():
            names = [name for name, source in sources.items() if source == path.resolve()]
            if not names:
                raise ValueError('Original evidence file disappeared from publication: ' + str(path))
            original_mapping[str(path.relative_to(run))] = names
    for name, path in sources.items():
        if path.suffix == ".json":
            strict_json(path)
    for name in ("demo.mp4", "demo.gif", "demo.png"):
        if name not in sources:
            raise FileNotFoundError(f"Closed final media missing: {name}")
        if rendered["media_sha256"].get(name) != file_identity(sources[name])["sha256"]:
            raise ValueError(f"Closed media checksum differs: {name}")
    snapshots = {name: file_identity(path) for name, path in sources.items()}
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.parent / (target.name + ".packaging-" + uuid.uuid4().hex)
    staging.mkdir()
    try:
        artifacts = {name: write_artifact(path, staging, name, chunk_bytes)
                     for name, path in sources.items()}
        for name, path in sources.items():
            if file_identity(path) != snapshots[name]:
                raise ValueError(f"Closed evidence changed during packaging: {name}")
        manifest = {
            "status": "completed_native_sequence" if complete else "unsuccessful_native_trial",
            "original_report_passed": report["passed"], "independent_audit_passed": audit["passed"],
            "original_report_partial": report["partial"], "aborted": report["aborted"],
            "rendered_evidence_scope": rendered.get("scope"),
            "independent_primary_auditor_sha256": file_identity(frozen_auditor)['sha256'],
            "independent_audit_report_reader_sha256": audit['auditor_source_sha256'],
            "independent_audit_kind": 'output_only_compatibility_derivative' if compatibility else 'original_frozen_reader',
            "auditor_compatibility": compatibility,
            "actual_unaborted_qualified_phase_sequence_completed": complete,
            "qualified_turn_phase_count": len(turns),
            "first_physics_time_s": float(times[0]), "last_physics_time_s": float(times[-1]),
            "sample_count": len(times), "trajectory_sha256": trace_identity["sha256"],
            "model_fingerprint": report["model_fingerprint"],
            "model_xml_sha256": report["model_xml_sha256"],
            "scene_source_sha256": report["scene_source_sha256"],
            "controller_sha256": report["controller_sha256"],
            "controller_module_sha256": report["controller_module_sha256"],
            "runtime": report["runtime"], "maximum_stored_file_bytes": chunk_bytes,
            "complete_original_feedback_ledger_sha256": feedback["sha256"],
            "original_feedback_observed_physics_steps": feedback["observed_physics_steps"],
            "original_feedback_declared_columns": feedback["columns"],
            "original_feedback_npz_array_names": original_feedback_arrays,
            "feedback_helper_source_sha256": feedback["helper_source_sha256"],
            "all_original_closed_run_files_included": True,
            "original_run_file_to_published_artifact_names": original_mapping,
            "original_run_file_identities": {name:snapshots[aliases[0]]
                for name,aliases in original_mapping.items()},
            "closure_native_exit_code": closure['native_exit_code'],
            "original_closure_identity_sha256": file_identity(run/'run_publication_identity_after.json')['sha256'],
            "external_begin_end_logs_provenance": [str(Path(p).resolve()) for p in provenance],
            "software_proof_is_this_run_original_never_historical_relabelled": True,
            "software_tests_passed_original": proof['tests_passed'],
            "software_proof_source_hash_count_original": len(proof['source_hashes']),
            "software_proof_sha256_original": file_identity(run/'software_tests.json')['sha256'],
            "archived_whole_producer_dependencies_match_original_software_proof": True,
            "archives_are_lossless": True, "dropping_or_quantization": False,
            "artifacts": artifacts,
            "scope": "Exact native table-supported trial evidence. Original acceptance results remain authoritative, including every failed gate. Saved-state replay uses no integration/interpolation/freebody posing. Material calibration, broader numerical convergence and learned-policy robustness are not certified.",
            "first_demo_preserved": "media/m8_table_pickup/full is unchanged"}
        (staging/"package_manifest.json").write_text(json.dumps(manifest,indent=2,allow_nan=False)+"\n")
        helper = Path(__file__).parent/"reassemble_archives.py"
        shutil.copyfile(helper, staging/"reassemble_archives.py")
        shutil.copyfile(Path(__file__).parent/'restore_original_run_layout.py',
                        staging/'restore_original_run_layout.py')
        shutil.copyfile(__file__, staging/"publication_helper_source.py")
        (staging/"README.md").write_text(make_readme(target, manifest))
        stored = sorted(p for p in staging.rglob("*") if p.is_file())
        if any(p.stat().st_size > chunk_bytes for p in stored):
            raise ValueError("A stored package file exceeds the requested chunk cap")
        (staging/"SHA256SUMS").write_text("".join(
            f'{file_identity(p)["sha256"]}  {p.relative_to(staging)}\n' for p in stored))
        if target.exists():
            raise FileExistsError(target)
        staging.rename(target)
        return {"target": str(target.relative_to(ROOT)), "artifacts": len(artifacts),
            "stored_files": len(stored)+1, "trace_sha256": trace_identity["sha256"],
            "complete_sequence": complete, "report_passed": report["passed"],
            "independent_audit_passed": audit["passed"]}
    except Exception:
        # This staging directory was uniquely created by this invocation and
        # has never been published or shared with an existing package.
        shutil.rmtree(staging)
        raise


def raw_bundle(run, target, *, chunk_bytes=DEFAULT_CHUNK_BYTES, provenance=()):
    """Preserve an exceptional CLOSED producer output without inventing physics.

    Arbitrary original NPZ/JSON/log bytes are copied verbatim. This path does
    not require a final report, completed force ledgers, or a simulator/auditor.
    A missing producer exit identity is always a LIVE/nonclosed refusal.
    """
    run, target = Path(run).resolve(), Path(target).resolve()
    if target.is_relative_to(run):
        raise ValueError('Raw publication target must be outside original native evidence')
    if target.exists():
        raise FileExistsError(target)
    if not 1 <= chunk_bytes <= DEFAULT_CHUNK_BYTES:
        raise ValueError('Chunk size must be 1..45,000,000 bytes')
    beginning, closure = closed_identity(run)
    sources = {}
    original_paths = sorted(path for path in run.rglob('*') if path.is_file())
    for path in original_paths:
        add_source(sources, str(path.relative_to(run)), path)
    for index, path in enumerate(provenance):
        path = Path(path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        add_source(sources, f'execution_provenance/{index:02d}_{path.name}', path)
    snapshots = {name: file_identity(path) for name, path in sources.items()}
    final_names = ('insertion_trace.npz', 'insertion_validation.json',
                   'left_pad_force_history.npz', 'table_support_force_history.npz',
                   'native_feedback_force_history.npz')
    missing = [name for name in final_names if not (run/name).is_file()]
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.parent/(target.name+'.packaging-'+uuid.uuid4().hex)
    staging.mkdir()
    try:
        artifacts = {}
        for name, path in sources.items():
            artifact = write_artifact(path, staging/'original_run', name, chunk_bytes)
            if artifact['storage'] == 'single_file':
                artifact['path'] = 'original_run/'+artifact['path']
            else:
                for chunk in artifact['chunks']:
                    chunk['path'] = 'original_run/'+chunk['path']
            artifacts[name] = artifact
        if sorted(path for path in run.rglob('*') if path.is_file()) != original_paths:
            raise ValueError('Closed raw evidence directory changed during publication')
        for name, path in sources.items():
            if file_identity(path) != snapshots[name]:
                raise ValueError('Closed raw evidence changed during publication: '+name)
        sources_agree = (closure.get('source_hashes_unchanged') is True and
                         beginning['source_hashes_before'] == closure['source_hashes_after'])
        manifest = {
            'status': 'closed_raw_evidence_only',
            'physics_success_claim': False, 'physical_qualification_audited': False,
            'final_report_or_ledger_synthesized': False,
            'producer_commit': beginning['producer_commit'],
            'native_exit_code': closure['native_exit_code'],
            'source_hashes_unchanged': sources_agree,
            'original_declared_source_hashes_unchanged': closure.get('source_hashes_unchanged'),
            'original_source_hashes_before': beginning['source_hashes_before'],
            'original_source_hashes_after': closure['source_hashes_after'],
            'missing_final_artifacts': missing,
            'original_partial_trace_present': (run/'insertion_trace_partial.npz').is_file(),
            'original_closed_directory_file_count': len(original_paths),
            'all_existing_original_files_included': True,
            'original_file_identities': {str(path.relative_to(run)):
                snapshots[str(path.relative_to(run))] for path in original_paths},
            'known_identity_artifact_paths': [name for name in sources if
                name in ('run_publication_identity.json', 'run_publication_identity_after.json',
                         'software_tests.json', 'software_tests.log', 'manifest.json', 'launch_source.py')
                or name.startswith(('recorded_sources/', 'frozen_audit_sources/'))],
            'runtime_identity_scope': 'Known source/proof/runtime declarations are retained in their original identity, source, and raw trace bytes. No missing runtime measurement or report field is synthesized.',
            'artifacts': artifacts, 'maximum_stored_file_bytes': chunk_bytes,
            'archives_are_lossless': True, 'dropping_or_quantization': False,
            'scope': 'Exceptional CLOSED raw producer evidence only. Missing final reports/ledgers stay missing. Original native exit, partial trace, failures, identities, arbitrary JSON/NPZ and logs remain exact bytes. No native model, contact, force, capture, lead, reset or completed-task proof is asserted.'}
        (staging/'package_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
        shutil.copyfile(Path(__file__).parent/'reassemble_archives.py',staging/'reassemble_archives.py')
        shutil.copyfile(Path(__file__).parent/'restore_original_run_layout.py',staging/'restore_original_run_layout.py')
        shutil.copyfile(__file__,staging/'publication_helper_source.py')
        (staging/'README.md').write_text(
            '# Closed raw native evidence\n\n'+manifest['scope']+'\n\n'
            f"Native exit code: {closure['native_exit_code']}. Missing final artifacts: {', '.join(missing) or 'none'}.\n\n"
            'Every existing original file is preserved, without JSON normalization or NPZ re-encoding. '
            'Storage lives under `original_run/`; artifact names in the manifest restore the exact original relative filenames. '
            'Large files are consecutive <=45,000,000-byte chunks, each with offset/size/SHA and a complete whole-file SHA.\n\n'
            'From this package directory, using standard Python:\n\n```sh\n'
            'python reassemble_archives.py --verify-only\n'
            'python reassemble_archives.py --output /absolute/path/to/a/new/restored_original_run\n```\n\n'
            'The output contains the original `insertion_trace_partial.npz` when that is all the producer wrote. '
            'Do not rename it as a completed recording, manufacture a validation report or force ledger, or infer missing physical stages. '
            'Frozen auditor source snapshots are provenance, not a complete standalone Python application/runtime. '
            'Any later native inspection requires the complete matching producer checkout/runtime and separate clearly labeled derivatives.\n')
        stored = sorted(path for path in staging.rglob('*') if path.is_file())
        if any(path.stat().st_size > chunk_bytes for path in stored):
            raise ValueError('A stored raw package file exceeds the requested chunk cap')
        (staging/'SHA256SUMS').write_text(''.join(
            f"{file_identity(path)['sha256']}  {path.relative_to(staging)}\n" for path in stored))
        if target.exists():
            raise FileExistsError(target)
        staging.rename(target)
        return {'target': str(target), 'status': manifest['status'],
            'physics_success_claim': False, 'artifacts': len(artifacts),
            'stored_files': len(stored)+1, 'missing_final_artifacts': missing,
            'native_exit_code': closure['native_exit_code']}
    except Exception:
        shutil.rmtree(staging)
        raise


def legacy_make_readme(target, manifest):
    path = str(target.relative_to(ROOT))
    result = "passed" if manifest["original_report_passed"] else "failed"
    completed = "completed" if manifest["actual_unaborted_qualified_phase_sequence_completed"] else "did not complete"
    return f'''# Native table-supported M8 trial\n\nThe original recorded sequence {completed}; its original overall validation {result}.\nRead `validation.json` and `independent_supported_audit.json` for every result\nand its scope. Video and stills replay exact native qpos/qvel with the state-refresh method\nbound in `render_manifest.json`. They do not integrate, interpolate or manually\npose free workpieces. Force captions use the original solved samples at saved\ntime minus one native timestep; replay forces are not original evidence.\nPlayback is normal 1×.\n\n{manifest["rendered_evidence_scope"]}\n\nThe free female block rests on the plain solid table, with left fingers\nstabilizing its sides. The separate male bolt starts on its physical side rest.\nThe table remains solid under the bore; the recorded tip/table clearance guard\nlimits this to running-thread travel, without a full head-seating claim.\n\nEvery original trace and force-ledger byte is preserved. Files larger than\n45,000,000 bytes are split into consecutive fixed-size binary chunks, with\nwhole-file size/SHA256 and each chunk offset/size/SHA256 in `package_manifest.json`.\nNo rows are dropped and no state, force, or contact values are quantized. The\nlast chunk may be smaller. GitHub may not preview NPZ/chunk files.\n\n## Verify and reassemble\n\nFrom the repository checkout, use standard Python; no simulator is needed:\n\n```sh\npython {path}/reassemble_archives.py --verify-only\npython {path}/reassemble_archives.py --output outputs/m8_table_supported/reassembled\n```\n\nThe helper verifies every stored chunk and reconstructed whole file, restores\nall artifacts to their original filenames and relative archive directories,\nand refuses to overwrite differing files. An existing byte-identical file is\naccepted only after the source chunks are verified again.\n\n## Replay and inspect\n\nAfter preparing the verified native MuJoCo runtime according to\n`docs/m8_setup.md`, inspect the archived trajectory independently. This uses the\nmatched native GCC CPU engine and bindings with the pinned source/engine patch;\nthe stock MuJoCo wheel is insufficient. Runtime core/plugin SHA identities are\npreserved. A rebuild on another machine must be compared before claiming\nbyte-identical native libraries:\n\n```sh\nscripts/run_m8.sh scripts/audit_m8_supported_trace.py outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/repeated_audit.json\nscripts/run_m8.sh -m yam_twin.m8_supported_demo --replay outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/replayed --video --fps 12 --slow-motion 1\n```\n\nThose generic commands inspect/replay recorded states; they do not regenerate\na controller rollout. The generic supported_demo replay uses mj_forward and\ncan recompute unused replay forces; it is a NEW replay, not reproduction of the\ngeometry-only publication render. For actual-render reproduction, follow\nREPLAY_ACTUAL_MEDIA.md and its archived renderer source when supplied. Exact production sources are retained in\n`controller_source.py`, `scene_source.py`, `recorded_sources/`, and\n`frozen_audit_sources/`. `run_publication_identity.json` records the producer\ncommit. Software proof and its unchanged-source manifest remain separate from\nphysical qualification. Supplemental audit revisions, when present, retain their\nown source hashes and revision record rather than relabeling the original audit.\nSee `docs/m8_table_supported.md` for fresh rollout\ncommands, input configuration, and reconstruction limits.\n\nThe first carried-block demonstration remains unchanged at\n`media/m8_table_pickup/full`. This package does not certify calibrated materials,\nfull seating/preload, broad numerical convergence, or learned-policy robustness.\n'''


def make_readme(target, manifest):
    text=legacy_make_readme(target,manifest)
    addition=f'''The exact original producer software proof contains **{manifest["software_tests_passed_original"]} tests**
and **{manifest["software_proof_source_hash_count_original"]} source hashes**; it remains separate from native
physical acceptance and earlier historical proofs. Every original closed run
artifact is retained, including the complete `native_feedback_force_history.npz`
and its own metadata, control/event declarations, executed finite motor wrenches
and torques, open-contact histories, per-grasp masks, original loaded-interior
observations, all native state arrays, and original before/after provenance.
No feedback column, row, dtype or binary archive byte is dropped or re-encoded.
Extra sibling execution manifests/logs supplied to publication are preserved
under `execution_provenance/` with original identities. The package manifest
lists every original feedback array name and whole-file SHA. Reassembly restores
the exact whole NPZ and standard filenames before audits or replay.

To restore the EXACT ORIGINAL native-run names and directories, use the verified
original-layout helper rather than renaming only the trace:

```sh
python {target.relative_to(ROOT)}/restore_original_run_layout.py --verify-only
python {target.relative_to(ROOT)}/restore_original_run_layout.py --output outputs/m8_table_supported/original_run_layout
```

It verifies stored/chunk/whole SHA first, then restores all original files from
the manifest's native filename/identity mapping, including `insertion_trace.npz`,
`insertion_validation.json`, `renderer_source.py` and `manifest.json`. It refuses
to overwrite changed destinations. Invoke the original closure verifier and
matched archived/native model auditors on this original layout in the complete
producer checkout. `frozen_audit_sources/` is provenance, not a standalone runtime.

'''
    if manifest['auditor_compatibility'] is not None:
        compatibility = manifest['auditor_compatibility']
        addition += ('The frozen original supported reader remains `'
            + compatibility['original_frozen_auditor_sha256']+'`; its original exception is preserved. '
            'The independently generated report is explicitly an output-only compatibility derivative using `'
            + compatibility['corrected_output_only_reader_sha256']+'`, with the exact narrow source diff, '
            'boundary diagnosis, regression proof and sidecar under `auditor_compatibility/`. '
            'Only the initial same-row versus quiet-regrasp next-tick guard boundary is corrected. '
            'The original402/65 proof, native source/report and physical acceptance outcomes are unchanged.\n\n')
    return text.replace('Every original trace and force-ledger byte is preserved.',addition+'Every original trace and force-ledger byte is preserved.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("render", type=Path, nargs='?')
    parser.add_argument("target", type=Path, nargs='?')
    parser.add_argument("--evidence-only", action="store_true")
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--chunk-bytes", type=int, default=DEFAULT_CHUNK_BYTES)
    parser.add_argument("--audit-dir", type=Path)
    parser.add_argument("--provenance", type=Path, action="append", default=[],
        help="Exact extra begin/end producer manifests, logs or proof artifacts, retained verbatim")
    parser.add_argument('--raw-evidence-bundle', action='store_true',
        help='Exceptional CLOSED producer byte preservation; no final report/ledger or physics-success claim')
    parser.add_argument('--raw-target', type=Path)
    parser.add_argument('--auditor-compatibility', type=Path,
        help='Exact approved output-only reader sidecar; never replaces the original frozen reader/proof')
    args = parser.parse_args()
    if args.raw_evidence_bundle:
        if (args.raw_target is None or args.render is not None or args.target is not None
                or args.require_complete or args.audit_dir is not None
                or args.auditor_compatibility is not None):
            parser.error('Raw mode requires run --raw-evidence-bundle --raw-target TARGET, without render/target/qualification options')
        print(json.dumps(raw_bundle(args.run,args.raw_target,chunk_bytes=args.chunk_bytes,
            provenance=args.provenance),indent=2))
        return
    if args.render is None or args.target is None or args.raw_target is not None:
        parser.error('Final audited mode requires run render target')
    print(json.dumps(package(args.run,args.render,args.target,evidence_only=args.evidence_only,
        require_complete=args.require_complete,chunk_bytes=args.chunk_bytes,
        audit_dir=args.audit_dir,provenance=args.provenance,
        auditor_compatibility=args.auditor_compatibility),indent=2))


if __name__ == "__main__":
    main()
