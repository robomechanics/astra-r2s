"""Publish closed native supported-demo evidence without dropping archive bytes.

This output-only helper never changes physics/controller/auditor sources. Large
files use fixed-size binary chunks, each <=45 MB, plus whole-file SHA256 hashes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import uuid

ROOT = Path("/workspace/astra-r2s")
DEFAULT_CHUNK_BYTES = 45_000_000


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


def package(run, render, target, *, evidence_only=False, require_complete=False,
            chunk_bytes=DEFAULT_CHUNK_BYTES, audit_dir=None):
    run, render, target = map(lambda p: Path(p).resolve(), (run, render, target))
    if target.exists():
        raise FileExistsError(target)
    if not 1 <= chunk_bytes <= DEFAULT_CHUNK_BYTES:
        raise ValueError("Chunk size must be 1..45,000,000 bytes")
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
    audit = strict_json(audit_path)
    if audit["trajectory_sha256"] != trace_identity["sha256"]:
        raise ValueError("Independent audit belongs to another native trace")
    frozen_auditor = run / "frozen_audit_sources/scripts/audit_m8_supported_trace.py"
    if file_identity(frozen_auditor)["sha256"] != audit["auditor_source_sha256"]:
        raise ValueError("Final audit source differs from the frozen run auditor")
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
    for field in ("left_pad_force_history", "table_support_force_history"):
        ledger = report[field]
        if file_identity(run/ledger["filename"])["sha256"] != ledger["sha256"]:
            raise ValueError(f"Original solved force ledger differs: {field}")
    sources = {
        "trace.npz": run/"insertion_trace.npz", "validation.json": run/"insertion_validation.json",
        "scene.xml": run/"scene.xml", "scene_source.py": run/"scene_source.py",
        "controller_source.py": run/"controller_source.py",
        "recorded_app_renderer_source.py": run/"renderer_source.py",
        "engagement_observer_source.py": run/"engagement_observer_source.py",
        "supported_scene.zip": run/"supported_scene.zip",
        "left_pad_force_history.npz": run/"left_pad_force_history.npz",
        "table_support_force_history.npz": run/"table_support_force_history.npz",
        "independent_supported_audit.json": audit_path,
        "software_tests.json": run/"software_tests.json",
        "software_tests.log": run/"software_tests.log",
        "software_manifest.json": run/"manifest.json",
        "verify_software_sources.py": run/"verify_sources.py",
        "run_publication_identity.json": run/"run_publication_identity.json"}
    for subtree in ("recorded_sources", "frozen_audit_sources"):
        for path in sorted((run/subtree).rglob("*")):
            if path.is_file():
                sources[str(path.relative_to(run))] = path
    for path in sorted(render.rglob("*")):
        if path.is_file() and path.name != "SHA256SUMS":
            sources[str(path.relative_to(render))] = path
    for name in ("independent_left_pad_audit.json", "independent_free_joint_audit.json",
                 "independent_reset_audit.json", "source_identity.json"):
        if (run/name).exists():
            sources[name] = run/name
    if audit_directory != run:
        for path in sorted(audit_directory.rglob("*")):
            if path.is_file():
                sources["supplemental_audits/" + str(path.relative_to(audit_directory))] = path
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
            "independent_primary_auditor_sha256": audit["auditor_source_sha256"],
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
            "archives_are_lossless": True, "dropping_or_quantization": False,
            "artifacts": artifacts,
            "scope": "Exact native table-supported trial evidence. Original acceptance results remain authoritative, including every failed gate. Saved-state replay uses no integration/interpolation/freebody posing. Material calibration, broader numerical convergence and learned-policy robustness are not certified.",
            "first_demo_preserved": "media/m8_table_pickup/full is unchanged"}
        (staging/"package_manifest.json").write_text(json.dumps(manifest,indent=2,allow_nan=False)+"\n")
        helper = Path(__file__).parent/"reassemble_archives.py"
        shutil.copyfile(helper, staging/"reassemble_archives.py")
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


def make_readme(target, manifest):
    path = str(target.relative_to(ROOT))
    result = "passed" if manifest["original_report_passed"] else "failed"
    completed = "completed" if manifest["actual_unaborted_qualified_phase_sequence_completed"] else "did not complete"
    return f'''# Native table-supported M8 trial\n\nThe original recorded sequence {completed}; its original overall validation {result}.\nRead `validation.json` and `independent_supported_audit.json` for every result\nand its scope. Video and stills replay exact native qpos/qvel with `mj_forward`\nonly. They do not integrate, interpolate, manually pose free workpieces, or\nreconstruct original solved forces. Playback is normal 1×.\n\n{manifest["rendered_evidence_scope"]}\n\nThe free female block rests on the plain solid table, with left fingers\nstabilizing its sides. The separate male bolt starts on its physical side rest.\nThe table remains solid under the bore; the recorded tip/table clearance guard\nlimits this to running-thread travel, without a full head-seating claim.\n\nEvery original trace and force-ledger byte is preserved. Files larger than\n45,000,000 bytes are split into consecutive fixed-size binary chunks, with\nwhole-file size/SHA256 and each chunk offset/size/SHA256 in `package_manifest.json`.\nNo rows are dropped and no state, force, or contact values are quantized. The\nlast chunk may be smaller. GitHub may not preview NPZ/chunk files.\n\n## Verify and reassemble\n\nFrom the repository checkout, use standard Python; no simulator is needed:\n\n```sh\npython {path}/reassemble_archives.py --verify-only\npython {path}/reassemble_archives.py --output outputs/m8_table_supported/reassembled\n```\n\nThe helper verifies every stored chunk and reconstructed whole file, restores\nall artifacts to their original filenames and relative archive directories,\nand refuses to overwrite differing files. An existing byte-identical file is\naccepted only after the source chunks are verified again.\n\n## Replay and inspect\n\nAfter preparing the verified native MuJoCo runtime according to\n`docs/m8_setup.md`, inspect the archived trajectory independently. This uses the\nmatched native GCC CPU engine and bindings with the pinned source/engine patch;\nthe stock MuJoCo wheel is insufficient. Runtime core/plugin SHA identities are\npreserved. A rebuild on another machine must be compared before claiming\nbyte-identical native libraries:\n\n```sh\nscripts/run_m8.sh scripts/audit_m8_supported_trace.py outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/repeated_audit.json\nscripts/run_m8.sh -m yam_twin.m8_supported_demo --replay outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/replayed --video --fps 12 --slow-motion 1\n```\n\nThose commands inspect/replay recorded states; they do not regenerate a\ncontroller rollout. Exact production sources are retained in\n`controller_source.py`, `scene_source.py`, `recorded_sources/`, and\n`frozen_audit_sources/`. `run_publication_identity.json` records the producer\ncommit. Software proof and its unchanged-source manifest remain separate from\nphysical qualification. Supplemental audit revisions, when present, retain their\nown source hashes and revision record rather than relabeling the original audit.\nSee `docs/m8_table_supported.md` for fresh rollout\ncommands, input configuration, and reconstruction limits.\n\nThe first carried-block demonstration remains unchanged at\n`media/m8_table_pickup/full`. This package does not certify calibrated materials,\nfull seating/preload, broad numerical convergence, or learned-policy robustness.\n'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("render", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("--evidence-only", action="store_true")
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--chunk-bytes", type=int, default=DEFAULT_CHUNK_BYTES)
    parser.add_argument("--audit-dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(package(args.run,args.render,args.target,evidence_only=args.evidence_only,
        require_complete=args.require_complete,chunk_bytes=args.chunk_bytes,
        audit_dir=args.audit_dir),indent=2))


if __name__ == "__main__":
    main()
