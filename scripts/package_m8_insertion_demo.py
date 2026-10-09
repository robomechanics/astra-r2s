"""Freeze a completed M8 insertion recording and its declared provenance.

Packaging performs no simulation and cannot establish physics qualification.
Mutable partial traces are never used. An existing destination is not replaced.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import inspect
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def _snapshot(path, snapshots):
    path = Path(path).resolve()
    if path not in snapshots:
        before = path.stat()
        value = path.read_bytes()
        after = path.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (
                after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError(f"Input changed while being read: {path}")
        snapshots[path] = value
    return snapshots[path]


def _source_object(source, name):
    tree = ast.parse(source)
    node = next((n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                 and n.name == name), None)
    if node is None:
        return None
    start = min([node.lineno, *(d.lineno for d in node.decorator_list)]) - 1
    return "".join(inspect.getblock(source.splitlines(keepends=True)[start:]))


def _controller_identity(path, run_dir, snapshots):
    source = _snapshot(path, snapshots).decode()
    tree = ast.parse(source)
    lists = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == "controller_sha256":
                    for child in ast.walk(value):
                        if isinstance(child, ast.GeneratorExp) and isinstance(child.generators[0].iter, ast.Tuple):
                            names = child.generators[0].iter.elts
                            if all(isinstance(n, ast.Name) for n in names):
                                lists.append([n.id for n in names])
    if len(lists) != 1:
        raise ValueError("Cannot resolve the archived controller's declared source-hash list")
    imports = {}
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module:
            for item in node.names:
                imports[item.asname or item.name] = (node.module, item.name)
    fragments, helper_sources = [], {}
    for name in lists[0]:
        fragment = _source_object(source, name)
        if fragment is None:
            module, original = imports[name]
            if module == "m8_insertion_engagement":
                helper = run_dir / "engagement_observer_source.py"
            else:
                archived = run_dir / "recorded_sources" / "yam_twin" / f"{module}.py"
                helper = archived if archived.exists() else ROOT / "yam_twin" / f"{module}.py"
            value = _snapshot(helper, snapshots)
            fragment = _source_object(value.decode(), original)
            helper_sources[f"recorded_sources/yam_twin/{module}.py"] = value
        if fragment is None:
            raise ValueError(f"Cannot inspect declared controller source object: {name}")
        fragments.append(fragment)
    return sha256("\n".join(fragments).encode()), lists[0], helper_sources


def _model_identity(xml, run_dir, snapshots):
    root = ET.fromstring(xml)
    hashes = {}
    for mesh in root.findall("asset/mesh"):
        name = mesh.get("file")
        if not name:
            continue
        path = Path(name)
        if path.is_absolute() and not path.exists() and "assets" in path.parts:
            # Recorded XML keeps its original absolute paths; verify the same
            # repository asset bytes after a checkout moves to another machine.
            path = ROOT.joinpath(*path.parts[path.parts.index("assets"):])
        elif not path.is_absolute():
            candidates = (run_dir / path, ROOT / path)
            path = next((p for p in candidates if p.exists()), candidates[0])
        value = sha256(_snapshot(path, snapshots))
        if path.name in hashes and hashes[path.name] != value:
            raise ValueError("Mesh basenames collide in the portable scene identity")
        hashes[path.name] = value
        mesh.set("file", path.name)
    value = {"xml": ET.tostring(root, encoding="unicode"), "mesh_sha256": hashes}
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()), hashes


def load_completed_run(run_dir, *, evidence_only=False, legacy_inline_observer=False):
    """Read final files, verify declared identities, and return immutable bytes."""
    run_dir = Path(run_dir).resolve()
    snapshots = {}
    trace = _snapshot(run_dir / "insertion_trace.npz", snapshots)
    validation = _snapshot(run_dir / "insertion_validation.json", snapshots)
    report = json.loads(validation)
    with np.load(io.BytesIO(trace), allow_pickle=False) as archive:
        arrays = {key: archive[key].copy() for key in archive.files}
    recorded = json.loads(str(arrays["metadata_json"]))
    if recorded != report:
        raise ValueError("Final trace metadata does not exactly match the validation report")
    if type(report.get("passed")) is not bool or type(report.get("partial")) is not bool:
        raise ValueError("Final report lacks explicit outcome and scope")
    if report["partial"] and not evidence_only:
        raise ValueError("A bounded partial-task outcome requires --evidence-only")
    times = arrays["time"]
    rows = json.loads(str(arrays["info_json"]))
    if (times.ndim != 1 or not len(times) or not np.isfinite(times).all()
            or not np.all(np.diff(times) > 0) or len(rows) != len(times)):
        raise ValueError("Trace times and sampled diagnostics are not finite and aligned")
    for field in ("qpos", "qvel", "controller"):
        if arrays[field].ndim != 2 or len(arrays[field]) != len(times) or not np.isfinite(arrays[field]).all():
            raise ValueError(f"Invalid recorded {field} array")
    if report["passed"] and (report.get("aborted") or not all(
            check.get("passed") is True for check in report["acceptance_checks"].values())):
        raise ValueError("Reported pass contradicts its acceptance checks or abort")
    xml = _snapshot(run_dir / "scene.xml", snapshots)
    if sha256(xml) != report["model_xml_sha256"]:
        raise ValueError("Frozen scene XML does not match the declared SHA")
    fingerprint, meshes = _model_identity(xml, run_dir, snapshots)
    if fingerprint != report["model_fingerprint"]:
        raise ValueError("Frozen scene and repository mesh bytes do not match the model fingerprint")
    controller_sha, names, helpers = _controller_identity(run_dir / "controller_source.py", run_dir, snapshots)
    if controller_sha != report["controller_sha256"]:
        raise ValueError("Archived controller/helper definitions do not match the declared function SHA")
    observer_sha = report.get("engagement_observer_source_sha256")
    warnings = []
    if observer_sha:
        observer = _snapshot(run_dir / "engagement_observer_source.py", snapshots)
        if sha256(observer) != observer_sha:
            raise ValueError("Frozen engagement observer does not match its declared SHA")
    elif evidence_only or legacy_inline_observer:
        if "LoadedFlankWindow" in names:
            raise ValueError("A modular observer cannot be treated as an inline legacy criterion")
        warnings.append("Legacy capture criterion is inline in the hash-verified controller archive; no separate observer module SHA exists")
    else:
        raise ValueError("Current publication requires engagement_observer_source_sha256")
    return {"report": report, "times": times, "rows": rows, "arrays": arrays,
            "checks": {"trace_report_identity": True, "finite_aligned_states": True,
                "model_xml_sha256": report["model_xml_sha256"], "model_fingerprint": fingerprint,
                "mesh_sha256": meshes, "controller_sha256": controller_sha,
                "hashed_controller_objects": names, "observer_sha256": observer_sha,
                "observer_identity_kind": "separate_hash_verified_module" if observer_sha else "legacy_inline_controller_criterion",
                "legacy_inline_observer_verified": bool(not observer_sha and "run_insertion_demo" in names),
                "warnings": warnings},
            "source_bytes": snapshots, "helper_sources": helpers,
            "trace_sha256": sha256(trace)}


def _closed_media(path, snapshots):
    # Same-user ffmpeg/Pillow writers must be closed before freezing media.
    for pid in Path("/proc").iterdir():
        if not pid.name.isdigit():
            continue
        try:
            for descriptor in (pid / "fd").iterdir():
                try:
                    if descriptor.resolve() == path.resolve():
                        flags = next(line.split()[1] for line in
                            (pid / "fdinfo" / descriptor.name).read_text().splitlines()
                            if line.startswith("flags:"))
                        if int(flags, 8) & os.O_ACCMODE:
                            raise ValueError(f"Media still has an open writer: {path}")
                except (OSError, StopIteration):
                    continue
        except OSError:
            continue
    value = _snapshot(path, snapshots)
    if path.suffix.lower() == ".png":
        valid = value.startswith(b"\x89PNG\r\n\x1a\n") and value.endswith(b"IEND\xaeB`\x82")
    elif path.suffix.lower() == ".gif":
        valid = value.startswith((b"GIF87a", b"GIF89a")) and value.endswith(b";")
    elif path.suffix.lower() == ".pdf":
        valid = value.startswith(b"%PDF-") and value.rstrip().endswith(b"%%EOF")
    else:
        offset, boxes = 0, []
        while offset + 8 <= len(value):
            size, kind = int.from_bytes(value[offset:offset+4], "big"), value[offset+4:offset+8]
            header = 8
            if size == 1:
                size, header = int.from_bytes(value[offset+8:offset+16], "big"), 16
            elif size == 0:
                size = len(value) - offset
            if size < header or offset + size > len(value):
                break
            boxes.append(kind)
            offset += size
        valid = offset == len(value) and all(b in boxes for b in (b"ftyp", b"moov", b"mdat"))
    if not valid:
        raise ValueError(f"Media container is incomplete or invalid: {path}")
    return value


def package_run(run_dir, target, *, rendered_media_dir=None, evidence_only=False,
                legacy_inline_observer=False):
    run_dir, target = Path(run_dir).resolve(), Path(target).resolve()
    if target.exists():
        raise ValueError(f"Refusing to replace an existing package: {target}")
    loaded = load_completed_run(run_dir, evidence_only=evidence_only,
                                legacy_inline_observer=legacy_inline_observer)
    snapshots, report = loaded["source_bytes"], loaded["report"]
    files = {"trace.npz": snapshots[run_dir / "insertion_trace.npz"],
             "validation.json": snapshots[run_dir / "insertion_validation.json"],
             "scene.xml": snapshots[run_dir / "scene.xml"],
             "controller_source.py": snapshots[run_dir / "controller_source.py"],
             **loaded["helper_sources"]}
    if report.get("engagement_observer_source_sha256"):
        files["engagement_observer_source.py"] = snapshots[run_dir / "engagement_observer_source.py"]
    modern_observer = bool(report.get("engagement_observer_source_sha256"))
    audit_name = "independent_capture_audit.json" if modern_observer else "independent_insertion_audit.json"
    audit_path = run_dir / audit_name
    audit_bytes = _snapshot(audit_path, snapshots)
    audit = json.loads(audit_bytes)
    if (audit["trajectory_sha256"] != loaded["trace_sha256"]
            or audit["recorded_metadata"] != report or audit["model_fingerprint"] != report["model_fingerprint"]
            or not audit["controller_source_matches_recorded"]
            or audit["inspected_controller_source"]["controller_sha256"] != report["controller_sha256"]
            or audit["inspected_controller_source"]["module_sha256"] != sha256(files["controller_source.py"])
            or not audit["runtime_core_and_plugin_identities_match_recorded"]):
        raise ValueError("Independent audit does not link to this exact trace/model/controller/runtime")
    removed = audit.pop("recomputed_rows", [])
    omitted = {"recomputed_rows": len(removed)}
    if modern_observer:
        observer_audit = audit["capture_observer_audit"]
        if (observer_audit["archived_source_sha256"] != report["engagement_observer_source_sha256"]
                or not observer_audit["archived_source_matches_recorded"]
                or not observer_audit["archived_version_matches_recorded"]):
            raise ValueError("Capture audit does not verify the recorded observer archive/version")
        if report["passed"] and not observer_audit["sampled_ready_flags_match_recorded_limits"]:
            raise ValueError("Recorded pass contradicts capture-observer threshold audit")
        omitted["capture_observer_audit.recorded_window_rows"] = len(observer_audit.pop("recorded_window_rows", []))
    source_names = ({"audit_m8_insertion_trace.py": audit["geometry_auditor_sha256"],
                     "audit_m8_insertion_capture.py": audit["capture_auditor_sha256"]}
                    if modern_observer else {"audit_m8_insertion_trace.py": audit["auditor_sha256"]})
    auditor_sources_verified = True
    for name, expected in source_names.items():
        candidates = (run_dir / "audit_sources" / name, ROOT / "scripts" / name)
        matching = next((p for p in candidates if p.exists() and sha256(p.read_bytes()) == expected), None)
        if matching is None:
            if not evidence_only and not legacy_inline_observer:
                raise ValueError(f"The audit's exact declared source is unavailable: {name}")
            loaded["checks"]["warnings"].append(f"Exact legacy audit source unavailable: {name}; verify its declared SHA before regeneration")
            auditor_sources_verified = False
        else:
            files["audit_sources/" + name] = _snapshot(matching, snapshots)
    try:
        relative_target = target.relative_to(ROOT)
    except ValueError:
        relative_target = target
    primary_script = "audit_m8_insertion_capture.py" if modern_observer else "audit_m8_insertion_trace.py"
    script_path = (relative_target / "audit_sources" / primary_script if auditor_sources_verified
                   else Path("scripts") / primary_script)
    audit_output_name = "independent_capture_audit.json" if modern_observer else "independent_audit.json"
    audit["compaction"] = {"omitted_fields_and_row_counts": omitted,
        "original_audit_sha256": sha256(audit_bytes),
        "regenerate_command": "bash scripts/run_m8.sh " + shlex.quote(str(script_path)) + " "
            + shlex.quote(str(relative_target / "trace.npz")) + " --output "
            + shlex.quote(str(relative_target / "independent_audit_full.json")),
        "required_auditor_source_sha256": source_names,
        "note": "Saved-pose replay cannot reconstruct original all-substep normal impulses or support forces"}
    files[audit_output_name] = (json.dumps(audit, indent=2) + "\n").encode()
    media_manifest = None
    if rendered_media_dir is not None:
        directory = Path(rendered_media_dir).resolve()
        media_manifest = json.loads(_snapshot(directory / "render_manifest.json", snapshots))
        if media_manifest.get("trajectory_sha256") != loaded["trace_sha256"]:
            raise ValueError("Rendered media manifest does not link to this completed trace")
        for key, extension in (("video", ".mp4"), ("screenshot", ".png")):
            name = media_manifest.get(key)
            if name:
                path = directory / Path(name).name
                if path.exists():
                    files["demo" + extension] = _closed_media(path, snapshots)
        video = media_manifest.get("video")
        if video:
            gif = directory / Path(video).with_suffix(".gif").name
            if gif.exists():
                files["demo.gif"] = _closed_media(gif, snapshots)
        files["render_manifest.json"] = snapshots[directory / "render_manifest.json"]
        chart_path = directory / "insertion_trajectory.json"
        if chart_path.exists():
            chart_bytes = _snapshot(chart_path, snapshots)
            chart = json.loads(chart_bytes)
            if (chart.get("source_trace_sha256") != loaded["trace_sha256"]
                    or chart.get("source_report_sha256") != sha256(files["validation.json"])):
                raise ValueError("Trajectory chart does not link to the exact final trace/report")
            plot_source = _snapshot(ROOT / "scripts/plot_m8_insertion_trace.py", snapshots)
            if sha256(plot_source) != chart["script_sha256"]:
                raise ValueError("Trajectory chart source does not match its declared SHA")
            for key in ("png", "pdf"):
                path = directory / Path(chart[key]).name
                files["trajectory." + key] = _closed_media(path, snapshots)
            files["trajectory_plot.json"] = chart_bytes
    # Check every snapshotted source again before publication. No live partial
    # file is included, and an encoding process cannot race an accepted copy.
    for path, value in snapshots.items():
        if path.read_bytes() != value:
            raise ValueError(f"Input changed before package publication: {path}")
    status = ("recorded_validation_pass" if report["passed"] and not report["partial"]
              else "aborted_trial" if report.get("aborted")
              else "bounded_unsuccessful_trial" if report["partial"] else "unsuccessful_trial")
    manifest = {"schema_version": 1, "status": status, "evidence_only": evidence_only,
        "legacy_inline_observer": legacy_inline_observer,
        "recorded_validation_passed": report["passed"], "partial_task_scope": report["partial"],
        "physics_claim_scope": "Packaging preserves recorded outcomes; it does not add physics qualification",
        "packager_source_sha256": sha256(Path(__file__).read_bytes()),
        "source_run_directory": str(run_dir), "trajectory_sha256": loaded["trace_sha256"],
        "sample_count": len(loaded["times"]), "final_recorded_time_s": float(loaded["times"][-1]),
        "identity_checks": loaded["checks"], "recorded_runtime": report["runtime"],
        "independent_audit_file": audit_output_name,
        "audit_source_sha256": source_names,
        "audit_source_archives_verified": auditor_sources_verified,
        "media_trajectory_link_verified": media_manifest is not None,
        "asset_scope": "scene.xml requires repository mesh assets and the verified thread plugin",
        "files": {name: {"sha256": sha256(value), "bytes": len(value)} for name, value in files.items()}}
    target.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{target.name}.package-", dir=target.parent))
    try:
        for name, value in files.items():
            destination = stage / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(value)
        (stage / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        if target.exists():
            raise ValueError("Destination appeared during packaging; refusing to replace it")
        stage.rename(target)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
    return {"package": str(target), "status": status, "files": len(files),
            "trajectory_sha256": loaded["trace_sha256"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("--rendered-media-dir", type=Path)
    parser.add_argument("--evidence-only", action="store_true", help="Permit bounded partial-task evidence and explicitly recorded legacy inline criteria")
    parser.add_argument("--legacy-inline-observer", action="store_true", help="Verify legacy inline capture logic through the archived controller SHA; preserve the actual outcome")
    args = parser.parse_args()
    print(json.dumps(package_run(args.run_dir, args.target,
        rendered_media_dir=args.rendered_media_dir, evidence_only=args.evidence_only,
        legacy_inline_observer=args.legacy_inline_observer), indent=2))


if __name__ == "__main__":
    main()
