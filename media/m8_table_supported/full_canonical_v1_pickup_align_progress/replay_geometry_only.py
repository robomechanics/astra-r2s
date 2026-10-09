"""Verify or replay this progress package without integration or force solves."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parent


def digest(value):
    return hashlib.sha256(value).hexdigest()


def repository_root():
    return next(parent for parent in PACKAGE.parents
                if (parent / "thread_lab/runtime.py").is_file())


def portable_model_bytes():
    xml = (PACKAGE / "scene.xml").read_bytes()
    root = ET.fromstring(xml)
    assets, hashes = {}, {}
    for mesh in root.findall("asset/mesh"):
        filename = mesh.get("file")
        if filename:
            name = Path(filename).name
            value = (PACKAGE / "portable_assets" / name).read_bytes()
            if name in hashes and hashes[name] != digest(value):
                raise ValueError("Mesh basename collision")
            assets[name], hashes[name] = value, digest(value)
            mesh.set("file", name)
    normalized = ET.tostring(root, encoding="unicode")
    fingerprint = digest(json.dumps({"xml": normalized, "mesh_sha256": hashes},
        sort_keys=True, separators=(",", ":")).encode())
    return xml, normalized, assets, hashes, fingerprint


def verify_package():
    sums = PACKAGE / "SHA256SUMS"
    checked = 0
    for line in sums.read_text().splitlines():
        expected, relative = line.split("  ", 1)
        path = PACKAGE / relative
        if not path.resolve().is_relative_to(PACKAGE):
            raise ValueError("Checksum path escapes package")
        if digest(path.read_bytes()) != expected:
            raise ValueError("Package bytes changed: " + relative)
        checked += 1
    declaration = json.loads((PACKAGE / "progress_snapshot.json").read_text())
    identity = json.loads((PACKAGE / "run_publication_identity.json").read_text())
    proof = json.loads((PACKAGE / "original_software_proof/software_tests.json").read_text())
    if identity["source_hashes_before"] != proof["source_hashes"]:
        raise ValueError("Software proof differs from producer65 source binding")
    for relative, expected in identity["source_hashes_before"].items():
        if digest((PACKAGE / "producer_sources" / relative).read_bytes()) != expected:
            raise ValueError("Archived producer source changed: " + relative)
    xml, _, _, _, fingerprint = portable_model_bytes()
    if digest(xml) != declaration["model_xml_sha256"] or fingerprint != declaration["model_fingerprint"]:
        raise ValueError("Portable XML/mesh identity differs from original recording")
    if digest((PACKAGE / "insertion_trace_partial.npz").read_bytes()) != declaration["snapshot_trajectory_sha256"]:
        raise ValueError("Original immutable snapshot differs")
    return {"checked_package_files": checked,
        "archived_producer_sources_matched": len(identity["source_hashes_before"]),
        "producer_commit": identity["producer_commit"],
        "model_fingerprint": fingerprint, "scope": declaration["scope"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    verification = verify_package()
    if args.verify_only:
        print(json.dumps(verification, indent=2))
        return
    if args.output is None or args.output.resolve().is_relative_to(PACKAGE):
        parser.error("Choose --output outside the immutable progress package")
    repo = repository_root()
    sys.path.insert(0, str(repo))
    identity = json.loads((PACKAGE / "run_publication_identity.json").read_text())
    for relative, expected in identity["source_hashes_before"].items():
        if digest((repo / relative).read_bytes()) != expected:
            raise ValueError("Use the matching producer source: " + relative)
    os.environ["MUJOCO_GL"] = "egl"
    os.environ["LP_NUM_THREADS"] = "2"
    import mujoco
    from thread_lab.runtime import require_micron_engine
    from yam_twin import m8_supported_demo as demo
    from yam_twin import m8_scene
    from scripts import audit_m8_insertion_trace as audit
    expected_runtime = json.loads((PACKAGE / "exact_recorded_config.json").read_text())["runtime"]
    actual_runtime = require_micron_engine()
    if (actual_runtime["mujoco_version"] != expected_runtime["mujoco_version"] or
            [item["sha256"] for item in actual_runtime["libraries"]] !=
            [item["sha256"] for item in expected_runtime["libraries"]]):
        raise ValueError("Replay requires the recorded matched MuJoCo runtime")
    if digest((repo / "thread_lab/plugins/m8_sdf.cc").read_bytes()) != expected_runtime["thread_plugin_source_sha256"]:
        raise ValueError("Replay requires the recorded geometry plugin source")

    def recorded_portable_model(path, metadata):
        xml, normalized, assets, hashes, fingerprint = portable_model_bytes()
        if digest(xml) != metadata["model_xml_sha256"] or fingerprint != metadata["model_fingerprint"]:
            raise ValueError("Saved-state model identity mismatch")
        if not m8_scene._loaded:
            mujoco.mj_loadPluginLibrary(str(m8_scene.build_plugin()))
            m8_scene._loaded = True
        model = mujoco.MjSpec.from_string(normalized, assets=assets).compile()
        return model, {"model_xml_sha256": digest(xml), "model_fingerprint": fingerprint,
            "mesh_sha256": hashes, "method": "Original XML bytes checked; only mesh paths normalized in memory using exact packaged asset bytes"}

    def geometry_refresh(model, data):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_camlight(model, data)

    args.output.mkdir(parents=True, exist_ok=True)
    original_forward, original_model = demo.mujoco.mj_forward, audit.recorded_model
    try:
        demo.mujoco.mj_forward = geometry_refresh
        audit.recorded_model = recorded_portable_model
        result = demo.render_recording(PACKAGE / "insertion_trace_partial.npz",
            args.output / "pickup_align_progress.mp4", fps=12, slow_motion=1.)
    finally:
        demo.mujoco.mj_forward, audit.recorded_model = original_forward, original_model
    result.update({"package_verification": verification,
        "refresh": ["mj_kinematics", "mj_comPos", "mj_camlight"],
        "integration_or_collision_or_force_solve_called": False,
        "canonical_renderer_cameras_and_visibility_unchanged": True,
        "overlay_scope": "Archived original solved samples at saved time minus one timestep",
        "replay_helper_sha256": digest(Path(__file__).read_bytes()),
        "scope": verification["scope"]})
    (args.output / "portable_render_manifest.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
