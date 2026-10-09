"""Verify the built Python extensions and core as one recorded ABI pair."""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import tempfile
import zipfile


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--expected-core", type=Path)
    parser.add_argument("--fingerprint")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    previous = None
    if not args.write:
        previous = json.loads(args.record.read_text())
        if args.fingerprint and previous.get("build_fingerprint") != args.fingerprint:
            raise RuntimeError("The cached build fingerprint has changed.")

    import mujoco

    package = Path(mujoco.__file__).resolve().parent
    core_path = package / "libmujoco.so.3.15.0"
    core_hash = digest(core_path)
    extensions = {str(p.relative_to(package)): digest(p)
                  for p in sorted(package.rglob("*.so"))}
    if previous is not None:
        if previous.get("library_sha256") != core_hash:
            raise RuntimeError("The installed core does not match the verified build.")
        if previous.get("extension_sha256") != extensions:
            raise RuntimeError("The installed Python extensions do not match the verified build.")
    if args.expected_core and digest(args.expected_core) != core_hash:
        raise RuntimeError("The bindings package and requested core have different hashes.")
    if mujoco.__version__ != "3.15.0":
        raise RuntimeError("M8 experiments require the pinned MuJoCo 3.15.0 bindings.")

    core = ctypes.CDLL(str(core_path))
    floor = core.astra_sdf_min_alpha
    initial = core.astra_sdf_initial_alpha
    for function in (floor, initial):
        function.restype = ctypes.c_double
        function.argtypes = []
    assert floor() == 1e-7
    assert initial() == .002
    mapped = {line.split()[-1] for line in Path("/proc/self/maps").read_text().splitlines()
              if "/libmujoco.so" in line}
    assert mapped == {str(core_path.resolve())}, mapped

    # Exercise the C++ string-bearing MjSpec API, where mixed compiler ABIs
    # previously corrupted names despite ordinary mj_step calls working.
    spec = mujoco.MjSpec.from_string(
        '<mujoco model="m8_matched_abi"><worldbody><body name="nut">'
        '<freejoint/><geom name="nut_geom" type="sphere" size=".004" mass=".005"/>'
        '</body></worldbody></mujoco>')
    assert spec.modelname == "m8_matched_abi", repr(spec.modelname)
    spec.modelname = "m8_matched_abi_roundtrip"
    assert spec.modelname == "m8_matched_abi_roundtrip"
    spec.worldbody.add_body(name="new_body").add_geom(
        name="new_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[.003, 0, 0])
    with tempfile.TemporaryDirectory() as directory:
        archive = Path(directory) / "abi.zip"
        spec.to_zip(archive)
        with zipfile.ZipFile(archive) as zipped:
            assert any(name.endswith(".xml") and "m8_matched_abi_roundtrip" in name
                       for name in zipped.namelist())
        restored = mujoco.MjSpec.from_zip(archive)
        assert restored.modelname == "m8_matched_abi_roundtrip"
        assert restored.compile().nbody == spec.compile().nbody

    report = {"version": mujoco.__version__, "package": str(package),
              "library": str(core_path), "library_sha256": core_hash,
              "extension_sha256": extensions,
              "mapped_mujoco_libraries": sorted(mapped),
              "sdf_min_alpha": floor(), "sdf_initial_alpha": initial(),
              "spec_modelname_and_zip_roundtrip": "passed"}
    if args.fingerprint:
        report["build_fingerprint"] = args.fingerprint
    elif previous is not None and "build_fingerprint" in previous:
        report["build_fingerprint"] = previous["build_fingerprint"]
    if args.write:
        args.record.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key != "extension_sha256"}, indent=2))


if __name__ == "__main__":
    main()
