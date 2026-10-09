"""Record and verify the engine actually mapped into this process."""
from __future__ import annotations

import ctypes
import hashlib
from pathlib import Path

import mujoco


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def engine_info():
    mapped = set()
    maps = Path("/proc/self/maps")
    if maps.exists():
        for line in maps.read_text().splitlines():
            parts = line.split()
            if len(parts) >= 6 and "/libmujoco.so" in parts[5]:
                mapped.add((parts[5], parts[-1] == "(deleted)"))
    libraries = []
    for path, deleted in sorted(mapped):
        if deleted:
            libraries.append({"path": path, "deleted": True, "sha256": None,
                              "sdf_minimum_search_step_m": None,
                              "sdf_initial_search_step_m": None})
            continue
        entry = {"path": path, "sha256": sha256(path)}
        try:
            marker = ctypes.CDLL(path).astra_sdf_min_alpha
            marker.restype = ctypes.c_double
            marker.argtypes = []
            entry["sdf_minimum_search_step_m"] = marker()
        except AttributeError:
            entry["sdf_minimum_search_step_m"] = None
        try:
            initial = ctypes.CDLL(path).astra_sdf_initial_alpha
            initial.restype = ctypes.c_double
            initial.argtypes = []
            entry["sdf_initial_search_step_m"] = initial()
        except AttributeError:
            entry["sdf_initial_search_step_m"] = None
        libraries.append(entry)
    source = Path(__file__).parent / "plugins" / "m8_sdf.cc"
    return {"mujoco_version": mujoco.__version__, "libraries": libraries,
            "thread_plugin_source_sha256": sha256(source)}


def require_micron_engine():
    info = engine_info()
    libs = info["libraries"]
    if (len(libs) != 1 or libs[0]["sdf_minimum_search_step_m"] != 1e-7
            or libs[0]["sdf_initial_search_step_m"] != .002):
        raise RuntimeError("M8 acceptance requires the verified micron-search engine. "
                           "Run through scripts/run_m8.sh; stock MuJoCo results are diagnostic only.")
    return info
