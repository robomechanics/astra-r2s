#!/usr/bin/env python3
"""Compare the current SDF against its preserved, unoptimized reference.

The default 5,000-point check is quick. Full evidence:
  scripts/run_m8.sh scripts/check_m8_optimization.py --points 500000 \
      --timing-points 300000 --pin-cpu --trace-duration .2

Temporary copies use distinct registration names. The reference fixture itself
is never changed. No forces, contact settings, or geometry are approximated.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import mujoco
import numpy as np


REPO = Path(__file__).resolve().parents[1]
REFERENCE_HASH = "fc1d703d5406b7856d70d6e8ead3c64d3aec1811af58423610ecd167e39989af"
PARAMS = np.array([.008, .00125, .0065, .007268, .013, .000360, 1., 0.])
LABELS = ("reference", "optimized")
BATCH_API = r'''
extern "C" void astra_distance_batch(const double* p,const double* a,double* out,int n) {
  for(int i=0;i<n;i++) out[i]=distance(p+3*i,a);
}
extern "C" void astra_gradient_batch(const double* p,const double* a,double* out,int n) {
  constexpr double e=1e-8;
  for(int i=0;i<n;i++) for(int k=0;k<3;k++) {
    double lo[3]={p[3*i],p[3*i+1],p[3*i+2]}, hi[3]={p[3*i],p[3*i+1],p[3*i+2]};
    lo[k]-=e;hi[k]+=e;
    out[3*i+k]=(distance(hi,a)-distance(lo,a))/(2*e);
  }
}
'''


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def save(path, report):
    path.write_text(json.dumps(report, indent=2) + "\n")


class Kernel:
    def __init__(self, source, label, directory):
        self.label = label
        self.plugin_name = f"astra.m8_thread.check_{label}"
        text = source.replace('p.name="astra.m8_thread"',
                              f'p.name="{self.plugin_name}"') + BATCH_API
        path = directory / f"{label}.cc"
        library = directory / f"{label}.so"
        path.write_text(text)
        package = Path(mujoco.__file__).parent
        core = next(package.glob("libmujoco.so.*"))
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-shared",
                        "-fPIC", "-O3", str(path), "-I" + str(package / "include"),
                        str(core), "-Wl,-rpath," + str(package), "-o", str(library)],
                       check=True)
        self.library = ctypes.CDLL(str(library))
        for name in ("astra_distance_batch", "astra_gradient_batch"):
            method = getattr(self.library, name)
            method.argtypes = [ctypes.POINTER(ctypes.c_double)] * 3 + [ctypes.c_int]
            method.restype = None

    def evaluate(self, points, attrs, *, gradient=False):
        points = np.ascontiguousarray(points, dtype=np.float64)
        attrs = np.ascontiguousarray(attrs, dtype=np.float64)
        output = np.empty((len(points), 3) if gradient else len(points))
        pointer = lambda x: x.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        method = getattr(self.library, "astra_gradient_batch" if gradient
                         else "astra_distance_batch")
        method(pointer(points), pointer(attrs), pointer(output), len(points))
        return output


def populations(total):
    rng = np.random.default_rng(29481)
    weights = np.array([20, 4, 6, 20, 4, 4] + [6] * 7)
    counts = total * weights // 100
    counts[:total - counts.sum()] += 1
    pitch, chamfer = PARAMS[1], PARAMS[5]
    h = np.sqrt(3) * pitch / 2
    major = PARAMS[3] / 2 + 3 * h / 8
    minor = PARAMS[3] / 2 - h / 4
    half = PARAMS[2] / 2
    cases = []
    n = counts[0]
    cases.append(("uniform_volume", rng.uniform([-.009, -.009, -.006],
                                                [.009, .009, .006], (n, 3)), PARAMS))
    n = counts[1]
    axis = rng.uniform([-1e-10, -1e-10, -.006], [1e-10, 1e-10, .006], (n, 3))
    axis[:6, :2] = 0
    cases.append(("axis", axis, PARAMS))
    n = counts[2]
    angles = rng.uniform(-np.pi, np.pi, n)
    radius = rng.choice([minor, major, major + chamfer, .0065, .013 / np.sqrt(3)], n)
    radius += rng.uniform(-3e-8, 3e-8, n)
    z = rng.choice([-half, half, -half + chamfer, half - chamfer, 0], n)
    z += rng.uniform(-3e-8, 3e-8, n)
    # Include the crest/flank/valley transitions in the helical phase.
    selected = np.arange(n) % 2 == 0
    phases = rng.choice([-pitch / 2, -3 * pitch / 8, -pitch / 16,
                        pitch / 16, 3 * pitch / 8, pitch / 2], selected.sum())
    z[selected] = phases + pitch * angles[selected] / (2 * np.pi)
    z[selected] += rng.uniform(-3e-8, 3e-8, selected.sum())
    cases.append(("feature_limits", np.column_stack([radius * np.cos(angles),
                                                    radius * np.sin(angles), z]), PARAMS))
    n = counts[3]
    pads = np.column_stack([rng.choice([-1, 1], n) * rng.uniform(.0058, .008, n),
                            rng.uniform(-.0045, .0045, n), rng.uniform(-.004, .004, n)])
    cases.append(("outer_gripper", pads, PARAMS))
    for index, jitter in ((4, False), (5, True)):
        n = counts[index]
        radius = rng.uniform(major + 1e-8, major + chamfer, n)
        t = (major + chamfer - radius) / (np.sqrt(2) - 1)
        points = np.column_stack([radius, np.zeros(n), half + t])
        if jitter:
            points += rng.uniform(-2e-8, 2e-8, points.shape)
        cases.append(("bound_equality_end" + ("_jitter" if jitter else ""), points, PARAMS))
    for i, n in enumerate(counts[6:12]):
        attrs = PARAMS.copy()
        attrs[1] = rng.uniform(.0008, .002)
        attrs[2] = rng.uniform(.004, .012)
        attrs[3] = rng.uniform(.0068, .0077)
        attrs[4] = rng.uniform(.011, .016)
        attrs[5] = rng.uniform(0, .001)
        attrs[7] = rng.uniform(-.002, .002)
        points = rng.uniform([-.01, -.01, -.01], [.01, .01, .01], (n, 3))
        cases.append((f"parameter_variant_{i}", points, attrs))
    attrs = PARAMS.copy()
    attrs[2], attrs[3], attrs[5], attrs[6] = .030, .007100, .000956, 0.
    points = rng.uniform([-.008, -.008, -.003], [.008, .008, .033], (counts[12], 3))
    cases.append(("male_control", points, attrs))
    return cases


def compare(kernels, total, identity):
    report = dict(identity, points=total, gradient_step_m=1e-8, populations={})
    for name, points, attrs in populations(total):
        entry = {"points": len(points), "params": attrs.tolist()}
        for gradient, mode in ((False, "distance"), (True, "gradient")):
            a, b = [kernel.evaluate(points, attrs, gradient=gradient) for kernel in kernels]
            finite = bool(np.all(np.isfinite(a)) and np.all(np.isfinite(b)))
            entry[mode] = {"bitwise_equal": a.tobytes() == b.tobytes(), "finite": finite,
                           "max_abs_error": float(np.max(np.abs(a - b)))}
        report["populations"][name] = entry
        print(json.dumps({"population": name, "points": len(points),
                          "bitwise_equal": all(entry[x]["bitwise_equal"] for x in ("distance", "gradient"))}), flush=True)
    report["passed"] = all(entry[mode]["bitwise_equal"] and entry[mode]["finite"]
                           for entry in report["populations"].values()
                           for mode in ("distance", "gradient"))
    return report


def timings(kernels, total, identity):
    rng = np.random.default_rng(29481)
    samples = {
        "outer_gripper": np.column_stack([rng.choice([-1, 1], total) * rng.uniform(.0058, .008, total),
                                           rng.uniform(-.0045, .0045, total), rng.uniform(-.004, .004, total)]),
        "uniform_volume": rng.uniform([-.009, -.009, -.006], [.009, .009, .006], (total, 3)),
        "thread_near_surface": rng.uniform([.0032, -.0001, -.003], [.0043, .0001, .003], (total, 3)),
    }
    report = dict(identity, queries_per_batch=total, repeats=9, populations={})
    for name, points in samples.items():
        modes = {}
        for gradient, mode in ((False, "distance"), (True, "gradient")):
            cpu, wall = [[], []], [[], []]
            for repeat in range(9):
                for index in ([0, 1] if repeat % 2 == 0 else [1, 0]):
                    start_wall, start_cpu = time.perf_counter(), time.process_time()
                    kernels[index].evaluate(points, PARAMS, gradient=gradient)
                    cpu[index].append(time.process_time() - start_cpu)
                    wall[index].append(time.perf_counter() - start_wall)
            c = [float(np.median(x)) for x in cpu]
            w = [float(np.median(x)) for x in wall]
            modes[mode] = {"median_cpu_seconds": dict(zip(LABELS, c)), "cpu_speedup": c[0] / c[1],
                           "median_wall_seconds": dict(zip(LABELS, w)), "wall_speedup": w[0] / w[1]}
        report["populations"][name] = modes
        print(json.dumps({"timing_population": name, "modes": modes}), flush=True)
    report["scope"] = "Geometry microbenchmarks; not end-to-end policy-training throughput."
    return report


def contact_trace(kernel, duration):
    from thread_lab.model import ThreadConfig, model_xml
    config = ThreadConfig()
    xml = model_xml(config).replace('plugin="astra.m8_thread"', f'plugin="{kernel.plugin_name}"')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    bid = model.body("nut").id
    mujoco.mj_forward(model, data)
    states, contacts = [], []
    velocity, force = np.zeros(6), np.zeros(6)
    start = time.perf_counter()
    for _ in range(round(duration / config.timestep)):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
        data.xfrc_applied[bid] = [0, 0, 0, 0, 0, -.00005 - .00001 * velocity[2]]
        mujoco.mj_step(model, data)
        states.append(np.concatenate([[data.time, data.ncon], data.qpos, data.qvel,
                                      data.qacc, data.qfrc_constraint, data.energy]))
        digest = hashlib.sha256()
        for i in range(data.ncon):
            contact = data.contact[i]
            mujoco.mj_contactForce(model, data, i, force)
            values = np.concatenate([[contact.dist, contact.dim, contact.efc_address],
                                     contact.geom, contact.pos, contact.frame,
                                     contact.friction, contact.solref, contact.solimp, force])
            digest.update(values.tobytes())
        contacts.append(digest.digest())
    trace = np.asarray(states)
    return trace, b"".join(contacts), time.perf_counter() - start


def trajectories(kernels, duration, identity, output):
    from thread_lab.runtime import require_micron_engine
    engine = require_micron_engine()
    records = []
    for kernel in kernels:
        print(json.dumps({"contact_trace": kernel.label, "duration_s": duration}), flush=True)
        records.append(contact_trace(kernel, duration))
    a, ac, at = records[0]
    b, bc, bt = records[1]
    np.savez_compressed(output / "contact_trace.npz", reference=a, optimized=b)
    equal = a.tobytes() == b.tobytes()
    report = dict(identity, engine=engine, duration_s=duration, physics_steps=len(a),
                  state_columns="time, contact_count, qpos, qvel, qacc, qfrc_constraint, energy",
                  state_bytes_equal=equal, contact_bytes_equal=ac == bc,
                  max_state_abs_error=float(np.max(np.abs(a - b))),
                  state_sha256={"reference": sha256(a.tobytes()), "optimized": sha256(b.tobytes())},
                  contact_sha256={"reference": sha256(ac), "optimized": sha256(bc)},
                  wall_seconds={"reference": at, "optimized": bt},
                  finite=bool(np.all(np.isfinite(a)) and np.all(np.isfinite(b))))
    report["passed"] = equal and ac == bc and report["finite"]
    return report


def cached_probe(path, identity, output):
    from thread_lab.benchmark import probe
    from thread_lab.model import ThreadConfig, model_xml
    from thread_lab.runtime import require_micron_engine
    current_engine = require_micron_engine()
    metadata = json.loads(path.with_suffix(".json").read_text())
    old_engine = metadata["engine"]
    core_identity = lambda x: [{k: v for k, v in lib.items() if k != "path"} for lib in x["libraries"]]
    config = ThreadConfig()
    matched = (metadata["config"] == config.as_dict()
               and metadata["model_xml_sha256"] == sha256(model_xml(config).encode())
               and old_engine["thread_plugin_source_sha256"] == REFERENCE_HASH
               and core_identity(old_engine) == core_identity(current_engine)
               and metadata["applied_torque_Nm"] == -.00005
               and metadata["angular_drag_Nms"] == .00001
               and metadata["axial_force_N"] == 0.)
    if not matched:
        raise RuntimeError("Cached baseline does not match the reference source, engine, model and loads")
    old = np.load(path.with_suffix(".npz"))["trace"]
    print(json.dumps({"cached_probe": str(path), "duration_s": metadata["duration_s"]}), flush=True)
    report, new = probe(config, duration=metadata["duration_s"], output=output / "optimized_probe")
    equal = old.tobytes() == new.tobytes() and old.shape == new.shape and old.dtype == new.dtype
    result = dict(identity, baseline=str(path), baseline_fingerprint_verified=matched,
                  duration_s=metadata["duration_s"], trace_shape=list(new.shape),
                  trace_bytes_equal=equal, max_abs_error=float(np.max(np.abs(old - new))),
                  trace_sha256={"reference": sha256(old.tobytes()), "optimized": sha256(new.tobytes())},
                  wall_seconds={"reference_historical": metadata["wall_seconds"], "optimized": report["wall_seconds"]},
                  timing_note="The historical and current wall times were not measured concurrently; not a throughput comparison.",
                  passed=equal)
    return result


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--points", type=int, default=5000)
    parser.add_argument("--timing-points", type=int, default=0)
    parser.add_argument("--trace-duration", type=float, default=0.)
    parser.add_argument("--cached-baseline", type=Path)
    parser.add_argument("--pin-cpu", action="store_true")
    parser.add_argument("--output", type=Path, default=REPO / "outputs/m8/optimization")
    args = parser.parse_args()
    if args.points < 100 or args.timing_points < 0 or args.trace_duration < 0:
        parser.error("Use at least 100 comparison points and nonnegative timing/trace lengths")
    cpu = None
    if args.pin_cpu:
        cpu = max(os.sched_getaffinity(0))
        os.sched_setaffinity(0, {cpu})
    reference = (REPO / "tests/fixtures/m8_sdf_reference.cc").read_text()
    optimized = (REPO / "thread_lab/plugins/m8_sdf.cc").read_text()
    if sha256(reference.encode()) != REFERENCE_HASH:
        raise RuntimeError("The frozen reference fixture has changed")
    identity = {"reference_source_sha256": REFERENCE_HASH,
                "optimized_source_sha256": sha256(optimized.encode()),
                "mujoco_version": mujoco.__version__, "pinned_cpu": cpu}
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="m8-kernel-check-") as temporary:
        kernels = [Kernel(source, label, Path(temporary))
                   for source, label in zip((reference, optimized), LABELS)]
        report = compare(kernels, args.points, identity)
        save(args.output / "comparison.json", report)
        passed = report["passed"]
        if args.timing_points:
            save(args.output / "timing.json", timings(kernels, args.timing_points, identity))
        if args.trace_duration:
            report = trajectories(kernels, args.trace_duration, identity, args.output)
            save(args.output / "trajectory.json", report)
            passed &= report["passed"]
        if args.cached_baseline:
            report = cached_probe(args.cached_baseline, identity, args.output)
            save(args.output / "cached_probe.json", report)
            passed &= report["passed"]
    print(json.dumps({"passed": bool(passed), "output": str(args.output)}), flush=True)
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
