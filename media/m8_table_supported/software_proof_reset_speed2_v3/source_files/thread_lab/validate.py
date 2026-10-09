"""Acceptance checks for free thread contact, including numerical convergence."""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np

from .benchmark import probe
from .model import ThreadConfig, model_xml
from .runtime import require_micron_engine


def _engine_identity(info):
    """File locations do not change the physics of byte-identical cores."""
    return {"mujoco_version": info["mujoco_version"],
            "thread_plugin_source_sha256": info["thread_plugin_source_sha256"],
            "libraries": [{k:v for k,v in lib.items() if k != "path"}
                          for lib in info["libraries"]]}


def validate(output, *, reuse=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    engine = require_micron_engine()
    baseline = ThreadConfig()
    cases = {}
    traces = {}
    specs = [
        ("nominal", baseline, -.00005, 1e-5),
        ("reverse", baseline, .0001, 1e-5),
        ("backdrive", replace(baseline, friction=0), 0., 1e-5),
        ("self_lock", baseline, 0., 0.),
        ("no_contact", replace(baseline, with_bolt=False, gravity=0), -.00005, 1e-5),
        ("coarse", replace(baseline, timestep=.0001), -.00005, 1e-5),
        ("fine", replace(baseline, timestep=.000025), -.00005, 1e-5),
        ("seeds80", replace(baseline, sdf_initpoints=80), -.00005, 1e-5),
    ]
    for name, c, torque, drag in specs:
        path = output / name
        cached = None
        if reuse and path.with_suffix(".json").exists() and path.with_suffix(".npz").exists():
            cached = json.loads(path.with_suffix(".json").read_text())
            fingerprint = hashlib.sha256(model_xml(c).encode()).hexdigest()
            if not (cached.get("model_xml_sha256") == fingerprint
                    and _engine_identity(cached["engine"]) == _engine_identity(engine)
                    and cached.get("duration_s") == .6
                    and cached.get("applied_torque_Nm") == torque
                    and cached.get("angular_drag_Nms") == drag):
                cached = None
        if cached is None:
            r, trace = probe(c, duration=.6, torque=torque, angular_drag=drag, output=path)
        else:
            r = cached
            trace = np.load(path.with_suffix(".npz"))["trace"]
        cases[name], traces[name] = r, trace
        print(json.dumps({"case": name, "lead_mm": r["fitted_lead_mm"],
                          "reported_depth_um": r["worst_reported_sdf_depth_um"]}), flush=True)
    checks = {}
    for name in ("nominal", "reverse", "backdrive", "coarse", "fine", "seeds80"):
        checks[name+"_lead"] = cases[name]["lead_error_percent"] is not None and cases[name]["lead_error_percent"] < 2
    checks["torque_direction"] = cases["nominal"]["turns"] < -.1 and cases["reverse"]["turns"] > .1
    checks["backdrive_gravity"] = cases["backdrive"]["axial_travel_mm"] < -.04
    checks["backdrive_energy"] = cases["backdrive"]["rigid_body_energy_change_J"] < 0 and cases["backdrive"]["external_work_J"] <= 0
    hold = traces["self_lock"]
    late = hold[:, 0] > .4
    checks["self_lock_late_drift_below_1um"] = float(np.ptp(hold[late, 3])) < 1e-6
    checks["self_lock_late_rotation_below_0_005rad"] = float(np.ptp(np.unwrap(hold[late, 4]))) < .005
    checks["no_contact_no_feed"] = abs(cases["no_contact"]["axial_travel_mm"]) < 1e-9 and abs(cases["no_contact"]["turns"]) > .1
    travels = [cases[n]["axial_travel_mm"] for n in ("coarse", "nominal", "fine")]
    variation = float(np.ptp(travels) / abs(travels[1]))
    seed_variation = abs(cases["seeds80"]["axial_travel_mm"] / cases["nominal"]["axial_travel_mm"] - 1)
    checks["timestep_travel_variation_below_2pct"] = variation < .02
    checks["search_travel_variation_below_2pct"] = seed_variation < .02
    checks["reported_sdf_depth_below_10um"] = all(r["worst_reported_sdf_depth_um"] < 10 for r in cases.values())
    checks["finite_without_warnings"] = all(r["finite"] and not r["warnings"] for r in cases.values())
    checks = {key: bool(value) for key, value in checks.items()}
    report = dict(passed=all(checks.values()), checks=checks, cases=cases,
                  timestep_travel_variation_percent=100*variation,
                  search_travel_variation_percent=100*seed_variation,
                  scope="Ideal world-wrench probes of a free pre-engaged nut; physical gripping is validated separately.",
                  depth_note="Reported SDF intersection depth, not independent geometric overlap. Flat-face overlap is approximately 2x this metric.")
    (output / "validation.json").write_text(json.dumps(report, indent=2)+"\n")
    return report


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--output", type=Path, default=Path("outputs/m8/acceptance"))
    p.add_argument("--reuse", action="store_true")
    a = p.parse_args()
    r = validate(a.output, reuse=a.reuse)
    print(json.dumps({"passed": r["passed"], "checks": r["checks"]}, indent=2))
    raise SystemExit(0 if r["passed"] else 1)


if __name__ == "__main__":
    main()
