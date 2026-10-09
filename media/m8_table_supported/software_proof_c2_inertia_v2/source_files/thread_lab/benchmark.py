"""Contact-only torque probes with machine-readable physics diagnostics."""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .model import ThreadConfig, make_model, model_xml
from .runtime import engine_info
import hashlib


def probe(config=None, *, duration=1., torque=-.00005, axial_force=0.,
          angular_drag=.00001, output=None):
    """Apply a world wrench to the nut as a unit-test load.

    This is an ideal wrench benchmark, separate from the hand policy task.
    It never writes position or derives an axial command from rotation.
    """
    c = config or ThreadConfig()
    model = make_model(c)
    data = mujoco.MjData(model)
    bid = model.body("nut").id
    mujoco.mj_forward(model, data)
    runtime = engine_info()
    initial_energy = float(sum(data.energy))
    initial_z = float(data.xpos[bid, 2])
    records = []
    velocity = np.zeros(6)
    force = np.zeros(6)
    work = 0.
    min_distance = 0.
    max_normal = 0.
    st = time.monotonic()
    stride = max(1, round(.001 / c.timestep))
    for k in range(round(duration / c.timestep)):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
        actual_torque = torque - angular_drag * velocity[2]
        data.xfrc_applied[bid] = [0, 0, axial_force, 0, 0, actual_torque]
        work += (axial_force * velocity[5] + actual_torque * velocity[2]) * c.timestep
        mujoco.mj_step(model, data)
        for i in range(data.ncon):
            min_distance = min(min_distance, float(data.contact[i].dist))
            mujoco.mj_contactForce(model, data, i, force)
            max_normal = max(max_normal, float(force[0]))
        if k % stride == 0:
            r = data.xmat[bid].reshape(3, 3)
            yaw = np.arctan2(r[1, 0], r[0, 0])
            tilt = np.arccos(np.clip(r[2, 2], -1, 1))
            records.append([data.time, *data.xpos[bid], yaw, tilt, data.ncon,
                            min_distance, work, *velocity, actual_torque, *data.energy])
    trace = np.asarray(records)
    yaw = np.unwrap(trace[:, 4])
    steady = trace[:, 0] >= min(.1, duration / 4)
    fitted = float(np.polyfit(yaw[steady], trace[steady, 3], 1)[0] * 2*np.pi) if np.ptp(yaw[steady]) > .15 else None
    residual = trace[:, 3] - initial_z - c.pitch * (yaw - yaw[0]) / (2*np.pi)
    warnings = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    result = dict(config=c.as_dict(), engine=runtime,
                  model_xml_sha256=hashlib.sha256(model_xml(c).encode()).hexdigest(),
                  duration_s=duration, applied_torque_Nm=torque,
                  axial_force_N=axial_force, angular_drag_Nms=angular_drag,
                  wall_seconds=time.monotonic()-st, turns=float((yaw[-1]-yaw[0])/(2*np.pi)),
                  axial_travel_mm=float((trace[-1, 3]-initial_z)*1000),
                  fitted_lead_mm=None if fitted is None else fitted*1000,
                  lead_error_percent=None if fitted is None else abs(fitted/c.pitch-1)*100,
                  max_helix_residual_mm=float(np.max(np.abs(residual))*1000),
                  worst_penetration_um=-min_distance*1e6, max_contact_normal_N=max_normal,
                  worst_reported_sdf_depth_um=-min_distance*1e6,
                  sdf_depth_note="Reported SDF intersection depth; flat-face geometric overlap is approximately twice this value. Not an independent overlap bound.",
                  max_tilt_deg=float(np.max(trace[:, 5])*180/np.pi),
                  external_work_J=work, warnings=warnings,
                  rigid_body_energy_change_J=float(sum(data.energy))-initial_energy,
                  energy_note="Rigid-body kinetic and gravitational potential only; soft-contact energy is excluded.",
                  finite=bool(np.all(np.isfinite(data.qpos))))
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.with_suffix(".json").write_text(json.dumps(result, indent=2)+"\n")
        np.savez_compressed(path.with_suffix(".npz"), trace=trace, yaw=yaw)
    return result, trace


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--duration", type=float, default=1.)
    p.add_argument("--torque", type=float, default=-.00005)
    p.add_argument("--load", type=float, default=0.)
    p.add_argument("--friction", type=float, default=.15)
    p.add_argument("--dt", type=float, default=.00005)
    p.add_argument("--points", type=int, default=40)
    p.add_argument("--guided", action="store_true")
    p.add_argument("--no-bolt", action="store_true")
    p.add_argument("--output", default="outputs/m8/probe")
    args = p.parse_args()
    c = ThreadConfig(timestep=args.dt, friction=args.friction, sdf_initpoints=args.points,
                     guided=args.guided, with_bolt=not args.no_bolt)
    if args.no_bolt:
        c = replace(c, gravity=0)
    result, _ = probe(c, duration=args.duration, torque=args.torque, axial_force=args.load, output=args.output)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
