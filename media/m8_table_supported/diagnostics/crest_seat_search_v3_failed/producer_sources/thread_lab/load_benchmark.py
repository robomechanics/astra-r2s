"""Independent torque/load and self-locking experiments for the M8 thread.

The nut has independent axial-slide and yaw DOFs, representing an aligned
bearing fixture. Only yaw is servoed; a prescribed constant axial force is
applied without any axial position or velocity control. The measured contact
reaction must create any observed thread feed. This is not a robotic grasp
or an assembly policy demonstration.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from .model import ThreadConfig, make_model
from .runtime import engine_info
from .theory import (
    contact_pitch_diameter,
    is_self_locking,
    lowering_torque,
    raising_torque,
    self_locking_threshold,
)


TRACE_FIELDS = (
    "time_s", "z_m", "yaw_rad", "vz_ms", "yaw_velocity_rads",
    "target_yaw_velocity_rads", "applied_torque_Nm", "axial_force_N",
    "contact_force_z_N", "contact_torque_z_Nm", "contact_count",
    "minimum_contact_distance_m", "max_contact_normal_N",
    "kinetic_energy_J", "motor_work_J", "axial_load_work_J",
)


def _setup(config, friction_impedance_ratio):
    config = config or ThreadConfig()
    if not math.isfinite(friction_impedance_ratio) or friction_impedance_ratio < 1:
        raise ValueError("Friction impedance ratio must be finite and at least one")
    c = replace(config, guided=True, with_gripper=False, gravity=0,
                contact_impratio=friction_impedance_ratio)
    model = make_model(c)
    if model.nv != 2 or model.nu or model.neq or np.any(model.dof_frictionloss):
        raise RuntimeError("Load fixture must have two independent passive DOFs, without hidden drives or coupling")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = model.body("nut").id
    geom = model.geom("nut_thread").id
    iz = int(model.joint("nut_z").qposadr[0])
    iyaw = int(model.joint("nut_yaw").qposadr[0])
    vz = int(model.joint("nut_z").dofadr[0])
    wyaw = int(model.joint("nut_yaw").dofadr[0])
    return c, model, data, body, geom, iz, iyaw, vz, wyaw


def _contact_wrench(model, data, body, nut_geom, velocities=None):
    """Sum physical contact forces/moments on the nut in world coordinates."""
    wrench = np.zeros(6)
    local = np.zeros(6)
    minimum, normal, count = 0.0, 0.0, 0
    diagnostics = np.zeros(10)
    for i in range(data.ncon):
        contact = data.contact[i]
        if nut_geom not in (contact.geom1, contact.geom2):
            continue
        count += 1
        mujoco.mj_contactForce(model, data, i, local)
        sign = 1 if contact.geom2 == nut_geom else -1
        frame = np.asarray(contact.frame).reshape(3, 3)
        force = sign * (frame.T @ local[:3])
        moment = sign * (frame.T @ local[3:])
        wrench[:3] += force
        wrench[3:] += np.cross(contact.pos - data.xipos[body], force) + moment
        minimum = min(minimum, float(contact.dist))
        normal = max(normal, float(local[0]))
        if velocities is not None and local[0] > 1e-10:
            axial_speed, yaw_speed, next_axial_speed, next_yaw_speed = velocities
            lever = contact.pos-data.xipos[body]
            point_velocity = np.array([-yaw_speed*lever[1], yaw_speed*lever[0], axial_speed])
            relative_velocity = sign*(frame @ point_velocity)
            next_point_velocity = np.array([-next_yaw_speed*lever[1], next_yaw_speed*lever[0], next_axial_speed])
            next_relative_velocity = sign*(frame @ next_point_velocity)
            fn = float(local[0])
            mu1, mu2 = contact.friction[:2]
            cone_ratio = math.hypot(local[1]/(mu1*fn), local[2]/(mu2*fn)) if min(mu1,mu2)>0 else 0.
            diagnostics += [fn, fn*cone_ratio, fn*abs(frame[0,2]),
                            fn*np.linalg.norm(relative_velocity[1:]),
                            fn*abs(relative_velocity[0]),
                            float(local[1:3] @ relative_velocity[1:]),
                            fn*math.hypot(lever[0],lever[1]),
                            float(local[1:3] @ next_relative_velocity[1:]),
                            fn*np.linalg.norm(next_relative_velocity[1:]),
                            fn*abs(next_relative_velocity[0])]
    return wrench, minimum, normal, count, diagnostics


def _save(output, report, trace):
    if output is None:
        return
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    Path(str(path)+".json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(Path(str(path)+".npz"), trace=trace,
                        fields=np.asarray(TRACE_FIELDS))


def velocity_probe(config=None, *, axial_load=1.0, yaw_velocity=0.2,
                   duration=1.0, preload_seconds=0.10, ramp_seconds=0.10,
                   load_ramp_seconds=0.05, torque_limit=0.03,
                   kp=0.001, ki=0.3, friction_impedance_ratio=1.0, output=None):
    """Measure torque required for a finite yaw-speed servo under axial load.

    Positive yaw raises against a downward load; negative yaw lowers with it.
    The PI controller receives actual yaw speed only. It has no thread-pitch
    input, analytical feedforward, axial reference, or coordinate overwrites.
    ``preload_seconds`` commands zero yaw speed while clearance is taken up;
    the finite holding torque in that stage must not be called self-locking.
    """
    if (not math.isfinite(axial_load) or axial_load <= 0
            or not math.isfinite(yaw_velocity) or yaw_velocity == 0
            or duration <= preload_seconds + ramp_seconds
            or min(preload_seconds, ramp_seconds, load_ramp_seconds) < 0
            or load_ramp_seconds > preload_seconds
            or min(torque_limit, kp, ki) <= 0):
        raise ValueError("Positive load, timing and gains; nonzero finite yaw speed required")
    c, model, data, body, nut_geom, iz, iyaw, vz, wyaw = _setup(config, friction_impedance_ratio)
    # Identify the loaded core while its file is still available. Reinstalling
    # an environment during a live experiment can unlink that mapped inode.
    engine_snapshot = engine_info()
    if kp * c.timestep / c.nut_axial_inertia >= 1:
        raise ValueError("Yaw speed gain is too stiff for the explicit servo timestep")
    integral = 0.0
    motor_work = load_work = 0.0
    records = []
    minimum = max_normal = 0.0
    saturation_steps = 0
    stride = max(1, round(0.001 / c.timestep))
    steady_start = max(preload_seconds + ramp_seconds, duration*0.65)
    steady_values = np.zeros(5)
    steady_samples = 0
    steady_speed_square = 0.0
    contact_diagnostics = np.zeros(10)
    start = time.monotonic()
    for step in range(round(duration / c.timestep)):
        t = float(data.time)
        fraction = np.clip((t-preload_seconds) / max(ramp_seconds, c.timestep), 0, 1)
        target = yaw_velocity * float(fraction)
        applied_load = axial_load * min(1., t/max(load_ramp_seconds, c.timestep))
        speed = float(data.qvel[wyaw])
        error = target - speed
        candidate = integral + error * c.timestep
        raw = kp * error + ki * candidate
        torque = float(np.clip(raw, -torque_limit, torque_limit))
        # Conditional integration avoids accumulating error while torque-limited.
        if abs(raw) < torque_limit or raw * error < 0:
            integral = candidate
        if abs(raw) >= torque_limit:
            saturation_steps += 1
        z_before = c.initial_z + float(data.qpos[iz])
        yaw_before = float(data.qpos[iyaw])
        v_before = float(data.qvel[vz])
        energy = 0.5*c.nut_mass*v_before*v_before + 0.5*c.nut_axial_inertia*speed*speed
        data.xfrc_applied[body] = [0, 0, -applied_load, 0, 0, torque]
        mujoco.mj_step(model, data)
        # Discrete work uses the actual displacement, not commanded velocity.
        motor_work += torque * (float(data.qpos[iyaw]) - yaw_before)
        load_work -= applied_load * (c.initial_z + float(data.qpos[iz]) - z_before)
        wrench, distance, normal, count, diagnostics = _contact_wrench(
            model, data, body, nut_geom,
            velocities=(v_before,speed,float(data.qvel[vz]),float(data.qvel[wyaw]))
                if t>=steady_start else None)
        minimum = min(minimum, distance)
        max_normal = max(max_normal, normal)
        if t >= steady_start:
            # Accumulate at every physics step. A downsampled trace can alias
            # contact chatter and bias mean force, torque, or speed.
            steady_values += [torque, speed, wrench[2], wrench[5], count]
            steady_speed_square += speed*speed
            steady_samples += 1
            contact_diagnostics += diagnostics
        if step % stride == 0:
            records.append([t, z_before, yaw_before, v_before, speed, target,
                            torque, -applied_load, wrench[2], wrench[5], count,
                            distance, normal, energy, motor_work, load_work])
    trace = np.asarray(records)
    steady = trace[:, 0] >= steady_start
    selected = trace[steady]
    angular_travel = float(np.ptp(selected[:, 2]))
    fitted = None
    if angular_travel >= 0.02:
        fitted = float(np.polyfit(selected[:, 2], selected[:, 1], 1)[0]*2*math.pi)
    actual_torque, observed_speed, mean_fz, mean_tz, mean_count = steady_values/steady_samples
    actual_torque, observed_speed = float(actual_torque), float(observed_speed)
    diameter = contact_pitch_diameter(c.male_pitch_diameter, c.female_pitch_diameter)
    options = dict(pitch=c.pitch, pitch_diameter=diameter)
    prediction = (raising_torque(axial_load, c.friction, **options) if yaw_velocity > 0
                  else -lowering_torque(axial_load, c.friction, **options))
    # The actual flank lever arm spans a finite radial interval. These bounds
    # are computed from the declared solid geometry, independently of forces.
    h = math.sqrt(3)*c.pitch/2
    radius_bounds = (c.female_pitch_diameter/2-h/4,
                     c.male_pitch_diameter/2+3*h/8)
    torque_bounds = []
    for radius in radius_bounds:
        kwargs = dict(pitch=c.pitch, pitch_diameter=2*radius)
        torque_bounds.append(raising_torque(axial_load,c.friction,**kwargs)
                            if yaw_velocity>0 else -lowering_torque(axial_load,c.friction,**kwargs))
    torque_bounds.sort()
    # Near the neutral backdrive threshold, relative torque error is ill-conditioned.
    torque_error = abs(actual_torque-prediction)
    torque_tolerance = max(0.05*abs(prediction), 0.000005)
    speed_error = abs(observed_speed-yaw_velocity)/abs(yaw_velocity)
    lead_error = None if fitted is None else abs(fitted/c.pitch-1)
    force_balance = float(mean_fz)-axial_load
    torque_balance = float(mean_tz)+actual_torque
    speed_std = math.sqrt(max(0., steady_speed_square/steady_samples-observed_speed**2))
    final_kinetic = (0.5*c.nut_mass*float(data.qvel[vz])**2
                     + 0.5*c.nut_axial_inertia*float(data.qvel[wyaw])**2)
    warnings = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    checks = dict(
        finite=bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(trace))),
        no_solver_warnings=not warnings,
        yaw_speed_within_5_percent=speed_error <= 0.05,
        yaw_speed_std_below_10_percent=speed_std <= 0.10*abs(yaw_velocity),
        sufficient_steady_angular_travel=angular_travel >= 0.02,
        contact_lead_within_2_percent=lead_error is not None and lead_error <= 0.02,
        torque_matches_theory=torque_error <= torque_tolerance,
        torque_within_fixed_geometry_bounds=torque_bounds[0]-.000005 <= actual_torque <= torque_bounds[1]+.000005,
        force_balance_within_2_percent=abs(force_balance) <= max(.02*axial_load, .001),
        torque_balance=abs(torque_balance) <= max(.02*abs(actual_torque), .000001),
        penetration_below_10_um=-minimum <= .000010,
    )
    report = dict(
        experiment="aligned independent slide/yaw, torque-only yaw-speed servo",
        note="Short steady window is a load diagnostic, not a several-revolution pitch certification.",
        config=c.as_dict(), engine=engine_snapshot, mode="raise" if yaw_velocity > 0 else "lower",
        friction_impedance_ratio=friction_impedance_ratio,
        duration_s=duration, preload_seconds=preload_seconds, ramp_seconds=ramp_seconds,
        load_ramp_seconds=load_ramp_seconds, steady_statistics_physics_steps=steady_samples,
        yaw_velocity_target_rads=yaw_velocity, axial_load_N=axial_load,
        controller=dict(kp_Nms=kp, ki_Nm=ki, torque_limit_Nm=torque_limit,
                        saturation_fraction=saturation_steps/round(duration/c.timestep)),
        analytical_contact_pitch_diameter_m=diameter,
        analytical_flank_contact_radius_range_m=list(radius_bounds),
        analytical_torque_geometry_range_Nm=torque_bounds,
        predicted_torque_Nm=prediction, measured_steady_torque_Nm=actual_torque,
        torque_error_Nm=torque_error, torque_tolerance_Nm=torque_tolerance,
        steady_yaw_velocity_rads=observed_speed, steady_yaw_speed_std_rads=speed_std,
        steady_angular_travel_rad=angular_travel,
        measured_lead_mm=None if fitted is None else fitted*1000,
        lead_error_percent=None if lead_error is None else lead_error*100,
        steady_contact_force_z_N=float(mean_fz), steady_contact_count=float(mean_count),
        steady_contact_torque_z_Nm=float(mean_tz),
        steady_contact_diagnostics=dict(
            normal_force_weighted_cone_fraction=float(contact_diagnostics[1]/max(contact_diagnostics[0],1e-30)),
            normal_force_weighted_normal_axial_component=float(contact_diagnostics[2]/max(contact_diagnostics[0],1e-30)),
            normal_force_weighted_tangential_slip_ms=float(contact_diagnostics[3]/max(contact_diagnostics[0],1e-30)),
            normal_force_weighted_normal_speed_ms=float(contact_diagnostics[4]/max(contact_diagnostics[0],1e-30)),
            mean_contact_friction_power_W=float(contact_diagnostics[5]/steady_samples),
            friction_power_note="First power/slip values pair the impulse with pre-step velocity; compare post-step values for the semi-implicit solve.",
            normal_force_weighted_contact_radius_m=float(contact_diagnostics[6]/max(contact_diagnostics[0],1e-30)),
            mean_post_step_contact_friction_power_W=float(contact_diagnostics[7]/steady_samples),
            normal_force_weighted_post_step_tangential_slip_ms=float(contact_diagnostics[8]/max(contact_diagnostics[0],1e-30)),
            normal_force_weighted_post_step_normal_speed_ms=float(contact_diagnostics[9]/max(contact_diagnostics[0],1e-30))),
        axial_force_balance_error_N=force_balance, torque_balance_error_Nm=torque_balance,
        worst_penetration_um=-minimum*1e6, max_contact_normal_N=max_normal,
        motor_work_J=motor_work, axial_load_work_J=load_work,
        final_kinetic_energy_J=final_kinetic,
        net_external_work_minus_kinetic_J=motor_work+load_work-final_kinetic,
        energy_note="Contact compliance stores energy; residual is not a measured friction dissipation.",
        trace_timing_note="Trace time, pose, velocity and kinetic energy precede the step; contact wrench and cumulative work include that step. Report work totals and final kinetic energy are terminal values.",
        checks=checks, accepted=bool(all(checks.values())), warnings=warnings,
        wall_seconds=time.monotonic()-start,
    )
    _save(output, report, trace)
    return report, trace


def self_lock_probe(config=None, *, axial_load=1.0, duration=0.08,
                    load_ramp_seconds=0.02, friction_impedance_ratio=1.0, output=None):
    """Apply axial load with zero applied yaw torque and no yaw servo."""
    if axial_load <= 0 or duration <= load_ramp_seconds or load_ramp_seconds < 0:
        raise ValueError("Positive axial load and duration required")
    c, model, data, body, nut_geom, iz, iyaw, vz, wyaw = _setup(config, friction_impedance_ratio)
    engine_snapshot = engine_info()
    records = []
    work = 0.0
    minimum = max_normal = 0.0
    stride = max(1, round(.001/c.timestep))
    start = time.monotonic()
    for step in range(round(duration/c.timestep)):
        z_before = c.initial_z + float(data.qpos[iz])
        yaw = float(data.qpos[iyaw])
        axial_speed = float(data.qvel[vz])
        yaw_speed = float(data.qvel[wyaw])
        energy = .5*c.nut_mass*axial_speed**2 + .5*c.nut_axial_inertia*yaw_speed**2
        applied_load = axial_load * min(1., float(data.time)/max(load_ramp_seconds,c.timestep))
        data.xfrc_applied[body] = [0, 0, -applied_load, 0, 0, 0]
        mujoco.mj_step(model, data)
        work -= applied_load*(c.initial_z+float(data.qpos[iz])-z_before)
        wrench, distance, normal, count, _ = _contact_wrench(model, data, body, nut_geom)
        minimum = min(minimum, distance)
        max_normal = max(max_normal, normal)
        if step % stride == 0:
            records.append([float(data.time)-c.timestep, z_before, yaw, axial_speed,
                            yaw_speed, 0, 0, -applied_load, wrench[2], wrench[5],
                            count, distance, normal, energy, 0, work])
    trace = np.asarray(records)
    diameter = contact_pitch_diameter(c.male_pitch_diameter, c.female_pitch_diameter)
    threshold = self_locking_threshold(c.pitch, diameter)
    expected_hold = is_self_locking(c.friction, pitch=c.pitch, pitch_diameter=diameter)
    late = trace[:, 0] > duration*.65
    drift = float(np.ptp(trace[late, 1]))
    terminal_speed = float(np.mean(trace[late, 4]))
    # Thread clearance may move axially before either flank carries load.
    holding = drift < .000002 and abs(terminal_speed) < .1
    backdriving = drift > .000010 and terminal_speed < -.2
    warnings = {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number}
    final_kinetic = (.5*c.nut_mass*float(data.qvel[vz])**2
                     + .5*c.nut_axial_inertia*float(data.qvel[wyaw])**2)
    finite = bool(np.all(np.isfinite(data.qpos))
                  and np.all(np.isfinite(data.qvel))
                  and np.all(np.isfinite(trace)))
    report = dict(
        experiment="zero-yaw-torque axial load, independent slide/yaw",
        config=c.as_dict(), engine=engine_snapshot, axial_load_N=axial_load, duration_s=duration,
        friction_impedance_ratio=friction_impedance_ratio,
        load_ramp_seconds=load_ramp_seconds,
        predicted_self_lock=expected_hold, predicted_friction_threshold=threshold,
        observed_hold=holding, observed_backdrive=backdriving,
        late_axial_drift_um=drift*1e6, late_yaw_velocity_rads=terminal_speed,
        late_measurement_window_s=duration*.35,
        holding_gate=dict(axial_drift_below_um=2., absolute_mean_yaw_speed_below_rads=.1),
        backdrive_gate=dict(axial_drift_above_um=10., mean_yaw_speed_below_rads=-.2),
        axial_travel_mm=float((c.initial_z+data.qpos[iz]-c.initial_z)*1000),
        yaw_travel_rad=float(data.qpos[iyaw]),
        worst_penetration_um=-minimum*1e6, max_contact_normal_N=max_normal,
        load_work_J=work, final_kinetic_energy_J=final_kinetic,
        finite=finite,
        accepted=bool((holding if expected_hold else backdriving)
                      and finite and not warnings and -minimum <= .000010),
        warnings=warnings, wall_seconds=time.monotonic()-start,
        note="No yaw holding controller; late drift excludes initial clearance take-up.",
        trace_timing_note="Trace time, pose, velocity and kinetic energy precede the step; contact wrench and cumulative work include that step. Final kinetic energy is computed at the terminal state.",
    )
    _save(output, report, trace)
    return report, trace


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--frictions", default="0,0.05,0.08,0.15,0.25")
    parser.add_argument("--loads", default="1,10")
    parser.add_argument("--modes", default="raise,lower")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=.2)
    parser.add_argument("--dt", type=float, default=.00005)
    parser.add_argument("--points", type=int, default=40)
    parser.add_argument("--contact-time", type=float, default=.0005)
    parser.add_argument("--margin", type=float, default=0.0)
    parser.add_argument("--impratio", type=float, default=1.0)
    parser.add_argument("--self-lock-only", action="store_true")
    parser.add_argument("--output", default="outputs/m8/guided_load")
    args = parser.parse_args()
    frictions = [float(x) for x in args.frictions.split(",")]
    loads = [float(x) for x in args.loads.split(",")]
    modes = args.modes.split(",")
    if any(x not in ("raise", "lower") for x in modes):
        parser.error("modes must contain only raise or lower")
    root = Path(args.output)
    reports = []
    for friction in frictions:
        c = ThreadConfig(guided=True, gravity=0, friction=friction,
                         timestep=args.dt, sdf_initpoints=args.points,
                         contact_time_constant=args.contact_time,
                         contact_margin=args.margin)
        if not args.self_lock_only:
            for load in loads:
                for mode in modes:
                    report, _ = velocity_probe(c, axial_load=load,
                        yaw_velocity=args.speed if mode == "raise" else -args.speed,
                        duration=args.duration, friction_impedance_ratio=args.impratio,
                        output=root/f"mu_{friction:g}_load_{load:g}_{mode}")
                    reports.append(report)
                    print(json.dumps({k: report[k] for k in (
                        "mode", "axial_load_N", "measured_steady_torque_Nm",
                        "predicted_torque_Nm", "measured_lead_mm",
                        "worst_penetration_um", "accepted", "wall_seconds")}), flush=True)
        report, _ = self_lock_probe(c, axial_load=min(loads),
                                   friction_impedance_ratio=args.impratio,
                                   output=root/f"mu_{friction:g}_self_lock")
        reports.append(report)
        print(json.dumps({k: report[k] for k in (
            "predicted_self_lock", "observed_hold", "observed_backdrive",
            "late_axial_drift_um", "worst_penetration_um", "accepted")}), flush=True)
    root.mkdir(parents=True, exist_ok=True)
    (root/"suite.json").write_text(json.dumps(dict(reports=reports,
        accepted=all(r["accepted"] for r in reports)), indent=2)+"\n")


if __name__ == "__main__":
    main()
