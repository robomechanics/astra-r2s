"""Disengaged M8 start benchmark, independent of the YAM manipulation demo.

A fixed female and free 6-DOF headed male receive a bounded ideal fixture
wrench. The axial command is a constant force plus velocity drag; it never
uses yaw, pitch, depth, engagement, or a prescribed helical displacement.
This is a geometry/contact diagnostic, not evidence of robot pickup.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from thread_lab.build_plugin import build_plugin
from thread_lab.model import ThreadConfig, model_xml
from thread_lab.runtime import require_micron_engine
from yam_twin.m8_scene import bolt_mass_properties


def headed_bolt_properties(config, head_af=.020, head_height=.008):
    """Independent steel hex-head + threaded-shaft union mass properties."""
    shaft = bolt_mass_properties(config, samples=262_144)
    density = 7850.
    area = np.sqrt(3.) * head_af**2 / 2
    head_mass = density * area * head_height
    head_center = np.array([0., 0., -head_height/2])
    head_polar = density * 5*np.sqrt(3)*head_af**4 / 72 * head_height
    head_inertia = np.diag([head_polar/2 + head_mass*head_height**2/12]*2 + [head_polar])
    shaft_mass = shaft["mass_kg"]
    shaft_center = np.asarray(shaft["centroid_m"])
    mass = head_mass + shaft_mass
    center = (head_mass*head_center+shaft_mass*shaft_center)/mass
    inertia = head_inertia + np.asarray(shaft["inertia_kg_m2"])
    for submass, subcenter in ((head_mass, head_center), (shaft_mass, shaft_center)):
        offset = subcenter-center
        inertia += submass * (np.dot(offset, offset)*np.eye(3)-np.outer(offset, offset))
    return {"mass_kg": float(mass), "centroid_m": center.tolist(),
            "inertia_kg_m2": inertia.tolist(), "steel_density_kg_m3": density,
            "shaft_quadrature_samples": shaft["samples"],
            "head_across_flats_m": head_af, "head_height_m": head_height}


def benchmark_model(config, *, initial_gap=.0005, initial_yaw=.37):
    """Axes point up: insert from below, equivalent to rotating both frames."""
    root = ET.fromstring(model_xml(replace(config, initial_z=0., gravity=0.)))
    world = root.find("worldbody")
    female = world.find("body[@name='nut']")
    female.remove(female.find("freejoint"))
    female.remove(female.find("inertial"))
    male_geom = world.find("geom[@name='bolt_thread']")
    world.remove(male_geom)
    initial_base = -config.nut_height/2-config.bolt_length-initial_gap
    body = ET.SubElement(world, "body", name="male_bolt", pos=f"0 0 {initial_base:.17g}",
                         quat=f"{np.cos(initial_yaw/2):.17g} 0 0 {np.sin(initial_yaw/2):.17g}")
    ET.SubElement(body, "freejoint", name="male_bolt_free")
    properties = headed_bolt_properties(config)
    inertia = np.asarray(properties["inertia_kg_m2"])
    ET.SubElement(body, "inertial", pos=" ".join(map(str, properties["centroid_m"])),
                  mass=str(properties["mass_kg"]), fullinertia=" ".join(map(str,
                  [inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]])))
    body.append(male_geom)
    # Explicit solid hex head, attached only to the male. It cannot reach the
    # female in this short start benchmark; preserve its contact capability.
    af, height = .020, .008
    vertices = [(af/np.sqrt(3)*np.cos((k+.5)*np.pi/3),
                 af/np.sqrt(3)*np.sin((k+.5)*np.pi/3), z)
                for z in (-height, 0.) for k in range(6)]
    ET.SubElement(root.find("asset"), "mesh", name="male_head_mesh",
                  vertex=" ".join(str(v) for point in vertices for v in point))
    ET.SubElement(body, "geom", name="male_head", type="mesh", mesh="male_head_mesh",
                  rgba=".52 .56 .60 1", mass="0", contype="1", conaffinity="2")
    xml = ET.tostring(root, encoding="unicode")
    mujoco.mj_loadPluginLibrary(str(build_plugin()))
    return mujoco.MjModel.from_xml_string(xml), xml, properties


def run(args):
    config = ThreadConfig(timestep=args.dt, bolt_length=.016, nut_height=.016,
                          nut_across_flats=.016, sdf_initpoints=args.points, gravity=0.)
    engine = require_micron_engine()
    model, xml, properties = benchmark_model(config, initial_gap=args.gap,
                                             initial_yaw=args.yaw)
    data = mujoco.MjData(model)
    bid = model.body("male_bolt").id
    male_gid, female_gid = model.geom("bolt_thread").id, model.geom("nut_thread").id
    velocity = np.zeros(6)
    force = np.zeros(6)
    mujoco.mj_forward(model, data)
    initial_base = float(data.xpos[bid, 2])
    rows = []
    worst_depth = 0.
    max_radial = 0.
    max_tilt = 0.
    contact_substeps = 0
    first_contact = None
    max_normal = 0.
    wall_start = time.monotonic()
    failed = None
    sample_stride = max(1, round(.001/config.timestep))
    for step in range(round(args.duration/config.timestep)):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, bid, velocity, 0)
        rotation = data.xmat[bid].reshape(3, 3)
        ramp = min(data.time/.3, 1.)
        target_omega = args.speed * ramp*ramp*(3-2*ramp)
        # Ideal bounded coaxial handling fixture; independent lateral/tilt
        # impedance does not couple the two unconstrained axial/yaw freedoms.
        wrench = np.zeros(6)
        wrench[:2] = np.clip(-5000*data.xipos[bid, :2]-10*velocity[3:5], -2., 2.)
        wrench[2] = np.clip(args.feed-args.axial_drag*velocity[5], -.2, .2)
        wrench[3:5] = np.clip((.3*np.cross(rotation[:, 2], [0., 0., 1.])
                              -.002*velocity[:3])[:2], -.02, .02)
        wrench[5] = np.clip(.002*(target_omega-velocity[2]), -.002, .002)
        data.xfrc_applied[bid] = wrench
        mujoco.mj_step(model, data)
        normal = 0.
        thread_count = 0
        for i in range(data.ncon):
            contact = data.contact[i]
            if {int(contact.geom1), int(contact.geom2)} == {male_gid, female_gid}:
                thread_count += 1
                worst_depth = max(worst_depth, -float(contact.dist))
                mujoco.mj_contactForce(model, data, i, force)
                normal += float(force[0])
        max_normal = max(max_normal, normal)
        if thread_count:
            contact_substeps += 1
            if first_contact is None:
                first_contact = float(data.time)
        rotation = data.xmat[bid].reshape(3, 3)
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
        tilt = float(np.arccos(np.clip(rotation[2, 2], -1, 1)))
        radial = float(np.linalg.norm(data.xpos[bid, :2]))
        max_radial = max(max_radial, radial)
        max_tilt = max(max_tilt, tilt)
        if step % sample_stride == 0:
            rows.append([data.time, *data.xpos[bid], yaw, tilt, radial, thread_count,
                         normal, worst_depth, *velocity, *wrench])
        if not np.all(np.isfinite(data.qpos)) or any(w.number for w in data.warning):
            failed = "nonfinite_or_solver_warning"
            break
        if worst_depth > 10e-6 or radial > 150e-6 or tilt > np.deg2rad(2):
            failed = "depth_radial_or_tilt_guard"
            break
    trace = np.asarray(rows)
    yaw = np.unwrap(trace[:, 4])
    # Geometric criterion declared before execution: at least one pitch of
    # shaft overlap, measured contacts, and at least .2 s after first contact.
    overlap = trace[:, 3]+config.bolt_length+config.nut_height/2
    steady = (overlap > config.pitch) & (trace[:, 7] > 0)
    if first_contact is not None:
        steady &= trace[:, 0] > first_contact+.2
    fitted_lead = None
    steady_turns = 0.
    residual_range = None
    if np.count_nonzero(steady) >= 3 and np.ptp(yaw[steady]) > .15:
        fitted_lead = float(np.polyfit(yaw[steady], trace[steady, 3], 1)[0]*2*np.pi)
        steady_turns = float(np.ptp(yaw[steady])/(2*np.pi))
        residual_range = float(np.ptp(trace[steady, 3]-config.pitch*yaw[steady]/(2*np.pi)))
    lead_error = None if fitted_lead is None else abs(fitted_lead/config.pitch-1)*100
    # Separately test only shared fully-formed flanks. The original broad
    # >one-pitch test remains above and in the acceptance result; it also
    # includes conical end contacts and is not silently reclassified.
    H = np.sqrt(3.)*config.pitch/2
    female_end_extent = 5*H/8 + .000360
    full_flank_overlap_min = .000956 + female_end_extent + config.pitch
    full_flank = steady & (overlap > full_flank_overlap_min)
    full_flank_lead = None
    full_flank_turns = 0.
    full_flank_residual_range = None
    if np.count_nonzero(full_flank) >= 3 and np.ptp(yaw[full_flank]) > .15:
        full_flank_lead = float(np.polyfit(yaw[full_flank], trace[full_flank, 3], 1)[0]*2*np.pi)
        full_flank_turns = float(np.ptp(yaw[full_flank])/(2*np.pi))
        full_flank_residual_range = float(np.ptp(trace[full_flank, 3]-config.pitch*yaw[full_flank]/(2*np.pi)))
    full_flank_error = None if full_flank_lead is None else abs(full_flank_lead/config.pitch-1)*100
    checks = {
        "initially_disengaged": args.gap > 0,
        "contact_captured": first_contact is not None,
        "sustained_thread_lead": lead_error is not None and lead_error < 2 and steady_turns > .10,
        "reported_depth_below_10um": worst_depth < 10e-6,
        "coaxial_fixture_bounds": max_radial < 150e-6 and max_tilt < np.deg2rad(2),
        "solver_finite": failed is None,
        "no_axial_yaw_equality": model.neq == 0,
    }
    checks = {key: bool(value) for key, value in checks.items()}
    full_flank_lead_passed = bool(full_flank_error is not None and full_flank_error < 2 and full_flank_turns > .10)
    full_flank_start_passed = full_flank_lead_passed and all(value for key, value in checks.items()
                                                          if key != "sustained_thread_lead")
    report = {"scope": "Fixed-female, free-6DOF headed-male bounded-wrench thread-start benchmark; not robot pickup",
              "gravity_note": "Zero gravity and axes up, equivalent geometric start to rotating both SDF frames 180 degrees for shaft-down insertion",
              "passed": all(checks.values()), "checks": checks, "guard_abort": failed,
              "config": config.as_dict(), "inputs": vars(args) | {"output": str(args.output)},
              "engine": engine, "initial_gap_m": args.gap, "initial_yaw_rad": args.yaw,
              "first_thread_contact_time_s": first_contact, "contact_substeps": contact_substeps,
              "total_axial_travel_mm": float((trace[-1, 3]-initial_base)*1000),
              "steady_turns": steady_turns, "fitted_lead_mm": None if fitted_lead is None else fitted_lead*1000,
              "lead_error_percent": lead_error, "steady_helix_residual_range_um": None if residual_range is None else residual_range*1e6,
              "full_flank_start_passed": full_flank_start_passed,
              "full_flank_diagnostic": {"minimum_total_overlap_m": float(full_flank_overlap_min),
                  "female_chamfer_maximum_axial_extent_m": float(female_end_extent),
                  "male_tip_chamfer_m": .000956, "fully_formed_shared_flank_length_minimum_m": config.pitch,
                  "turns": full_flank_turns, "lead_error_percent": full_flank_error,
                  "fitted_lead_mm": None if full_flank_lead is None else full_flank_lead*1000,
                  "helix_residual_range_um": None if full_flank_residual_range is None else full_flank_residual_range*1e6,
                  "note": "Separate stricter geometry-based window. Original broad >one-pitch lead test and its failures are preserved."},
              "worst_reported_sdf_depth_um": worst_depth*1e6,
              "max_radial_um": max_radial*1e6, "max_tilt_deg": np.rad2deg(max_tilt),
              "max_total_thread_normal_N": max_normal, "mass_properties": properties,
              "wall_seconds": time.monotonic()-wall_start,
              "xml_sha256": hashlib.sha256(xml.encode()).hexdigest(),
              "probe_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "initial_qpos": model.qpos0.tolist(),
              "warnings": {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number},
              "steady_definition": "shaft overlap > one pitch, contact present, >0.2 s since first contact; free axial DOF, constant feed plus velocity drag"}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"scene.xml").write_text(xml)
    np.savez_compressed(args.output/"trace.npz", trace=trace, yaw=yaw,
                        fields=np.asarray(["time", "x", "y", "z", "yaw", "tilt", "radial", "thread_contacts", "normal", "worst_depth", "wx", "wy", "wz", "vx", "vy", "vz", "fx", "fy", "fz", "tx", "ty", "tz"]))
    (args.output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({key: report[key] for key in ("passed", "full_flank_start_passed", "full_flank_diagnostic", "guard_abort", "first_thread_contact_time_s", "fitted_lead_mm", "lead_error_percent", "steady_turns", "worst_reported_sdf_depth_um", "total_axial_travel_mm", "wall_seconds")}, indent=2), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/m8_insertion/start_probe"))
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=.00005)
    parser.add_argument("--points", type=int, default=40)
    parser.add_argument("--gap", type=float, default=.0005)
    parser.add_argument("--yaw", type=float, default=.37)
    parser.add_argument("--speed", type=float, default=3.)
    parser.add_argument("--feed", type=float, default=.05)
    parser.add_argument("--axial-drag", type=float, default=10.)
    run(parser.parse_args())
