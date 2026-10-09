"""Compare frozen ideal-fixture M8 starts without changing their failed gates."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from thread_lab.model import ThreadConfig
from yam_twin.m8_insertion_mechanics import fully_formed_flank_interval


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalized_xml_sha256(path):
    root = ET.fromstring(Path(path).read_text())
    option = root.find("option")
    for key in ("timestep", "sdf_initpoints"):
        option.attrib.pop(key)
    return hashlib.sha256(ET.tostring(root, encoding="utf-8")).hexdigest()


def change_percent(value, reference):
    if value is None or reference is None or reference == 0:
        return None
    return float(100*(value-reference)/abs(reference))


def analyze(directory):
    directory = Path(directory)
    report = json.loads((directory/"report.json").read_text())
    with np.load(directory/"trace.npz", allow_pickle=False) as saved:
        trace, yaw = saved["trace"].copy(), saved["yaw"].copy()
    if trace.ndim != 2 or trace.shape[1] != 22 or yaw.shape != (len(trace),):
        raise ValueError(f"Unexpected trace layout: {directory}")
    if not np.isfinite(trace).all() or not np.isfinite(yaw).all() or not np.all(np.diff(trace[:, 0]) > 0):
        raise ValueError(f"Nonfinite or nonmonotonic trace: {directory}")
    config = ThreadConfig(**report["config"])
    complete_ring = []
    for row in trace:
        # Only tilt is needed by the complete-ring interval. The benchmark
        # female frame is fixed and identity; relative axial base is row z.
        cosine, sine = np.cos(row[5]), np.sin(row[5])
        rotation = np.array([[cosine, 0., sine], [0., 1., 0.], [-sine, 0., cosine]])
        interval = fully_formed_flank_interval(row[1:4], rotation, config)
        complete_ring.append(interval["one_pitch_available"])
    complete_ring = np.asarray(complete_ring)
    measured = complete_ring & (trace[:, 7] > 0) & (trace[:, 8] > 1e-5)
    if report["first_thread_contact_time_s"] is not None:
        measured &= trace[:, 0] > report["first_thread_contact_time_s"]+.2
    fitted_lead = turns = residual_range = late_travel = None
    if measured.sum() >= 3 and np.ptp(yaw[measured]) > .15:
        fitted_lead = float(np.polyfit(yaw[measured], trace[measured, 3], 1)[0]*2*np.pi*1000)
        turns = float(np.ptp(yaw[measured])/(2*np.pi))
        residual_range = float(np.ptp(trace[measured, 3]-config.pitch*yaw[measured]/(2*np.pi))*1e6)
        late_travel = float((trace[measured, 3][-1]-trace[measured, 3][0])*1000)
    lead_error = None if fitted_lead is None else abs(fitted_lead/(1000*config.pitch)-1)*100
    libraries = report["engine"]["libraries"]
    if len(libraries) != 1:
        raise ValueError(f"Expected exactly one verified core: {directory}")
    physical_inputs = {k: v for k, v in report["inputs"].items() if k not in ("output", "dt", "points")}
    physical_config = {k: v for k, v in report["config"].items() if k not in ("timestep", "sdf_initpoints")}
    identities = {
        "probe_source_sha256": report["probe_source_sha256"],
        "thread_plugin_source_sha256": report["engine"]["thread_plugin_source_sha256"],
        "mujoco_version": report["engine"]["mujoco_version"],
        "mujoco_core_sha256": libraries[0]["sha256"],
        "sdf_minimum_search_step_m": libraries[0]["sdf_minimum_search_step_m"],
        "sdf_initial_search_step_m": libraries[0]["sdf_initial_search_step_m"],
        "scene_xml_except_timestep_and_search_count_sha256": normalized_xml_sha256(directory/"scene.xml"),
    }
    copied_source = directory/"probe_source.py"
    source_snapshot_matches = copied_source.exists() and sha256(copied_source) == report["probe_source_sha256"]
    xml_matches = sha256(directory/"scene.xml") == report["xml_sha256"]
    unchanged_absolute_guards = {k: v for k, v in report["checks"].items() if k != "sustained_thread_lead"}
    observed = {
        "directory": str(directory), "dt_us": config.timestep*1e6,
        "sdf_initpoints": config.sdf_initpoints, "sampled_end_time_s": float(trace[-1, 0]),
        "total_axial_travel_mm": report["total_axial_travel_mm"],
        "late_complete_ring_fitted_lead_mm": fitted_lead,
        "late_complete_ring_lead_error_percent": lead_error,
        "late_complete_ring_turns": turns,
        "late_complete_ring_travel_mm": late_travel,
        "late_complete_ring_residual_range_um": residual_range,
        "first_thread_contact_time_s": report["first_thread_contact_time_s"],
        "first_complete_ring_geometry_time_s": float(trace[complete_ring, 0][0]) if complete_ring.any() else None,
        "complete_ring_contact_sample_count": int(measured.sum()),
        "complete_ring_geometry_sample_count": int(complete_ring.sum()),
        "maximum_reported_sdf_depth_um": report["worst_reported_sdf_depth_um"],
        "maximum_radial_offset_um": report["max_radial_um"],
        "maximum_tilt_deg": report["max_tilt_deg"],
        "warnings": report["warnings"], "guard_abort": report["guard_abort"],
        "original_broad_fit_passed": report["passed"],
        "original_broad_fit_lead_error_percent": report["lead_error_percent"],
        "original_broad_fit_checks": report["checks"],
        "declared_coaxial_full_flank_diagnostic": report["full_flank_diagnostic"],
        "declared_coaxial_full_flank_start_passed": report["full_flank_start_passed"],
        "unchanged_absolute_guards": unchanged_absolute_guards,
        "late_complete_ring_absolute_lead_passed": bool(lead_error is not None and lead_error < 2 and turns > .1),
        "source_snapshot_matches_report": source_snapshot_matches,
        "xml_matches_report": xml_matches,
        "raw_artifact_sha256": {name: sha256(directory/name) for name in ("trace.npz", "report.json", "scene.xml")},
        "identities": identities, "physical_inputs": physical_inputs,
        "physical_config": physical_config, "mass_properties": report["mass_properties"],
    }
    return observed


def compare(baseline, cases, output):
    base = analyze(baseline)
    observations = {name: analyze(directory) for name, directory in cases}
    comparisons = {}
    for name, observed in observations.items():
        travel_change = change_percent(observed["total_axial_travel_mm"], base["total_axial_travel_mm"])
        lead_change = change_percent(observed["late_complete_ring_fitted_lead_mm"], base["late_complete_ring_fitted_lead_mm"])
        late_travel_change = change_percent(observed["late_complete_ring_travel_mm"], base["late_complete_ring_travel_mm"])
        declared_lead_change = change_percent(observed["declared_coaxial_full_flank_diagnostic"]["fitted_lead_mm"],
                                             base["declared_coaxial_full_flank_diagnostic"]["fitted_lead_mm"])
        identities_match = observed["identities"] == base["identities"]
        inputs_match = observed["physical_inputs"] == base["physical_inputs"] and observed["physical_config"] == base["physical_config"]
        mass_matches = observed["mass_properties"] == base["mass_properties"]
        travel_passed = bool(travel_change is not None and abs(travel_change) < 2.)
        lead_passed = bool(lead_change is not None and abs(lead_change) < 2.)
        declared_lead_passed = bool(declared_lead_change is not None and abs(declared_lead_change) < 2.)
        late_travel_passed = bool(late_travel_change is not None and abs(late_travel_change) < 2.)
        depth_change = change_percent(observed["maximum_reported_sdf_depth_um"], base["maximum_reported_sdf_depth_um"])
        depth_comparison_passed = bool(depth_change is not None and abs(depth_change) < 2.)
        absolute_passed = (all(base["unchanged_absolute_guards"].values())
                           and base["late_complete_ring_absolute_lead_passed"]
                           and base["declared_coaxial_full_flank_start_passed"]
                           and all(observed["unchanged_absolute_guards"].values())
                           and observed["late_complete_ring_absolute_lead_passed"]
                           and observed["declared_coaxial_full_flank_start_passed"])
        provenance_passed = (identities_match and inputs_match and mass_matches
            and observed["source_snapshot_matches_report"] and observed["xml_matches_report"]
            and base["source_snapshot_matches_report"] and base["xml_matches_report"])
        comparisons[name] = {
            "travel_relative_change_percent": travel_change,
            "travel_comparison_passed_below_2_percent": travel_passed,
            "late_complete_ring_travel_relative_change_percent": late_travel_change,
            "late_complete_ring_travel_comparison_passed_below_2_percent": late_travel_passed,
            "late_complete_ring_lead_relative_change_percent": lead_change,
            "late_complete_ring_lead_comparison_passed_below_2_percent": lead_passed,
            "declared_coaxial_lead_relative_change_percent": declared_lead_change,
            "declared_coaxial_lead_comparison_passed_below_2_percent": declared_lead_passed,
            "reported_sdf_depth_relative_change_percent": depth_change,
            "reported_sdf_depth_diagnostic_below_2_percent": depth_comparison_passed,
            "depth_diagnostic_note": "Disclosed additional diagnostic, not part of the requested travel/lead gate. The unchanged absolute reported-depth guard is 10um.",
            "first_contact_time_difference_ms": None if observed["first_thread_contact_time_s"] is None else 1000*(observed["first_thread_contact_time_s"]-base["first_thread_contact_time_s"]),
            "complete_ring_geometry_time_difference_ms": None if observed["first_complete_ring_geometry_time_s"] is None else 1000*(observed["first_complete_ring_geometry_time_s"]-base["first_complete_ring_geometry_time_s"]),
            "unchanged_absolute_guards_passed": absolute_passed,
            "source_runtime_and_normalized_xml_identities_match": identities_match,
            "physical_inputs_and_config_match": inputs_match,
            "mass_properties_match": mass_matches,
            "raw_provenance_passed": provenance_passed,
            "travel_and_lead_refinement_passed": travel_passed and late_travel_passed and lead_passed and declared_lead_passed and absolute_passed and provenance_passed,
        }
    result = {
        "scope": "Frozen fixed-female/free-six-DOF headed-male ideal-fixture start only; not whole robot, load suite, seating, or trained-policy qualification",
        "strict_relative_travel_and_lead_limit_percent": 2.,
        "limits_note": "Relative total travel, late full-ring travel and lead changes must be strictly below 2%; original 2% absolute pitch, 10um reported depth, 150um radial and 2deg tilt guards remain. Proxy-depth change is disclosed, not included in the requested 2% travel/lead convergence gate.",
        "comparison_source_sha256": sha256(__file__),
        "shared_geometry_source_sha256": sha256(Path(__file__).resolve().parents[1]/"yam_twin/m8_insertion_mechanics.py"),
        "baseline": base, "cases": observations, "comparisons": comparisons,
        "travel_and_lead_refinement_passed": bool(comparisons and all(x["travel_and_lead_refinement_passed"] for x in comparisons.values())),
        "reported_sdf_depth_diagnostic_below_2_percent_passed": bool(comparisons and all(x["reported_sdf_depth_diagnostic_below_2_percent"] for x in comparisons.values())),
        "original_broad_fit_failures_preserved": True,
        "late_complete_ring_method": "Same geometric whole-ring interval for all cases, positive measured thread/end normal above 1e-5 N, >0.2 s since first contact; fit measured axial position against unwrapped measured yaw. No state/force/control changes.",
    }
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    lines = ["# Frozen M8 start refinement", "", result["scope"], "",
             "| Case | dt, µs | Starts | Travel, mm | Late full-ring lead, mm/rev | Δ total travel | Δ late travel | Δ lead | Peak reported depth, µm | Travel/lead refinement |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"]
    for name, observed in [("baseline", base), *observations.items()]:
        cmp = comparisons.get(name)
        def number(value, digits=6):
            return "unavailable" if value is None else f"{value:.{digits}f}"
        lines.append(f"| {name} | {observed['dt_us']:.1f} | {observed['sdf_initpoints']} | {number(observed['total_axial_travel_mm'])} | {number(observed['late_complete_ring_fitted_lead_mm'])} | {'reference' if cmp is None else number(cmp['travel_relative_change_percent'], 4)+'%'} | {'reference' if cmp is None else number(cmp['late_complete_ring_travel_relative_change_percent'], 4)+'%'} | {'reference' if cmp is None else number(cmp['late_complete_ring_lead_relative_change_percent'], 4)+'%'} | {number(observed['maximum_reported_sdf_depth_um'], 4)} | {'reference' if cmp is None else ('pass' if cmp['travel_and_lead_refinement_passed'] else 'FAIL')} |")
    lines += ["", "Every relative comparison requires strictly less than 2%. Late-window travel is checked separately so initial gap/lead-in cannot hide a local difference. Original broad fits and their failures remain in the JSON and original reports. The late formed-flank fit is a separate diagnostic.", "", "Proxy depth is a reported SDF metric, not certified solid overlap. Numerical refinement of this fixture alone does not qualify the whole bimanual manipulation task.", ""]
    for name, cmp in comparisons.items():
        if not cmp["reported_sdf_depth_diagnostic_below_2_percent"]:
            lines += [f"Peak reported depth **fails** the separate 2% comparison for `{name}`: {cmp['reported_sdf_depth_relative_change_percent']:+.2f}%. Travel/lead convergence does not establish convergence of this depth proxy.", ""]
    lines += ["![Measured start refinement](refinement.png)", "",
              "[Full JSON and failed gates](summary.json) · [Artifact hashes](manifest.json) · [Baseline trace](../start_probe_extended/trace.npz) · [25 µs trace](dt25_points40/trace.npz) · [80-point trace](dt50_points80/trace.npz)", "",
              "Recompute the comparison from the checked-in traces without rerunning physics:", "", "```bash",
              "scripts/run_m8.sh scripts/compare_m8_thread_start.py \\",
              "  --baseline media/m8_insertion/start_probe_extended \\",
              "  --case dt25_points40 media/m8_insertion/refinement/dt25_points40 \\",
              "  --case dt50_points80 media/m8_insertion/refinement/dt50_points80 \\",
              "  --output outputs/m8_insertion/refinement_recomputed", "```", ""]
    (output/"README.md").write_text("\n".join(lines))
    print(json.dumps({"travel_and_lead_refinement_passed": result["travel_and_lead_refinement_passed"], "comparisons": comparisons}, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=Path("outputs/m8_insertion/start_probe_extended"))
    parser.add_argument("--case", nargs=2, action="append", metavar=("NAME", "DIRECTORY"))
    parser.add_argument("--output", type=Path, default=Path("outputs/m8_insertion/refinement"))
    args = parser.parse_args()
    cases = args.case or [("dt25_points40", "outputs/m8_insertion/refinement_dt25_points40"),
                         ("dt50_points80", "outputs/m8_insertion/refinement_dt50_points80")]
    compare(args.baseline, cases, args.output)
