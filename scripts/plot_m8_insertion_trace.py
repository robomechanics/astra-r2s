"""Plot measured, completed YAM M8 rollout evidence without any simulation.

Only the final insertion_trace.npz and matching insertion_validation.json are
accepted. A stopped or failed trial needs --evidence-only, and its figure is
explicitly labeled as such. Entry/cone travel never receives a pitch reference.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re

os.environ.setdefault("MPLCONFIGDIR", "/workspace/.cache/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


COLORS = {
    "position": "#17639a", "yaw": "#272b31", "reference": "#a13b78",
    "overlap": "#73818c", "ring": "#087f74", "threshold": "#b15d12",
    "capture": "#794bb5", "right_1": "#17639a", "right_2": "#40a2b3",
    "left_1": "#ad5725", "left_2": "#e3a043", "support": "#be4155",
}
PHASE_COLORS = {
    "Pickup": "#dce8f5", "Carry / align": "#d4eeee",
    "Thread entry / search": "#fae3be", "Release / regrasp": "#e9e4f3",
    "Turn test": "#d5eadb", "Other": "#eceef0",
}


def _phase_category(phase):
    if phase in {"settle_table", "reach_left_block", "close_left_block", "settle_left_block",
                 "lift_left_block", "transport_left_block", "hold_left_block",
                 "secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt"}:
        return "Pickup"
    if phase in {"transport_bolt", "align_over_hole", "feed_to_entry"}:
        return "Carry / align"
    if phase.startswith(("start_thread_", "stop_start_")):
        return "Thread entry / search"
    if phase.startswith(("turn_", "stop_")):
        return "Turn test"
    if phase.startswith(("release_", "open_", "reset_open_", "regrip_", "settle_regrip_")):
        return "Release / regrasp"
    return "Other"


def _phase_segments(rows, times):
    """Contiguous saved phases; boundaries are approximate at sample resolution."""
    segments, start = [], 0
    for index in range(1, len(rows) + 1):
        if index == len(rows) or rows[index]["phase"] != rows[start]["phase"]:
            segments.append({"phase": rows[start]["phase"], "start": start,
                             "end": index - 1,
                             "left": float(times[max(0, start - 1)]),
                             "right": float(times[index - 1])})
            start = index
    return segments


def _qualified_turns(report, rows, segments, *, legacy_inline_observer_verified=False):
    """Authorize comparison lines from the saved checks, never from cone travel.

    A failed overall run may still contain individually measured valid turns;
    the comparison is allowed only when the completed trace contains observer
    provenance and the corresponding lead, tracking, and torque checks pass.
    Legacy inline criteria require explicit opt-in and a verified archived
    controller before their measured turn checks can authorize a reference.
    """
    checks = report.get("acceptance_checks", {})
    needed = ("observed_metric_lead", "closed_turn_tracking", "contact_torque_turns_bolt")
    if (report.get("partial") or not (report.get("engagement_observer_source_sha256")
                                     or legacy_inline_observer_verified)
            or not all(checks.get(name, {}).get("passed") is True for name in needed)):
        return []
    pitch = float(report["scene_config"]["base"]["thread"]["pitch"])
    stroke = float(report["control_config"]["arm"]["stroke_angle_rad"])
    summaries = {summary["phase"]: summary for summary in report.get("phases", [])}
    qualified = []
    for segment in segments:
        name = segment["phase"]
        if not re.fullmatch(r"turn_\d+", name):
            continue
        summary = summaries.get(name, {})
        advance = summary.get("bolt_insertion_advance_m")
        rotation = summary.get("bolt_clockwise_rotation_rad")
        if advance is None or rotation is None:
            continue
        expected = pitch * float(rotation) / (2 * np.pi)
        if (summary.get("started_engaged") is not True
                or abs(float(rotation) - stroke) >= .03
                or abs(float(advance) - expected) >= .02 * pitch * stroke / (2 * np.pi)
                or float(summary.get("sampled_peak_pad_contact_torque_Nm", 0)) <= 1e-6
                or not all(row.get("thread_engaged") is True
                           for row in rows[segment["start"]:segment["end"] + 1])):
            continue
        qualified.append(segment)
    return qualified


def _values(rows, field):
    result = np.asarray([row[field] for row in rows], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"Nonfinite recorded plot field: {field}")
    return result


def _pad_values(rows, field):
    values = np.asarray([row[field]["pad_normal_force_N"] for row in rows], dtype=float)
    if values.shape != (len(rows), 2) or not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError(f"Invalid recorded physical pad normals: {field}")
    return values


def _outcome(report, evidence_only, legacy_inline_observer_verified=False):
    if report.get("partial"):
        return "EARLY STOPPED TRIAL — EVIDENCE ONLY", "#9a4f12"
    if report.get("passed") is not True:
        return "FAILED TRIAL — EVIDENCE ONLY", "#a12e43"
    if legacy_inline_observer_verified:
        return "COMPLETED RUN — LEGACY INLINE CAPTURE CRITERION", "#167052"
    if evidence_only:
        return "COMPLETED RUN — EVIDENCE VIEW", "#4a5662"
    return "COMPLETED RUN — SAVED ACCEPTANCE CHECKS PASSED", "#167052"


def plot_completed_run(run, output_dir, *, evidence_only=False,
                       legacy_inline_observer=False, basename="insertion_trajectory"):
    """Render an already identity-validated bundle; integration is never invoked."""
    report, rows = run["report"], run["rows"]
    times = np.asarray(run["times"], dtype=float)
    if not len(rows) or times.shape != (len(rows),) or not np.isfinite(times).all():
        raise ValueError("Plot needs a nonempty, aligned final trace")
    if len(times) > 1 and np.any(np.diff(times) <= 0):
        raise ValueError("Trace sample times must increase")
    if not np.array_equal(times, _values(rows, "time")):
        raise ValueError("Recorded diagnostic row times do not match the trace time array")
    if (report.get("partial") or report.get("passed") is not True) and not evidence_only:
        raise ValueError("Stopped or failed trials require --evidence-only")
    if not basename or Path(basename).name != basename:
        raise ValueError("Plot basename must be a single filename stem")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    pitch = float(report["scene_config"]["base"]["thread"]["pitch"])
    z = _values(rows, "bolt_base_insertion_m")
    yaw = _values(rows, "bolt_yaw_unwrapped_rad")
    overlap = _values(rows, "thread_overlap_m")
    full_ring = _values(rows, "formed_flank_overlap_m")
    radial = _values(rows, "radial_offset_m")
    tilt = _values(rows, "bolt_tilt_rad")
    bolt_support = _values(rows, "bolt_world_support_contacts")
    block_support = _values(rows, "block_world_support_contacts")
    right_pads, left_pads = _pad_values(rows, "contact"), _pad_values(rows, "left_contact")
    segments = _phase_segments(rows, times)
    legacy_verified = bool(legacy_inline_observer
                           and run["checks"].get("legacy_inline_observer_verified") is True)
    if legacy_inline_observer and not legacy_verified:
        raise ValueError("Explicit legacy mode needs verified archived inline controller provenance")
    qualified = _qualified_turns(report, rows, segments,
                                 legacy_inline_observer_verified=legacy_verified)
    status, status_color = _outcome(report, evidence_only, legacy_verified)
    capture_time = report.get("acceptance_checks", {}).get(
        "started_previously_separate_threads", {}).get("engagement_time_s")
    if capture_time is not None and not np.isfinite(float(capture_time)):
        raise ValueError("Capture tag time must be finite")
    has_window_observer = bool(report.get("engagement_observer_source_sha256"))
    # Off-axis pickup projections and the distant approach are not thread entry.
    # The top panel retains the entire recorded axial trajectory.
    aligned = ((radial <= 150e-6) & (tilt <= np.deg2rad(2)) & (overlap >= -pitch))
    table_pickup = bool(report["scene_config"].get("pickup_from_table", False))
    insertion_display = np.ones(len(rows), dtype=bool)
    if table_pickup:
        alignment_start = next((index for index, row in enumerate(rows)
                                if row["phase"] == "align_over_hole"), len(rows))
        insertion_display = ((np.arange(len(rows)) >= alignment_start)
                             & (radial <= 150e-6) & (tilt <= np.deg2rad(2)))
        block_world_height = float(report["scene_config"]["block_position"][2]) + _values(rows, "block_lift_m")
        bolt_world_height = np.asarray([row["bolt_world_position"][2] for row in rows], dtype=float)
        if not np.isfinite(block_world_height).all() or not np.isfinite(bolt_world_height).all():
            raise ValueError("Pickup world-height diagnostics must be finite")
    displayed_yaw = np.where(insertion_display, yaw, np.nan) if table_pickup else yaw
    displayed_z = np.where(insertion_display, z, np.nan) if table_pickup else z
    masked_overlap = np.where(aligned, overlap * 1e3, np.nan)
    masked_ring = np.where(aligned, full_ring * 1e3, np.nan)

    style = {"font.family": "DejaVu Sans", "font.size": 10,
             "axes.labelsize": 10, "axes.titlesize": 11,
             "axes.spines.top": False, "axes.spines.right": False,
             "axes.edgecolor": "#c4cbd1", "text.color": "#24313b",
             "axes.labelcolor": "#24313b", "xtick.color": "#4a5662",
             "ytick.color": "#4a5662", "grid.color": "#dbe1e6",
             "grid.alpha": .75, "grid.linewidth": .6,
             "savefig.facecolor": "white", "svg.fonttype": "none"}
    with plt.rc_context(style):
        fig = plt.figure(figsize=(12.8, 12.0 if table_pickup else 10.2), facecolor="white")
        heights = [.16, .9, 1.4, 1.25, 1.05, .85] if table_pickup else [.16, 1.4, 1.25, 1.05, .85]
        grid = fig.add_gridspec(len(heights), 1, height_ratios=heights,
                               left=.095, right=.905, bottom=.145, top=.815, hspace=.33)
        band = fig.add_subplot(grid[0])
        axes = [fig.add_subplot(grid[i], sharex=band) for i in range(1, len(heights))]
        pickup_ax = axes[0] if table_pickup else None
        ax, geometry_ax, pad_ax, support_ax = axes[1:] if table_pickup else axes
        fig.text(.095, .958, "YAM · M8 pickup and thread insertion", fontsize=20,
                 weight="bold", va="top")
        fig.text(.095, .92, status, color=status_color, fontsize=11, weight="bold")
        fig.text(.095, .891, "Measured rollout samples · physical pad forces · no additional physics steps",
                 fontsize=10, color="#596975")

        used_categories = []
        for segment in segments:
            category = _phase_category(segment["phase"])
            color = PHASE_COLORS[category]
            band.axvspan(segment["left"], segment["right"], color=color, linewidth=0)
            if category not in used_categories:
                used_categories.append(category)
            for axis in axes:
                axis.axvline(segment["left"], color="#96a0a8", linewidth=.45, alpha=.2)
            if re.fullmatch(r"start_thread_\d+|turn_\d+", segment["phase"]):
                label = segment["phase"].replace("start_thread_", "entry ").replace("turn_", "turn ")
                geometry_ax.text((segment["left"] + segment["right"]) / 2, 1.055, label,
                                 ha="center", fontsize=8, transform=geometry_ax.get_xaxis_transform(),
                                 color="#697781")
        band.set_ylim(0, 1)
        band.set_axis_off()
        phase_handles = [Line2D([0], [0], color=PHASE_COLORS[c], linewidth=7, label=c)
                         for c in used_categories]
        fig.legend(handles=phase_handles, loc="upper left", bbox_to_anchor=(.095, .865),
                   ncol=min(5, len(phase_handles)), frameon=False, fontsize=8.5,
                   handlelength=1.2, columnspacing=1.25, borderaxespad=0)

        if table_pickup:
            pickup_ax.plot(times, block_world_height * 1e3, color=COLORS["left_1"], linewidth=1.4,
                           label="Block center · world Z")
            pickup_ax.plot(times, bolt_world_height * 1e3, color=COLORS["position"], linewidth=1.4,
                           label="Bolt shaft base · world Z")
            pickup_ax.set_ylabel("World height (mm)")
            pickup_ax.set_title("A  Actual pickup and carry height", loc="left", pad=9)
            pickup_ax.legend(loc="upper right", frameon=True, framealpha=.92, fontsize=8.2)

        position_line, = ax.plot(times, displayed_z * 1e3, color=COLORS["position"], linewidth=1.7,
                                 label="Female-frame axial coordinate" if table_pickup else "Measured axial base position")
        ax.set_ylabel("Female-frame axial\ncoordinate (mm)" if table_pickup else "Bolt axial base\nposition (mm)")
        yaw_ax = ax.twinx()
        yaw_line, = yaw_ax.plot(times, displayed_yaw, color=COLORS["yaw"], linewidth=1.2,
                               alpha=.85, label="Relative yaw (axes within 2°)" if table_pickup else "Measured unwrapped yaw")
        yaw_ax.spines["right"].set_visible(True)
        yaw_ax.set_ylabel("Relative yaw (rad)", color=COLORS["yaw"])
        position_handles = [position_line, yaw_line]
        for index, segment in enumerate(qualified):
            anchor = max(0, segment["start"] - 1)
            selected = np.arange(anchor, segment["end"] + 1)
            reference = z[anchor] + pitch * (yaw[selected] - yaw[anchor]) / (2 * np.pi)
            if table_pickup:
                reference = np.where(insertion_display[selected], reference, np.nan)
            line, = ax.plot(times[selected], reference * 1e3, "--", color=COLORS["reference"],
                            linewidth=1.4, label="Pitch comparison (qualified turns only)")
            if index == 0:
                position_handles.append(line)
        ax.legend(handles=position_handles, loc="lower left", frameon=True,
                  facecolor="white", framealpha=.92, fontsize=8.5)
        ax.set_title("B  Insertion and rotation after alignment over the hole" if table_pickup else
                     "A  Recorded axial position and actual rotation", loc="left", pad=9)
        if table_pickup:
            ax.text(.995, .94, "Shown from alignment stage onward, within 150 µm offset / 2° tilt.",
                    transform=ax.transAxes, ha="right", va="top", fontsize=7.7, color="#4f626d",
                    bbox={"facecolor": "white", "alpha": .87, "edgecolor": "none"})
            if not np.any(insertion_display):
                ax.text(.5, .5, "No aligned insertion stage was recorded", transform=ax.transAxes,
                        ha="center", va="center", fontsize=10, color="#596975")

        geometry_ax.plot(times, masked_overlap, color=COLORS["overlap"], linewidth=1.1,
                         label="Nominal axial overlap")
        geometry_ax.plot(times, masked_ring, color=COLORS["ring"], linewidth=1.8,
                         label="Potential complete-ring length (tilt aware)")
        geometry_ax.axhline(pitch * 1e3, color=COLORS["threshold"], linestyle="--", linewidth=1.2,
                            label=f"One complete pitch: {pitch * 1e3:.3f} mm")
        geometry_ax.set_ylabel("Potential overlap /\nfull-ring length (mm)")
        geometry_ax.set_title("C  Geometric threshold and observer tag" if table_pickup else
                             "B  Geometric threshold and observer tag", loc="left", pad=13)
        geometry_ax.legend(loc="upper left", frameon=True, framealpha=.92, fontsize=8.2)
        geometry_ax.text(.995, .05, "Near entry (gap ≤ one pitch), within 150 µm offset / 2° tilt.\n"
                         "One complete ring is potential overlap; it alone does not prove engagement.",
                         transform=geometry_ax.transAxes, ha="right", va="bottom", fontsize=7.7,
                         color="#4f626d", bbox={"facecolor": "white", "alpha": .87, "edgecolor": "none"})

        pad_ax.plot(times, right_pads[:, 0], color=COLORS["right_1"], linewidth=1.2, label="Right pad 1 · bolt")
        pad_ax.plot(times, right_pads[:, 1], color=COLORS["right_2"], linewidth=1.2, label="Right pad 2 · bolt")
        pad_ax.plot(times, left_pads[:, 0], color=COLORS["left_1"], linestyle="--", linewidth=1.1,
                    label="Left pad 1 · block")
        pad_ax.plot(times, left_pads[:, 1], color=COLORS["left_2"], linestyle="--", linewidth=1.1,
                    label="Left pad 2 · block")
        pad_ax.set_ylim(bottom=-.03 * max(1., float(max(right_pads.max(), left_pads.max()))))
        pad_ax.set_ylabel("Contact normal force (N)")
        pad_ax.set_title("D  Resolved friction-grip contact loads" if table_pickup else
                        "C  Resolved friction-grip contact loads", loc="left", pad=9)
        pad_ax.legend(loc="upper right", frameon=True, framealpha=.92, fontsize=8.2, ncol=2)

        support_ax.step(times, bolt_support, where="post", color=COLORS["support"], linewidth=1.4,
                        label="Bolt ↔ world / declared rest")
        support_ax.step(times, block_support, where="post", color=COLORS["ring"], linestyle="--",
                        linewidth=1.4, label="Block ↔ world")
        support_ax.set_ylabel("World-support\ncontacts")
        support_ax.set_xlabel("Simulation time (s)")
        support_ax.set_title("E  Recorded support counts" if table_pickup else
                            "D  Recorded support counts", loc="left", pad=9)
        support_ax.set_ylim(-.4, max(1., float(max(bolt_support.max(), block_support.max()))) + 1)
        support_ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True, nbins=4))
        support_ax.legend(loc="upper right", frameon=True, framealpha=.92, fontsize=8.2)
        support_ax.text(.005, .075, "Saved samples; between-sample maxima remain in the rollout report.",
                        transform=support_ax.transAxes, fontsize=7.7, color="#4f626d")

        for axis in axes:
            axis.grid(axis="y")
            if axis is not support_ax:
                axis.tick_params(labelbottom=False)
        xpad = .008 * max(1., float(times[-1] - times[0]))
        band.set_xlim(float(times[0]) - xpad, float(times[-1]) + xpad)

        if capture_time is not None:
            for axis in axes:
                axis.axvline(float(capture_time), color=COLORS["capture"], linestyle=":", linewidth=1.2)
            tag_scope = ("Loaded-flank candidate; lead checked in later turns"
                         if has_window_observer else
                         "Legacy inline capture criterion; later turn tests check lead"
                         if legacy_verified else "Legacy tag; observer provenance unavailable")
            geometry_ax.annotate(f"Capture tag {float(capture_time):.3f} s\n" + tag_scope,
                                 xy=(float(capture_time), pitch * 1e3), xycoords="data",
                                 xytext=(.995, .65), textcoords="axes fraction", ha="right", va="center",
                                 color=COLORS["capture"], fontsize=8,
                                 arrowprops={"arrowstyle": "-", "color": COLORS["capture"], "linewidth": .8},
                                 bbox={"facecolor": "white", "alpha": .94, "edgecolor": "none"})
        else:
            geometry_ax.text(.995, .68, "No capture tag recorded", transform=geometry_ax.transAxes,
                             ha="right", color=COLORS["capture"], fontsize=8.5,
                             bbox={"facecolor": "white", "alpha": .9, "edgecolor": "none"})

        if qualified:
            qualification = "Pitch references: " + ", ".join(segment["phase"] for segment in qualified)
        else:
            qualification = "No lead-qualified turn segments; no pitch reference is drawn."
        fig.text(.095, .095, qualification, fontsize=8.5, color=COLORS["reference"])
        fig.text(.095, .072, "Entry / chamfer travel is shown as measured motion and is never labeled lead-qualified.",
                 fontsize=8.5, color="#596975")
        provenance_note = ("Observer provenance verified; the capture tag is an observation, not a motion constraint."
                           if has_window_observer else
                           "Legacy inline capture criterion verified in archived controller; lead references require actual turn checks."
                           if legacy_verified else
                           "Legacy trial: loaded-flank observer provenance absent; no engagement or lead qualification is asserted.")
        fig.text(.095, .049, provenance_note, fontsize=8, color="#596975")
        png, pdf = output_dir / f"{basename}.png", output_dir / f"{basename}.pdf"
        fig.savefig(png, dpi=180)
        fig.savefig(pdf, metadata={"Title": "Measured YAM M8 insertion rollout",
                                   "Subject": status, "Creator": Path(__file__).name})
        plt.close(fig)

    return {"png": str(png), "pdf": str(pdf), "status": status,
            "plot_scope": "Recorded samples; no integration or force reconstruction",
            "prealignment_yaw_masked": table_pickup,
            "pickup_world_height_panel": table_pickup,
            "insertion_coordinate_mask_scope": ("align_over_hole onward, radial<=150um, tilt<=2deg"
                                                  if table_pickup else None),
            "lead_qualified_turn_segments": [segment["phase"] for segment in qualified],
            "capture_tag_time_s": capture_time,
            "observer_provenance_present": has_window_observer,
            "legacy_inline_observer_verified": legacy_verified,
            "sample_count": len(rows), "last_sample_time_s": float(times[-1]),
            "pitch_m": pitch, "identity_checks": run["checks"],
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Directory containing the immutable final trace and report")
    parser.add_argument("--output-dir", type=Path, help="Defaults to the completed run directory")
    parser.add_argument("--basename", default="insertion_trajectory")
    parser.add_argument("--evidence-only", action="store_true",
                        help="Permit a stopped/failed legacy trial and label its limitations explicitly")
    parser.add_argument("--legacy-inline-observer", action="store_true",
                        help="Explicitly accept a hash-verified archived inline criterion, preserving the real outcome")
    args = parser.parse_args()
    from package_m8_insertion_demo import load_completed_run

    run = load_completed_run(args.run_dir, evidence_only=args.evidence_only,
                             legacy_inline_observer=args.legacy_inline_observer)
    output_dir = args.output_dir or args.run_dir
    result = plot_completed_run(run, output_dir, evidence_only=args.evidence_only,
                                legacy_inline_observer=args.legacy_inline_observer, basename=args.basename)
    result["source_trace"] = str(args.run_dir / "insertion_trace.npz")
    result["source_report"] = str(args.run_dir / "insertion_validation.json")
    # Hash the very bytes the identity loader validated, not a later reread.
    snapshots = run["source_bytes"]
    result["source_trace_sha256"] = hashlib.sha256(
        snapshots[args.run_dir.resolve() / "insertion_trace.npz"]).hexdigest()
    result["source_report_sha256"] = hashlib.sha256(
        snapshots[args.run_dir.resolve() / "insertion_validation.json"]).hexdigest()
    metadata_path = output_dir / f"{args.basename}.json"
    metadata_path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"png": result["png"], "pdf": result["pdf"],
                      "metadata": str(metadata_path), "status": result["status"],
                      "lead_qualified_turn_segments": result["lead_qualified_turn_segments"]}, indent=2))


if __name__ == "__main__":
    main()
