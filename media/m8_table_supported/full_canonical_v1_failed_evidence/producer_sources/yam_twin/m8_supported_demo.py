"""Run or replay the separate native table-supported M8 assembly trajectory."""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from thread_lab.runtime import require_micron_engine
from .m8_insertion_demo import InsertionRenderer
from .m8_supported_scene import supported_config, build_spec, scene_xml
from .m8_supported_simulation import SupportedControlConfig, run_supported_demo


class SupportedRenderer(InsertionRenderer):
    def frame(self, data, sample=None, report=None, *, static=False, slow_motion=1.):
        sample, report = sample or {}, report or {}
        canvas = super().frame(data, sample, report, static=static, slow_motion=slow_motion)
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((0, 0, 610, 96), fill="#101924")
        draw.text((20, 20), "M8 bolt pickup · block stays on table", font=self.big, fill="white")
        draw.text((20, 59), "Left stabilizes · native table bears block weight", font=self.small, fill="#c5d6e4")
        draw.rectangle((864, 630, 1296, 720), fill="#101924")
        table = sample.get("table_support", {})
        if not static:
            table_force = table.get("table_upward_force_N")
            weight = sample.get("known_block_weight_N")
            if table_force is not None and weight is not None:
                draw.text((880, 634), f"Table up: {table_force:.3f} N · block mg: {weight:.3f} N",
                          font=self.small, fill="#c5d6e4")
            draw.text((880, 657), f"Block lift: {sample.get('block_lift_m', 0.)*1000:.4f} mm",
                      font=self.small, fill="#c5d6e4")
            draw.text((880, 680), "Original solved forces · free block / bolt", font=self.small, fill="#c5d6e4")
        else:
            draw.text((880, 653), "STATIC PREVIEW · no load proof", font=self.small, fill="#c5d6e4")
        return canvas


def render_recording(trajectory, output, *, fps=12, slow_motion=1., still_time=None,
                     stills_only=False):
    """Replay exact archived states/XML without integration or interpolation."""
    require_micron_engine()
    trajectory, output = Path(trajectory), Path(output)
    with np.load(trajectory, allow_pickle=False) as saved:
        times, qpos, qvel = (saved[key].copy() for key in ("time", "qpos", "qvel"))
        samples = json.loads(str(saved["info_json"]))
        report = json.loads(str(saved["metadata_json"]))
    from scripts.audit_m8_insertion_trace import recorded_model
    model, identity = recorded_model(trajectory, report)
    if (qpos.shape != (len(times), model.nq) or qvel.shape != (len(times), model.nv)
            or len(samples) != len(times) or not len(times)
            or not np.isfinite(qpos).all() or not np.isfinite(qvel).all()):
        raise ValueError("Recording lacks finite states aligned with archived model")
    if not np.isfinite(slow_motion) or slow_motion <= 0 or not 1 <= fps <= 60:
        raise ValueError("Replay speed must be finite/positive and FPS1–60")
    data, renderer = mujoco.MjData(model), SupportedRenderer(model)
    duration = float(times[-1]-times[0])
    frames = 0 if stills_only else max(1, int(np.ceil(duration*fps*slow_motion)))
    indices = [min(len(times)-1, int(np.searchsorted(times,
        times[0]+frame/(fps*slow_motion)))) for frame in range(frames)]
    still_index = len(times)-1 if still_time is None else min(len(times)-1,
        int(np.searchsorted(times, still_time)))
    writer = None if stills_only else imageio.get_writer(output, fps=fps, codec="libx264", quality=8,
        macro_block_size=1, ffmpeg_params=["-movflags", "+faststart", "-pix_fmt", "yuv420p"])
    try:
        for frame, index in enumerate([*indices, still_index]):
            data.qpos[:], data.qvel[:], data.time = qpos[index], qvel[index], times[index]
            mujoco.mj_forward(model, data)
            canvas = renderer.frame(data, samples[index], report, slow_motion=slow_motion)
            if frame == len(indices):
                canvas.save(output.with_suffix(".png"))
            elif writer is not None:
                writer.append_data(np.asarray(canvas))
    finally:
        if writer is not None:
            writer.close()
        renderer.close()
    return {"video": None if stills_only else str(output), "screenshot": str(output.with_suffix(".png")),
        "frames": frames, "fps": fps, "slow_motion": slow_motion,
        "source_recording": str(trajectory), "trajectory_sha256": hashlib.sha256(trajectory.read_bytes()).hexdigest(),
        "archived_model_identity": identity, "recorded_validation_passed": report.get("passed", False),
        "force_timing": report.get("native_force_recording_note"),
        "scope": "Actual saved native post-integration qpos/qvel replay; displayed original solved forces are from time minus timestep"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/m8_supported/demo"))
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--maximum-phases", type=int)
    parser.add_argument("--dt", type=float, default=.00005)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--slow-motion", type=float, default=1.)
    parser.add_argument("--still-time", type=float)
    parser.add_argument("--stills-only", action="store_true")
    parser.add_argument("--starting-angular-speed", type=float, default=1.)
    parser.add_argument("--angular-speed", type=float, default=2.)
    parser.add_argument("--maximum-entry-dwell", type=float, default=3.)
    parser.add_argument("--maximum-starting-strokes", type=int, default=5)
    parser.add_argument("--qualifying-strokes", type=int, default=2)
    parser.add_argument("--axial-damping", type=float, default=200.)
    args = parser.parse_args()
    require_micron_engine()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.replay:
        print(json.dumps(render_recording(args.replay, args.output/"supported_demo.mp4",
            fps=args.fps, slow_motion=args.slow_motion, still_time=args.still_time,
            stills_only=args.stills_only), indent=2))
        return
    scene = supported_config()
    scene = replace(scene, base=replace(scene.base, thread=replace(scene.thread, timestep=args.dt)))
    (args.output/"scene.xml").write_text(scene_xml(scene))
    build_spec(scene).to_zip(args.output/"supported_scene.zip")
    if args.export_only:
        print(args.output/"supported_scene.zip")
        return
    defaults = SupportedControlConfig()
    control = replace(defaults, arm=replace(defaults.arm, angular_speed_rad_s=args.angular_speed),
        starting_angular_speed_rad_s=args.starting_angular_speed,
        maximum_entry_dwell_s=args.maximum_entry_dwell,
        maximum_starting_strokes=args.maximum_starting_strokes, qualifying_turns=args.qualifying_strokes,
        axial_velocity_damping_Ns_per_m=args.axial_damping)
    (args.output/"renderer_source.py").write_bytes(Path(__file__).read_bytes())
    report = run_supported_demo(args.output, scene_config=scene, control_config=control,
                                maximum_phases=args.maximum_phases)
    if args.video:
        render_recording(args.output/"insertion_trace.npz", args.output/"supported_demo.mp4",
            fps=args.fps, slow_motion=args.slow_motion, still_time=args.still_time)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
