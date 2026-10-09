"""Render the real pickup/insertion scene or replay a saved physical rollout."""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MESA_SHADER_CACHE_DIR", "/workspace/.cache/mesa")
os.environ.setdefault("LP_NUM_THREADS", "2")

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from thread_lab.runtime import require_micron_engine
from .m8_demo import font, recorded_scene
from .m8_insertion_scene import (InsertionConfig, build_model, build_spec,
    scene_fingerprint, scene_xml)


def recorded_config(report):
    values = dict(report["scene_config"])
    base = recorded_scene({"scene_config": values.pop("base")})
    for key in ("block_position", "hole_offset", "bolt_head_position"):
        if key in values:
            values[key] = tuple(values[key])
    return InsertionConfig(base=base, **values)


def initialize_preview(model, data, config):
    """Set robot joint poses only; this static preview integrates no physics."""
    from .m8_insertion_simulation import initialize_insertion_pose, InsertionControlConfig
    initialize_insertion_pose(model, data, config, InsertionControlConfig())
    mujoco.mj_forward(model, data)


class InsertionRenderer:
    def __init__(self, model):
        self.model = model
        self.overview = mujoco.Renderer(model, height=720, width=864)
        self.bolt = mujoco.Renderer(model, height=330, width=432)
        self.hole = mujoco.Renderer(model, height=280, width=432)
        self.wrist = mujoco.Renderer(model, height=150, width=224)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3:] = False
        self.overview_camera = mujoco.MjvCamera()
        self.overview_camera.lookat[:] = [.30, -.035, .20]
        self.overview_camera.distance = 1.15
        self.overview_camera.azimuth, self.overview_camera.elevation = 160., -40.
        self.bolt_camera = mujoco.MjvCamera()
        self.bolt_camera.distance = .065
        self.bolt_camera.azimuth, self.bolt_camera.elevation = 140., -35.
        self.hole_camera = mujoco.MjvCamera()
        self.hole_camera.distance = .065
        self.hole_camera.azimuth, self.hole_camera.elevation = 130., -65.
        self.big, self.normal, self.small = font(27), font(20), font(16)

    def close(self):
        for renderer in (self.overview, self.bolt, self.hole, self.wrist):
            renderer.close()

    def frame(self, data, sample=None, report=None, *, static=False, slow_motion=1.5):
        sample, report = sample or {}, report or {}
        canvas = Image.new("RGB", (1296, 720), "#101924")
        self.overview.update_scene(data, camera=self.overview_camera, scene_option=self.options)
        canvas.paste(Image.fromarray(self.overview.render()), (0, 0))
        head = data.site_xpos[self.model.site("bolt_head_grasp").id]
        tip = data.site_xpos[self.model.site("bolt_tip").id]
        entry = data.site_xpos[self.model.site("hole_entry").id]
        self.bolt_camera.lookat[:] = (head + tip)/2
        self.hole_camera.lookat[:] = entry
        for renderer, camera, position in ((self.bolt, self.bolt_camera, (864, 0)),
                                           (self.hole, self.hole_camera, (864, 350))):
            renderer.update_scene(data, camera=camera, scene_option=self.options)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            canvas.paste(Image.fromarray(renderer.render()), position)
        self.wrist.update_scene(data, camera="right_camera_d405", scene_option=self.options)
        canvas.paste(Image.fromarray(self.wrist.render()), (624, 16))
        overlay = Image.new("RGBA", canvas.size)
        shade = ImageDraw.Draw(overlay)
        for box in ((0, 0, 610, 96), (0, 620, 864, 720),
                    (864, 0, 1296, 36), (864, 350, 1296, 386), (624, 16, 848, 43)):
            shade.rectangle(box, fill=(12, 22, 32, 218))
        canvas = Image.alpha_composite(canvas.convert("RGBA"), overlay).convert("RGB")
        draw = ImageDraw.Draw(canvas)
        table_pickup = report.get("scene_config", {}).get("pickup_from_table", False)
        title = "M8 block + bolt pickup" if table_pickup else "M8 bolt pickup + threaded block"
        draw.text((20, 20), title, font=self.big, fill="white")
        draw.text((20, 59), "Joint motors → fingers → free bolt / free block",
                  font=self.small, fill="#c5d6e4")
        draw.text((632, 20), "Right wrist camera", font=self.small, fill="#c5d6e4")
        draw.text((880, 7), "Male bolt · AF20 head · 16 mm shaft", font=self.small, fill="white")
        draw.text((880, 357), "Female M8 × 1.25 · open bore", font=self.small, fill="white")
        phase = str(sample.get("phase", "Pickup ready"))
        draw.text((20, 635), phase.replace("_", " "), font=self.big, fill="white")
        note = "STATIC SCENE PREVIEW · NO ROLLOUT" if static else (
            f"{data.time:.2f} s physics · {slow_motion:g}× slow motion")
        draw.text((20, 678), note, font=self.small, fill="#c5d6e4")
        if not static:
            normals = sample.get("contact", {}).get("pad_normal_force_N", [])
            left_normals = sample.get("left_contact", {}).get("pad_normal_force_N", [])
            if len(left_normals) == 2:
                draw.text((390, 632), f"Left pads: {left_normals[0]:.2f} / {left_normals[1]:.2f} N"
                          f" · table: {sample.get('block_world_support_contacts', '?')}",
                          font=self.small, fill="#c5d6e4")
            if len(normals) == 2:
                draw.text((390, 654), f"Right pads: {normals[0]:.2f} / {normals[1]:.2f} N"
                          f" · rest: {sample.get('bolt_world_support_contacts', '?')}",
                          font=self.small, fill="#c5d6e4")
            status = ("Full physical rollout passed" if report.get("passed") and not report.get("partial")
                      else "Diagnostic rollout · inspect checks")
            draw.text((390, 681), status, font=self.small, fill="#68e0c5" if report.get("passed") else "#f1c46c")
        draw.text((882, 653), "Both parts start separately on table" if table_pickup else
                  "Left secures block · right picks up bolt", font=self.small, fill="#c5d6e4")
        draw.text((882, 680), "Actual collision geometry · no grasp weld", font=self.small, fill="#c5d6e4")
        return canvas


def render_preview(config, output):
    model = build_model(config)
    data = mujoco.MjData(model)
    initialize_preview(model, data, config)
    renderer = InsertionRenderer(model)
    try:
        renderer.frame(data, report={"scene_config": {"pickup_from_table":
                       getattr(config, "pickup_from_table", False)}}, static=True).save(output)
    finally:
        renderer.close()
    return {"screenshot": str(output), "physics_rollout": False,
            "model_fingerprint": scene_fingerprint(config)}


def render_recording(trajectory, output, *, fps=12, slow_motion=1.5, still_time=None,
                     stills_only=False):
    require_micron_engine()
    with np.load(trajectory, allow_pickle=False) as saved:
        times, qpos, qvel = (saved[key].copy() for key in ("time", "qpos", "qvel"))
        samples = json.loads(str(saved["info_json"]))
        report = json.loads(str(saved["metadata_json"]))
    from scripts.audit_m8_insertion_trace import recorded_model
    model, _ = recorded_model(Path(trajectory), report)
    if (qpos.shape != (len(times), model.nq) or qvel.shape != (len(times), model.nv)
            or len(samples) != len(times) or not len(times)
            or not np.isfinite(qpos).all() or not np.isfinite(qvel).all()):
        raise ValueError("The recording lacks finite states aligned with the compiled model")
    data = mujoco.MjData(model)
    renderer = InsertionRenderer(model)
    duration = float(times[-1]-times[0])
    frames = 0 if stills_only else max(1, int(np.ceil(duration*fps*slow_motion)))
    indices = [min(len(times)-1, int(np.searchsorted(times,
               times[0]+frame/(fps*slow_motion)))) for frame in range(frames)]
    still_index = len(times)//2 if still_time is None else min(len(times)-1,
        int(np.searchsorted(times, still_time)))
    writer = None if stills_only else imageio.get_writer(output, fps=fps, codec="libx264", quality=8,
        macro_block_size=1, ffmpeg_params=["-movflags", "+faststart"])
    try:
        for frame, index in enumerate([*indices, still_index]):
            data.qpos[:], data.qvel[:], data.time = qpos[index], qvel[index], times[index]
            mujoco.mj_forward(model, data)
            canvas = renderer.frame(data, samples[index], report, slow_motion=slow_motion)
            if frame == len(indices):
                canvas.save(Path(output).with_suffix(".png"))
            if frame < len(indices):
                writer.append_data(np.asarray(canvas))
    finally:
        if writer is not None:
            writer.close()
        renderer.close()
    return {"video": str(output), "screenshot": str(Path(output).with_suffix(".png")),
            "frames": frames, "source_recording": str(trajectory),
            "trajectory_sha256": hashlib.sha256(Path(trajectory).read_bytes()).hexdigest(),
            "recorded_validation_passed": report.get("passed", False)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/m8_table_pickup/demo"))
    parser.add_argument("--preview", action="store_true", help="Static scene with robot pickup poses")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--export-only", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--dt", type=float, default=.00005)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--slow-motion", type=float, default=1.5)
    parser.add_argument("--still-time", type=float)
    parser.add_argument("--stills-only", action="store_true")
    parser.add_argument("--maximum-phases", type=int)
    parser.add_argument("--stroke-degrees", type=float, default=180.)
    parser.add_argument("--angular-speed", type=float, default=2.)
    parser.add_argument("--maximum-starting-strokes", type=int, default=5)
    parser.add_argument("--qualifying-strokes", type=int, default=2)
    parser.add_argument("--axial-damping", type=float,
                        help="Native arm axial velocity damping in N s/m (table: 50, legacy: 0)")
    parser.add_argument("--preheld-block", action="store_true",
                        help="Use the archived initial left-touching block scene")
    args = parser.parse_args()
    if not 1 <= args.fps <= 60 or args.slow_motion <= 0:
        parser.error("fps must be 1–60 and slow motion must be positive")
    require_micron_engine()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.replay:
        print(json.dumps(render_recording(args.replay, args.output/"insertion_demo.mp4",
            fps=args.fps, slow_motion=args.slow_motion, still_time=args.still_time,
            stills_only=args.stills_only)))
        return
    from .m8_insertion_scene import table_pickup_config
    config = InsertionConfig() if args.preheld_block else table_pickup_config()
    config = replace(config, base=replace(config.base,
        thread=replace(config.thread, timestep=args.dt)))
    if args.preview:
        print(json.dumps(render_preview(config, args.output/"insertion_preview.png")))
        return
    (args.output/"scene.xml").write_text(scene_xml(config))
    build_spec(config).to_zip(args.output/"insertion_scene.zip")
    if args.export_only:
        print(args.output/"insertion_scene.zip")
        return
    from .m8_simulation import YamM8ControlConfig
    from .m8_insertion_simulation import run_insertion_demo, InsertionControlConfig
    control = InsertionControlConfig(arm=replace(YamM8ControlConfig(),
        stroke_angle_rad=np.deg2rad(args.stroke_degrees), angular_speed_rad_s=args.angular_speed),
        maximum_starting_strokes=args.maximum_starting_strokes,
        qualifying_turns=args.qualifying_strokes,
        axial_velocity_damping_Ns_per_m=(args.axial_damping if args.axial_damping is not None
                                        else 0. if args.preheld_block else 50.))
    report = run_insertion_demo(args.output, scene_config=config, control_config=control,
                                maximum_phases=args.maximum_phases)
    if args.video:
        render_recording(args.output/"insertion_trace.npz", args.output/"insertion_demo.mp4",
            fps=args.fps, slow_motion=args.slow_motion, still_time=args.still_time)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
