"""Run and render the contact-driven M8 task in the actual dual-YAM workcell."""
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
from PIL import Image, ImageDraw, ImageFont

from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine
from .m8_scene import YamM8Config, build_model, build_spec, scene_xml, scene_fingerprint


def recorded_scene(report):
    """Rebuild the configuration that produced a trajectory, including the fit."""
    values = dict(report["scene_config"])
    thread = values.pop("thread", report.get("thread_config", {}))
    for key in ("bolt_position", "bolt_quaternion", "block_size", "bolt_offset", "left_grasp_offset", "left_block_contact_impedance"):
        if key in values:
            values[key] = tuple(values[key])
    return YamM8Config(thread=ThreadConfig(**thread), **values)


def font(size):
    return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)


def render_recording(trajectory, output, *, fps=12, slow_motion=1.5,
                     still_time=None, stills_only=False):
    """Replay actual recorded states; no physical rollout or interpolation."""
    require_micron_engine()
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with np.load(trajectory, allow_pickle=False) as archive:
        times, qpos = archive["time"].copy(), archive["qpos"].copy()
        qvel = archive["qvel"].copy()
        report = json.loads(str(archive["metadata_json"]))
        samples = json.loads(str(archive["info_json"]))
    scene = recorded_scene(report)
    model = build_model(scene)
    if qpos.shape != (len(times), model.nq) or qvel.shape != (len(times), model.nv):
        raise ValueError("The recording does not match the compiled YAM M8 scene")
    if len(times) != len(samples) or not len(times) or not np.isfinite(qpos).all():
        raise ValueError("The recording is missing finite, aligned sample data")
    expected_model = report.get("model_fingerprint")
    expected_xml = report.get("model_xml_sha256")
    if expected_model:
        model_matches = scene_fingerprint(scene) == expected_model
    elif expected_xml:
        model_matches = hashlib.sha256(scene_xml(scene).encode()).hexdigest() == expected_xml
    else:
        raise ValueError("The recording lacks scene provenance")
    if not model_matches:
        raise ValueError("The recorded scene source or model settings have changed")
    data = mujoco.MjData(model)
    bolt_id = model.body("bolt_frame").id
    overview = mujoco.Renderer(model, height=720, width=864)
    detail = mujoco.Renderer(model, height=330, width=432)
    wrist = mujoco.Renderer(model, height=150, width=224)
    overview_camera = mujoco.MjvCamera()
    overview_camera.lookat[:] = [.28, 0., .20]
    overview_camera.distance = 1.15
    overview_camera.azimuth, overview_camera.elevation = 160., -40.
    options = mujoco.MjvOption()
    options.geomgroup[3:] = False
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    turning = [i for i, s in enumerate(samples) if s["phase"].startswith("turn_")]
    still_index = (turning[len(turning)//2] if turning else len(times)//2) if still_time is None else min(
        len(times)-1, int(np.searchsorted(times, still_time)))
    duration = float(times[-1] - times[0])
    frames = max(1, int(np.ceil(duration * slow_motion * fps)))
    indices = [] if stills_only else [min(len(times)-1, int(np.searchsorted(
        times, times[0] + i / (fps * slow_motion)))) for i in range(frames)]
    writer = None if stills_only else imageio.get_writer(output, fps=fps, codec="libx264", quality=8,
        macro_block_size=1, ffmpeg_params=["-movflags", "+faststart"])
    big, normal, small = font(27), font(20), font(16)
    try:
        for frame, index in enumerate([*indices, still_index]):
            data.qpos[:] = qpos[index]
            data.qvel[:] = qvel[index]
            data.time = times[index]
            mujoco.mj_forward(model, data)
            sample = samples[index]
            overview.update_scene(data, camera=overview_camera, scene_option=options)
            canvas = Image.new("RGB", (1296, 720), "#101924")
            canvas.paste(Image.fromarray(overview.render()), (0, 0))
            detail.update_scene(data, camera="threadcloseup", scene_option=options)
            detail.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            canvas.paste(Image.fromarray(detail.render()), (864, 0))
            wrist.update_scene(data, camera="right_camera_d405", scene_option=options)
            wrist.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            canvas.paste(Image.fromarray(wrist.render()), (624, 16))
            draw = ImageDraw.Draw(canvas, "RGBA")
            draw.rectangle((0, 0, 606, 94), fill=(16, 25, 36, 230))
            draw.text((20, 18), "M8 contact · dual-YAM workcell", font=big, fill="white")
            draw.text((20, 59), "Joint motors → fingers → free block / nut", font=small, fill="#c6d5e4")
            draw.rectangle((624, 16, 848, 42), fill=(16, 25, 36, 230))
            draw.text((636, 20), "Right wrist camera", font=small, fill="#c6d5e4")
            draw.rectangle((864, 0, 1296, 38), fill=(16, 25, 36, 220))
            draw.text((882, 9), "Recorded thread / finger contacts", font=small, fill="white")
            draw.rectangle((0, 619, 864, 720), fill=(16, 25, 36, 235))
            draw.text((20, 635), sample["phase"].replace("_", " "), font=big, fill="white")
            draw.text((20, 677), f"{times[index]:.2f} s physics · {slow_motion:g}× slow motion", font=small, fill="#c6d5e4")
            draw.text((485, 637), "Left holds block · right turns nut", font=small, fill="#c6d5e4")
            draw.text((485, 674), "M8 bore · AF20 nut · 16 mm shaft", font=small, fill="#c6d5e4")
            draw.text((882, 348), f"Nut turns: {sample['clockwise_turns']:.3f}", font=normal, fill="white")
            draw.text((882, 380), f"Advance: {sample['axial_advance_mm']:.3f} mm", font=normal, fill="white")
            contact = sample.get("contact", {})
            torque = np.asarray(contact.get("pad_wrench_world", [0.]*6))[3:]
            axis = data.xmat[bolt_id].reshape(3, 3)[:, 2]
            pad_torque = abs(float(torque @ axis)) * 1000
            draw.text((882, 415), f"Pad torque: {pad_torque:.3f} mNm", font=small, fill="#c6d5e4")
            normal_forces = contact.get("pad_normal_force_N", [0., 0.])
            draw.text((882, 442), "Pad normals: " + " / ".join(f"{v:.2f}" for v in normal_forces) + " N",
                      font=small, fill="#c6d5e4")
            draw.text((882, 469), f"Reported SDF depth: {sample['reported_sdf_depth_m']*1e6:.2f} µm",
                      font=small, fill="#c6d5e4")
            left, top, width, height = 884, 511, 365, 137
            max_turns = max(1., max(s["clockwise_turns"] for s in samples))
            max_advance = max(scene.thread.pitch*1000*max_turns*1.1,
                              max(s["axial_advance_mm"] for s in samples)*1.1)
            point = lambda a, b: (left+width*a/max_turns, top+height-height*b/max_advance)
            draw.line([(left, top), (left, top+height), (left+width, top+height)], fill="#60768a", width=2)
            draw.line([point(0, 0), point(max_turns, scene.thread.pitch*1000*max_turns)], fill="#6686aa", width=2)
            points = [point(s["clockwise_turns"], s["axial_advance_mm"]) for s in samples[:index+1]]
            if len(points) > 1:
                draw.line(points, fill="#53dec4", width=3)
            draw.text((882, 656), "Observed advance vs. clockwise turns", font=small, fill="#53dec4")
            draw.text((882, 681), "Blue: 1.25 mm/rev reference", font=small, fill="#93aec7")
            if report.get("partial", False):
                status = ("PARTIAL PHYSICS RUN" if "acceptance_checks" in report or "hold_checks" in report
                          else "PHYSICS RUN IN PROGRESS")
                draw.text((20, 106), status, font=small, fill="#ffbd68")
            elif not report.get("passed", False):
                draw.text((20, 106), "DIAGNOSTIC: validation incomplete / failed", font=small, fill="#ffbd68")
            if frame < len(indices):
                writer.append_data(np.asarray(canvas))
            else:
                canvas.save(output.with_suffix(".png"))
    finally:
        if writer is not None:
            writer.close()
        overview.close()
        detail.close()
        wrist.close()
    return {"video": None if stills_only else str(output), "screenshot": str(output.with_suffix(".png")),
            "frames": 0 if stills_only else frames, "source_recording": str(trajectory),
            "still_time_s": float(times[still_index]), "recorded_validation_passed": report.get("passed", False)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/yam_m8/demo"))
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--replay", type=Path, help="Render recorded physics without rerunning it")
    parser.add_argument("--stills-only", action="store_true")
    parser.add_argument("--still-time", type=float)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--slow-motion", type=float, default=1.5)
    parser.add_argument("--strokes", type=int, default=3)
    parser.add_argument("--stroke-degrees", type=float, default=120.)
    parser.add_argument("--angular-speed", type=float, default=1.)
    parser.add_argument("--dt", type=float, default=.000025)
    parser.add_argument("--maximum-phases", type=int, help="Diagnostic partial run; full-demo gates remain required")
    parser.add_argument("--export-only", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.fps <= 60 or args.slow_motion <= 0:
        parser.error("fps must be 1–60 and slow motion must be positive")
    require_micron_engine()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.replay:
        result = render_recording(args.replay, args.output/"yam_m8_demo.mp4", fps=args.fps,
            slow_motion=args.slow_motion, still_time=args.still_time, stills_only=args.stills_only)
        print(json.dumps(result))
        return
    scene = YamM8Config()
    scene = replace(scene, thread=replace(scene.thread, timestep=args.dt))
    (args.output/"scene.xml").write_text(scene_xml(scene))
    build_spec(scene).to_zip(args.output/"yam_m8_scene.zip")
    if args.export_only:
        print(args.output/"yam_m8_scene.zip")
        return
    from .m8_simulation import YamM8ControlConfig, run_demo
    control = YamM8ControlConfig(strokes=args.strokes, stroke_angle_rad=np.deg2rad(args.stroke_degrees),
                                angular_speed_rad_s=args.angular_speed)
    report = run_demo(args.output, scene_config=scene, control_config=control,
                      maximum_phases=args.maximum_phases)
    if args.video:
        render_recording(args.output/"yam_m8_trace.npz", args.output/"yam_m8_demo.mp4", fps=args.fps,
            slow_motion=args.slow_motion, still_time=args.still_time, stills_only=args.stills_only)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
