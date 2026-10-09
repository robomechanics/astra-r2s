"""Render recorded contact rollouts; rendering never generates their motion."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("LP_NUM_THREADS", "2")
os.environ.setdefault("MESA_SHADER_CACHE_DIR", "/workspace/.cache/mesa")

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .model import ThreadConfig, make_model
from .gripper import GripperConfig


def render_rollout(path, output, *, fps=20, slow_motion=4, report_path=None,
                   view="clear", still_time=None, stills_only=False,
                   azimuth=90., elevation=-16.):
    archive = np.load(path, allow_pickle=False)
    samples = json.loads(str(archive["info_json"]))
    report = json.loads(str(archive["metadata_json"])) if "metadata_json" in archive else samples
    if isinstance(report, list):
        report = json.loads(Path(report_path or Path(path).with_suffix(".json")).read_text())
    config = ThreadConfig(**report["config"])
    gripper_config = GripperConfig(**report["gripper_config"]) if report.get("gripper_config") else None
    model = make_model(config, gripper_config=gripper_config)
    if view not in ("clear", "thread-view", "opaque"):
        raise ValueError("Unknown rendering view")
    # Presentation only: these RGBA changes do not alter contact shapes,
    # masses, forces, or any state from the recorded rollout.
    if view != "opaque":
        for gid in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if name.startswith(("contact_hand_palm_", "contact_backing_")):
                model.geom_rgba[gid, 3] = .12
            elif name.startswith("contact_pad_"):
                model.geom_rgba[gid] = [.08, .25, .31, 1.]
        if view == "thread-view":
            model.geom_rgba[model.geom("nut_thread").id] = [.95, .71, .26, .52]
    model.light_ambient[:] = [.18, .18, .20]
    data = mujoco.MjData(model)
    qpos, times = archive["qpos"], archive["time"]
    nid = model.body("nut").id
    qaddr = int(model.jnt_qposadr[model.joint("nut_free").id])
    quats = qpos[:, qaddr+3:qaddr+7]
    w, x, y, z = quats.T
    yaw = np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    turns = -(yaw-yaw[0])/(2*np.pi)
    advance = (qpos[0, qaddr+2]-qpos[:, qaddr+2])*1000
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .015]
    camera.distance, camera.azimuth, camera.elevation = .074, azimuth, elevation
    renderer = mujoco.Renderer(model, height=720, width=800)
    options = mujoco.MjvOption()
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 22)
    big = ImageFont.truetype(font_path, 29)
    small = ImageFont.truetype(font_path, 18)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    preliminary = report.get("preliminary_stock_engine", False)
    samples = samples if isinstance(samples, list) else report.get("trace", [])
    if still_time is None:
        turning = [i for i, s in enumerate(samples) if s.get("phase") == "turn_1"]
        still_index = turning[len(turning)//2] if turning else len(times)//2
    else:
        still_index = min(len(times)-1, int(np.searchsorted(times, still_time)))
    writer = None if stills_only else imageio.get_writer(output, fps=fps, codec="libx264", quality=8,
        macro_block_size=1, pixelformat="yuv420p", ffmpeg_params=["-movflags", "+faststart"])
    frames = max(1, round((times[-1]-times[0])*slow_motion*fps))
    try:
        indices = [] if stills_only else [min(len(times)-1, int(np.searchsorted(
            times, times[0] + frame/(fps*slow_motion)))) for frame in range(frames)]
        # Always render the chosen contact-loaded screenshot separately. A
        # video's midpoint can fall during the open-hand reset phase.
        for frame, k in enumerate([*indices, still_index]):
            data.qpos[:] = qpos[k]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=options)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            canvas = Image.new("RGB", (1280, 720), "#101924")
            canvas.paste(Image.fromarray(renderer.render()), (0, 0))
            draw = ImageDraw.Draw(canvas)
            draw.text((24, 24), "M8 contact experiment", font=big, fill="white")
            draw.text((24, 66), "Pre-engaged nut · frictional proxy fingers", font=small, fill="#d2e0ee")
            draw.text((832, 28), "Recorded physics rollout", font=font, fill="white")
            draw.text((832, 68), f"{slow_motion:g}× slow motion · μ = {config.friction:g} assumed", font=small, fill="#a9bdd1")
            if preliminary:
                draw.text((832, 104), "PRELIMINARY: stock-engine run", font=small, fill="#ffbd68")
            elif report.get("passed") is False:
                draw.text((832, 104), "Experiment: validation gates failed", font=small, fill="#ffbd68")
            draw.text((832, 158), f"Time: {times[k]:.2f} s", font=font, fill="white")
            draw.text((832, 202), f"Nut turns: {turns[k]:.3f}", font=font, fill="white")
            draw.text((832, 246), f"Advance: {advance[k]:.3f} mm", font=font, fill="white")
            if samples:
                s = samples[min(k, len(samples)-1)]
                contact = s.get("contact", {})
                torque = contact.get("pad_wrench_world", [0]*6)[5] if contact else s.get('pad_torque', s.get('pad_torque_Nm', 0))
                draw.text((832, 292), f"Pad torque |z|: {abs(torque)*1000:.3f} mNm", font=small, fill="#a9bdd1")
                normal = contact.get("pad_normal_force_N", [0]) if contact else s.get('normal', s.get('pad_normal_forces_N', [0]))
                draw.text((832, 322), f"Pad normals: {' / '.join(f'{n:.2f}' for n in normal)} N", font=small, fill="#a9bdd1")
                if 'phase' in s:
                    draw.text((832, 356), s['phase'].replace('_', ' '), font=small, fill="white")
            left, top, width, height = 850, 402, 365, 196
            draw.line([(left, top), (left, top+height), (left+width, top+height)], fill="#566b7f", width=2)
            max_turn = max(1., float(turns.max()))
            max_advance = max(1.4, float(advance.max())*1.08)
            def point(a, b):
                return (left+width*a/max_turn, top+height-height*b/max_advance)
            draw.line([point(0, 0), point(max_turn, max_turn*config.pitch*1000)], fill="#6686aa", width=2)
            curve = [point(a, b) for a,b in zip(turns[:k+1], advance[:k+1])]
            if len(curve)>1:
                draw.line(curve, fill="#53dec4", width=3)
            draw.text((850, 616), "Observed advance vs. clockwise turns", font=small, fill="#53dec4")
            draw.text((850, 644), f"Blue: {config.pitch*1000:g} mm/rev reference", font=small, fill="#8aa7cb")
            if view != "opaque":
                label = "Housings translucent for visibility"
                if view == "thread-view":
                    label += " · nut translucent"
                draw.rectangle((16, 664, 785, 707), fill="#101924")
                draw.text((24, 675), label, font=small, fill="#d2e0ee")
            if frame < len(indices):
                writer.append_data(np.asarray(canvas))
            else:
                canvas.save(output.with_suffix(".png"))
    finally:
        if writer is not None:
            writer.close()
        renderer.close()
    return {"video": None if stills_only else str(output), "still": str(output.with_suffix(".png")),
            "frames": 0 if stills_only else frames, "source_rollout": str(path), "view": view,
            "still_time_s": float(times[still_index]),
            "still_phase": samples[still_index].get("phase") if samples else None,
            "preliminary": preliminary}


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("trajectory", type=Path)
    p.add_argument("--output", type=Path, default=Path("outputs/m8/contact_demo.mp4"))
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--slow-motion", type=float, default=4)
    p.add_argument("--report", type=Path, help="Metadata sidecar when archive stores samples only")
    p.add_argument("--view", choices=["clear", "thread-view", "opaque"], default="clear")
    p.add_argument("--still-time", type=float, help="Recorded physics time for the screenshot; default is mid-turn")
    p.add_argument("--stills-only", action="store_true", help="Render one recorded frame without encoding a video")
    p.add_argument("--azimuth", type=float, default=90.)
    p.add_argument("--elevation", type=float, default=-16.)
    a = p.parse_args()
    print(json.dumps(render_rollout(a.trajectory, a.output, fps=a.fps, slow_motion=a.slow_motion,
        report_path=a.report, view=a.view, still_time=a.still_time, stills_only=a.stills_only,
        azimuth=a.azimuth, elevation=a.elevation)))


if __name__ == "__main__":
    main()
