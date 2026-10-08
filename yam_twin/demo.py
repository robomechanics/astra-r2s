"""Run, render, or inspect the reconstructed dual-YAM demonstration."""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

# Choose headless Mesa before importing MuJoCo/PyOpenGL. --viewer uses GLFW.
if "--viewer" not in __import__("sys").argv:
    os.environ.setdefault("MUJOCO_GL","egl")
os.environ.setdefault("MESA_SHADER_CACHE_DIR","/workspace/.cache/mesa")
os.environ.setdefault("LP_NUM_THREADS","2")

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .scene import generate_scene, build_model, INSERTION
from .simulation import TwinSimulation, DURATION, DRIVE_END


def font(size):
    for path in ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf","/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"]:
        if Path(path).exists():
            return ImageFont.truetype(path,size)
    return ImageFont.load_default()


class DemoRenderer:
    def __init__(self,simulation,width=960,height=720):
        self.sim = simulation
        self.width = width; self.height = height
        self.render = mujoco.Renderer(simulation.model,height,width)
        self.inset = mujoco.Renderer(simulation.model,168,224)
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3:] = 0

    def frame(self):
        s = self.sim
        self.render.update_scene(s.data,camera="overview",scene_option=self.options)
        frame = Image.fromarray(self.render.render())
        draw = ImageDraw.Draw(frame,"RGBA")
        for side,x in [("left",12),("right",self.width-236)]:
            self.inset.update_scene(s.data,camera=f"{side}_camera_d405",scene_option=self.options)
            self.inset.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            wrist = Image.fromarray(self.inset.render())
            frame.paste(wrist,(x,12))
            draw.rounded_rectangle((x,12,x+224,42),radius=3,fill=(10,18,28,205))
            draw.text((x+8,16),side.capitalize()+" wrist",fill=(108,171,255,255),font=font(18))
        y = self.height-88
        draw.rectangle((0,y,self.width,self.height),fill=(10,18,28,230))
        draw.text((20,y+10),s.phase,fill=(245,247,250,255),font=font(21))
        depth = float(-s.data.qpos[s.depth_q]*1000)
        draw.text((20,y+44),f"{s.data.time:05.1f} s  |  screw depth {depth:.2f} / {INSERTION*1000:.1f} mm",fill=(185,201,218,255),font=font(17))
        draw.text((self.width-320,y+12),"VIDEO-INFORMED RECONSTRUCTION",fill=(185,201,218,255),font=font(12))
        draw.rounded_rectangle((self.width-320,y+44,self.width-20,y+61),radius=7,fill=(55,67,81,255))
        progress = min(1.,max(0.,depth/(INSERTION*1000)))
        draw.rounded_rectangle((self.width-320,y+44,self.width-320+max(14,300*progress),y+61),radius=7,fill=(67,198,170,255))
        return np.asarray(frame)

    def closeup(self):
        self.render.update_scene(self.sim.data,camera="closeup",scene_option=self.options)
        return self.render.render().copy()

    def close(self):
        self.render.close();self.inset.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=Path("outputs"))
    parser.add_argument("--video",action="store_true",help="Render an MP4 with wrist-camera insets")
    parser.add_argument("--viewer",action="store_true",help="Live MuJoCo viewer (needs local display)")
    parser.add_argument("--seconds",type=float,default=DURATION)
    parser.add_argument("--fps",type=int,default=24)
    parser.add_argument("--export-only",action="store_true")
    parser.add_argument("--replay",type=Path,help="Render a saved trajectory without rerunning physics")
    parser.add_argument("--allow-incomplete",action="store_true",help="Permit a partial demonstration without seating the screw")
    args = parser.parse_args()
    if not 0 < args.seconds <= DURATION or not 1 <= args.fps <= 60:
        parser.error(f"seconds must be in (0,{DURATION}], fps in [1,60]")
    args.output.mkdir(parents=True,exist_ok=True)
    generate_scene(args.output/"scene.xml")
    if args.export_only:
        print(args.output/"scene.xml")
        return
    if args.replay:
        render_trajectory(args.replay,args.output,args.fps)
        return
    sim = TwinSimulation()
    renderer = DemoRenderer(sim) if args.video else None
    writer = imageio.get_writer(args.output/"twin_demo.mp4",fps=args.fps,codec="libx264",quality=8,macro_block_size=16) if args.video else None
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model,sim.data)
    positions=[];velocities=[];times=[]
    began = time.monotonic()
    milestones = {4:"pickup",9:"presentation",16:"screwing",DRIVE_END:"seated",DURATION:"complete"}
    saved=set()
    try:
        for i in range(1,math_ceil(args.seconds*args.fps)+1):
            target = min(i/args.fps,args.seconds)
            sim.advance(target)
            positions.append(sim.data.qpos.copy());velocities.append(sim.data.qvel.copy());times.append(float(sim.data.time))
            if writer:
                rendered = renderer.frame()
                writer.append_data(rendered)
                for t,name in milestones.items():
                    if target >= t and name not in saved:
                        Image.fromarray(rendered).save(args.output/f"{name}.png")
                        if name in ["screwing","seated"]:
                            Image.fromarray(renderer.closeup()).save(args.output/f"{name}_closeup.png")
                        saved.add(name)
            if viewer:
                if not viewer.is_running():
                    break
                viewer.sync()
                time.sleep(max(0.,target-(time.monotonic()-began)))
            if i%max(1,args.fps*4)==0:
                print(f"{target:.1f}s: {sim.phase}; insertion {sim.records[-1]['insertion_mm']:.2f}mm",flush=True)
    finally:
        if writer:writer.close()
        if renderer:renderer.close()
        if viewer:viewer.close()
    report=sim.report()
    (args.output/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
    with (args.output/"metrics.csv").open("w",newline="") as f:
        csvwriter=csv.DictWriter(f,fieldnames=list(sim.records[0]));csvwriter.writeheader();csvwriter.writerows(sim.records)
    np.savez_compressed(args.output/"trajectory.npz",time=np.array(times),qpos=np.array(positions),qvel=np.array(velocities),phase=np.array([r["phase"] for r in sim.records]))
    print(json.dumps(report,indent=2))
    if not report["success"] and not args.allow_incomplete:
        raise SystemExit("Demonstration did not complete successfully; see validation.json")


def math_ceil(x):
    return int(np.ceil(x))


def render_trajectory(path,output,fps):
    trajectory=np.load(path,allow_pickle=False)
    model=build_model(); data=mujoco.MjData(model)
    if trajectory["qpos"].shape[1] != model.nq:
        raise ValueError("Saved trajectory does not match this model")
    state=SimpleNamespace(model=model,data=data,phase="",depth_q=model.joint("screw_depth").qposadr[0])
    renderer=DemoRenderer(state)
    duration=float(trajectory["time"][-1])
    milestones={4:"pickup",9:"presentation",16:"screwing",DRIVE_END:"seated",duration:"complete"}
    saved=set()
    try:
        with imageio.get_writer(output/"twin_demo.mp4",fps=fps,codec="libx264",quality=8,macro_block_size=16) as writer:
            for i in range(1,math_ceil(duration*fps)+1):
                t=min(i/fps,duration)
                idx=min(int(np.searchsorted(trajectory["time"],t)),len(trajectory["time"])-1)
                data.qpos[:]=trajectory["qpos"][idx]
                data.qvel[:]=trajectory["qvel"][idx]
                data.time=float(trajectory["time"][idx])
                state.phase=str(trajectory["phase"][idx])
                mujoco.mj_forward(model,data)
                frame=renderer.frame();writer.append_data(frame)
                for timestamp,name in milestones.items():
                    if t >= timestamp and name not in saved:
                        Image.fromarray(frame).save(output/f"{name}.png")
                        if name in ["screwing","seated"]:
                            Image.fromarray(renderer.closeup()).save(output/f"{name}_closeup.png")
                        saved.add(name)
                if i%(fps*4)==0:print(f"Rendered {t:.1f}/{duration:.1f}s",flush=True)
    finally:
        renderer.close()


if __name__ == "__main__":
    main()
