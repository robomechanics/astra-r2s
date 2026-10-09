"""Render this immutable phase-only snapshot with geometry refresh only.

The canonical renderer, cameras, view options, overlays, and saved-state
selection are unchanged. Its mj_forward refresh is replaced in this process
by mj_kinematics, mj_comPos, and mj_camlight. No integration, collision
discovery, or dynamics/contact-force solve is called by this helper.
"""
from pathlib import Path
import hashlib
import json
import os
import sys

os.environ["MUJOCO_GL"] = "egl"
os.environ["LP_NUM_THREADS"] = "2"
REPO = Path("/workspace/astra-r2s")
sys.path.insert(0, str(REPO))

import mujoco
import numpy as np
from yam_twin import m8_supported_demo as demo


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "insertion_trace_partial.npz"
OUTPUT = HERE / "pickup_align_progress.mp4"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def geometry_refresh(model, data):
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_camlight(model, data)


def main():
    manifest = json.loads((HERE / "progress_snapshot.json").read_text())
    identity = json.loads((HERE / "run_publication_identity.json").read_text())
    if digest(SOURCE) != manifest["snapshot_trajectory_sha256"]:
        raise RuntimeError("Immutable progress snapshot changed")
    for relative, expected in identity["source_hashes_before"].items():
        if digest(REPO / relative) != expected:
            raise RuntimeError("Producer-bound source changed: " + relative)
    with np.load(SOURCE, allow_pickle=False) as saved:
        times = saved["time"].copy()
        samples = json.loads(str(saved["info_json"]))
    fps = 12
    frames = max(1, int(np.ceil(float(times[-1]-times[0])*fps)))
    selected = [min(len(times)-1, int(np.searchsorted(times, times[0]+frame/fps)))
                for frame in range(frames)]
    original_forward = demo.mujoco.mj_forward
    try:
        demo.mujoco.mj_forward = geometry_refresh
        result = demo.render_recording(SOURCE, OUTPUT, fps=fps, slow_motion=1.,
                                       still_time=float(times[-1]))
    finally:
        demo.mujoco.mj_forward = original_forward
    for relative, expected in identity["source_hashes_before"].items():
        if digest(REPO / relative) != expected:
            raise RuntimeError("Producer-bound source changed during render: " + relative)
    result.update({
        "scope": manifest["scope"],
        "producer_commit": identity["producer_commit"],
        "refresh": ["mj_kinematics", "mj_comPos", "mj_camlight"],
        "integration_called": False,
        "mj_forward_called": False,
        "collision_discovery_called": False,
        "dynamics_or_contact_force_solve_called": False,
        "pose_interpolation_or_manual_pose_called": False,
        "canonical_renderer_and_camera_configuration_unchanged": True,
        "canonical_view_options_unchanged": True,
        "canonical_view_option_note": "Renderer uses its original geomgroup[3:] visibility filter; no per-object hiding or additional visibility changes.",
        "overlay_force_scope": "Archived original native solved force samples at saved time minus one timestep; no forces are reconstructed for these media.",
        "render_helper_sha256": digest(__file__),
        "renderer_sha256": digest(REPO / "yam_twin/m8_supported_demo.py"),
        "base_renderer_sha256": digest(REPO / "yam_twin/m8_insertion_demo.py"),
        "source_hashes_before_and_after_render_matched": True,
        "frame_sample_indices": selected,
        "frame_saved_times_s": [float(times[index]) for index in selected],
        "frame_original_phases": [samples[index]["phase"] for index in selected],
        "screenshot_saved_sample_index": len(times)-1,
        "screenshot_saved_time_s": float(times[-1]),
        "screenshot_original_phase": samples[-1]["phase"],
        "video_sha256": digest(OUTPUT),
        "screenshot_sha256": digest(OUTPUT.with_suffix(".png")),
    })
    (HERE / "render_manifest.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({key: result[key] for key in
        ("video", "screenshot", "frames", "fps", "slow_motion", "scope")}, indent=2))


if __name__ == "__main__":
    main()
