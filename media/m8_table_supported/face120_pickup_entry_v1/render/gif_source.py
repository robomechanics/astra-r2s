"""Encode every existing replay-video frame into a normal-speed GIF."""
from pathlib import Path
import hashlib
import json
import imageio.v2 as imageio
from PIL import Image

OUT = Path('/workspace/astra-r2s/outputs/m8_table_supported/face120_prefix_v1_render')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    manifest = json.loads((OUT/'render_manifest.json').read_text())
    fps = manifest['fps']
    reader = imageio.get_reader(OUT/'demo.mp4')
    frames = []
    for frame in reader:
        image = Image.fromarray(frame)
        image = image.resize((960, round(image.height * 960/image.width)))
        frames.append(image.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
    reader.close()
    assert len(frames) == manifest['video_frames']
    # GIF stores centisecond units. Alternating 80/90 ms frames preserve
    # normal-speed total time instead of rounding every frame down to 80 ms.
    duration = [10*(round((i+1)*100/fps)-round(i*100/fps)) for i in range(len(frames))]
    frames[0].save(OUT/'demo.gif',save_all=True,append_images=frames[1:],
        duration=duration,loop=0,optimize=False,disposal=2)
    result = {
        'scope': 'GIF presentation of every existing exact-state replay-video frame. Native state, force and contact archives unchanged; resizing and GIF palette encoding affect presentation only.',
        'source_video_sha256': sha(OUT/'demo.mp4'),
        'source_render_manifest_sha256': sha(OUT/'render_manifest.json'),
        'source_sha256': sha(__file__),
        'fps': fps, 'playback_speed': 1, 'frames': len(frames),
        'requested_frame_duration_ms': duration,
        'requested_duration_ms': sum(duration),
        'gif_timing_note': 'Alternating 80/90 ms GIF centisecond frame durations preserve total normal 1x time within one centisecond; MP4 uses exact 12 fps.',
        'gif_sha256': sha(OUT/'demo.gif')}
    (OUT/'gif_manifest.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (OUT/'gif_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:result[k] for k in ['frames','playback_speed','requested_duration_ms','gif_sha256']},indent=2))


if __name__ == '__main__':
    main()
