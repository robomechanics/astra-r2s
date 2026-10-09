"""Display-only 640-pixel GIF from the CLOSED original rendered GIF.

Preserve every decoded source frame, its order, duration and loop. Only raster
resize/quantization changes. No engine, model, arrays, state interpolation,
physics, renderer, native/audit updates or new acceptance proof.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

RENDER_MANIFEST_SHA = 'fc9ef4152dc122f7b242cc27139d04c8a6a9d54e8d19b7f61db28c3b7a098337'
REVIEWER_SHA = '8d58268df1ca851f9bda89f0899349bc0f33cce0c1ec0cf4bd74bea8789dbd4e'
LIMIT = 45_000_000


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda:stream.read(1024*1024),b''):h.update(data)
    return h.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(),parse_constant=lambda value:
        (_ for _ in ()).throw(ValueError(value)))


def inventory(root):
    result = {}
    for p in sorted(root.rglob('*')):
        if p.is_symlink():raise ValueError('Symlink renderer input refused')
        if p.is_file():result[p.relative_to(root).as_posix()] = sha(p)
    return result


def main(args):
    root,render,output = args.repository_root.resolve(),args.render.resolve(),args.output.resolve()
    if (output.exists() or output.is_relative_to(render) or render.is_relative_to(output)
            or not output.is_relative_to(root/'outputs/m8_table_supported/full_reset_speed2_v3_closed_side_artifacts')):
        raise ValueError('Fresh display-only SIDE output outside original render is required')
    mpath,rpath = render/'render_manifest.json',render/'media_identity_review.json'
    # Refuse before ANY raster import/decode unless actual frozen CLOSED media passed.
    if sha(mpath) != RENDER_MANIFEST_SHA:raise ValueError('Original closed render anchor differs')
    m,r = read(mpath),read(rpath)
    if (r['passed'] is not True or r['verification_source_sha256'] != REVIEWER_SHA
            or r['render_manifest_sha256'] != RENDER_MANIFEST_SHA
            or m['native_closed_result'] is not True or m['mode'] != 'closed_native'
            or m['frames'] != 648 or m['gif_frames'] != 648 or m['gif_duration_ms'] != 54000
            or r['mp4_frames'] != 648 or r['gif_duration_ms'] != 54000
            or r['original_native_exit_code'] != m['original_native_exit_code']):
        raise ValueError('Exact original passed media review is required; not a new physics gate')
    before = inventory(render)
    for name,digest in m['media_sha256'].items():
        if before.get(name) != digest:raise ValueError('Original rendered media differs: '+name)
    sources,runtime = m['source74_before_after_render'],m['original_run_files_sha256_before_after']
    if len(sources) != 74:raise ValueError('Original source map differs')
    for name,digest in sources.items():
        if sha(root/name) != digest:raise ValueError('Current frozen producer source differs')
    from PIL import Image, ImageSequence, GifImagePlugin, __version__ as pillow_version
    gif = render/'demo.gif';output.mkdir(parents=True)
    target = output/'preview.gif';durations=[];source_pixels=[];expected_pixels=[]
    with Image.open(gif) as original, target.open('xb') as writer:
        size = original.size;loop = original.info.get('loop')
        if type(loop) is not int or original.n_frames != 648:raise ValueError('Original GIF frame/loop metadata differs')
        preview_size = (640,round(size[1]*640/size[0]))
        for i,frame in enumerate(ImageSequence.Iterator(original)):
            duration = frame.info['duration']
            if type(duration) is not int or duration <= 0 or duration%10:raise ValueError('Invalid original GIF duration')
            rgb = frame.convert('RGB')
            source_pixels.append(hashlib.sha256(rgb.tobytes()).hexdigest())
            image = rgb.resize(preview_size,Image.Resampling.LANCZOS).quantize(
                colors=128,method=Image.Quantize.MEDIANCUT,dither=Image.Dither.NONE)
            if i == 0:
                header,_ = GifImagePlugin.getheader(image,info={'loop':loop,'optimize':False})
                for block in header:writer.write(block)
            expected_pixels.append(hashlib.sha256(image.convert('RGB').tobytes()).hexdigest())
            # Emit each FULL canvas explicitly. Pillow save_all can otherwise
            # merge newly identical downscaled frames and change frame counts.
            for block in GifImagePlugin.getdata(image,duration=duration,disposal=2,include_color_table=True):
                writer.write(block)
            durations.append(duration)
        writer.write(b';')
    if len(durations) != 648 or sum(durations) != 54000 or target.stat().st_size >= LIMIT:
        raise ValueError('Display preview must retain all648 original frames/durations and remain below45MB')
    actual_durations=[];actual_pixels=[]
    with Image.open(target) as preview:
        if preview.size != preview_size or preview.n_frames != 648 or preview.info.get('loop') != loop:
            raise ValueError('Encoded preview dimensions/frame count/loop differ')
        for frame in ImageSequence.Iterator(preview):
            actual_durations.append(frame.info['duration'])
            actual_pixels.append(hashlib.sha256(frame.convert('RGB').tobytes()).hexdigest())
    if actual_durations != durations or actual_pixels != expected_pixels:
        raise ValueError('Preview frame order/timing or decoded quantized raster differs')
    if inventory(render) != before:raise ValueError('Original rendered inputs changed')
    for name,digest in sources.items():
        if sha(root/name) != digest:raise ValueError('Frozen producer source changed')
    helper = Path(__file__).resolve();shutil.copyfile(helper,output/'preview_helper_source.py')
    manifest=dict(scope='Display derivative only of the CLOSED original GIF. No new native, media-render, force, capture or policy acceptance proof.',
        original_render_manifest_sha256=RENDER_MANIFEST_SHA,original_media_review_sha256=sha(rpath),
        original_media_reviewer_source_sha256=REVIEWER_SHA,original_render_files_sha256_before_after=before,
        source74_sha256_before_after=sources,original_run_files_sha256_pointer_only=runtime,
        original_gif_sha256=sha(gif),original_gif_bytes=gif.stat().st_size,original_size_px=list(size),
        preview_gif_sha256=sha(target),preview_bytes=target.stat().st_size,preview_size_px=list(preview_size),
        frames=648,frame_order='Every original decoded GIF frame, indices0through647, no merge/drop/reorder',
        original_frame_decoded_rgb_sha256=source_pixels,preview_frame_decoded_rgb_sha256=actual_pixels,
        original_and_preview_frame_durations_ms=durations,duration_ms=sum(durations),loop=loop,
        raster_transform=dict(resize='Pillow LANCZOS',width_px=640,height_rule='nearest integer preserving original aspect',
            quantization='MEDIANCUT128colors',dither='NONE',full_canvas_frames=True,disposal=2,pillow_version=pillow_version),
        state_or_timestamp_interpolation=False,model_or_engine_imported=False,physics_or_force_solve=False,
        original_input_bytes_unchanged=True,helper_sha256=sha(helper))
    (output/'preview_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),frames=648,duration_ms=54000,size_px=list(preview_size),
        preview_bytes=target.stat().st_size,preview_sha256=sha(target),manifest_sha256=sha(output/'preview_manifest.json'),
        display_only=True),indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,required=True)
    p.add_argument('render',type=Path);p.add_argument('output',type=Path);main(p.parse_args())
