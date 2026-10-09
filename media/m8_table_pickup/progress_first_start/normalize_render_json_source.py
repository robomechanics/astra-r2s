"""Preserve a Python-JSON manifest and fix only nonfinite container metadata."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import sys

directory = Path(sys.argv[1] if len(sys.argv) > 1 else
    'media/m8_table_pickup/progress_first_start')
manifest_path = directory/'render_manifest.json'
original_path = directory/'render_manifest_original_python_json.txt'
checksums_path = directory/'checksums.json'
checksums = json.loads(checksums_path.read_text())
for name, identity in checksums.items():
    path = directory/name
    assert path.stat().st_size == identity['bytes'], name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256'], name
original = original_path.read_bytes() if original_path.exists() else manifest_path.read_bytes()
if not original_path.exists():
    with original_path.open('xb') as stream:
        stream.write(original)
value = json.loads(original)
changes = []

def finite_container_metadata(item, path):
    if isinstance(item, dict):
        return {key: finite_container_metadata(child, path+'/'+key)
                for key, child in item.items()}
    if isinstance(item, list):
        return [finite_container_metadata(child, path+'/'+str(index))
                for index, child in enumerate(item)]
    if isinstance(item, float) and not math.isfinite(item):
        changes.append({'path': path, 'original_python_json_value': str(item),
                        'strict_json_value': None})
        return None
    return item

value['video']['metadata'] = finite_container_metadata(value['video']['metadata'],
    '/video/metadata')
assert changes == [{'path': '/video/metadata/nframes',
                    'original_python_json_value': 'inf', 'strict_json_value': None}]
value['container_metadata_normalization'] = {
    'note': 'The previously published render_manifest.json contained the nonstandard Python-JSON value Infinity in video.metadata.nframes. Only that diagnostic container-metadata field is normalized to null. The separately decoded frame count remains 142. No video, trace, archived source, physical metadata or physical result changed.',
    'original_file': original_path.name,
    'original_sha256': hashlib.sha256(original).hexdigest(),
    'normalized_fields': changes,
    'serialization': 'Strict JSON with allow_nan=false',
    'helper_source': 'normalize_render_json_source.py',
}
manifest_path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
helper_path = directory/'normalize_render_json_source.py'
if Path(__file__).resolve() != helper_path.resolve():
    shutil.copyfile(__file__, helper_path)

def reject_constant(value):
    raise ValueError('Nonfinite nonstandard JSON constant: '+value)

strict_json_files = []
for path in sorted(directory.rglob('*.json')):
    json.loads(path.read_text(), parse_constant=reject_constant)
    strict_json_files.append(str(path.relative_to(directory)))
new_checksums = {str(path.relative_to(directory)): {'bytes': path.stat().st_size,
    'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    for path in sorted(directory.rglob('*')) if path.is_file() and path.name != 'checksums.json'}
checksums_path.write_text(json.dumps(new_checksums, indent=2, allow_nan=False)+'\n')
json.loads(checksums_path.read_text(), parse_constant=reject_constant)
for name, identity in new_checksums.items():
    path = directory/name
    assert path.stat().st_size == identity['bytes'], name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256'], name
for name, identity in checksums.items():
    if name not in ('render_manifest.json', 'normalize_render_json_source.py'):
        assert new_checksums[name] == identity, name
print(json.dumps({'original_sha256': hashlib.sha256(original).hexdigest(),
    'strict_manifest_sha256': new_checksums['render_manifest.json']['sha256'],
    'strict_json_files_verified': strict_json_files,
    'checksummed_files_verified': len(new_checksums),
    'only_normalized_field': changes[0]['path'],
    'all_existing_physical_and_archived_source_bytes_unchanged': True}, indent=2, allow_nan=False))
