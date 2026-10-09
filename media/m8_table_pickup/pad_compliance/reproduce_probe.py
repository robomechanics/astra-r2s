"""Reproduce the archived diagnostic with checked helpers and published input.

Only root, input and output paths are changed in the preserved original probe.
The original numerical loop and pair-parameter overrides remain byte-for-byte.
"""
from pathlib import Path
import hashlib
import json
import sys

package = Path(__file__).resolve().parent
root = next(p for p in package.parents if (p / 'thread_lab/runtime.py').is_file())
sys.path.insert(0, str(root))
manifest = json.loads((package / 'manifest.json').read_text())
source_path = package / 'probe_source.py'
source = source_path.read_text()
if hashlib.sha256(source_path.read_bytes()).hexdigest() != manifest['original_probe_source_sha256']:
    raise ValueError('Original probe source changed')
for relative, expected in manifest['helper_source_sha256'].items():
    if hashlib.sha256((root / relative).read_bytes()).hexdigest() != expected:
        raise ValueError(f'Helper differs from frozen reproduction snapshot: {relative}')
trace = root / manifest['published_baseline_trajectory']
if hashlib.sha256(trace.read_bytes()).hexdigest() != manifest['baseline_trajectory_sha256']:
    raise ValueError('Published baseline trace changed')
from thread_lab.runtime import require_micron_engine
runtime = require_micron_engine()
expected = manifest['native_runtime_verified_at_packaging']
if ([x['sha256'] for x in runtime['libraries']] != [x['sha256'] for x in expected['libraries']]
        or runtime['thread_plugin_source_sha256'] != expected['thread_plugin_source_sha256']):
    raise ValueError('Native core/plugin differs from the measured diagnostic runtime')
substitutions = {
    "ROOT=Path('/workspace/astra-r2s')": f'ROOT=Path({str(root)!r})',
    "p=ROOT/'outputs/m8_insertion/table_left_smoke_v2/insertion_trace.npz'":
        f"p=ROOT/{manifest['published_baseline_trajectory']!r}",
    "out=ROOT/'outputs/m8_insertion/pad_compliance_review'":
        "out=ROOT/'outputs/m8_table_pickup/pad_compliance_reproduction'",
}
for old, new in substitutions.items():
    if source.count(old) != 1:
        raise ValueError(f'Cannot apply the documented path-only substitution: {old}')
    source = source.replace(old, new)
exec(compile(source, str(source_path), 'exec'), {'__name__': '__main__', '__file__': str(source_path)})
