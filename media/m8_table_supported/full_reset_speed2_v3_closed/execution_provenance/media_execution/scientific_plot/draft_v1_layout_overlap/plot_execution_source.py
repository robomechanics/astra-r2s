"""Actual bounded standalone plot execution; never a physics audit."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path('/workspace/astra-r2s')
OUT = Path(__file__).resolve().parent
SOURCE = OUT/'plot_qualified_half_turns.py'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stamp():
    return datetime.now(timezone.utc).isoformat()


log = OUT/'plot_stdout_stderr.log'
record = OUT/'plot_execution.json'
assert not log.exists() and not record.exists()
source_sha = sha(SOURCE)
outer_sha = sha(Path(__file__))
command = ['python', str(SOURCE.relative_to(ROOT))]
environment = os.environ.copy()
environment['PYTHONDONTWRITEBYTECODE'] = '1'
environment['MPLCONFIGDIR'] = '/tmp/astra-m8-qualified-plot-cache'
started, before = stamp(), time.perf_counter()
with log.open('xb') as stream:
    child = subprocess.run(command, cwd=ROOT, env=environment,
        stdout=stream, stderr=subprocess.STDOUT, check=False)
finished, elapsed = stamp(), time.perf_counter()-before
unchanged = sha(SOURCE) == source_sha and sha(Path(__file__)) == outer_sha
result = {'kind':'Actual standalone qualified-half-turn plot execution',
    'command':command, 'cwd':str(ROOT),
    'environment_overrides':{'PYTHONDONTWRITEBYTECODE':'1', 'MPLCONFIGDIR':environment['MPLCONFIGDIR']},
    'started_utc':started, 'finished_utc':finished, 'wall_seconds':elapsed, 'exit_code':child.returncode,
    'plot_source_sha256_before_after':source_sha, 'execution_source_sha256':outer_sha,
    'source_unchanged':unchanged, 'stdout_stderr_sha256':sha(log),
    'plot_manifest_sha256':sha(OUT/'plot_manifest.json') if (OUT/'plot_manifest.json').is_file() else None,
    'figure_sha256':sha(OUT/'qualified_half_turns.png') if (OUT/'qualified_half_turns.png').is_file() else None,
    'scope':'Standalone scientific visualization of original closed raw geometry, outside immutable native/audit trees. No new audit, fit, test, model, dynamics integration or force reconstruction. Original native and official acceptance remain separate.'}
with record.open('x') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps(result))
assert child.returncode == 0 and unchanged and result['figure_sha256'] and result['plot_manifest_sha256']
