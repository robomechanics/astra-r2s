"""Versioned software proof for a later supported-controller producer."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
destination = Path(args.output)
destination.mkdir(parents=True, exist_ok=False)
paths = sorted(set(Path('tests').glob('test_*.py'))
    | set(Path('yam_twin').glob('m8*.py'))
    | {Path('yam_twin/kinematics.py')}
    | set(Path('thread_lab').glob('*.py'))
    | {Path('thread_lab/plugins/m8_sdf.cc')}
    | set(Path('scripts').glob('*m8*.py')))

def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}

runtime_command = ['scripts/run_m8.sh', '-c',
    'import json;from thread_lab.runtime import require_micron_engine;print(json.dumps(require_micron_engine()))']
runtime = json.loads(subprocess.check_output(runtime_command,text=True))
runtime_before = {library['path']: hashlib.sha256(Path(library['path']).read_bytes()).hexdigest()
                  for library in runtime['libraries']}
if any(library['sha256'] != runtime_before[library['path']] for library in runtime['libraries']):
    raise ValueError('Mapped native runtime differs from its disk bytes')
before = hashes()
started = time.monotonic()
command = ['scripts/run_m8.sh', '-m', 'pytest', '-q']
result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT)
log = destination / 'pytest_all.log'
log.write_text(result.stdout)
count = re.search(r'(\d+) passed', result.stdout)
after = hashes()
runtime_after = {name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in runtime_before}
report = dict(passed=result.returncode == 0 and before == after and runtime_before == runtime_after,
    tests_passed=int(count.group(1)) if count else None,
    wall_seconds=time.monotonic()-started, source_hashes=before,
    source_hashes_unchanged=before == after, command=command,
    wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    runtime=runtime, runtime_file_sha256_before=runtime_before,
    runtime_file_sha256_after=runtime_after, runtime_files_unchanged=runtime_before == runtime_after,
    scope='Software tests for this exact supported-controller source; independent from historical producer proofs and native physics qualification.')
(destination / 'software_proof.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
(destination / 'verification_source.py').write_bytes(Path(__file__).read_bytes())
print(result.stdout)
print(json.dumps({k:v for k,v in report.items() if k != 'source_hashes'}, indent=2))
raise SystemExit(0 if report['passed'] else 1)
