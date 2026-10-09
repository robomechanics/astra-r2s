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

before = hashes()
started = time.monotonic()
command = ['scripts/run_m8.sh', '-m', 'pytest', '-q']
result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT)
log = destination / 'pytest_all.log'
log.write_text(result.stdout)
count = re.search(r'(\d+) passed', result.stdout)
after = hashes()
report = dict(passed=result.returncode == 0 and before == after,
    tests_passed=int(count.group(1)) if count else None,
    wall_seconds=time.monotonic()-started, source_hashes=before,
    source_hashes_unchanged=before == after, command=command,
    wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    scope='Software tests for this exact supported-controller source; independent from historical producer proofs and native physics qualification.')
(destination / 'software_proof.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
(destination / 'verification_source.py').write_bytes(Path(__file__).read_bytes())
print(result.stdout)
print(json.dumps({k:v for k,v in report.items() if k != 'source_hashes'}, indent=2))
raise SystemExit(0 if report['passed'] else 1)
