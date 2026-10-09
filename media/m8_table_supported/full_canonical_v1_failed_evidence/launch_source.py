"""Bind one fresh, unspliced native run to a completed software proof.

Output-only orchestration: no mutation of application, contacts or object state.
The native child's original exit code is retained, including failed trials.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--proof', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--producer', required=True)
args = parser.parse_args()
proof = args.proof.resolve()
output = args.output.resolve()
report = json.loads((proof / 'software_proof.json').read_text())
if not report['passed'] or not report['source_hashes_unchanged'] or not report['tests_passed']:
    raise ValueError('A completed, source-stable software proof is required')
expected = report['source_hashes']
before = {name: digest(ROOT / name) for name in expected}
if before != expected or git('rev-parse', 'HEAD') != args.producer:
    raise ValueError('Producer HEAD/source files differ from the declared tested producer')
output.mkdir(parents=True, exist_ok=False)
shutil.copy2(proof / 'software_proof.json', output / 'software_tests.json')
shutil.copy2(proof / 'pytest_all.log', output / 'software_tests.log')
shutil.copy2(proof / 'verification_source.py', output / 'verify_sources.py')
shutil.copy2(Path(__file__), output / 'launch_source.py')
manifest = {name: {'bytes': (output / name).stat().st_size,
                  'sha256': digest(output / name)}
            for name in ('software_tests.json', 'software_tests.log', 'verify_sources.py', 'launch_source.py')}
save(output / 'manifest.json', {'scope': report['scope'], 'files': manifest})
audit_names = ('scripts/audit_m8_supported_trace.py', 'scripts/audit_m8_insertion_trace.py',
               'scripts/audit_m8_free_joint_properties.py', 'scripts/audit_m8_left_pad_force_history.py',
               'thread_lab/runtime.py')
audit_hashes = {}
for name in audit_names:
    target = output / 'frozen_audit_sources' / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / name, target)
    audit_hashes[name] = digest(target)
command = ['scripts/run_m8.sh', '-m', 'yam_twin.m8_supported_demo',
           '--output', str(output), '--dt', '.00005', '--starting-angular-speed', '1',
           '--angular-speed', '2', '--maximum-entry-dwell', '10',
           '--maximum-starting-strokes', '5', '--qualifying-strokes', '2',
           '--axial-damping', '200']
identity = {'producer_commit': args.producer,
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'command': command, 'source_hashes_before': before,
            'frozen_audit_sources': audit_hashes,
            'scope': 'One fresh native table-supported attempt from initial independent table/rest spawns. No cold checkpoint or stitched states. Software proof is distinct from trajectory qualification.'}
save(output / 'run_publication_identity.json', identity)
start = time.monotonic()
with (output / 'native_stdout_stderr.log').open('x') as log:
    result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
after = {name: digest(ROOT / name) for name in expected}
identity.update({'finished_utc': datetime.now(timezone.utc).isoformat(),
                 'wall_seconds': time.monotonic() - start,
                 'native_exit_code': result.returncode,
                 'source_hashes_after': after,
                 'source_hashes_unchanged': before == after,
                 'repository_head_after': git('rev-parse', 'HEAD'),
                 'repository_head_scope': 'Media/documentation-only commits may advance HEAD; exact tested source bytes remain the binding.'})
save(output / 'run_publication_identity_after.json', identity)
print(json.dumps({key: identity[key] for key in ('producer_commit', 'native_exit_code',
                                                'wall_seconds', 'source_hashes_unchanged')}, indent=2))
raise SystemExit(result.returncode if before == after else 2)
