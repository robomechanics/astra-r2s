"""Bind one fresh, unspliced reset-speed2 native run to its completed software proof.

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
parser.add_argument('--repository-root',type=Path,default=ROOT)
parser.add_argument('--prepare-only',action='store_true')
args = parser.parse_args()
ROOT = args.repository_root.resolve()
proof = args.proof.resolve()
output = args.output.resolve()
report = json.loads((proof / 'software_proof.json').read_text())
if not report['passed'] or not report['source_hashes_unchanged'] or not report['tests_passed']:
    raise ValueError('A completed, source-stable software proof is required')
expected = report['source_hashes']
before = {name: digest(ROOT / name) for name in expected}
if before != expected or git('rev-parse', 'HEAD') != args.producer:
    raise ValueError('Producer HEAD/source files differ from the declared tested producer')
if not report['runtime_files_unchanged']:
    raise ValueError('Software proof lacks stable native runtime binding')
runtime_before = {name:digest(Path(name)) for name in report['runtime_file_sha256_before']}
if runtime_before != report['runtime_file_sha256_before'] or runtime_before != report['runtime_file_sha256_after']:
    raise ValueError('Native runtime differs from completed software proof')
if output.exists():
    raise FileExistsError('Never overwrite a native trajectory')
if args.prepare_only:
    print(json.dumps({'prepared':True,'native_integration_steps':0,'producer_commit':args.producer,
        'source_files':len(before),'runtime_files':runtime_before,'command_scope':'Unspliced fresh independent table/rest spawns with finite free-wrist reset-speed2 only'},indent=2))
    raise SystemExit(0)
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
           '--angular-speed', '2', '--reset-speed', '2', '--maximum-entry-dwell', '10',
           '--maximum-starting-strokes', '5', '--qualifying-strokes', '2',
           '--axial-damping', '200']
identity = {'producer_commit': args.producer,
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'command': command, 'source_hashes_before': before,
            'runtime':report['runtime'],'runtime_file_sha256_before':runtime_before,
            'frozen_audit_sources': audit_hashes,
            'scope': 'One fresh native table-supported attempt from initial independent table/rest spawns, with the same frozen free-hand reset path at peak2rad/s through unchanged finite robot controls. No cold checkpoint or stitched states. Software proof is distinct from trajectory qualification.'}
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
                 'runtime_file_sha256_after':{name:digest(Path(name)) for name in runtime_before},
                 'repository_head_after': git('rev-parse', 'HEAD'),
                 'repository_head_scope': 'Media/documentation-only commits may advance HEAD; exact tested source bytes remain the binding.'})
identity['runtime_files_unchanged'] = identity['runtime_file_sha256_after']==runtime_before
identity['native_stdout_stderr_sha256'] = digest(output/'native_stdout_stderr.log')
save(output / 'run_publication_identity_after.json', identity)
print(json.dumps({key: identity[key] for key in ('producer_commit', 'native_exit_code',
                                                'wall_seconds', 'source_hashes_unchanged')}, indent=2))
raise SystemExit(result.returncode if before == after and identity['runtime_files_unchanged'] else 2)
