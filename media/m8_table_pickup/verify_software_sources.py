import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

paths = sorted(set(Path('tests').glob('test_*.py'))
    | set(Path('yam_twin').glob('m8*.py'))
    | set(Path('scripts').glob('*m8_insertion*.py'))
    | {Path('scripts/audit_m8_free_joint_properties.py'),
       Path('scripts/audit_m8_left_pad_force_history.py')})

def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}

before = hashes()
start = time.monotonic()
command = ['scripts/run_m8.sh', '-m', 'pytest', '-q']
result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT)
output = Path('outputs/m8_table_pickup')
log = output / 'pytest_all.log'
log.write_text(result.stdout)
count = re.search(r'(\d+) passed', result.stdout)
after = hashes()
report = dict(passed=result.returncode == 0 and before == after,
    tests_passed=int(count.group(1)) if count else None,
    wall_seconds=time.monotonic()-start, source_hashes=before,
    source_hashes_unchanged=before == after,
    command=command, output_log=str(log))
(output / 'pytest_all.json').write_text(json.dumps(report, indent=2)+'\n')
print(result.stdout)
print(json.dumps({k:v for k,v in report.items() if k != 'source_hashes'}, indent=2))
raise SystemExit(0 if report['passed'] else 1)
