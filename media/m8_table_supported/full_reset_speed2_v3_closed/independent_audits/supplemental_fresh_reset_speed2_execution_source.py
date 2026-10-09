"""One parent-authorized post-closure execution of the frozen a9c2 reader."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3'
OUT = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3_audits'
INPUTS = {
    ROOT/'outputs/m8_table_supported/audit_fresh_reset_speed2_v3.py':
        ('supplemental_fresh_reset_speed2_audit_source.py', 'a9c2b710f9f840677402dbc290e263bb29d17fb502408ce2072d4589a59acaea'),
    ROOT/'outputs/m8_table_supported/fresh_reset_speed2_v3_supplemental_workflow.md':
        ('supplemental_fresh_reset_speed2_workflow.md', 'b9c578f9bfa985fa598d90c19c3ec84281106c392e5a3cfd8ad8bf497d68b038'),
    ROOT/'outputs/m8_table_supported/full_c2_inertia_v2_audits/supplemental_open_reset_contact_audit.py':
        ('reused_b714_arithmetic_source.py', 'b714fed754d21b4a4684394802912b33b6f562b4e3936e90d70e3ad56fb23a0e'),
}


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def inventory(directory):
    return {str(p.relative_to(directory)):sha(p) for p in sorted(directory.rglob('*')) if p.is_file()}


def stamp():
    return datetime.now(timezone.utc).isoformat()


after = RUN/'run_publication_identity_after.json'
assert after.is_file() and sha(after) == 'f91f91beaaf7a8ee493fc124b068d469c72fee06be38f9920dce955e53aa2792'
serial = OUT/'serial_audit_result.json'
assert sha(serial) == '92c636a188d2f86a8c17c312f03e379d8c402ae2b8d19f686d8c2ba6293e8666'
result = json.loads(serial.read_text())
assert result['reader_execution_completed'] is True and result['identity_and_original_bytes_unchanged'] is True
original_before = inventory(RUN)
assert original_before == result['original_run_files_sha256_after']
for source, (name, expected) in INPUTS.items():
    assert sha(source) == expected and not (OUT/name).exists()
    shutil.copyfile(source, OUT/name)
    assert sha(OUT/name) == expected
record = OUT/'supplemental_fresh_reset_speed2_execution.json'
log = OUT/'supplemental_fresh_reset_speed2_stdout_stderr.log'
report = OUT/'supplemental_fresh_reset_speed2_audit.json'
assert not any(path.exists() for path in (record, log, report))
command = ['python', 'outputs/m8_table_supported/audit_fresh_reset_speed2_v3.py',
    'outputs/m8_table_supported/full_reset_speed2_v3',
    'outputs/m8_table_supported/full_reset_speed2_v3_audits',
    '--arithmetic-source', 'outputs/m8_table_supported/full_c2_inertia_v2_audits/supplemental_open_reset_contact_audit.py',
    '--output', 'outputs/m8_table_supported/full_reset_speed2_v3_audits/supplemental_fresh_reset_speed2_audit.json']
env = os.environ.copy()
env['PYTHONDONTWRITEBYTECODE'] = '1'
started, before = stamp(), time.perf_counter()
with log.open('xb') as stream:
    child = subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, check=False)
finished, wall_seconds = stamp(), time.perf_counter()-before
original_after = inventory(RUN)
source_unchanged = all(sha(source) == expected and sha(OUT/name) == expected
    for source, (name, expected) in INPUTS.items())
report_sha = sha(report) if report.is_file() else None
execution = {
    'kind':'Actual one-shot post-closure supplemental reader execution',
    'command':command, 'environment_override':{'PYTHONDONTWRITEBYTECODE':'1'}, 'cwd':str(ROOT),
    'started_utc':started, 'finished_utc':finished, 'wall_seconds':wall_seconds,
    'reader_exit_code':child.returncode, 'original_native_child_exit_code':0,
    'reader_source_sha256':INPUTS[ROOT/'outputs/m8_table_supported/audit_fresh_reset_speed2_v3.py'][1],
    'workflow_sha256':INPUTS[ROOT/'outputs/m8_table_supported/fresh_reset_speed2_v3_supplemental_workflow.md'][1],
    'reused_arithmetic_source_sha256':INPUTS[ROOT/'outputs/m8_table_supported/full_c2_inertia_v2_audits/supplemental_open_reset_contact_audit.py'][1],
    'execution_source_sha256':sha(__file__), 'stdout_stderr_sha256':sha(log), 'report_sha256':report_sha,
    'original_after_sha256':sha(after), 'official_serial_result_sha256':sha(serial),
    'original_run_files_sha256_before':original_before, 'original_run_files_sha256_after':original_after,
    'original_bytes_unchanged':original_before == original_after,
    'reader_workflow_and_reused_source_unchanged':source_unchanged,
    'scope':'One execution of the exact frozen source after root-confirmed closure and three intact serial official audits. CLI exit is execution only; original native acceptance and official physics results remain separate. No source/runtime changes, tests, native imports, extra auditors or integrations.'}
with record.open('x') as stream:
    json.dump(execution, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps({'reader_exit_code':child.returncode, 'original_bytes_unchanged':original_before == original_after,
    'source_unchanged':source_unchanged, 'report_sha256':report_sha, 'wall_seconds':wall_seconds}), flush=True)
assert child.returncode == 0 and report_sha and original_before == original_after and source_unchanged
