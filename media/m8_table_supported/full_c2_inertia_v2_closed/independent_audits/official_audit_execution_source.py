"""Serial read-only original-source auditors for the CLOSED fresh v2 attempt."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT / 'outputs/m8_table_supported/full_c2_inertia_v2'
OUT = ROOT / 'outputs/m8_table_supported/full_c2_inertia_v2_audits'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            h.update(data)
    return h.hexdigest()

def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

assert (RUN / 'run_publication_identity_after.json').is_file()
identity = json.loads((OUT / 'source_identity_before.json').read_text())
assert identity['closure_native_exit_code'] == 1
after_sha = sha(RUN / 'run_publication_identity_after.json')
assert after_sha == identity['after_identity_sha256']
trace_sha = sha(RUN / 'insertion_trace.npz')
source = ROOT / 'outputs/m8_table_supported/run_fresh_v2_official_audits.py'
source_sha = sha(source)
for script, name in (
    ('scripts/audit_m8_supported_trace.py', 'independent_supported_audit'),
    ('scripts/audit_m8_left_pad_force_history.py', 'independent_left_pad_force_history_audit'),
    ('scripts/audit_m8_free_joint_properties.py', 'independent_free_joint_properties'),
):
    report = OUT / (name + '.json')
    log = OUT / (name + '.log')
    record = OUT / (name + '_execution.json')
    assert not any(p.exists() for p in (report, log, record))
    expected = identity['frozen_audit_sources'][script]
    assert sha(ROOT / script) == expected
    cmd = ['scripts/run_m8.sh', script, str(RUN / 'insertion_trace.npz'), '--output', str(report)]
    started, before = utc(), time.perf_counter()
    with log.open('xb') as stream:
        child = subprocess.run(cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=False)
    finished, elapsed = utc(), time.perf_counter() - before
    assert sha(ROOT / script) == expected
    assert sha(RUN / 'run_publication_identity_after.json') == after_sha
    assert sha(RUN / 'insertion_trace.npz') == trace_sha
    assert sha(source) == source_sha
    result = dict(command=cmd, cwd=str(ROOT), started_utc=started, finished_utc=finished,
        wall_seconds=elapsed, exit_code=child.returncode, original_native_child_exit_code=1,
        auditor_source_sha256_before_after=expected, log_sha256=sha(log),
        report_sha256=sha(report) if report.is_file() else None,
        original_after_sha256=after_sha, original_trace_sha256=trace_sha,
        execution_wrapper_sha256=source_sha,
        scope='Original frozen auditor, read-only derivative; native failure retained. No trajectory reintegration or original-file rewrite.')
    with record.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(dict(auditor=script, exit_code=child.returncode, wall_seconds=elapsed,
        report_present=report.is_file())), flush=True)
