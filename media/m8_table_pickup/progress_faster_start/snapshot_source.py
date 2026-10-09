from pathlib import Path
import hashlib, io, json, os, time, zipfile
import numpy as np

directory = Path('outputs/m8_table_pickup/full_faster_entry')
source = directory/'insertion_trace_partial.npz'
target = directory/'progress_first_faster_start.npz'
if target.exists():
    raise FileExistsError(target)
for attempt in range(50):
    raw = source.read_bytes()
    if raw != source.read_bytes():
        time.sleep(.1)
        continue
    try:
        with zipfile.ZipFile(io.BytesIO(raw)) as zipped:
            assert zipped.testzip() is None
        with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
            arrays = {name:saved[name].copy() for name in saved.files}
        rows = json.loads(str(arrays['info_json']))
        assert len(rows) == len(arrays['time'])
        assert not any(x['phase']=='start_thread_2' for x in rows)
        assert all(x['thread_engaged'] is False for x in rows)
        break
    except (zipfile.BadZipFile,EOFError,OSError):
        if attempt == 49:
            raise
        time.sleep(.1)
else:
    raise ValueError('No stable closed phase-prefix archive')
scratch=target.with_suffix('.tmp')
with scratch.open('xb') as stream:
    stream.write(raw)
    stream.flush()
    os.fsync(stream.fileno())
os.replace(scratch,target)
report = {'source':str(source),'snapshot':str(target),
    'snapshot_sha256':hashlib.sha256(raw).hexdigest(), 'snapshot_bytes':len(raw),
    'two_consecutive_reads_identical':True,'archive_crc_readability_verified':True,
    'byte_exact_copy':True,
    'snapshot_wall_time_UTC':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime(target.stat().st_mtime)),
    'samples':len(rows),'last_time_s':float(arrays['time'][-1]),
    'last_phase':rows[-1]['phase'],'second_start_included':False,
    'capture_included':False}
target.with_suffix('.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,indent=2,allow_nan=False))
