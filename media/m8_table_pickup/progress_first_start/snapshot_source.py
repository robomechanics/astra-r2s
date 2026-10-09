"""Close-check and byte-copy a concurrently written phase-prefix archive."""
from pathlib import Path
import hashlib
import json
import os
import time
import zipfile
import numpy as np

directory = Path('outputs/m8_table_pickup/full_damped')
source = directory / 'insertion_trace_partial.npz'
target = directory / 'progress_first_start.npz'
if target.exists():
    raise FileExistsError(target)
for attempt in range(10):
    raw = source.read_bytes()
    scratch = target.with_suffix('.tmp')
    try:
        with scratch.open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        with zipfile.ZipFile(scratch) as zipped:
            if zipped.testzip() is not None:
                raise ValueError('Bad ZIP member')
        with np.load(scratch, allow_pickle=False) as saved:
            values = {name: saved[name].copy() for name in saved.files}
        rows = json.loads(str(values['info_json']))
        metadata = json.loads(str(values['metadata_json']))
        if len(rows) != len(values['time']):
            raise ValueError('State/sample alignment failed')
        if rows[-1]['phase'] != 'settle_regrip_search_2':
            raise ValueError(f"Unexpected prefix endpoint: {rows[-1]['phase']}")
        os.replace(scratch, target)
        manifest = {
            'source': str(source), 'immutable_snapshot': str(target),
            'snapshot_wall_time_UTC': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'byte_exact_copy': True, 'archive_closed_readable': True,
            'trajectory_sha256': hashlib.sha256(raw).hexdigest(),
            'bytes': len(raw), 'samples': len(rows),
            'first_physics_time_s': float(values['time'][0]),
            'last_physics_time_s': float(values['time'][-1]),
            'last_phase': rows[-1]['phase'],
            'phase_sequence': list(dict.fromkeys(row['phase'] for row in rows)),
            'model_fingerprint': metadata['model_fingerprint'],
            'model_xml_sha256': metadata['model_xml_sha256'],
            'scene_source_sha256': metadata['scene_source_sha256'],
            'controller_sha256': metadata['controller_sha256'],
            'status': 'incomplete_progress_prefix',
            'scope': 'Both genuine pickups and first cone-start recovery. This prefix ends after opening, reset and regrasp, before the second starting stroke. No finished rollout, formed-thread support or qualified-thread proof is claimed.',
            'metadata_partial_note': 'The original metadata partial field records the requested phase selection, not completion. This byte-exact prefix remains incomplete regardless of its original partial field.'
        }
        target.with_suffix('.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(json.dumps(manifest, indent=2))
        break
    except (zipfile.BadZipFile, EOFError, OSError) as error:
        scratch.unlink(missing_ok=True)
        if attempt == 9:
            raise
        time.sleep(.1)
