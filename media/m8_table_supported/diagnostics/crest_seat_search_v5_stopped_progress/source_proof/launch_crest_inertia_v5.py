"""One separately authorized cold native V5 experiment; no source mutation."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess
import time


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    folder=Path(__file__).resolve().parent
    root=folder.parents[2]
    manifest_path=folder/'crest_seat_search_v5_frozen_inputs.json'
    manifest=json.loads(manifest_path.read_text())
    output=root/manifest['output']
    if output.exists():
        raise FileExistsError('Never overwrite a native trial')
    parent=root/'outputs/m8_table_supported/full_canonical_v1'
    source_before={name:sha(root/name) for name in manifest['producer_source_65']}
    frozen_before={name:sha(folder/name) for name in manifest['source_files']}
    parents_before={name:sha(parent/name) for name in manifest['parent_input_sha256']}
    runtime_before={name:sha(name) for name in manifest['runtime_file_sha256']}
    if (source_before!=manifest['producer_source_65'] or frozen_before!=manifest['source_files']
            or parents_before!=manifest['parent_input_sha256'] or runtime_before!=manifest['runtime_file_sha256']):
        raise RuntimeError('Frozen source/input/native-runtime binding changed before launch')
    identity={'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'producer_source_65_before':source_before,'source_files_before':frozen_before,
        'parent_input_sha256_before':parents_before,'runtime_file_sha256_before':runtime_before,
        'runtime':manifest['runtime'],'command':manifest['command'],
        'execution_repository_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        'frozen_inputs_manifest_sha256':sha(manifest_path),
        'scope':'Separate cold approximate robot-arm hybrid inertia diagnostic. Original65/402 proof unchanged, no trajectory splice or physical success implied.'}
    before_path=folder/'crest_seat_search_v5_execution_before.json'
    if before_path.exists():
        raise FileExistsError('A prior V5 launch identity already exists')
    before_path.write_text(json.dumps(identity,indent=2)+'\n')
    start=time.perf_counter()
    log=folder/'crest_seat_search_v5.log'
    with log.open('x') as stream:
        result=subprocess.run(manifest['command'],cwd=root,stdout=stream,stderr=subprocess.STDOUT)
    after={'closed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'native_exit_code':result.returncode,'wall_seconds':time.perf_counter()-start,
        'execution_before_sha256':sha(before_path),'native_stdout_stderr_sha256':sha(log),
        'source_65_after':{name:sha(root/name) for name in source_before},
        'source_files_after':{name:sha(folder/name) for name in frozen_before},
        'parent_input_sha256_after':{name:sha(parent/name) for name in parents_before},
        'runtime_file_sha256_after':{name:sha(name) for name in runtime_before},
        'frozen_inputs_manifest_sha256_after':sha(manifest_path)}
    after['all_bindings_unchanged']=(after['source_65_after']==source_before
        and after['source_files_after']==frozen_before and after['parent_input_sha256_after']==parents_before
        and after['runtime_file_sha256_after']==runtime_before
        and after['frozen_inputs_manifest_sha256_after']==identity['frozen_inputs_manifest_sha256'])
    report=output/'insertion_validation.json'
    if report.exists():
        closed=json.loads(report.read_text())
        after.update({'native_final_time_s':closed.get('aborted',{}).get('time') if closed.get('aborted') else None,
            'diagnostic_completed':closed.get('diagnostic_completed'),
            'original_validation_sha256':sha(report),'aborted':closed.get('aborted')})
    (folder/'crest_seat_search_v5_execution_after.json').write_text(json.dumps(after,indent=2)+'\n')
    print(json.dumps(after,indent=2))
    raise SystemExit(result.returncode if after['all_bindings_unchanged'] else 2)


if __name__=='__main__':
    main()
