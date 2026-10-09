"""Record one authorized read-only media stage command and exact stdout/stderr.

Does not integrate physics, import the media stage, modify native files, or infer
success. This wrapper preserves failures and leaves stage atomic-copy checks to
its source-bound snapshot/renderer/publication helper.
"""
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for raw in iter(lambda:stream.read(1024*1024),b''):h.update(raw)
    return h.hexdigest()


def now():return datetime.now(timezone.utc).isoformat()


def main(args):
    root=args.repository_root.resolve();target=args.record_dir.resolve()
    command=args.command[1:] if args.command and args.command[0]=='--' else args.command
    assert command and not target.exists()
    target.mkdir(parents=True)
    source=Path(__file__).resolve();(target/'stage_recorder_source.py').write_bytes(source.read_bytes())
    record=dict(schema='reset-speed2-v3-media-stage-execution-v1',stage=args.stage,
        command=command,cwd=str(root),started_utc=now(),recorder_source_sha256=sha(source),
        scope='Recorded execution of additive read-only snapshot/geometry/media/publication stage. Not a native rollout or native qualification.')
    (target/'execution_started.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    with (target/'stdout.log').open('wb') as stdout,(target/'stderr.log').open('wb') as stderr:
        result=subprocess.run(command,cwd=root,stdout=stdout,stderr=stderr,check=False)
    record.update(finished_utc=now(),exit_code=result.returncode,stage_command_succeeded=result.returncode==0,
        stdout_sha256=sha(target/'stdout.log'),stderr_sha256=sha(target/'stderr.log'))
    (target/'execution_finished.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    print(json.dumps(record,indent=2))
    raise SystemExit(result.returncode)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repository-root',type=Path,default=Path('/workspace/astra-r2s'))
    p.add_argument('--record-dir',type=Path,required=True);p.add_argument('--stage',required=True)
    p.add_argument('command',nargs=argparse.REMAINDER);main(p.parse_args())
