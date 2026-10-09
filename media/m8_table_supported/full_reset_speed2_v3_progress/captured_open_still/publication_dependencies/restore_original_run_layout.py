"""Restore verified package artifacts to exact original native relative names.

Standard library only. Every stored checksum/chunk/whole artifact is verified
before any original-layout output is written. Differing existing files are
never overwritten. This helper does not import or invoke a simulator.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import types


def identity(path):
    value = hashlib.sha256()
    size = 0
    with Path(path).open('rb') as stream:
        for raw in iter(lambda:stream.read(1024*1024), b''):
            size += len(raw)
            value.update(raw)
    return {'bytes':size,'sha256':value.hexdigest()}


def relative(value):
    path = Path(value)
    if path.is_absolute() or not path.parts or '..' in path.parts:
        raise ValueError('Unsafe original relative path: '+str(value))
    return path


def restore_original(manifest_path, output=None, *, verify_only=False):
    if output is None and not verify_only:
        raise ValueError('An output directory is required for original-layout restoration')
    manifest_path = Path(manifest_path).resolve()
    package = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    checksums = {}
    for line in (package/'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ',1)
        path = (package/relative(name)).resolve()
        if not path.is_relative_to(package) or identity(path)['sha256'] != digest:
            raise ValueError('Stored package file failed SHA verification: '+name)
        checksums[name] = digest
    helper = package/'reassemble_archives.py'
    if 'package_manifest.json' not in checksums or 'reassemble_archives.py' not in checksums:
        raise ValueError('Package metadata/reassembler checksum binding is absent')
    reassembler = types.ModuleType('verified_native_reassembler')
    reassembler.__file__ = str(helper)
    exec(compile(helper.read_bytes(),str(helper),'exec'),reassembler.__dict__)
    if 'original_run_file_to_published_artifact_names' in manifest:
        mapping = manifest['original_run_file_to_published_artifact_names']
        originals = manifest['original_run_file_identities']
    elif manifest.get('status') == 'closed_raw_evidence_only':
        originals = manifest['original_file_identities']
        mapping = {name:[name] for name in originals}
    else:
        raise ValueError('Package does not declare a verifiable original-run layout')
    if not mapping or set(mapping) != set(originals):
        raise ValueError('Original filename/identity coverage differs')
    destination = None if verify_only else Path(output).resolve()
    if destination is not None and destination.is_relative_to(package):
        raise ValueError('Restore output must be outside the immutable package')
    if destination is not None and destination.exists() and not destination.is_dir():
        raise ValueError('Restore output is not a directory')
    with tempfile.TemporaryDirectory(prefix='m8_original_layout_verified_') as scratch:
        scratch = Path(scratch)
        reassembler.restore(manifest_path,scratch)
        verified = {}
        for original, aliases in mapping.items():
            relative(original)
            if not isinstance(aliases,list) or not aliases:
                raise ValueError('Original artifact has no verified alias: '+original)
            expected = originals[original]
            for alias in aliases:
                relative(alias)
                if alias not in manifest['artifacts'] or identity(scratch/alias) != expected:
                    raise ValueError('Published alias differs from original identity: '+original)
            verified[original] = scratch/aliases[0]
            if destination is not None:
                target = destination/relative(original)
                if not target.resolve().is_relative_to(destination):
                    raise ValueError('Original restore path escapes output: '+original)
                if target.exists() and identity(target) != expected:
                    raise FileExistsError('Existing original-layout destination differs: '+str(target))
        installed = 0
        if destination is not None:
            for original, source in verified.items():
                target = destination/relative(original)
                expected = originals[original]
                if target.exists():
                    if identity(target) != expected:
                        raise FileExistsError(target)
                    continue
                target.parent.mkdir(parents=True,exist_ok=True)
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(dir=target.parent,prefix=target.name+'.',
                            suffix='.verified',delete=False) as writer:
                        temporary = Path(writer.name)
                        with source.open('rb') as reader:
                            for raw in iter(lambda:reader.read(1024*1024),b''):
                                writer.write(raw)
                    if identity(temporary) != expected:
                        raise ValueError('Verified original bytes changed during copy: '+original)
                    # An atomic hard link fails if a concurrent destination
                    # appeared; it can never replace differing existing bytes.
                    os.link(temporary,target)
                    installed += 1
                finally:
                    if temporary is not None:
                        temporary.unlink(missing_ok=True)
        return {'verified_original_files':len(verified),'newly_installed_files':installed,
            'original_relative_names_restored':True,'lossless':True,
            'verify_only':verify_only,'output':None if destination is None else str(destination),
            'scope':'Exact original closed-run layout/bytes only; no simulator, integration, rewritten report/ledger or physics-success inference.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,
        default=Path(__file__).resolve().parent/'package_manifest.json')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--verify-only',action='store_true')
    args = parser.parse_args()
    if args.output is None and not args.verify_only:
        parser.error('--output is required unless --verify-only')
    print(json.dumps(restore_original(args.manifest,args.output,verify_only=args.verify_only),indent=2))


if __name__ == '__main__':
    main()
