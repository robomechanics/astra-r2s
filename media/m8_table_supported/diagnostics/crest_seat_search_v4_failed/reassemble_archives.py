"""Verify and losslessly reassemble exact native supported-demo artifacts.

Standard library only. No MuJoCo, NumPy, integration, or quantization is used.
Every stored chunk and every reconstructed whole file is verified by SHA256.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import tempfile


def relative_path(value):
    path = Path(value)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError(f"Unsafe relative artifact path: {value}")
    return path


def digest(path):
    h = hashlib.sha256()
    size = 0
    with path.open("rb") as source:
        for raw in iter(lambda: source.read(1024 * 1024), b""):
            h.update(raw)
            size += len(raw)
    return size, h.hexdigest()


def restore(manifest_path, output=None, *, verify_only=False):
    manifest_path = Path(manifest_path).resolve()
    root = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    verified = 0
    for name, artifact in manifest["artifacts"].items():
        destination_name = relative_path(name)
        chunks = artifact["chunks"] if artifact["storage"] == "lossless_binary_chunks" else [{
            "path": artifact["path"], "offset": 0,
            "bytes": artifact["bytes"], "sha256": artifact["sha256"]}]
        target = None if verify_only else Path(output) / destination_name
        if target is not None:
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                if digest(target) != (artifact["bytes"], artifact["sha256"]):
                    raise FileExistsError(f"Existing destination differs: {target}")
                # Still verify all source chunks; an existing good destination
                # does not hide a corrupted downloaded archive.
        temporary_path = None
        writer = None
        if target is not None and not target.exists():
            writer = tempfile.NamedTemporaryFile(mode="wb", dir=target.parent,
                prefix=target.name + ".", suffix=".reassembling", delete=False)
            temporary_path = Path(writer.name)
        whole = hashlib.sha256()
        offset = 0
        try:
            for chunk in chunks:
                source_path = root / relative_path(chunk["path"])
                if chunk["offset"] != offset:
                    raise ValueError(f"Unexpected chunk order/offset for {name}")
                chunk_digest = hashlib.sha256()
                count = 0
                with source_path.open("rb") as source:
                    for raw in iter(lambda: source.read(1024 * 1024), b""):
                        chunk_digest.update(raw)
                        whole.update(raw)
                        count += len(raw)
                        if writer is not None:
                            writer.write(raw)
                if count != chunk["bytes"] or chunk_digest.hexdigest() != chunk["sha256"]:
                    raise ValueError(f"Chunk size/SHA256 mismatch: {source_path}")
                offset += count
            if offset != artifact["bytes"] or whole.hexdigest() != artifact["sha256"]:
                raise ValueError(f"Whole-file size/SHA256 mismatch: {name}")
            if writer is not None:
                writer.flush()
                writer.close()
                writer = None
                # The final destination is installed only after verification.
                if target.exists():
                    raise FileExistsError(target)
                temporary_path.rename(target)
                temporary_path = None
            verified += 1
        finally:
            if writer is not None:
                writer.close()
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
    return {"verified_artifacts": verified, "lossless": True,
            "verify_only": verify_only, "output": None if verify_only else str(output)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
        default=Path(__file__).resolve().parent / "package_manifest.json")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if not args.verify_only and args.output is None:
        parser.error("--output is required unless --verify-only is selected")
    print(json.dumps(restore(args.manifest, args.output, verify_only=args.verify_only), indent=2))


if __name__ == "__main__":
    main()
