"""Standard-library regression checks for exact raw native-evidence publication.

These tests use temporary, deliberately opaque evidence bytes. They do not
decode NPZ arrays, import a simulator, integrate physics, or claim qualification.
Run after the publication helper is frozen:

    python -B outputs/m8_table_supported/test_publish_supported_feedback_preservation.py
"""
from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import zipfile


HERE = Path(__file__).resolve().parent


def load_local_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, HERE / filename)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


publisher = load_local_module("raw_supported_publisher", "publish_supported_feedback_run.py")
reassembler = load_local_module("raw_supported_reassembler", "reassemble_archives.py")


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


class PreservationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="m8-publisher-regression-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.root_patch = mock.patch.object(publisher, "ROOT", self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)

    def write(self, path, raw):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
        return path

    def json_file(self, path, value):
        return self.write(path, (json.dumps(value, indent=2, allow_nan=False) + "\n").encode())

    def closed_run(self, *, changed=False, exit_code=17):
        run = self.root / "run"
        run.mkdir()
        dependencies = {"yam_twin/m8_supported_start.py": sha256(b"original helper bytes\n")}
        after_dependencies = dict(dependencies)
        if changed:
            after_dependencies["yam_twin/m8_supported_start.py"] = sha256(b"changed helper bytes\n")
        before = {
            "producer_commit": "1" * 40,
            "source_hashes_before": dependencies,
            "scope": "Unsuccessful exceptional partial native producer; raw preservation only.",
        }
        after = {
            "producer_commit": before["producer_commit"],
            "native_exit_code": exit_code,
            "source_hashes_before": dependencies,
            "source_hashes_after": after_dependencies,
            "source_hashes_unchanged": not changed,
        }
        self.json_file(run / "run_publication_identity.json", before)
        self.json_file(run / "run_publication_identity_after.json", after)
        return run, before, after

    def load_manifest(self, target):
        return json.loads((target / "package_manifest.json").read_bytes())

    def assert_unpublished(self, target):
        self.assertFalse(target.exists())
        self.assertFalse(list(target.parent.glob(target.name + ".packaging-*")))

    def assert_original_artifacts(self, manifest, originals):
        for name, original in originals.items():
            with self.subTest(original=name):
                artifact = manifest["artifacts"][name]
                self.assertEqual(artifact["bytes"], len(original))
                self.assertEqual(artifact["sha256"], sha256(original))

    def test_add_source_rejects_same_name_different_path_even_with_identical_bytes(self):
        first = self.write(self.root / "producer" / "validation.json", b'{"raw": true}\n')
        second = self.write(self.root / "renderer" / "validation.json", first.read_bytes())
        sources = {}
        publisher.add_source(sources, "validation.json", first)
        with self.assertRaises(ValueError):
            publisher.add_source(sources, "validation.json", second)
        self.assertEqual(sources, {"validation.json": first.resolve()})

    def test_add_source_accepts_repeat_of_exact_same_resolved_path(self):
        original = self.write(self.root / "producer" / "trace.npz", b"opaque original bytes")
        sources = {}
        publisher.add_source(sources, "trace.npz", original)
        publisher.add_source(sources, "trace.npz", original.parent / ".." / "producer" / "trace.npz")
        self.assertEqual(sources, {"trace.npz": original.resolve()})

    def test_raw_bundle_rejects_live_run_without_native_closure(self):
        run, _, _ = self.closed_run()
        (run / "run_publication_identity_after.json").unlink()
        self.write(run / "partial.npz", b"still-live NPZ bytes\x00\xff")
        originals = {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()}
        target = self.root / "published-live"
        with self.assertRaises((FileNotFoundError, ValueError)):
            publisher.raw_bundle(run, target)
        self.assert_unpublished(target)
        self.assertEqual(originals, {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()})

    def test_raw_bundle_rejects_target_inside_original_run_without_changing_it(self):
        run, _, _ = self.closed_run()
        self.write(run / "native" / "partial.npz", b"unchanged original native archive\x00\xff")
        originals = {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()}
        original_directories = {str(p.relative_to(run)) for p in run.rglob("*") if p.is_dir()}
        target = run / "package"
        with self.assertRaises(ValueError):
            publisher.raw_bundle(run, target)
        self.assert_unpublished(target)
        self.assertEqual(original_directories, {str(p.relative_to(run)) for p in run.rglob("*") if p.is_dir()})
        self.assertEqual(originals, {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()})

    def test_raw_bundle_requires_actual_integer_native_exit_code(self):
        run, _, after = self.closed_run()
        for index, invalid in enumerate((None, 0.0, "0", False, True)):
            with self.subTest(native_exit_code=invalid):
                invalid_after = dict(after, native_exit_code=invalid)
                self.json_file(run / "run_publication_identity_after.json", invalid_after)
                target = self.root / f"invalid-exit-{index}"
                with self.assertRaises(ValueError):
                    publisher.raw_bundle(run, target)
                self.assert_unpublished(target)
        self.json_file(run / "run_publication_identity_after.json", {k: v for k, v in after.items() if k != "native_exit_code"})
        target = self.root / "missing-exit"
        with self.assertRaises(ValueError):
            publisher.raw_bundle(run, target)
        self.assert_unpublished(target)

    def test_raw_bundle_rejects_mismatched_producer_or_before_source_identity(self):
        run, _, after = self.closed_run()
        mismatches = (
            dict(after, producer_commit="2" * 40),
            dict(after, source_hashes_before={"yam_twin/m8_supported_start.py": "3" * 64}),
        )
        for index, changed_after in enumerate(mismatches):
            with self.subTest(mismatch=index):
                self.json_file(run / "run_publication_identity_after.json", changed_after)
                target = self.root / f"mismatched-identity-{index}"
                with self.assertRaises(ValueError):
                    publisher.raw_bundle(run, target)
                self.assert_unpublished(target)

    def test_raw_exceptional_partial_keeps_nonfinite_json_npz_logs_and_sources_verbatim(self):
        run, _, _ = self.closed_run(exit_code=17)
        cap = 131_072
        # A ZIP-form NPZ whose contents are deliberately opaque to publication.
        # Its raw payload contains all byte values, with no NumPy dependency.
        binary = io.BytesIO()
        with zipfile.ZipFile(binary, "w", compression=zipfile.ZIP_STORED) as archive:
            info = zipfile.ZipInfo("raw_native_state.npy", date_time=(2026, 1, 1, 0, 0, 0))
            archive.writestr(info, bytes(range(256)) * 1100 + b"\x00NaN\xffInfinity\r\n")
        self.write(run / "exceptional_partial_states.npz", binary.getvalue())
        self.write(run / "partial_status.json", b'{"energy": NaN, "force": Infinity, "neg": -Infinity}\r\n')
        self.write(run / "native.log", b"native failure\r\nraw stderr:\x00\xff\xfe\n")
        # Original filenames that coincide with generated publication helpers
        # must remain distinct and recoverable under the original-run storage.
        self.write(run / "package_manifest.json", b'{"original_native_debug": NaN}\n')
        self.write(run / "reassemble_archives.py", b"# producer's unrelated original helper\r\n")
        self.write(run / "README.md", b"Original unsuccessful run notes\x00\xff\n")
        self.write(run / "controller_source.py", b"# original producer source\r\nvalue = b'\\xff'\n")
        self.write(run / "recorded_sources" / "yam_twin" / "m8_supported_start.py", b"original helper bytes\n")
        provenance = self.write(self.root / "execution" / "stderr.log", b"closed producer stderr\x00\xff\r\n")
        originals = {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()}
        target = self.root / "raw-closed-evidence"
        publisher.raw_bundle(run, target, chunk_bytes=cap, provenance=(provenance,))
        manifest = self.load_manifest(target)
        self.assertIs(manifest["physics_success_claim"], False)
        self.assert_original_artifacts(manifest, originals)
        self.assertEqual(set(originals), set(manifest["artifacts"]) - {
            name for name in manifest["artifacts"] if name.startswith("execution_provenance/")
        })
        provenance_names = [name for name in manifest["artifacts"] if name.startswith("execution_provenance/")]
        self.assertEqual(len(provenance_names), 1)
        self.assert_original_artifacts(manifest, {provenance_names[0]: provenance.read_bytes()})
        self.assertIn("insertion_trace.npz", manifest["missing_final_artifacts"])
        self.assertIn("insertion_validation.json", manifest["missing_final_artifacts"])
        for name in ("left_pad_force_history.npz", "table_support_force_history.npz", "native_feedback_force_history.npz"):
            self.assertIn(name, manifest["missing_final_artifacts"])
            self.assertNotIn(name, manifest["artifacts"])
        self.assertNotIn("insertion_validation.json", manifest["artifacts"])
        self.assertNotIn("validation.json", manifest["artifacts"])
        artifact = manifest["artifacts"]["exceptional_partial_states.npz"]
        self.assertEqual(artifact["storage"], "lossless_binary_chunks")
        raw = originals["exceptional_partial_states.npz"]
        self.assertEqual([part["offset"] for part in artifact["chunks"]], list(range(0, len(raw), cap)))
        self.assertEqual([part["bytes"] for part in artifact["chunks"]], [len(raw[i:i + cap]) for i in range(0, len(raw), cap)])
        for part in artifact["chunks"]:
            expected = raw[part["offset"]:part["offset"] + part["bytes"]]
            self.assertEqual((target / part["path"]).read_bytes(), expected)
            self.assertEqual(part["sha256"], sha256(expected))
        restored = self.root / "restored"
        result = reassembler.restore(target / "package_manifest.json", restored)
        self.assertEqual(result["verified_artifacts"], len(manifest["artifacts"]))
        for name, original in originals.items():
            with self.subTest(restored=name):
                self.assertEqual((restored / name).read_bytes(), original)
        self.assertEqual((restored / provenance_names[0]).read_bytes(), provenance.read_bytes())
        self.assertEqual(originals, {str(p.relative_to(run)): p.read_bytes() for p in run.rglob("*") if p.is_file()})

    def test_raw_bundle_preserves_and_labels_changed_source_closure_without_success_claim(self):
        run, before, after = self.closed_run(changed=True, exit_code=1)
        target = self.root / "changed-source-evidence"
        publisher.raw_bundle(run, target)
        manifest = self.load_manifest(target)
        self.assertIs(manifest["physics_success_claim"], False)
        self.assertIs(manifest["source_hashes_unchanged"], False)
        self.assertEqual(json.loads((target / manifest["artifacts"]["run_publication_identity.json"]["path"]).read_bytes()), before)
        self.assertEqual(json.loads((target / manifest["artifacts"]["run_publication_identity_after.json"]["path"]).read_bytes()), after)
        self.assertNotIn("insertion_validation.json", manifest["artifacts"])

    def artifact_manifest(self, *, cap=7):
        raw = b"\x00raw\xffnative\r\n" * 7
        source = self.write(self.root / "source.bin", raw)
        package = self.root / "binary-package"
        package.mkdir()
        artifact = publisher.write_artifact(source, package, "original/partial.npz", cap)
        manifest_path = self.json_file(package / "package_manifest.json", {"artifacts": {"original/partial.npz": artifact}})
        return raw, package, artifact, manifest_path

    def test_write_artifact_exact_contiguous_chunks_and_whole_identity(self):
        raw, package, artifact, manifest_path = self.artifact_manifest(cap=7)
        self.assertEqual(artifact["bytes"], len(raw))
        self.assertEqual(artifact["sha256"], sha256(raw))
        for index, chunk in enumerate(artifact["chunks"]):
            expected = raw[index * 7:(index + 1) * 7]
            self.assertEqual(chunk["index"], index)
            self.assertEqual(chunk["offset"], index * 7)
            self.assertEqual(chunk["bytes"], len(expected))
            self.assertEqual(chunk["sha256"], sha256(expected))
            self.assertEqual((package / chunk["path"]).read_bytes(), expected)
        restored = self.root / "restored-binary"
        reassembler.restore(manifest_path, restored)
        self.assertEqual((restored / "original/partial.npz").read_bytes(), raw)

    def corrupt_chunk(self, package, artifact):
        chunk = package / artifact["chunks"][1]["path"]
        raw = bytearray(chunk.read_bytes())
        raw[0] ^= 1
        chunk.write_bytes(raw)

    def test_corrupted_chunk_rejected_without_installing_reconstructed_target(self):
        _, package, artifact, manifest_path = self.artifact_manifest()
        self.corrupt_chunk(package, artifact)
        restored = self.root / "rejected-restoration"
        with self.assertRaisesRegex(ValueError, "Chunk size/SHA256 mismatch"):
            reassembler.restore(manifest_path, restored)
        self.assertFalse((restored / "original/partial.npz").exists())
        self.assertFalse(list(restored.rglob("*.reassembling")))

    def test_existing_identical_restoration_cannot_hide_corrupted_source_chunk(self):
        raw, package, artifact, manifest_path = self.artifact_manifest()
        restored = self.root / "existing-restoration"
        reassembler.restore(manifest_path, restored)
        destination = restored / "original/partial.npz"
        self.corrupt_chunk(package, artifact)
        with self.assertRaisesRegex(ValueError, "Chunk size/SHA256 mismatch"):
            reassembler.restore(manifest_path, restored)
        self.assertEqual(destination.read_bytes(), raw)
        self.assertFalse(list(restored.rglob("*.reassembling")))

    def compatibility_fixture(self, name):
        """Copy approved source/proof bytes; use small opaque native fixtures.

        No approved auditor or its regression source is imported or executed.
        The synthetic original report remains explicitly unsuccessful.
        """
        base = self.root / name
        run, artifacts = base / "run", base / "compatibility"
        run.mkdir(parents=True)
        artifacts.mkdir()
        approved = HERE / "full_canonical_v1_audits"
        original = HERE / "full_canonical_v1" / "frozen_audit_sources" / "scripts" / "audit_m8_supported_trace.py"
        inputs = {
            "original_auditor": original,
            "corrected_auditor": approved / "audit_m8_supported_trace_initial_boundary_v1.py",
            "original_exception": approved / "independent_supported_audit.log",
            "narrow_diff": approved / "supported_auditor_initial_boundary_v1.diff",
            "boundary_diagnosis": approved / "initial_acquisition_reader_boundary_diagnosis.json",
            "regression_proof": approved / "initial_boundary_reader_software_proof_v1.json",
            "test_source": approved / "test_supported_initial_boundary_reader_v1.py",
        }
        paths = {}
        for key, source in inputs.items():
            paths[key] = self.write(artifacts / source.name, source.read_bytes())
        original_sha = sha256(paths["original_auditor"].read_bytes())
        corrected_sha = sha256(paths["corrected_auditor"].read_bytes())
        self.assertEqual(original_sha, "e46808601d7974ffdea1e5e20e07e0f28eb8e0a74a394545b276621a7e856702")
        self.assertEqual(corrected_sha, "abb4e724de91eeec9dcff9c481d1a89674fba5ea3e3c5c2d9a9b76a074f9cf1e")
        self.assertEqual(sha256(paths["test_source"].read_bytes()), "44c0a25331d66f8406c0117c2f3d7b9d519112ba18c5f17e56d0563bf54f995f")
        self.write(run / "frozen_audit_sources" / "scripts" / "audit_m8_supported_trace.py", original.read_bytes())
        trace = self.write(run / "insertion_trace.npz", b"opaque synthetic closed native fixture\x00\xff")
        report = {
            "passed": False,
            "acceptance_checks": {
                "formed_thread_capture": {"passed": False},
                "native_left_hold": {"passed": True},
            },
        }
        self.json_file(run / "insertion_validation.json", report)
        producer = "1" * 40
        self.json_file(run / "run_publication_identity.json", {"producer_commit": producer})
        closure = {"producer_commit": producer, "native_exit_code": 1}
        self.json_file(run / "run_publication_identity_after.json", closure)
        audit = {
            "trajectory_sha256": sha256(trace.read_bytes()),
            "auditor_source_sha256": corrected_sha,
            "original_report_passed": report["passed"],
            "original_acceptance_checks": json.loads(json.dumps(report["acceptance_checks"])),
            "passed": False,
        }
        audit_path = self.json_file(artifacts / "corrected_report.json", audit)
        paths["corrected_report"] = audit_path
        declaration = {
            "schema": "supported-auditor-compatibility-v1",
            "correction_kind": "initial-settle-bolt-same-row-grasp-flag",
            "canonical_source_or_original_report_modified": False,
            "original_auditor_sha256": original_sha,
            "corrected_auditor_sha256": corrected_sha,
            "original_producer_commit": producer,
            "original_trace_sha256": audit["trajectory_sha256"],
            "original_native_exit_code": closure["native_exit_code"],
            "original_report_passed": report["passed"],
            "correction_scope": "Synthetic publication regression; approved output-only reader pair, no physical qualification.",
        }
        for key, path in paths.items():
            declaration[key] = {"path": str(path.relative_to(artifacts)), "sha256": sha256(path.read_bytes())}
        sidecar = self.json_file(artifacts / "compatibility.json", declaration)
        return {
            "run": run, "audit_path": audit_path, "audit": audit,
            "report": report, "closure": closure, "sidecar": sidecar,
            "declaration": declaration, "paths": paths,
        }

    def validate_compatibility(self, fixture):
        return publisher.validate_auditor_compatibility(
            fixture["sidecar"], fixture["run"], fixture["audit_path"],
            fixture["audit"], fixture["report"], fixture["closure"],
        )

    def rewrite_compatibility_artifact(self, fixture, key, raw):
        path = fixture["paths"][key]
        self.write(path, raw)
        fixture["declaration"][key]["sha256"] = sha256(raw)

    def store_compatibility_sidecar(self, fixture):
        self.json_file(fixture["sidecar"], fixture["declaration"])

    def test_known_output_only_auditor_compatibility_preserves_original_failed_results(self):
        fixture = self.compatibility_fixture("valid-reader")
        original_report = json.loads(json.dumps(fixture["report"]))
        summary, paths = self.validate_compatibility(fixture)
        self.assertEqual(summary["original_frozen_auditor_sha256"], fixture["declaration"]["original_auditor_sha256"])
        self.assertEqual(summary["corrected_output_only_reader_sha256"], fixture["declaration"]["corrected_auditor_sha256"])
        self.assertEqual(summary["regression_tests_passed"], 8)
        self.assertIs(summary["original_source_report_acceptance_and_proof_preserved"], True)
        self.assertEqual(paths, dict(fixture["paths"], sidecar=fixture["sidecar"]))
        self.assertEqual(fixture["report"], original_report)
        self.assertIs(fixture["report"]["passed"], False)
        self.assertIs(fixture["audit"]["original_acceptance_checks"]["formed_thread_capture"]["passed"], False)

    def test_published_compatibility_sidecar_resolves_all_exact_companion_paths_after_relocation(self):
        fixture = self.compatibility_fixture("portable-reader-sidecar")
        # Exercise a nested declared companion path as well as plain names.
        key = "narrow_diff"
        old_path = fixture["paths"][key]
        nested = fixture["sidecar"].parent / "reader_patch" / old_path.name
        self.write(nested, old_path.read_bytes())
        fixture["paths"][key] = nested
        fixture["declaration"][key]["path"] = str(nested.relative_to(fixture["sidecar"].parent))
        self.store_compatibility_sidecar(fixture)
        _, paths = self.validate_compatibility(fixture)
        sources = publisher.compatibility_publication_sources(fixture["sidecar"], paths)
        expected_names = {"auditor_compatibility/" + fixture["sidecar"].name}
        expected_names.update("auditor_compatibility/" + fixture["declaration"][companion]["path"] for companion in fixture["paths"])
        self.assertEqual(set(sources), expected_names)
        self.assertEqual(len(sources), 9)
        relocated = self.root / "relocated-publication"
        for name, source in sources.items():
            self.write(relocated / name, source.read_bytes())
        sidecar = relocated / "auditor_compatibility" / fixture["sidecar"].name
        self.assertEqual(sidecar.read_bytes(), fixture["sidecar"].read_bytes())
        declaration = json.loads(sidecar.read_bytes())
        for companion, original in fixture["paths"].items():
            with self.subTest(companion=companion):
                resolved = publisher.compatibility_artifact(sidecar, declaration, companion)
                self.assertEqual(resolved, (sidecar.parent / declaration[companion]["path"]).resolve())
                self.assertEqual(resolved.read_bytes(), original.read_bytes())

    def test_auditor_compatibility_rejects_wrong_native_bindings_or_relabelled_acceptance(self):
        changes = (
            ("sidecar", "original_trace_sha256", "0" * 64),
            ("sidecar", "original_producer_commit", "2" * 40),
            ("sidecar", "original_native_exit_code", 2),
            ("sidecar", "original_native_exit_code", True),
            ("sidecar", "original_report_passed", True),
            ("audit", "original_report_passed", True),
            ("audit", "auditor_source_sha256", "0" * 64),
            ("audit", "original_acceptance_checks", {"formed_thread_capture": {"passed": True}, "native_left_hold": {"passed": True}}),
            ("audit", "original_acceptance_checks", {"formed_thread_capture": {"passed": 0}, "native_left_hold": {"passed": True}}),
        )
        for index, (scope, field, value) in enumerate(changes):
            with self.subTest(scope=scope, field=field, value=value):
                fixture = self.compatibility_fixture(f"binding-{index}")
                if scope == "sidecar":
                    fixture["declaration"][field] = value
                else:
                    fixture["audit"][field] = value
                    raw = (json.dumps(fixture["audit"], indent=2, allow_nan=False) + "\n").encode()
                    self.rewrite_compatibility_artifact(fixture, "corrected_report", raw)
                self.store_compatibility_sidecar(fixture)
                with self.assertRaises(ValueError):
                    self.validate_compatibility(fixture)

    def test_auditor_compatibility_rejects_changed_reader_diff_exception_or_artifact_hash(self):
        for index, change in enumerate(("reader", "diff", "exception", "artifact_hash")):
            with self.subTest(change=change):
                fixture = self.compatibility_fixture(f"changed-artifact-{index}")
                if change == "reader":
                    raw = fixture["paths"]["corrected_auditor"].read_bytes() + b"\n# Unreviewed reader byte change\n"
                    self.rewrite_compatibility_artifact(fixture, "corrected_auditor", raw)
                    fixture["declaration"]["corrected_auditor_sha256"] = sha256(raw)
                elif change == "diff":
                    raw = fixture["paths"]["narrow_diff"].read_bytes().replace(b"@@ -512,7 +512,7 @@", b"@@ -513,7 +513,7 @@", 1)
                    self.rewrite_compatibility_artifact(fixture, "narrow_diff", raw)
                elif change == "exception":
                    self.rewrite_compatibility_artifact(fixture, "original_exception", b"different native reader failure\n")
                else:
                    fixture["declaration"]["boundary_diagnosis"]["sha256"] = "0" * 64
                self.store_compatibility_sidecar(fixture)
                with self.assertRaises(ValueError):
                    self.validate_compatibility(fixture)

    def test_auditor_compatibility_rejects_missing_or_falsely_bound_regression_proof(self):
        changes = (
            ("missing", "regression_proof", None),
            ("missing", "test_source", None),
            ("proof", "passed", False),
            ("proof", "sources_unchanged", False),
            ("proof", "tests_passed", 0),
            ("proof", "tests_passed", True),
            ("proof", "corrected_auditor_sha256", "0" * 64),
            ("proof", "test_source_sha256", "0" * 64),
        )
        for index, (scope, field, value) in enumerate(changes):
            with self.subTest(scope=scope, field=field, value=value):
                fixture = self.compatibility_fixture(f"bad-proof-{index}")
                if scope == "missing":
                    fixture["paths"][field].unlink()
                else:
                    proof = json.loads(fixture["paths"]["regression_proof"].read_bytes())
                    proof[field] = value
                    self.rewrite_compatibility_artifact(fixture, "regression_proof", (json.dumps(proof, indent=2) + "\n").encode())
                self.store_compatibility_sidecar(fixture)
                with self.assertRaises(ValueError):
                    self.validate_compatibility(fixture)

    def test_auditor_compatibility_rejects_changed_original_or_mismatched_frozen_baseline(self):
        for index, change in enumerate(("original", "frozen")):
            with self.subTest(change=change):
                fixture = self.compatibility_fixture(f"bad-baseline-{index}")
                raw = fixture["paths"]["original_auditor"].read_bytes() + b"\n# Different frozen reader bytes\n"
                if change == "original":
                    self.rewrite_compatibility_artifact(fixture, "original_auditor", raw)
                    fixture["declaration"]["original_auditor_sha256"] = sha256(raw)
                    self.store_compatibility_sidecar(fixture)
                else:
                    self.write(fixture["run"] / "frozen_audit_sources" / "scripts" / "audit_m8_supported_trace.py", raw)
                with self.assertRaises(ValueError):
                    self.validate_compatibility(fixture)

    def refresh_package_checksums(self, package):
        stored = sorted(path for path in package.rglob("*") if path.is_file() and path.name != "SHA256SUMS")
        raw = "".join(f"{sha256(path.read_bytes())}  {path.relative_to(package)}\n" for path in stored)
        self.write(package / "SHA256SUMS", raw.encode())

    def original_layout_package(self, name):
        base = self.root / name
        package = base / "package"
        package.mkdir(parents=True)
        originals = {
            "insertion_trace.npz": bytes(range(256)) * 1100 + b"opaque original native archive\x00\xff",
            "insertion_validation.json": b'{"passed": false, "checks": {"capture": false}}\r\n',
            "manifest.json": b'{\r\n  "producer": "original", "passed": false\r\n}\r\n',
            "recorded_sources/yam_twin/m8_supported_start.py": b"# Original producer dependency\r\ndef helper():\n    return b'\\x00\\xff'\n",
        }
        aliases = {
            "insertion_trace.npz": "trace.npz",
            "insertion_validation.json": "validation.json",
            "manifest.json": "software_manifest.json",
            "recorded_sources/yam_twin/m8_supported_start.py": "recorded_sources/yam_twin/m8_supported_start.py",
        }
        artifacts = {}
        for original, raw in originals.items():
            source = self.write(base / "sources" / original, raw)
            alias = aliases[original]
            artifacts[alias] = publisher.write_artifact(source, package, alias, 131_072)
        supplemental = self.write(base / "sources" / "supplemental_audit.json", b'{"output_only_reader": true}\n')
        artifacts["supplemental_audit.json"] = publisher.write_artifact(supplemental, package, "supplemental_audit.json", 131_072)
        manifest = {
            "status": "unsuccessful_native_trial",
            "artifacts": artifacts,
            "original_run_file_to_published_artifact_names": {original: [alias] for original, alias in aliases.items()},
            "original_run_file_identities": {original: {"bytes": len(raw), "sha256": sha256(raw)} for original, raw in originals.items()},
        }
        manifest_path = self.json_file(package / "package_manifest.json", manifest)
        self.write(package / "reassemble_archives.py", (HERE / "reassemble_archives.py").read_bytes())
        self.write(package / "README.md", b"Synthetic exact original-name restoration package.\n")
        self.refresh_package_checksums(package)
        return package, manifest_path, manifest, originals

    def test_original_run_layout_restores_aliases_chunked_trace_and_nested_sources_exactly(self):
        restore_helper = load_local_module("original_layout_restorer", "restore_original_run_layout.py")
        package, manifest_path, manifest, originals = self.original_layout_package("valid-original-layout")
        self.assertEqual(manifest["artifacts"]["trace.npz"]["storage"], "lossless_binary_chunks")
        self.assertGreater(len(manifest["artifacts"]["trace.npz"]["chunks"]), 1)
        output = self.root / "restored-original-run"
        package_before = {str(path.relative_to(package)): path.read_bytes() for path in package.rglob("*") if path.is_file()}
        verified = restore_helper.restore_original(manifest_path, output=output, verify_only=True)
        self.assertEqual(verified["verified_original_files"], len(originals))
        self.assertEqual(verified["newly_installed_files"], 0)
        self.assertIs(verified["lossless"], True)
        self.assertFalse(output.exists())
        restored = restore_helper.restore_original(manifest_path, output=output)
        self.assertEqual(restored["verified_original_files"], len(originals))
        self.assertEqual(restored["newly_installed_files"], len(originals))
        self.assertIs(restored["original_relative_names_restored"], True)
        actual = {str(path.relative_to(output)): path.read_bytes() for path in output.rglob("*") if path.is_file()}
        self.assertEqual(actual, originals)
        self.assertNotIn("trace.npz", actual)
        self.assertNotIn("validation.json", actual)
        self.assertNotIn("software_manifest.json", actual)
        self.assertNotIn("supplemental_audit.json", actual)
        self.assertEqual(package_before, {str(path.relative_to(package)): path.read_bytes() for path in package.rglob("*") if path.is_file()})

    def test_original_run_layout_refuses_changed_destination_or_bad_alias_before_installing_files(self):
        restore_helper = load_local_module("original_layout_refusal", "restore_original_run_layout.py")
        for index, rejection in enumerate(("changed_destination", "bad_alias")):
            with self.subTest(rejection=rejection):
                package, manifest_path, manifest, _ = self.original_layout_package(f"refused-original-layout-{index}")
                output = self.root / f"refused-native-output-{index}"
                if rejection == "changed_destination":
                    self.write(output / "manifest.json", b"Existing different original manifest; retain verbatim\x00\xff")
                    self.write(output / "unrelated" / "sentinel.bin", b"Untouched existing file\r\n")
                else:
                    # The first alias is valid. A second, existing package
                    # artifact with different whole bytes must not be ignored.
                    manifest["original_run_file_to_published_artifact_names"]["insertion_trace.npz"].append("validation.json")
                    self.json_file(manifest_path, manifest)
                    self.refresh_package_checksums(package)
                directories_before = {str(path.relative_to(output)) for path in output.rglob("*") if path.is_dir()} if output.exists() else set()
                files_before = {str(path.relative_to(output)): path.read_bytes() for path in output.rglob("*") if path.is_file()} if output.exists() else {}
                with self.assertRaises((FileExistsError, ValueError)):
                    restore_helper.restore_original(manifest_path, output=output)
                self.assertEqual(files_before, {str(path.relative_to(output)): path.read_bytes() for path in output.rglob("*") if path.is_file()} if output.exists() else {})
                self.assertEqual(directories_before, {str(path.relative_to(output)) for path in output.rglob("*") if path.is_dir()} if output.exists() else set())
                if rejection == "bad_alias":
                    self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
