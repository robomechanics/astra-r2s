# Recreate the published geometry-only media

The original native trial ran at producer commit `6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1`. Its original 402-test software proof covers 65 unchanged source files. All 65 exact bytes are in `producer_sources/`; `producer_source_manifest.json` binds them to the original beginning, closure and software proof. This software proof is separate from the failed native physical trial and the publisher's storage tests.

The exact executed rendering source is `renderer_sources/render_supported_failed_geometry.py`. Its original `ROOT` is the literal path `/workspace/astra-r2s`. To execute those exact source bytes, use a fresh workspace/container with the producer checkout at that path. A checkout at another path needs an explicitly separate path adaptation; such a modified script is not the exact executed rendering source.

Keep the newer published package in a separate review checkout. In a fresh workspace, prepare the exact producer checkout and matched native runtime using `docs/m8_setup.md`. The pinned GCC CPU MuJoCo engine, bindings and plugin are required; the stock wheel is insufficient. Original runtime core/plugin hashes are preserved. A rebuild on a different machine must be compared before claiming byte-identical native libraries.

Restore the complete verified package and the ORIGINAL native run layout to ignored output directories. The publisher's `restore_original_run_layout.py` verifies stored checksums and every reconstructed whole file, including native filename aliases; it does not invoke a simulator. Set `task_package` to the final package in the newer review checkout:

```sh
cd /workspace/astra-r2s
test "$(git rev-parse HEAD)" = 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1
task_package=/absolute/path/to/newer/review/checkout/media/m8_table_supported/full_canonical_v1_failed_evidence
task_restored=/workspace/astra-r2s/outputs/m8_table_supported/failed_v1_media_inputs
task_original=/workspace/astra-r2s/outputs/m8_table_supported/failed_v1_original_run
python "$task_package/reassemble_archives.py" --verify-only
python "$task_package/reassemble_archives.py" --output "$task_restored"
python "$task_package/restore_original_run_layout.py" --output "$task_original"
```

Before replay, compare every current producer source against the archived 65-file manifest. This rejects the current checkout if it contains later changes:

```sh
python - "$task_restored/producer_source_manifest.json" <<'PY'
import hashlib, json, sys
from pathlib import Path
root = Path('/workspace/astra-r2s')
identity = json.loads(Path(sys.argv[1]).read_text())
assert identity['source_hash_count'] == 65
for name, expected in identity['source_hashes'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
print('All65 exact original producer sources verified')
PY
scripts/run_m8.sh "$task_restored/renderer_sources/render_supported_failed_geometry.py" \
  "$task_original" outputs/m8_table_supported/failed_v1_geometry_replay
```

The output directory must not already exist. The executed helper independently checks the exact original report/trace, original source hashes, model archive and runtime before rendering, then checks source/runtime and original run files again afterward. It writes exact original qpos/qvel/time and refreshes ONLY `mj_kinematics`, `mj_comPos` and `mj_camlight`. It performs no `mj_forward`, collision discovery, dynamics/contact-force solve, integration, interpolation, workpiece posing or trajectory splice. The canonical renderer retains its original cameras and `geomgroup[3:]` filter; supplementary detail images use real cameras and default visible geometry, with no additional body hiding.

The scientific plot has its own exact executed source `scientific_plot/plot_supported_direction_failure.py` and hash/reference binding. It reads original dense raw ledgers, not sparse post-step qpos, and does not invoke a simulator. Its historical input path is the original `outputs/m8_table_supported/full_canonical_v1`; restore to that path in a fresh checkout if recreating its exact path-bound source.

The primary historical `e468` auditor raises an initial acquisition-row reader exception on this source. Its original exception and source remain intact. The corrected output-only reader and its narrow source diff, diagnosis and eight separate arithmetic regressions are explicitly bound under `auditor_compatibility/` and `supplemental_audits/`; neither the original software402/65 proof nor native acceptance results changed. A generic later `supported_demo --replay` invocation is a NEW replay with its own state refresh; it is not recreation of the exact published geometry-only media.
