#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_repo_root"
export UV_CACHE_DIR=/workspace/.cache/uv
export XDG_CACHE_HOME=/workspace/.cache
export WARP_CACHE_PATH=/workspace/.cache/warp
export MPLCONFIGDIR=/workspace/.cache/matplotlib
export MESA_SHADER_CACHE_DIR=/workspace/.cache/mesa
export PYTHONHASHSEED=0
mkdir -p "$UV_CACHE_DIR" "$XDG_CACHE_HOME" "$WARP_CACHE_PATH" "$MPLCONFIGDIR" "$MESA_SHADER_CACHE_DIR"
for task_tool in uv git gcc g++; do
  command -v "$task_tool" >/dev/null || { printf 'Required setup tool is missing: %s\n' "$task_tool" >&2; exit 1; }
done
uv venv --allow-existing /workspace/.venvs/astra-r2s
uv pip sync --python /workspace/.venvs/astra-r2s/bin/python --require-hashes requirements.lock
uv pip install --python /workspace/.venvs/astra-r2s/bin/python --no-deps -e .
# Genuine M8 contact uses a separately built GCC core and matching bindings.
# Keep the stock core and MJLab environments intact: their C++ ABIs differ.
/workspace/.venvs/astra-r2s/bin/python scripts/build_mujoco_thread_engine.py --jobs "${M8_BINDINGS_JOBS:-2}"
scripts/build_mujoco_bindings.sh
uv pip install --python /workspace/.venvs/m8-contact/bin/python --no-deps -e .
scripts/run_m8.sh -m thread_lab.build_plugin
uv venv --allow-existing /workspace/.venvs/mjlab-twin
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --index https://download.pytorch.org/whl/cpu 'torch==2.14.1+cpu'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-sources 'mjlab @ git+https://github.com/mujocolab/mjlab@033ae22a2c7a30a25a6fa77b16c113ed88dd1b55'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-deps -e .
