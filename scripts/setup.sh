#!/usr/bin/env bash
set -euo pipefail
cd /workspace/astra-r2s
export UV_CACHE_DIR=/workspace/.cache/uv
export XDG_CACHE_HOME=/workspace/.cache
export WARP_CACHE_PATH=/workspace/.cache/warp
export MPLCONFIGDIR=/workspace/.cache/matplotlib
uv venv --allow-existing /workspace/.venvs/astra-r2s
uv pip sync --python /workspace/.venvs/astra-r2s/bin/python --require-hashes requirements.lock
uv pip install --python /workspace/.venvs/astra-r2s/bin/python --no-deps -e .
uv venv --allow-existing /workspace/.venvs/mjlab-twin
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --index https://download.pytorch.org/whl/cpu 'torch==2.14.1+cpu'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-sources 'mjlab @ git+https://github.com/mujocolab/mjlab@033ae22a2c7a30a25a6fa77b16c113ed88dd1b55'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-deps -e .
