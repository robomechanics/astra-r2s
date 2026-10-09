#!/usr/bin/env bash
set -euo pipefail
task_repo_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
task_python="${ASTRA_PYTHON:-/workspace/.venvs/m8-contact/bin/python}"
task_record="${M8_BINDINGS_SDK:-/workspace/research/m8-contact-sdk}/bindings_verification.json"
if [[ ! -x "$task_python" || ! -f "$task_record" ]]; then
  printf '%s\n' "Set up the matched M8 environment first: $task_repo_root/scripts/setup.sh" >&2
  exit 1
fi
# The matched wheel already contains its GCC-built core. Preloading a different
# core can silently reintroduce the stock-wheel C++ ABI mismatch.
unset LD_PRELOAD
export PYTHONHASHSEED=0
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/workspace/.cache}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/workspace/.cache/matplotlib}"
export MESA_SHADER_CACHE_DIR="${MESA_SHADER_CACHE_DIR:-/workspace/.cache/mesa}"
export WARP_CACHE_PATH="${WARP_CACHE_PATH:-/workspace/.cache/warp}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
cd "$task_repo_root"
task_verify_args=(--record "$task_record")
if [[ -n "${ASTRA_MUJOCO_LIB:-}" ]]; then
  task_verify_args+=(--expected-core "$ASTRA_MUJOCO_LIB")
fi
"$task_python" "$task_repo_root/scripts/verify_m8_bindings.py" "${task_verify_args[@]}" >/dev/null
if (( $# == 0 )); then
  set -- -m thread_lab.benchmark
fi
exec "$task_python" "$@"
