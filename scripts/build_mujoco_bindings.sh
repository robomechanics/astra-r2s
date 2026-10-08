#!/usr/bin/env bash
# Build GCC Python bindings against the same patched GCC MuJoCo core.
# This avoids mixing PyPI wheel libc++ MjSpec objects with libstdc++ objects.
# Only the isolated m8-contact venv and /workspace/research build outputs change.
set -euo pipefail
task_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
task_source="${M8_MUJOCO_SOURCE:-/workspace/research/mujoco-3.15.0}"
task_core="${M8_MUJOCO_CORE:-/workspace/research/mujoco-3.15.0-thread-build/lib/libmujoco.so.3.15.0}"
task_venv="${M8_BINDINGS_VENV:-/workspace/.venvs/m8-contact}"
task_stage="${M8_BINDINGS_STAGE:-/workspace/research/mujoco-3.15.0-python-gcc}"
task_sdk="${M8_BINDINGS_SDK:-/workspace/research/m8-contact-sdk}"
task_jobs="${M8_BINDINGS_JOBS:-2}"
export UV_CACHE_DIR=/workspace/.cache/uv
export PYTHONHASHSEED=0
unset LD_PRELOAD
if [[ "$(git -C "$task_source" rev-parse HEAD)" != 9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5 ]]; then
  printf 'Expected the pinned official MuJoCo 3.15.0 source.\n' >&2
  exit 1
fi
while IFS= read -r task_changed_source; do
  if [[ -n "$task_changed_source" && "$task_changed_source" != src/engine/engine_collision_sdf.c ]]; then
    printf 'Unexpected local upstream source edit: %s\n' "$task_changed_source" >&2
    exit 1
  fi
done < <(git -C "$task_source" diff HEAD --name-only)
if [[ ! -x "$task_venv/bin/python" ]]; then
  uv venv "$task_venv"
fi
mkdir -p "$task_sdk"
# Preserve the core lock verbatim. Omit the stock MuJoCo wheel and the two
# build tools supplied by the separately pinned binding-build toolchain.
# Every remaining requirement still retains its original package hashes.
"$task_venv/bin/python" - "$task_repo_root/requirements.lock" "$task_sdk/runtime-bootstrap.lock" <<'PY'
from pathlib import Path
import re,sys
text=Path(sys.argv[1]).read_text()
blocks=re.split(r'(?m)(?=^[A-Za-z0-9][A-Za-z0-9_.-]*==)',text)
kept=[block for block in blocks
      if not re.match(r'(?:mujoco|cmake|ninja)==',block)]
Path(sys.argv[2]).write_text(''.join(kept))
PY
uv pip install --python "$task_venv/bin/python" --require-hashes --no-deps \
  -r "$task_sdk/runtime-bootstrap.lock"
uv pip install --python "$task_venv/bin/python" --require-hashes \
  -r "$task_repo_root/requirements-build.lock"
task_fingerprint="$("$task_venv/bin/python" - "$task_repo_root" "$task_core" <<'PY'
from pathlib import Path
import hashlib,json,sys,sysconfig
repo,core=map(Path,sys.argv[1:])
files=[repo/'requirements.lock',repo/'requirements-build.lock',
       repo/'scripts/build_mujoco_bindings.sh',repo/'scripts/verify_m8_bindings.py',core]
payload={'files':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
         'source_commit':'9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5',
         'python':sys.version,'soabi':sysconfig.get_config_var('SOABI'),
         'python_hash_seed':0}
print(hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest())
PY
)"
if "$task_venv/bin/python" "$task_repo_root/scripts/verify_m8_bindings.py" \
    --record "$task_sdk/bindings_verification.json" --expected-core "$task_core" \
    --fingerprint "$task_fingerprint" >/dev/null 2>&1; then
  printf 'Matched MuJoCo core/bindings already verified; cached build reused.\n'
  exit 0
fi
"$task_venv/bin/python" - "$task_source" "$task_core" "$task_stage" "$task_sdk" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys
source,core,stage,sdk=map(Path,sys.argv[1:])
for p in [stage,sdk/'lib',sdk/'plugin']:
    p.mkdir(parents=True,exist_ok=True)
shutil.copytree(source/'include',sdk/'include',dirs_exist_ok=True)
shutil.copy2(core,sdk/'lib/libmujoco.so.3.15.0')
link=sdk/'lib/libmujoco.so'
if not link.exists():link.symlink_to('libmujoco.so.3.15.0')
shutil.copytree(source/'python',stage,dirs_exist_ok=True)
shutil.copytree(source/'cmake',stage/'mujoco/cmake',dirs_exist_ok=True)
shutil.copytree(source/'simulate',stage/'mujoco/simulate',dirs_exist_ok=True)
shutil.copy2(source/'LICENSE',stage/'LICENSE')
# GLFW's normal header mode includes system GL headers, which this CPU cloud
# lacks. These adapters invoke MuJoCo rendering through its C API and make no
# direct GL calls, so GLFW's documented no-GL-header mode is sufficient. This
# changes build definitions only, not MuJoCo or binding implementation code.
with (stage/'mujoco/CMakeLists.txt').open('a') as cmake:
    cmake.write('\n# Headless cloud: avoid unused system OpenGL headers.\n'
                'target_compile_definitions(platform_ui_adapter PRIVATE GLFW_INCLUDE_NONE)\n'
                'target_compile_definitions(_simulate PRIVATE GLFW_INCLUDE_NONE)\n')
manifest={'source_commit':'9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5',
          'core_source':str(core),'sdk_core':str(sdk/'lib/libmujoco.so.3.15.0'),
          'core_sha256':hashlib.sha256(core.read_bytes()).hexdigest(),
          'compiler':'GCC/libstdc++',
          'headless_build_definitions':['GLFW_BUILD_WAYLAND=OFF','GLFW_BUILD_X11=OFF',
                                       'platform_ui_adapter:GLFW_INCLUDE_NONE',
                                       '_simulate:GLFW_INCLUDE_NONE']}
(sdk/'bindings_sdk_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
# Generate the same headers as official make_sdist.sh without importing a
# half-built mujoco package. These generators import the pure introspect module.
for task_generator in enum_traits function_traits spec_bindings; do
  case "$task_generator" in
    spec_bindings) task_output=specs.cc.inc ;;
    *) task_output="$task_generator.h" ;;
  esac
  PYTHONPATH="$task_stage/mujoco" "$task_venv/bin/python" \
    "$task_stage/mujoco/codegen/generate_$task_generator.py" > "$task_stage/mujoco/$task_output.tmp"
  if cmp -s "$task_stage/mujoco/$task_output.tmp" "$task_stage/mujoco/$task_output"; then
    rm "$task_stage/mujoco/$task_output.tmp"
  else
    mv "$task_stage/mujoco/$task_output.tmp" "$task_stage/mujoco/$task_output"
  fi
done
# setup.py otherwise expands -j to host CPU count, ignoring container quota.
# A transparent wrapper only limits build parallelism; source stays unchanged.
cat > "$task_sdk/cmake-wrapper.py" <<'PY'
#!/usr/bin/env python3
import os,sys
args=sys.argv[1:]
if '--build' in args:
    args=[('-j'+os.environ.get('M8_BINDINGS_JOBS','2')) if a.startswith('-j') else a for a in args]
os.execv(os.environ['M8_REAL_CMAKE'],[os.environ['M8_REAL_CMAKE'],*args])
PY
chmod +x "$task_sdk/cmake-wrapper.py"
export PATH="$task_venv/bin:$PATH"
export M8_REAL_CMAKE="$task_venv/bin/cmake"
export M8_BINDINGS_JOBS="$task_jobs"
export MUJOCO_CMAKE="$task_sdk/cmake-wrapper.py"
export MUJOCO_PATH="$task_sdk"
export MUJOCO_PLUGIN_PATH="$task_sdk/plugin"
task_pybind_cmake="$("$task_venv/bin/python" -m pybind11 --cmakedir)"
export MUJOCO_CMAKE_ARGS="-DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.15 -DMUJOCO_PYTHON_USE_SYSTEM_PYBIND11=ON -DGLFW_BUILD_WAYLAND=OFF -DGLFW_BUILD_X11=OFF -Dpybind11_DIR=$task_pybind_cmake"
cd "$task_stage"
"$task_venv/bin/python" -m build --no-isolation --wheel --outdir "$task_sdk/wheels"
if "$task_venv/bin/python" - "$task_sdk/wheels" <<'PY'
from pathlib import Path
import sys,zipfile
try:
    import mujoco
    package=Path(mujoco.__file__).resolve().parent
    wheels=list(Path(sys.argv[1]).glob('mujoco-3.15.0-*.whl'))
    if len(wheels)!=1:raise RuntimeError('Expected exactly one matched wheel.')
    with zipfile.ZipFile(wheels[0]) as archive:
        for name in archive.namelist():
            if not name.startswith('mujoco/') or name.endswith('/'):continue
            target=package/name.removeprefix('mujoco/')
            if not target.is_file() or target.read_bytes()!=archive.read(name):
                raise RuntimeError('Installed package differs from rebuilt wheel.')
except (ImportError,FileNotFoundError,RuntimeError):
    sys.exit(1)
PY
then
  printf 'Rebuilt wheel matches installed files; installation left intact.\n'
else
  uv pip install --python "$task_venv/bin/python" --reinstall --no-deps "$task_sdk"/wheels/mujoco-3.15.0-*.whl
fi
cd /tmp
"$task_venv/bin/python" "$task_repo_root/scripts/verify_m8_bindings.py" \
  --record "$task_sdk/bindings_verification.json" --expected-core "$task_core" \
  --fingerprint "$task_fingerprint" --write
