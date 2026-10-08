"""Build exact upstream MuJoCo with a transparent SDF search-scale patch.

No simulator wheel files are replaced. Build matched GCC Python bindings with
build_mujoco_bindings.sh, then launch experiments through run_m8.sh.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

COMMIT='9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5'
REPOSITORY='https://github.com/google-deepmind/mujoco.git'

def run(args,**kwargs):
    return subprocess.run(args,check=True,text=True,**kwargs)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=Path('/workspace/research/mujoco-3.15.0'))
    parser.add_argument('--build',type=Path,default=Path('/workspace/research/mujoco-3.15.0-thread-build'))
    parser.add_argument('--jobs',type=int,default=min(os.cpu_count() or 2,4))
    args=parser.parse_args();source=args.source.resolve();build=args.build.resolve()
    if not (source/'.git').exists():
        source.parent.mkdir(parents=True,exist_ok=True)
        run(['git','clone','--depth','1','--branch','3.15.0',REPOSITORY,str(source)])
    sha=run(['git','rev-parse','HEAD'],cwd=source,capture_output=True).stdout.strip()
    if sha!=COMMIT:raise RuntimeError(f'Expected official 3.15.0 commit {COMMIT}, got {sha}')
    remote=run(['git','remote','get-url','origin'],cwd=source,capture_output=True).stdout.strip()
    if remote!=REPOSITORY:raise RuntimeError(f'Unexpected source remote {remote}')
    changed=run(['git','diff','HEAD','--name-only'],cwd=source,capture_output=True).stdout.splitlines()
    if set(changed)-{'src/engine/engine_collision_sdf.c'}:
        raise RuntimeError(f'Preserve additional upstream edits and build from a fresh checkout: {changed}')
    target=source/'src/engine/engine_collision_sdf.c'
    original=run(['git','show',f'{COMMIT}:src/engine/engine_collision_sdf.c'],cwd=source,capture_output=True).stdout
    needle='const mjtNum amin = 1e-4;  // minimum value for alpha'
    if original.count(needle)!=1:raise RuntimeError('Pinned upstream source does not match expected patch')
    marker='\n// Exported marker: proves this research core is loaded rather than stock.\n#include <mujoco/mjexport.h>\nMJAPI mjtNum astra_sdf_min_alpha(void) { return 1e-7; }\n'
    initial_patch=original.replace(needle,'const mjtNum amin = 1e-7;  // 0.1 micrometer line-search floor for M8 contact research')+marker
    modified=initial_patch.replace('mjtNum alpha = 2.;  // initial line search factor scaling the gradient','mjtNum alpha = .002;  // 2 mm initial step for M8-scale SDF contact research')
    modified+='MJAPI mjtNum astra_sdf_initial_alpha(void) { return .002; }\n'
    if target.read_text() not in (original,initial_patch,modified):
        raise RuntimeError('Source has additional local edits; preserve them and use a fresh source directory')
    if target.read_text()!=modified:
        target.write_text(modified)
    patch=run(['git','diff','--','src/engine/engine_collision_sdf.c'],cwd=source,capture_output=True).stdout
    patch_path=Path(__file__).parent/'patches/mujoco-3.15.0-micron-sdf.patch'
    patch_path.parent.mkdir(parents=True,exist_ok=True);patch_path.write_text(patch)
    cmake=shutil.which('cmake') or str(Path(sys.executable).parent/'cmake')
    ninja=shutil.which('ninja') or str(Path(sys.executable).parent/'ninja')
    run([cmake,'-S',str(source),'-B',str(build),'-G','Ninja',
         '-DCMAKE_BUILD_TYPE=Release','-DCMAKE_MAKE_PROGRAM='+ninja,
         '-DMUJOCO_BUILD_EXAMPLES=OFF','-DMUJOCO_BUILD_SIMULATE=OFF',
         '-DMUJOCO_BUILD_TESTS=OFF','-DMUJOCO_TEST_PYTHON_UTIL=OFF',
         '-DMUJOCO_WITH_USD=OFF'])
    run([cmake,'--build',str(build),'--target','mujoco','--parallel',str(args.jobs)])
    library=build/'lib/libmujoco.so.3.15.0'
    report={'upstream_repository':REPOSITORY,'upstream_commit':COMMIT,'version':'3.15.0',
            'patch':str(patch_path),'patch_sha256':hashlib.sha256(patch.encode()).hexdigest(),
            'library':str(library),'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
            'amin_meters':1e-7,'initial_alpha_meters':.002}
    (build/'thread_engine_provenance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    print('Next: scripts/build_mujoco_bindings.sh, then scripts/run_m8.sh -m thread_lab.benchmark')

if __name__=='__main__':main()
