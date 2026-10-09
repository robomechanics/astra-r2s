from pathlib import Path
import subprocess
import os
import mujoco

def build_plugin() -> Path:
    if mujoco.__version__!='3.15.0':
        raise RuntimeError('The native thread contact plugin requires MuJoCo 3.15.0; use the pinned physics environment.')
    root=Path(__file__).resolve().parent
    package=Path(mujoco.__file__).parent
    library=next(package.glob('libmujoco.so.*'))
    output=root/'plugins'/'libm8_sdf.so'
    source=root/'plugins'/'m8_sdf.cc'
    if not output.exists() or source.stat().st_mtime > output.stat().st_mtime:
        temporary=output.with_name(f'.libm8_sdf.{os.getpid()}.so')
        subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O3',str(source),'-I'+str(package/'include'),str(library),'-Wl,-rpath,'+str(package),'-o',str(temporary)],check=True)
        os.replace(temporary,output)
    return output

if __name__=='__main__':
    print(build_plugin())
