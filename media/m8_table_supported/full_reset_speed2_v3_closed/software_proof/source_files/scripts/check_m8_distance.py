"""Independent 3D closest-surface reference for the plugin's local metric."""
import argparse
import ctypes,sys,json,math
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from thread_lab.build_plugin import build_plugin
lib=ctypes.CDLL(str(build_plugin()))
lib.astra_thread_distance.argtypes=[ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
lib.astra_thread_distance.restype=ctypes.c_double
P=.00125;H=math.sqrt(3)*P/2
attrs=np.array([.008,P,.030,.0071,.013,.000956,0,0])

def radial(v):
    v=(v+P/2)%P-P/2
    a=abs(v);r2=attrs[3]/2
    if a<=P/16:return r2+3*H/8
    if a>=3*P/8:
        R=H/6;w=P/2-a
        return r2-H/6-math.sqrt(max(R*R-w*w,0))
    return r2+math.sqrt(3)*(P/4-a)

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=Path('outputs/m8/geometry_distance.json'))
args=parser.parse_args()
results=[]
for phase in [0,.10*P,.25*P,.40*P,.5*P]:
 for offset in [-1e-4,-1e-5,1e-5,1e-4]:
    rho=radial(phase)+offset
    point=np.array([rho,0,.0125+phase])
    approx=abs(lib.astra_thread_distance(point.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),attrs.ctypes.data_as(ctypes.POINTER(ctypes.c_double))))
    def obj(x):
        v=x[0]*P;angle=x[1]
        rp=radial(phase+v)
        return ((rp*math.cos(angle)-rho)**2+(rp*math.sin(angle))**2+(v+P*angle/(2*math.pi))**2)*1e6
    fits=[minimize(obj,[start,0],bounds=[(-1,1),(-.6,.6)],method='L-BFGS-B',options={'ftol':1e-14,'gtol':1e-10,'maxiter':100}) for start in np.linspace(-.6,.6,9)]
    true=math.sqrt(min(f.fun for f in fits))*1e-3
    results.append({'phase_fraction':phase/P,'radial_offset_um':offset*1e6,'approx_um':approx*1e6,'closest_3d_um':true*1e6,'absolute_error_um':abs(approx-true)*1e6,'relative_error':abs(approx-true)/true})
out={'max_error_um':max(r['absolute_error_um'] for r in results),'max_relative_error':max(r['relative_error'] for r in results),'probes':results}
out['method']='Independent 3D Euclidean closest-surface optimization, nine bounded starts per probe; signed-field constructive caps are excluded.'
out['plugin_source_sha256']=__import__('hashlib').sha256((Path(__file__).resolve().parents[1]/'thread_lab/plugins/m8_sdf.cc').read_bytes()).hexdigest()
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items() if k!='probes'},indent=2))
print(args.output)
if out['max_error_um']>.01:
    raise SystemExit('Thread contact distance discrepancy exceeded the 0.01 micrometer audit threshold.')
