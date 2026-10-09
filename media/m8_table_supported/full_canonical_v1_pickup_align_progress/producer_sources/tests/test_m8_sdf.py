"""Geometry tests for the metric contact plugin, independent of a trajectory."""
import ctypes
import math
import numpy as np
import pytest

from thread_lab.build_plugin import build_plugin

P=.00125
H=math.sqrt(3)*P/2
MALE=np.array([.008,P,.030,.007100,.013,.000956,0,0],dtype=np.float64)
FEMALE=np.array([.008,P,.0065,.007268,.013,.000360,1,0],dtype=np.float64)

@pytest.fixture(scope='module')
def sdf():
    lib=ctypes.CDLL(str(build_plugin()))
    f=lib.astra_thread_distance
    f.argtypes=[ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_double)]
    f.restype=ctypes.c_double
    def sample(point,attrs):
        p=np.asarray(point,dtype=np.float64)
        return f(p.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),attrs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
    return sample

def test_crest_and_rounded_root_dimensions(sdf):
    r2=MALE[3]/2
    assert sdf([r2+3*H/8,0,.0125],MALE)==pytest.approx(0,abs=1e-12)
    assert sdf([r2-H/3,0,.0125+P/2],MALE)==pytest.approx(0,abs=1e-12)
    assert sdf([r2+3*H/8+1e-5,0,.0125],MALE)==pytest.approx(1e-5,rel=1e-8)
    assert sdf([r2+3*H/8-1e-5,0,.0125],MALE)==pytest.approx(-1e-5,rel=1e-8)
    assert sdf([r2-H/3+1e-5,0,.0125+P/2],MALE)==pytest.approx(1e-5,rel=1e-8)

def test_flank_distance_uses_closest_point(sdf):
    rho=MALE[3]/2+2e-5
    beta=1/math.sqrt(1+(P/(2*math.pi*rho))**2)
    expected=2e-5/math.sqrt(1+3/beta**2)
    assert sdf([rho,0,.0125+P/4],MALE)==pytest.approx(expected,rel=1e-7)

def test_internal_crest_has_correct_signed_distance(sdf):
    minor=FEMALE[3]/2-H/4
    assert sdf([minor,0,P/2],FEMALE)==pytest.approx(0,abs=1e-12)
    assert sdf([minor-1e-5,0,P/2],FEMALE)==pytest.approx(1e-5,rel=1e-8)
    assert sdf([minor+1e-5,0,P/2],FEMALE)==pytest.approx(-1e-5,rel=1e-8)

def test_matching_threads_do_not_report_false_contact(sdf):
    rng=np.random.default_rng(804)
    for _ in range(2000):
        rho=rng.uniform(0,.005)
        theta=rng.uniform(-math.pi,math.pi)
        localz=rng.uniform(-.0015,.0015)
        xy=[rho*math.cos(theta),rho*math.sin(theta)]
        dm=sdf([*xy,.0125+localz],MALE)
        dn=sdf([*xy,localz],FEMALE)
        assert dm+dn>0, (rho,theta,localz,dm,dn)

def test_right_hand_helical_phase_invariance(sdf):
    rho=.0036;theta=.37;z=.01268;turn=.61
    a=sdf([rho*math.cos(theta),rho*math.sin(theta),z],MALE)
    b=sdf([rho*math.cos(theta+turn),rho*math.sin(theta+turn),z+P*turn/(2*math.pi)],MALE)
    assert a==pytest.approx(b,abs=1e-12)

def test_axis_is_finite_and_nut_bore_is_open(sdf):
    for z in np.linspace(-.0035,.0035,21):
        assert math.isfinite(sdf([0,0,z],FEMALE))
        assert sdf([0,0,z],FEMALE)>0
    assert sdf([0,0,.0125],MALE)<0

def test_entry_chamfer_only_expands_the_bore_at_faces(sdf):
    rho=FEMALE[3]/2+3*H/8+.00015
    assert sdf([rho,0,0],FEMALE)<0
    assert sdf([rho,0,.00324],FEMALE)>0

def test_contact_distance_gradient_is_unit_length(sdf):
    for fraction in [0,.10,.25,.40,.50]:
        phase=fraction*P
        if fraction<=1/16: rho=MALE[3]/2+3*H/8
        elif fraction<3/8: rho=MALE[3]/2+math.sqrt(3)*(P/4-phase)
        else:
            R=H/6;v=P/2-phase
            rho=MALE[3]/2-H/6-math.sqrt(R*R-v*v)
        point=np.array([rho+1e-5,0,.0125+phase])
        eps=1e-8
        grad=[]
        for axis in range(3):
            lo=point.copy();hi=point.copy();lo[axis]-=eps;hi[axis]+=eps
            grad.append((sdf(hi,MALE)-sdf(lo,MALE))/(2*eps))
        assert np.linalg.norm(grad)==pytest.approx(1,abs=1e-4)
