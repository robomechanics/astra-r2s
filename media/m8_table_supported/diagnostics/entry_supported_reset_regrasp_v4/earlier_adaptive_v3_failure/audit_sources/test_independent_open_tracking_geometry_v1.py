"""Pure support-plane geometry tests; never load or integrate a native model."""
import ast
from pathlib import Path
import unittest

import numpy as np

SOURCE=Path(__file__).with_name('independent_open_tracking_geometry_v1.py')
tree=ast.parse(SOURCE.read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='inner_face_gap')
namespace={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),namespace)
gap=namespace['inner_face_gap']


class InnerPlaneTests(unittest.TestCase):
    def test_centered_full_hex_clearance_can_be_lost_by_native_translation(self):
        radius=.02/np.sqrt(3)
        angles=np.arange(6)*np.pi/3+np.pi/2
        vertices=np.c_[radius*np.cos(angles),radius*np.sin(angles),np.zeros(6)]
        center=np.array([0.,.0135,0.]);half=np.array([.0045,.0015,.00275])
        original,axis=gap(vertices,center,np.eye(3),half,np.zeros(3))
        shifted,_=gap(vertices+np.array([0.,.001,0.]),center,np.eye(3),half,np.array([0.,.001,0.]))
        self.assertAlmostEqual(original,(.024-2*radius)/2)
        self.assertGreater(original,0.);self.assertLess(shifted,0.)
        self.assertAlmostEqual(original-shifted,.001)
        np.testing.assert_array_equal(axis,[0.,-1.,0.])

    def test_projection_is_rotationally_invariant(self):
        vertices=np.array([[-.001,-.004,0.],[.001,.004,0.]])
        center=np.array([0.,.01,0.]);half=np.array([.002,.001,.002])
        R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        a,_=gap(vertices,center,np.eye(3),half,np.zeros(3))
        b,_=gap(vertices@R.T,R@center,R,half,np.zeros(3))
        self.assertAlmostEqual(a,b)


if __name__=='__main__':unittest.main(verbosity=2)
