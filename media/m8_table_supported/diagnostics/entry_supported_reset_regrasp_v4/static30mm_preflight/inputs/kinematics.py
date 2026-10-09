"""Bounded, warm-started Cartesian IK on the actual MuJoCo arm chains."""
from __future__ import annotations

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


HOME = np.array([0.0, 1.047, 1.05, -.9, 0.0, 0.0])


def frame(z, x_hint=(0, 0, 1)):
    z = np.asarray(z, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(x_hint, dtype=float)
    x = x - z * np.dot(x,z)
    if np.linalg.norm(x) < .01:
        x = np.array([1.,0,0]) - z*z[0]
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z,x), z))


def quat_wxyz(matrix):
    return np.roll(Rotation.from_matrix(matrix).as_quat(), 1)


class ArmIK:
    def __init__(self, model, side):
        self.model = model
        self.data = mujoco.MjData(model)
        self.side = side
        self.site = model.site(f"{side}_grasp_site").id
        self.joints = [model.joint(f"{side}_joint{i}").id for i in range(1,7)]
        self.qadr = model.jnt_qposadr[self.joints]
        self.dadr = model.jnt_dofadr[self.joints]
        self.bounds = model.jnt_range[self.joints].T
        self.q = HOME.copy()
        self.max_position_error = 0.
        self.max_orientation_error = 0.

    def pose(self,q=None):
        self.data.qpos[self.qadr] = self.q if q is None else q
        mujoco.mj_kinematics(self.model,self.data)
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3,3).copy()

    def solve(self,position,orientation, *, thorough=False):
        position = np.asarray(position)
        def residual(q):
            p,r = self.pose(q)
            return np.r_[p-position, .12*Rotation.from_matrix(orientation@r.T).as_rotvec()]
        seeds = [self.q]
        if thorough:
            seeds += [HOME, np.array([0,2.,2.,-.5,0,0]), np.array([.8,1.8,1.8,.3,0,0])]
        best = None
        for seed in seeds:
            result = least_squares(residual,np.clip(seed,*self.bounds),bounds=self.bounds,
                                   max_nfev=180 if thorough else 35, ftol=1e-8,xtol=1e-8,gtol=1e-8)
            if best is None or np.linalg.norm(result.fun) < np.linalg.norm(best.fun):
                best = result
            if np.linalg.norm(result.fun) < 1e-5:
                break
        self.q = best.x
        p,r = self.pose()
        pe = float(np.linalg.norm(p-position))
        re = float(np.linalg.norm(Rotation.from_matrix(orientation@r.T).as_rotvec()))
        self.max_position_error = max(self.max_position_error,pe)
        self.max_orientation_error = max(self.max_orientation_error,re)
        if thorough and (pe > .002 or re > .02):
            raise RuntimeError(f"Unreachable {self.side} pose: position error {pe:.4f}m, rotation error {re:.3f}rad")
        return self.q.copy()
