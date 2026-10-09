"""Bounded axial-free robot-arm inertia feedforward for the table-supported controller.

This is an approximate arm plant with fingers at their native configuration;
M_aa includes downstream finger rigid inertia and omits M_af*qdd_f. Native
finger motors, grasp contacts, free workpieces, threads and table stay native.
The five task rows project WORLD physical acceleration onto two current
transverse directions and three world rotation axes. The projected Jdot term
uses a frozen instantaneous basis, not the derivative of moving coordinates.
"""
from __future__ import annotations
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .m8_simulation import YamCartesianController, bounded_vector


class InertiaFeedforwardError(ValueError):
    """Finite/conditioning rejection; callers must close orderly with evidence."""


def _array(value, shape, label):
    try:
        result = np.asarray(value, dtype=float)
    except (ValueError, TypeError, OverflowError) as error:
        raise InertiaFeedforwardError(f'{label} requires finite numeric values') from error
    if result.shape != shape or not np.isfinite(result).all():
        raise InertiaFeedforwardError(f'{label} requires finite shape {shape}')
    return result.copy()


def _positive_scalar(value, label):
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise InertiaFeedforwardError(f'{label} must be finite and positive') from error
    if not np.isfinite(result) or result <= 0:
        raise InertiaFeedforwardError(f'{label} must be finite and positive')
    return result


def orbital_site_acceleration(lever_world_m, angular_velocity_world_rad_s,
                              angular_acceleration_world_rad_s2):
    """Site acceleration with a fixed desired head and scheduled lever rotation."""
    r = _array(lever_world_m, (3,), 'Scheduled world lever')
    w = _array(angular_velocity_world_rad_s, (3,), 'Scheduled world angular velocity')
    a = _array(angular_acceleration_world_rad_s2, (3,), 'Scheduled world angular acceleration')
    result = -np.cross(a, r)-np.cross(w, np.cross(w, r))
    if not np.isfinite(result).all():
        raise InertiaFeedforwardError('Scheduled orbital acceleration overflowed')
    return result


def hybrid5d_feedforward(mass_aa, jp, jr, jpdot, jrdot, retained_qdot,
                        transverse_basis_world, desired_site_acceleration_world,
                        desired_angular_acceleration_world, *,
                        maximum_mass_condition=1e12, maximum_mobility_condition=1e12):
    """Return an uncapped WORLD robot-site wrench with zero axial FF force.

    Acceleration components and Jdot products are instantaneous projections
    of WORLD acceleration; no E_dot coordinate derivative is required or used.
    Axial acceleration, position, thread pitch and groove yaw are not inputs.
    Strict finite/SPD/conditioning checks reject; there is no regularization.
    """
    mass = np.asarray(mass_aa, dtype=float)
    if mass.ndim != 2 or mass.shape[0] != mass.shape[1] or mass.shape[0] < 5:
        raise InertiaFeedforwardError('Arm mass must be square with at least five DOFs')
    n = mass.shape[0]
    mass = _array(mass, (n, n), 'Retained arm mass')
    jp, jr = _array(jp, (3, n), 'Retained position Jacobian'), _array(jr, (3, n), 'Retained rotation Jacobian')
    jpdot, jrdot = (_array(jpdot, (3, n), 'Retained position Jacobian derivative'),
                    _array(jrdot, (3, n), 'Retained rotation Jacobian derivative'))
    qdot = _array(retained_qdot, (n,), 'Coherent retained arm velocity')
    basis = _array(transverse_basis_world, (3, 2), 'World transverse basis')
    if not np.allclose(basis.T@basis, np.eye(2), atol=1e-10, rtol=0.):
        raise InertiaFeedforwardError('World transverse basis must be orthonormal')
    ades = _array(desired_site_acceleration_world, (3,), 'Desired world site acceleration')
    alphades = _array(desired_angular_acceleration_world, (3,), 'Desired world angular acceleration')
    mlimit = _positive_scalar(maximum_mass_condition, 'Mass condition limit')
    glimit = _positive_scalar(maximum_mobility_condition, 'Mobility condition limit')
    if not np.allclose(mass, mass.T, atol=1e-12, rtol=1e-10):
        raise InertiaFeedforwardError('Retained arm mass must be symmetric')
    mass = mass/2+mass.T/2
    eigm = np.linalg.eigvalsh(mass)
    if eigm[0] <= 0 or not np.isfinite(eigm).all():
        raise InertiaFeedforwardError('Retained arm mass must be positive definite')
    mcond = float(eigm[-1]/eigm[0])
    if not np.isfinite(mcond) or mcond > mlimit:
        raise InertiaFeedforwardError('Retained arm mass exceeds numerical condition bound')
    with np.errstate(over="ignore", invalid="ignore"):
        jac5 = np.vstack((basis.T@jp, jr))
        bias5 = np.r_[basis.T@(jpdot@qdot), jrdot@qdot]
        desired5 = np.r_[basis.T@ades, alphades]
    if not all(np.isfinite(x).all() for x in (jac5,bias5,desired5)):
        raise InertiaFeedforwardError("Hybrid task inputs overflowed")
    try:
        with np.errstate(over="ignore",invalid="ignore"):
            mobility5 = jac5@np.linalg.solve(mass, jac5.T)
    except np.linalg.LinAlgError as error:
        raise InertiaFeedforwardError('Arm mass solve failed') from error
    if not np.isfinite(mobility5).all():
        raise InertiaFeedforwardError("Hybrid mobility overflowed")
    mobility5 = mobility5/2+mobility5.T/2
    eigg = np.linalg.eigvalsh(mobility5)
    if eigg[0] <= 0 or not np.isfinite(eigg).all():
        raise InertiaFeedforwardError('Hybrid task must have positive full-rank mobility')
    gcond = float(eigg[-1]/eigg[0])
    if not np.isfinite(gcond) or gcond > glimit:
        raise InertiaFeedforwardError('Hybrid task exceeds numerical condition bound')
    try:
        wrench5 = np.linalg.solve(mobility5, desired5-bias5)
    except np.linalg.LinAlgError as error:
        raise InertiaFeedforwardError('Hybrid task solve failed') from error
    wrench_world = np.r_[basis@wrench5[:2], wrench5[2:]]
    if not np.isfinite(wrench_world).all():
        raise InertiaFeedforwardError('Hybrid wrench overflowed')
    return {'wrench_world_N_Nm': wrench_world, 'mass_aa': mass, 'jacobian5': jac5,
        'projected_world_jdot_qdot5': bias5, 'desired_world_acceleration5': desired5,
        'mobility5': mobility5, 'transverse_basis_world': basis,
        'mass_condition': mcond, 'mobility_condition': gcond,
        'minimum_mobility_eigenvalue': float(eigg[0])}


def combine_capped_robot_command(pd_wrench_world, ff_wrench_world, jp, jr,
                                arm_bias_torque, native_drag_torque, torque_caps,
                                *, maximum_force_N=8., maximum_torque_Nm=2.):
    """Cap total PD+FF Cartesian wrench, then add bias/drag once and motor clip."""
    pd = _array(pd_wrench_world, (6,), 'PD world wrench')
    ff = _array(ff_wrench_world, (6,), 'FF world wrench')
    n = np.asarray(arm_bias_torque).size
    jp, jr = _array(jp, (3, n), 'Arm position Jacobian'), _array(jr, (3, n), 'Arm rotation Jacobian')
    bias, drag = _array(arm_bias_torque, (n,), 'Native arm bias'), _array(native_drag_torque, (n,), 'Native arm drag')
    caps = _array(torque_caps, (n,), 'Native motor caps')
    if np.any(caps <= 0):
        raise InertiaFeedforwardError('Native motor caps must be positive')
    fcap = _positive_scalar(maximum_force_N, 'Cartesian force cap')
    tcap = _positive_scalar(maximum_torque_Nm, 'Cartesian torque cap')
    total = pd+ff
    if not np.isfinite(total).all():
        raise InertiaFeedforwardError('Combined world wrench overflowed')
    capped = np.r_[bounded_vector(total[:3], fcap), bounded_vector(total[3:], tcap)]
    motors_uncapped = bias+drag+jp.T@capped[:3]+jr.T@capped[3:]
    if not np.isfinite(motors_uncapped).all():
        raise InertiaFeedforwardError('Combined robot torque overflowed')
    motors = np.clip(motors_uncapped, -caps, caps)
    return {'pd_wrench_world_N_Nm': pd, 'ff_wrench_world_N_Nm': ff,
        'combined_uncapped_wrench_world_N_Nm': total, 'capped_wrench_world_N_Nm': capped,
        'motor_uncapped_torques_Nm': motors_uncapped, 'motor_torques_Nm': motors,
        'cartesian_force_clipped': bool(np.linalg.norm(total[:3]) > fcap),
        'cartesian_torque_clipped': bool(np.linalg.norm(total[3:]) > tcap),
        'motor_clipped': np.abs(motors_uncapped)>caps}


class HybridInertiaController(YamCartesianController):
    """Only real finite robot/finger commands; no free-workpiece drive or writes.

    M/J/Jdot/cvel/bias retain the previous solve's pre-integration state. A
    separate qvel copy from BEFORE that solve makes damping, Jdot*qdot and drag
    coherent with those caches; data.qvel is never overwritten. The controller
    therefore intentionally uses one retained native timestep of latency.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.full_mass = np.empty((self.model.nv, self.model.nv))
        self.jpdot, self.jrdot = np.zeros_like(self.jp), np.zeros_like(self.jr)
        self.retained_qdot = None
        self.retained_time_s = None
        self.last_inertia_record = None

    def capture_velocity_before_step(self):
        self.retained_qdot = self.data.qvel[self.dof_indices].copy()
        self.retained_time_s = float(self.data.time)

    def command(self, position, orientation, aperture, *, linear_velocity=None,
                angular_velocity=None, axial_float=False, axis_world=(0.,0.,1.),
                axial_feed_N=None, desired_site_acceleration_world=None,
                desired_angular_acceleration_world=None, transverse_basis_world=None,
                inertia_feedforward_enabled=False):
        if self.retained_qdot is None:
            raise InertiaFeedforwardError('Coherent native velocity must be captured at initialization')
        cfg = self.config
        from .m8_scene import jaw_positions
        targets = _array(jaw_positions(aperture,self.scene_config),(2,),"Native jaw targets")
        mujoco.mj_jacSite(self.model,self.data,self.jp,self.jr,self.site_id)
        jp, jr = self.jp[:,self.dof_indices], self.jr[:,self.dof_indices]
        qdot = self.retained_qdot.copy()
        p, r = self.pose()
        axis = _array(axis_world,(3,), 'Axial world direction')
        if np.linalg.norm(axis) <= 0:
            raise InertiaFeedforwardError('Axial direction must be nonzero')
        axis /= np.linalg.norm(axis)
        projection = np.eye(3)-np.outer(axis,axis) if axial_float else np.eye(3)
        target_v = np.zeros(3) if linear_velocity is None else _array(linear_velocity,(3,), 'Target site velocity')
        target_w = np.zeros(3) if angular_velocity is None else _array(angular_velocity,(3,), 'Target site angular velocity')
        self.last_position_error = _array(position,(3,), 'Target site position')-p
        target_r = _array(orientation,(3,3),'Target site rotation')
        if not np.allclose(target_r.T@target_r,np.eye(3),atol=1e-8,rtol=0.) or abs(np.linalg.det(target_r)-1)>1e-8:
            raise InertiaFeedforwardError('Target site rotation must be SO(3)')
        self.last_rotation_error = Rotation.from_matrix(target_r@r.T).as_rotvec()
        force = projection@(cfg.position_stiffness*self.last_position_error+cfg.position_damping*(target_v-jp@qdot))
        if axial_float:
            feed = cfg.axial_feed_N if axial_feed_N is None else float(axial_feed_N)
            if not np.isfinite(feed):
                raise InertiaFeedforwardError('Actual axial feed must be finite')
            force += feed*axis
        kr = cfg.left_rotation_stiffness if self.side=='left' else cfg.rotation_stiffness
        dr = cfg.left_rotation_damping if self.side=='left' else cfg.rotation_damping
        pd_wrench = np.r_[force, kr*self.last_rotation_error+dr*(target_w-jr@qdot)]
        inertia = None
        ff_wrench = np.zeros(6)
        if inertia_feedforward_enabled:
            if not axial_float:
                raise InertiaFeedforwardError('Hybrid inertia feedforward requires actual axial force-float')
            basis = _array(transverse_basis_world,(3,2),'World transverse task basis')
            if np.max(np.abs(basis.T@axis))>1e-10:
                raise InertiaFeedforwardError('Hybrid task basis must exclude the actual axial direction')
            mujoco.mj_fullM(self.model,self.data,self.full_mass)
            mujoco.mj_jacDot(self.model,self.data,self.jpdot,self.jrdot,
                            self.data.site_xpos[self.site_id],int(self.model.site_bodyid[self.site_id]))
            inertia = hybrid5d_feedforward(self.full_mass[np.ix_(self.dof_indices,self.dof_indices)],
                jp,jr,self.jpdot[:,self.dof_indices],self.jrdot[:,self.dof_indices],qdot,
                basis,desired_site_acceleration_world,desired_angular_acceleration_world)
            ff_wrench = inertia['wrench_world_N_Nm']
        bias = self.data.qfrc_bias[self.dof_indices].copy()
        drag = self.model.dof_damping[self.dof_indices]*qdot
        combined = combine_capped_robot_command(pd_wrench,ff_wrench,jp,jr,bias,drag,self.torque_caps,
            maximum_force_N=cfg.maximum_cartesian_force,maximum_torque_Nm=cfg.maximum_cartesian_torque)
        self.last_wrench[:] = combined['capped_wrench_world_N_Nm']
        self.last_motor_torques[:] = combined['motor_torques_Nm']
        self.data.ctrl[self.motor_ids] = self.last_motor_torques
        limits=self.model.actuator_ctrlrange[self.finger_ids]
        self.data.ctrl[self.finger_ids]=np.clip(targets,limits[:,0],limits[:,1])
        self.last_inertia_record = {'enabled':bool(inertia_feedforward_enabled),
            'retained_native_state_time_s':self.retained_time_s,
            'retained_arm_velocity_rad_s':qdot,'arm_jacobian_position':jp.copy(),
            'arm_jacobian_rotation':jr.copy(),'arm_bias_torque_Nm':bias,'native_drag_torque_Nm':drag,
            'arm_jacobian_position_derivative':self.jpdot[:,self.dof_indices].copy() if inertia is not None else np.zeros_like(jp),
            'arm_jacobian_rotation_derivative':self.jrdot[:,self.dof_indices].copy() if inertia is not None else np.zeros_like(jr),
            'requested_site_acceleration_world_m_s2':(np.zeros(3) if not inertia_feedforward_enabled else np.asarray(desired_site_acceleration_world).copy()),
            'requested_angular_acceleration_world_rad_s2':(np.zeros(3) if not inertia_feedforward_enabled else np.asarray(desired_angular_acceleration_world).copy()),
            **combined, 'inertia':inertia}
        return self.last_motor_torques.copy()
