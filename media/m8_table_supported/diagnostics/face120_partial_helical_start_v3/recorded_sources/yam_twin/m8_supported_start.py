"""Native observations for physical M8 weight transfer and seat-direction search.

Call immediately after mj_step and before any forward/kinematics refresh.
Returned contact forces, transforms and velocities belong to its original
pre-integration solve. A cold checkpoint diagnostic is not a full trajectory.
"""
from collections import deque
from pathlib import Path
import hashlib

import mujoco
import numpy as np

from .m8_insertion_mechanics import contact_is_on_full_flanks

OBSERVER_SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def contact_world_wrench(frame, local, position, origin, sign):
    frame, local = np.asarray(frame).reshape(3, 3), np.asarray(local)
    force = sign*(frame.T@local[:3])
    torque = sign*(frame.T@local[3:])+np.cross(np.asarray(position)-origin, force)
    return np.r_[force, torque]


def resolved_thread_wrench_on_bolt(model, data, thread, *, with_records=False):
    """Full signed native thread wrench on male; normals are separate diagnostics."""
    bolt, female = model.body('male_bolt').id, model.body('female_frame').id
    male_geom, female_geom = model.geom('bolt_thread').id, model.geom('female_thread').id
    br, fr = data.xmat[bolt].reshape(3, 3), data.xmat[female].reshape(3, 3)
    gravity = np.asarray(model.opt.gravity)
    if np.linalg.norm(gravity) <= 0:
        raise ValueError('Bolt-weight observation requires nonzero declared gravity')
    up = -gravity/np.linalg.norm(gravity)
    total, interior = np.zeros(6), np.zeros(6)
    normals = interior_normals = 0.
    count = interior_count = loaded_interior_count = 0
    local = np.zeros(6)
    records = []
    for index, c in enumerate(data.contact):
        pair = {int(c.geom1), int(c.geom2)}
        if pair != {male_geom, female_geom}:
            continue
        sign = -1. if int(c.geom1) == male_geom else 1.
        mujoco.mj_contactForce(model, data, index, local)
        frame = c.frame.reshape(3, 3)
        wrench = contact_world_wrench(frame, local, c.pos, data.xpos[bolt], sign)
        on_interior = contact_is_on_full_flanks(fr.T@(c.pos-data.xpos[female]),
                                               br.T@(c.pos-data.xpos[bolt]), thread)
        total += wrench
        normals += float(local[0])
        count += 1
        if on_interior:
            interior += wrench
            interior_normals += float(local[0])
            interior_count += 1
            loaded_interior_count += float(local[0]) > 1e-5
        if with_records:
            records.append({'geom1': model.geom(int(c.geom1)).name,
                'geom2': model.geom(int(c.geom2)).name, 'frame': frame.tolist(),
                'local_force_N_Nm': local.tolist(), 'contact_position_world_m': c.pos.tolist(),
                'bolt_origin_world_m': data.xpos[bolt].tolist(),
                'is_actual_interior_flank_contact': bool(on_interior),
                'wrench_on_bolt_world_N_Nm': wrench.tolist()})
    return {'thread_wrench_on_bolt_world_N_Nm': total.tolist(),
        'thread_gravity_opposing_force_N': float(np.dot(total[:3], up)),
        'thread_hole_axial_reaction_force_N': float(-np.dot(total[:3], fr[:, 2])),
        'thread_summed_normal_force_N': normals,
        'native_thread_contact_count': count,
        'actual_interior_flank_contact_count': interior_count,
        'loaded_actual_interior_flank_contact_count': loaded_interior_count,
        'interior_flank_summed_normal_force_N': interior_normals,
        'interior_flank_gravity_opposing_force_N': float(np.dot(interior[:3], up)),
        'noninterior_entry_contact_count': count-interior_count,
        'native_contact_records': records}


def native_weight_transfer_sample(model, data, right_controller, thread, *, with_records=False):
    """Observe actual original native load and relative kinematics, with no commands."""
    thread_load = resolved_thread_wrench_on_bolt(model, data, thread, with_records=with_records)
    bolt, hole, block = (model.body(n).id for n in ('male_bolt', 'female_frame', 'fixture_block'))
    br, hr = data.xmat[bolt].reshape(3, 3), data.xmat[hole].reshape(3, 3)
    relative, rr = hr.T@(data.xpos[bolt]-data.xpos[hole]), hr.T@br
    bv, hv, sv = np.zeros(6), np.zeros(6), np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, bolt, bv, 0)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, hole, hv, 0)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE,
                            right_controller.site_id, sv, 0)
    relative_velocity = bv[3:]-hv[3:]-np.cross(hv[:3], data.xpos[bolt]-data.xpos[hole])
    up = -np.asarray(model.opt.gravity)/np.linalg.norm(model.opt.gravity)
    hand = right_controller.contact_wrench_on('male_bolt')
    world_support = nonthread_seating = 0
    expected_thread = {model.geom('bolt_thread').id, model.geom('female_thread').id}
    for c in data.contact:
        pair = {int(c.geom1), int(c.geom2)}
        roots = {int(model.body_weldid[int(model.geom_bodyid[g])]) for g in pair}
        if bolt in roots and 0 in roots:
            world_support += 1
        if bolt in roots and block in roots and pair != expected_thread:
            nonthread_seating += 1
    unforced = bool(np.all(data.xfrc_applied[[bolt, hole, block]] == 0))
    for name in ('male_bolt_free', 'fixture_block_free'):
        start = int(model.jnt_dofadr[model.joint(name).id])
        unforced &= bool(np.all(data.qfrc_applied[start:start+6] == 0))
    return {**thread_load, 'hand_gravity_opposing_force_N': float(np.dot(hand['wrench_world'][:3], up)),
        'hand_hole_axial_reaction_force_N': float(-np.dot(hand['wrench_world'][:3], hr[:, 2])),
        'right_pad_gravity_opposing_force_N': float(np.dot(hand['pad_wrench_world'][:3], up)),
        'right_pad_wrench_on_bolt_world_N_Nm': hand['pad_wrench_world'],
        'hand_wrench_on_bolt_world_N_Nm': hand['wrench_world'],
        'right_pad_normal_force_N': hand['pad_normal_force_N'],
        'bolt_weight_N': float(model.body_subtreemass[bolt]*np.linalg.norm(model.opt.gravity)),
        'relative_bolt_axial_velocity_m_per_s': float(np.dot(relative_velocity, hr[:, 2])),
        'relative_bolt_angular_speed_rad_per_s': float(np.linalg.norm(bv[:3]-hv[:3])),
        'relative_hand_angular_speed_rad_per_s': float(np.linalg.norm(sv[:3]-hv[:3])),
        'radial_offset_m': float(np.linalg.norm(relative[:2])),
        'bolt_tilt_rad': float(np.arccos(np.clip(rr[2, 2], -1., 1.))),
        'bolt_world_support_contact_count': world_support,
        'nonthread_block_bolt_contact_count': nonthread_seating,
        'external_drive_zero': unforced,
        'source_sha256': OBSERVER_SOURCE_SHA256,
        'scope': 'Original pre-integration native contact solve and matching retained derived kinematics; never reconstructed from post-step saved qpos'}


class BoltWeightTransferWindow:
    """Aligned slow motion and measured gravity support over100ms; not capture."""
    version = 'diagnostic-native-bolt-weight-transfer-v1'

    def __init__(self, bolt_weight_N, *, duration_s=.100,
                 minimum_thread_weight_fraction=.90, maximum_hand_weight_fraction=.10,
                 velocity_limit_m_per_s=.0002, radial_limit_m=.000150, tilt_limit_rad=np.deg2rad(2)):
        values = (bolt_weight_N, duration_s, minimum_thread_weight_fraction,
                  maximum_hand_weight_fraction, velocity_limit_m_per_s, radial_limit_m, tilt_limit_rad)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError('Weight-transfer thresholds must be positive and finite')
        self.weight, self.duration = float(bolt_weight_N), float(duration_s)
        self.minimum_thread = float(minimum_thread_weight_fraction)
        self.maximum_hand = float(maximum_hand_weight_fraction)
        self.velocity_limit, self.radial_limit, self.tilt_limit = map(float,
            (velocity_limit_m_per_s, radial_limit_m, tilt_limit_rad))
        self.samples = deque()
        self.last_time, self.geometry_start = None, None
        self.ready = False
        self.elapsed = self.thread_impulse = self.hand_positive_impulse = 0.
        self.interior_duration = self.loaded_thread_duration = 0.
        self.contact_samples = 0

    def _accumulate(self, row, sign):
        _, dt, sample = row
        self.elapsed += sign*dt
        self.thread_impulse += sign*dt*sample['thread_gravity_opposing_force_N']
        self.hand_positive_impulse += sign*dt*max(sample['hand_gravity_opposing_force_N'], 0.)
        self.interior_duration += sign*dt*(sample['loaded_actual_interior_flank_contact_count'] > 0)
        self.loaded_thread_duration += sign*dt*(sample['thread_gravity_opposing_force_N'] > .1*self.weight)
        self.contact_samples += sign*(sample['native_thread_contact_count'] > 0)

    def _clear(self):
        self.samples.clear()
        self.geometry_start = None
        self.ready = False
        self.elapsed = self.thread_impulse = self.hand_positive_impulse = 0.
        self.interior_duration = self.loaded_thread_duration = 0.
        self.contact_samples = 0

    def observe(self, time_s, dt, sample, *, valid=True):
        values = (time_s, dt, sample['thread_gravity_opposing_force_N'],
            sample['hand_gravity_opposing_force_N'], sample['relative_bolt_axial_velocity_m_per_s'],
            sample['radial_offset_m'], sample['bolt_tilt_rad'])
        if not np.isfinite(values).all() or dt <= 0:
            raise ValueError('Native weight-transfer samples must be finite and timestep positive')
        counts = [sample[key] for key in ('native_thread_contact_count',
            'loaded_actual_interior_flank_contact_count', 'bolt_world_support_contact_count',
            'nonthread_block_bolt_contact_count')]
        if any(not np.isfinite(value) or value < 0 or int(value) != value for value in counts):
            raise ValueError('Native contact counts must be nonnegative integers')
        if sample['radial_offset_m'] < 0 or sample['bolt_tilt_rad'] < 0:
            raise ValueError('Radial offset and tilt must be nonnegative')
        if self.last_time is not None and (time_s <= self.last_time or
                not np.isclose(time_s-self.last_time, dt, rtol=1e-7, atol=1e-10)):
            raise ValueError('Observe every original native step with increasing, contiguous time')
        self.last_time = float(time_s)
        good = (valid and sample['external_drive_zero'] and not sample['bolt_world_support_contact_count']
            and not sample['nonthread_block_bolt_contact_count']
            and abs(sample['relative_bolt_axial_velocity_m_per_s']) <= self.velocity_limit
            and sample['radial_offset_m'] <= self.radial_limit and sample['bolt_tilt_rad'] <= self.tilt_limit)
        if not good:
            self._clear()
            return self.report()
        if self.geometry_start is None:
            self.geometry_start = time_s-dt
        row = (float(time_s), float(dt), dict(sample))
        self.samples.append(row)
        self._accumulate(row, 1)
        cutoff = time_s-self.duration
        while self.samples and self.samples[0][0] <= cutoff:
            self._accumulate(self.samples.popleft(), -1)
        report = self.report()
        self.ready = bool(time_s-self.geometry_start+1e-9 >= self.duration
            and report['observed_window_s']+1e-9 >= self.duration
            and report['mean_thread_weight_fraction'] >= self.minimum_thread
            and report['mean_positive_hand_upward_weight_fraction'] <= self.maximum_hand
            and sample['thread_gravity_opposing_force_N'] > .1*self.weight)
        return self.report()

    def report(self):
        duration = max(self.elapsed, 0.)
        thread, hand = self.thread_impulse, max(self.hand_positive_impulse, 0.)
        interior_duration = max(self.interior_duration, 0.)
        return {'observer': self.version, 'ready_for_diagnostic_release_attempt': self.ready,
            'bolt_weight_N': self.weight, 'required_window_s': self.duration,
            'observed_window_s': float(duration), 'original_native_samples': len(self.samples),
            'mean_thread_weight_fraction': thread/(duration*self.weight) if duration else None,
            'mean_positive_hand_upward_weight_fraction': hand/(duration*self.weight) if duration else None,
            'signed_thread_gravity_opposing_impulse_Ns': float(thread),
            'positive_hand_upward_impulse_Ns': float(hand),
            'loaded_actual_interior_flank_duration_s': float(interior_duration),
            'loaded_thread_gravity_support_duty_fraction': max(self.loaded_thread_duration, 0.)/duration if duration else None,
            'entry_only_native_contact_window': bool(self.contact_samples and interior_duration < 1e-12),
            'loaded_actual_interior_flank_contact_observed': bool(interior_duration > 1e-12),
            'scope': 'Measured bolt-weight transfer while aligned and slow. May still be cone-only; never a full-flank capture, passive reset or successful thread proof.'}


class SeatDropWindow:
    """Observe a real axial drop and stable stop; never declare engagement.

    No thread pitch or yaw enters this event. It selects a physical search
    direction and requires separate formed-contact/load/lead/reset evidence.
    """
    version = 'native-measured-seat-direction-event-v1'

    def __init__(self, reference_base_z_m, *, minimum_drop_m=50e-6,
                 required_stable_stop_s=.05, velocity_limit_m_per_s=.0002,
                 angular_velocity_limit_rad_per_s=.01,
                 minimum_loaded_contact_normal_force_N=1e-5):
        if not np.isfinite(reference_base_z_m):
            raise ValueError('Measured seat reference must be finite')
        thresholds = (minimum_drop_m, required_stable_stop_s,
                      velocity_limit_m_per_s, angular_velocity_limit_rad_per_s,
                      minimum_loaded_contact_normal_force_N)
        if not np.isfinite(thresholds).all() or min(thresholds) <= 0:
            raise ValueError('Measured seat event thresholds must be finite and positive')
        self.reference = float(reference_base_z_m)
        self.minimum_drop = float(minimum_drop_m)
        self.required_stop = float(required_stable_stop_s)
        self.velocity_limit = float(velocity_limit_m_per_s)
        self.angular_velocity_limit = float(angular_velocity_limit_rad_per_s)
        self.minimum_contact = float(minimum_loaded_contact_normal_force_N)
        self.stop_requested = False
        self.first_drop_time = None
        self.last_time = None
        self.stable_duration = 0.
        self.drop = 0.
        self.ready = False

    def observe(self, time_s, dt, measured_base_z_m, actual_axial_velocity_m_per_s,
                original_thread_normal_force_N, *, actual_relative_angular_speed_rad_per_s,
                valid, physically_stopped):
        values = (time_s, dt, measured_base_z_m, actual_axial_velocity_m_per_s,
                  original_thread_normal_force_N, actual_relative_angular_speed_rad_per_s)
        if not np.isfinite(values).all() or dt <= 0:
            raise ValueError('Seat observations must be finite and timestep positive')
        if self.last_time is not None and (time_s <= self.last_time or
                abs((time_s-self.last_time)-dt) > max(1e-9, dt*1e-6)):
            self.stable_duration = 0.
            self.ready = False
            raise ValueError('Seat observations require every contiguous native timestep')
        self.last_time = float(time_s)
        self.drop = float(measured_base_z_m-self.reference)
        if valid and self.drop >= self.minimum_drop and not self.stop_requested:
            self.stop_requested = True
            self.first_drop_time = float(time_s)
        stable = (valid and physically_stopped and self.stop_requested
            and self.drop >= self.minimum_drop
            and abs(actual_axial_velocity_m_per_s) <= self.velocity_limit
            and abs(actual_relative_angular_speed_rad_per_s) <= self.angular_velocity_limit
            and original_thread_normal_force_N > self.minimum_contact)
        self.stable_duration = self.stable_duration+dt if stable else 0.
        self.ready = bool(self.stable_duration+1e-12 >= self.required_stop)
        return self.report()

    def report(self):
        return {'observer': self.version, 'stop_requested_from_measured_drop': self.stop_requested,
            'first_actual_drop_time_s': self.first_drop_time,
            'actual_axial_drop_m': self.drop, 'minimum_actual_drop_m': self.minimum_drop,
            'stable_stop_duration_s': self.stable_duration,
            'required_stable_stop_duration_s': self.required_stop,
            'maximum_stopped_angular_speed_rad_per_s': self.angular_velocity_limit,
            'confirmed_search_direction_event': self.ready,
            'is_engagement_or_release_proof': False,
            'scope': 'Real native axial drop followed by continuously aligned, slow, contact-loaded physical stop. A direction-search event only; cone or arm motion may cause it.'}
