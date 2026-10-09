"""Experimental CLOSED search observer; never engagement or release proof.

The inputs are original native bolt/female axial pose, velocity and solved
thread normal load.  Tool pose, thread pitch, groove yaw and commanded motion
are deliberately absent.  This file is outside the frozen producer sources.
"""
import numpy as np

from yam_twin.m8_insertion_simulation import EntrySupportWindow


class CrestSeatDropWindowV3:
    version = "native-measured-crest-return-direction-impulse-event-v3"

    def __init__(self, reference_base_z_m, net_feed_N, *, minimum_drop_m=50e-6,
                 required_stop_s=.1, velocity_limit_m_per_s=.0002,
                 angular_velocity_limit_rad_per_s=.01):
        values = (reference_base_z_m, net_feed_N, minimum_drop_m,
                  required_stop_s, velocity_limit_m_per_s,
                  angular_velocity_limit_rad_per_s)
        if not np.isfinite(values).all() or min(values[1:]) <= 0:
            raise ValueError("Crest-return thresholds must be finite and positive")
        self.reference = float(reference_base_z_m)
        self.net_feed = float(net_feed_N)
        self.minimum_drop = float(minimum_drop_m)
        self.duration = float(required_stop_s)
        self.velocity_limit = float(velocity_limit_m_per_s)
        self.angular_limit = float(angular_velocity_limit_rad_per_s)
        self.epoch = 0
        self._clear_epoch()

    def _clear_epoch(self):
        self.crest = None
        self.crest_time = None
        self.first_drop_time = None
        self.stop_requested = False
        self.drop = 0.
        self.ready = False
        self.last_time = None
        self.entry = EntrySupportWindow(self.duration, self.velocity_limit,
                                        self.net_feed)

    def observe(self, time_s, dt, measured_base_z_m,
                actual_axial_velocity_m_per_s, original_thread_normal_force_N,
                *, actual_relative_angular_speed_rad_per_s, valid,
                physically_stopped):
        values = (time_s, dt, measured_base_z_m,
                  actual_axial_velocity_m_per_s, original_thread_normal_force_N,
                  actual_relative_angular_speed_rad_per_s)
        if (not np.isfinite(values).all() or dt <= 0
                or original_thread_normal_force_N < 0
                or actual_relative_angular_speed_rad_per_s < 0):
            self.epoch += 1
            self._clear_epoch()
            raise ValueError("Crest observations require finite native physical values")
        if (self.last_time is not None and (time_s <= self.last_time
                or abs(time_s-self.last_time-dt) > max(1e-9, dt*1e-6))):
            self.epoch += 1
            self._clear_epoch()
            raise ValueError("Crest observations require every contiguous native timestep")
        self.last_time = float(time_s)
        if not valid:
            self.epoch += 1
            self._clear_epoch()
            # Keep the invalid observation's native clock, but discard all
            # geometry and load history so it cannot seed a later return.
            self.last_time = float(time_s)
            return self.report()
        if self.crest is None:
            self.crest = float(measured_base_z_m)
            self.crest_time = float(time_s)
        elif not self.stop_requested and measured_base_z_m < self.crest:
            self.crest = float(measured_base_z_m)
            self.crest_time = float(time_s)
        self.drop = float(measured_base_z_m-self.crest)
        if self.drop >= self.minimum_drop and not self.stop_requested:
            # This can occur while the bolt is moving and the hand bears its
            # weight.  It requests CLOSED deceleration only; no readiness yet.
            self.stop_requested = True
            self.first_drop_time = float(time_s)
        stable = (physically_stopped and self.stop_requested
                  and self.drop >= self.minimum_drop
                  and actual_relative_angular_speed_rad_per_s <= self.angular_limit)
        self.ready = bool(self.entry.observe(original_thread_normal_force_N,
            actual_axial_velocity_m_per_s, dt, stable))
        return self.report()

    def report(self):
        return {
            "observer": self.version,
            "original_settled_reference_base_z_m": self.reference,
            "actual_crest_base_z_m": self.crest,
            "actual_crest_time_s": self.crest_time,
            "actual_crest_return_m": self.drop,
            "minimum_actual_return_m": self.minimum_drop,
            "first_actual_return_time_s": self.first_drop_time,
            "stop_requested_from_measured_return": bool(self.stop_requested),
            "confirmed_search_direction_event": bool(self.ready),
            "original_native_impulse_window": self.entry.report(),
            "invalid_or_discontinuous_epoch": self.epoch,
            "is_engagement_or_release_proof": False,
            "scope": "Measured return from a valid contiguous actual bolt/female shallow crest requests CLOSED deceleration only. Confirmation requires persistent return and original native aligned/slow/angular-stopped impulse, duty and current loaded endpoint over 100 ms. Separate original 100 ms native 90/10 weight-transfer proof is also required by the diagnostic before any forward scan. No pitch, groove phase, tool trajectory or engagement/opening qualification enters this observer.",
        }
