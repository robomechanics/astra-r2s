"""Pure C2 robot-command braking candidate; no object or physics state access.

Only the preceding scheduled robot clock and its derivatives seed this curve.
The cubic Hermite velocity joins (omega0, alpha0) to (0, 0), and its integral
defines the clock. Command bounds are checked analytically over the whole
curve. These bounds do not establish physical motor tracking or contact safety.
"""
from dataclasses import dataclass, field
import math
from typing import NamedTuple


class BrakeSample(NamedTuple):
    theta_rad: float
    omega_rad_s: float
    alpha_rad_s2: float
    jerk_rad_s3: float


def _finite_scalar(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be a finite scalar') from error
    if not math.isfinite(result):
        raise ValueError(f'{name} must be a finite scalar')
    return result


@dataclass(frozen=True)
class C2ReverseBrake:
    theta0_rad: float
    omega0_rad_s: float
    alpha0_rad_s2: float
    duration_s: float
    minimum_theta_rad: float = -2.7
    maximum_theta_rad: float = 0.
    maximum_speed_rad_s: float = 2.
    maximum_acceleration_rad_s2: float | None = None
    displacement_rad: float = field(init=False)
    final_theta_rad: float = field(init=False)
    peak_speed_rad_s: float = field(init=False)
    peak_acceleration_rad_s2: float = field(init=False)
    peak_jerk_rad_s3: float = field(init=False)

    def __post_init__(self):
        names = ('theta0_rad', 'omega0_rad_s', 'alpha0_rad_s2', 'duration_s',
                 'minimum_theta_rad', 'maximum_theta_rad', 'maximum_speed_rad_s')
        for name in names:
            object.__setattr__(self, name, _finite_scalar(getattr(self, name), name))
        if self.maximum_acceleration_rad_s2 is not None:
            object.__setattr__(self, 'maximum_acceleration_rad_s2',
                _finite_scalar(self.maximum_acceleration_rad_s2, 'maximum_acceleration_rad_s2'))
        if (self.duration_s <= 0 or self.maximum_speed_rad_s <= 0
                or self.minimum_theta_rad >= self.maximum_theta_rad
                or (self.maximum_acceleration_rad_s2 is not None
                    and self.maximum_acceleration_rad_s2 <= 0)):
            raise ValueError('Brake duration/caps must be positive and workspace ordered')
        scaled_alpha = self.duration_s*self.alpha0_rad_s2
        inverse_duration = 1./self.duration_s
        inverse_duration_squared = inverse_duration*inverse_duration
        displacement = self.duration_s*(self.omega0_rad_s/2.+scaled_alpha/12.)
        final = self.theta0_rad+displacement
        if not all(math.isfinite(value) for value in
                   (scaled_alpha, inverse_duration_squared, displacement, final)):
            raise ValueError('Derived brake values must remain finite')
        # v(s)=(1-s)^2*[omega0*(1+2*s)+T*alpha0*s]. The bracket is
        # linear, so these two endpoint checks prove no sign reversal.
        if self.omega0_rad_s > 0 or 3.*self.omega0_rad_s+scaled_alpha > 0:
            raise ValueError('Brake must retain a monotonic reverse clock without sign reversal')
        if not self.minimum_theta_rad <= final <= self.theta0_rad <= self.maximum_theta_rad:
            raise ValueError('Complete brake would exceed the declared clock workspace')
        object.__setattr__(self, 'displacement_rad', displacement)
        object.__setattr__(self, 'final_theta_rad', final)

        speed_locations = [0., 1.]
        denominator = 3.*scaled_alpha+6.*self.omega0_rad_s
        if denominator != 0:
            stationary_speed = scaled_alpha/denominator
            if 0 < stationary_speed < 1:
                speed_locations.append(stationary_speed)
        acceleration_locations = [0., 1.]
        denominator = 12.*self.omega0_rad_s+6.*scaled_alpha
        if denominator != 0:
            stationary_acceleration = (6.*self.omega0_rad_s+4.*scaled_alpha)/denominator
            if 0 < stationary_acceleration < 1:
                acceleration_locations.append(stationary_acceleration)
        peak_speed = max(abs(self._interior(s).omega_rad_s) for s in speed_locations)
        peak_acceleration = max(abs(self._interior(s).alpha_rad_s2)
                                for s in acceleration_locations)
        peak_jerk = max(abs(self._interior(s).jerk_rad_s3) for s in (0., 1.))
        if not all(math.isfinite(value) for value in (peak_speed, peak_acceleration, peak_jerk)):
            raise ValueError('Derived brake derivatives must remain finite')
        if peak_speed > self.maximum_speed_rad_s:
            raise ValueError('Complete brake would exceed the angular-speed cap')
        if (self.maximum_acceleration_rad_s2 is not None
                and peak_acceleration > self.maximum_acceleration_rad_s2):
            raise ValueError('Complete brake would exceed the angular-acceleration cap')
        object.__setattr__(self, 'peak_speed_rad_s', peak_speed)
        object.__setattr__(self, 'peak_acceleration_rad_s2', peak_acceleration)
        object.__setattr__(self, 'peak_jerk_rad_s3', peak_jerk)

    def _interior(self, s):
        scaled_alpha = self.duration_s*self.alpha0_rad_s2
        omega = (1.-s)**2*(self.omega0_rad_s*(1.+2.*s)+scaled_alpha*s)
        alpha = (self.omega0_rad_s*(6.*s*s-6.*s)
                 +scaled_alpha*(3.*s*s-4.*s+1.))/self.duration_s
        jerk = ((self.omega0_rad_s*(12.*s-6.)
                 +scaled_alpha*(6.*s-4.))/self.duration_s)/self.duration_s
        theta = self.theta0_rad+self.duration_s*(
            self.omega0_rad_s*(.5*s**4-s**3+s)
            +scaled_alpha*(.25*s**4-2.*s**3/3.+.5*s*s))
        return BrakeSample(theta, omega, alpha, jerk)

    def sample(self, elapsed_s):
        """Clamp finite query time; at/after T return the stationary endpoint.

        Negative queries select the start jet, whose velocity/acceleration
        belong to the preceding schedule. The endpoint is C2, with no C3 claim.
        """
        elapsed = _finite_scalar(elapsed_s, 'elapsed_s')
        if elapsed >= self.duration_s:
            return BrakeSample(self.final_theta_rad, 0., 0., 0.)
        return self._interior(max(elapsed, 0.)/self.duration_s)
