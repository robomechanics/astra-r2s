"""Versioned observation of intermittent, unilateral full-flank contact.

This module observes geometry and actual resolved contact forces. It neither
changes the contact law nor writes a controller command or object state.
"""
from __future__ import annotations

from collections import deque

import numpy as np


class LoadedFlankWindow:
    """Persistent geometry plus loaded normal impulse over a rolling window."""

    version = "loaded_full_flank_window_v1"

    def __init__(self, duration_s=.20, minimum_normal_impulse_Ns=.001,
                 minimum_loaded_duration_s=.0005, maximum_helix_phase_range_m=150e-6):
        values = (duration_s, minimum_normal_impulse_Ns, minimum_loaded_duration_s,
                  maximum_helix_phase_range_m)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Engagement window limits must be finite and positive")
        self.duration_s = float(duration_s)
        self.minimum_normal_impulse_Ns = float(minimum_normal_impulse_Ns)
        self.minimum_loaded_duration_s = float(minimum_loaded_duration_s)
        self.maximum_helix_phase_range_m = float(maximum_helix_phase_range_m)
        self.entries = deque()
        self.minima, self.maxima = deque(), deque()
        self.geometry_start_time = None
        self.impulse = 0.
        self.loaded = 0
        self.loaded_contact_count = 0
        self.loaded_duration = 0.
        self.last_time = None

    def update(self, time_s, dt, geometry_valid, normal_force_N, loaded_contacts,
               helix_phase_m):
        """Observe one actually integrated substep; return transparent metrics."""
        if (not np.isfinite((time_s, dt, normal_force_N, helix_phase_m)).all()
                or dt <= 0 or normal_force_N < 0 or loaded_contacts < 0):
            raise ValueError("Engagement samples must be finite and physically nonnegative")
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError("Engagement sample time must increase")
        self.last_time = float(time_s)
        if not geometry_valid:
            self.entries.clear(); self.minima.clear(); self.maxima.clear()
            self.geometry_start_time = None
            self.impulse = 0.; self.loaded = 0; self.loaded_contact_count = 0; self.loaded_duration = 0.
        else:
            if self.geometry_start_time is None:
                self.geometry_start_time = time_s-dt
            event = (float(time_s), float(normal_force_N*dt), int(loaded_contacts > 0),
                     int(loaded_contacts), float(helix_phase_m), float(dt if loaded_contacts else 0.))
            self.entries.append(event)
            self.impulse += event[1]; self.loaded += event[2]; self.loaded_contact_count += event[3]
            self.loaded_duration += event[5]
            while self.minima and self.minima[-1][1] >= helix_phase_m:
                self.minima.pop()
            while self.maxima and self.maxima[-1][1] <= helix_phase_m:
                self.maxima.pop()
            self.minima.append((time_s, helix_phase_m)); self.maxima.append((time_s, helix_phase_m))
            cutoff = time_s-self.duration_s
            while self.entries and self.entries[0][0] <= cutoff:
                old = self.entries.popleft()
                self.impulse -= old[1]; self.loaded -= old[2]; self.loaded_contact_count -= old[3]
                self.loaded_duration -= old[5]
            while self.minima and self.minima[0][0] <= cutoff:
                self.minima.popleft()
            while self.maxima and self.maxima[0][0] <= cutoff:
                self.maxima.popleft()
        elapsed = (time_s-self.geometry_start_time) if self.geometry_start_time is not None else 0.
        phase_range = float(self.maxima[0][1]-self.minima[0][1]) if self.entries else 0.
        ready = bool(elapsed + 1e-9 >= self.duration_s
                     and self.impulse >= self.minimum_normal_impulse_Ns
                     and self.loaded_duration + 1e-12 >= self.minimum_loaded_duration_s
                     and phase_range <= self.maximum_helix_phase_range_m)
        return {"version": self.version, "ready": ready,
                "continuous_geometry_elapsed_s": float(elapsed),
                "window_duration_s": self.duration_s,
                "normal_impulse_Ns": max(0., float(self.impulse)),
                "loaded_substeps": int(self.loaded),
                "loaded_contact_count": int(self.loaded_contact_count),
                "loaded_duration_s": max(0., float(self.loaded_duration)),
                "sample_count": len(self.entries),
                "loaded_substep_duty": float(self.loaded/len(self.entries)) if self.entries else 0.,
                "helix_phase_range_m": phase_range}
