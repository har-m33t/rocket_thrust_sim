from __future__ import annotations

import numpy as np


class ThrustCurve:
    """Interpolates a small thrust table into a smooth thrust command."""

    def __init__(self, thrust_scale: float = 1.0) -> None:
        self.thrust_scale = thrust_scale
        self.times_s = np.array([0.0, 0.5, 2.0, 8.0, 8.5], dtype=float)
        self.thrust_values_n = np.array([0.0, 800.0, 1200.0, 1200.0, 0.0], dtype=float)

    def thrust_at(self, time_s: float) -> float:
        """Return thrust at the current time.

        In a real program this data would usually come from static fire tests or
        manufacturer-provided thrust curves. Here it is intentionally hardcoded
        so the simulation architecture stays easy to explain in an interview.
        """

        return float(
            self.thrust_scale
            * np.interp(
                time_s,
                self.times_s,
                self.thrust_values_n,
                left=0.0,
                right=0.0,
            )
        )
