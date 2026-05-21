from __future__ import annotations

import numpy as np


class WindModel:
    """Provides a simple lateral wind disturbance with optional piecewise-constant gusts."""

    def __init__(
        self,
        base_force_n: float,
        gust_std_n: float,
        gust_interval_s: float,
        enable_random_gusts: bool,
        rng: np.random.Generator,
    ) -> None:
        self.base_force_n = base_force_n
        self.gust_std_n = gust_std_n
        self.gust_interval_s = max(gust_interval_s, 1e-6)
        self.enable_random_gusts = enable_random_gusts
        self.rng = rng
        self.current_bucket_index: int | None = None
        self.current_gust_n = 0.0

    def force_at(self, time_s: float) -> float:
        """Return the lateral wind disturbance at the requested time."""

        if not self.enable_random_gusts or self.gust_std_n <= 0.0:
            return self.base_force_n

        bucket_index = int(time_s / self.gust_interval_s)
        if bucket_index != self.current_bucket_index:
            self.current_bucket_index = bucket_index
            self.current_gust_n = float(self.rng.normal(0.0, self.gust_std_n))

        return self.base_force_n + self.current_gust_n
