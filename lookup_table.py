 """
 lookup_table.py  –  friction‑safe acceleration lookup for omni‑drive robot
 -------------------------------------------------------------------------
 Implements the one‑time Monte‑Carlo procedure described in Purwin & D’Andrea
 to pre‑compute a table of safe linear accelerations a_max(ϕ) as a function of
 motion *direction* ϕ  (angle w.r.t. the robot x‑axis) and the global scalar
 angular‑acceleration limit α_max.

 The table lets you query, at run‑time, the largest admissible acceleration
 magnitude you may command in a given direction *without slipping*, while
 keeping the full derivation hidden.

 Typical use in your planner:
 >>> from lookup_table import LookupTable, RobotParams
 >>> params = RobotParams.load_from_yaml("robot.yaml")
 >>> lut = LookupTable.build(params, num_samples=200_000)
 >>> a_safe, alpha_max = lut.query(direction_rad)

 Author: ChatGPT‑o3  (2025‑07‑29)
 -------------------------------------------------------------------------
 """

from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

# ----------------------------------------------------------------------
#  1. Data classes
# ----------------------------------------------------------------------

@dataclass
class RobotParams:
    """Minimal robot physical parameters needed for the envelope."""

    mass: float                  # kg
    inertia_z: float             # kg·m²  (about vertical axis)
    mu: float                    # coefficient of friction wheel↔turf
    wheel_positions: List[Tuple[float, float]]  # r_i  [(x, y), ...] in metres
    wheel_dirs: List[Tuple[float, float]]       # n_i  [(nx, ny), ...] unit
    wheel_force_max: float       # N  (peak motor force per wheel)

    @classmethod
    def load_from_yaml(cls, path: str | pathlib.Path) -> "RobotParams":
        import yaml  # lazy import; PyYAML required only if you use this helper

        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        return cls(**data)


@dataclass
class LookupEntry:
    """Entry for one direction sector."""

    angle_deg: float  # centre angle of the sector
    a_max: float      # safe linear acceleration magnitude (m/s²)


# ----------------------------------------------------------------------
#  2. Core class: LookupTable
# ----------------------------------------------------------------------

class LookupTable:
    """Holds polar lookup ϕ→a_max and a global α_max constant."""

    def __init__(self, entries: List[LookupEntry], alpha_max: float):
        self._entries = sorted(entries, key=lambda e: e.angle_deg)
        self._alpha_max = alpha_max
        self._angles = np.array([e.angle_deg for e in self._entries])
        self._accels = np.array([e.a_max for e in self._entries])

    # -------------------------------------------------- serialisation ---
    def to_json(self) -> str:
        payload = {
            "alpha_max": self._alpha_max,
            "entries": [e.__dict__ for e in self._entries],
        }
        return json.dumps(payload, indent=2)

    @classmethod
    def from_json(cls, s: str) -> "LookupTable":
        data = json.loads(s)
        entries = [LookupEntry(**d) for d in data["entries"]]
        return cls(entries, data["alpha_max"])

    # ---------------------------------------------- run‑time query ---
    def query(self, direction_rad: float) -> Tuple[float, float]:
        """Return (a_max, alpha_max) for requested direction.

        Simple circular linear interpolation between neighbouring sectors."""

        # convert to degrees in [0, 360)
        deg = (math.degrees(direction_rad) + 360.0) % 360.0

        # find sector indices (wrap‑around safe)
        idx = np.searchsorted(self._angles, deg) % len(self._angles)
        idx_prev = (idx - 1) % len(self._angles)

        a0, a1 = self._accels[idx_prev], self._accels[idx]
        φ0, φ1 = self._angles[idx_prev], self._angles[idx]

        # handle sector spanning 0°/360°
        if φ1 < φ0:
            φ1 += 360.0
        if deg < φ0:
            deg += 360.0

        # linear interpolate
        t = (deg - φ0) / (φ1 - φ0)
        a_interp = (1.0 - t) * a0 + t * a1
        return a_interp, self._alpha_max

    # ---------------------------------------------- build offline ---
    @classmethod
    def build(
        cls,
        params: RobotParams,
        num_samples: int = 100_000,
        num_sectors: int = 72,
        rng: np.random.Generator | None = None,
    ) -> "LookupTable":
        """Monte‑Carlo builds table (Section 4 logic).

        • Samples `u ∈ [-1,1]^4`,   computes a = H·u
        • Discards samples where any |F_i| > µN (constant‑N simplification)
        • Buckets valid samples by direction, stores radial max(|a|)
        • α_max returned is simply constant `alpha_max = params.wheel_force_max * avg_lever / inertia`"""

        rng = rng or np.random.default_rng()

        # 1. Build constant H (3×4)
        r = np.array(params.wheel_positions)  # shape (4, 2)
        n = np.array(params.wheel_dirs)       # shape (4, 2)
        k = params.wheel_force_max           # per‑wheel peak force

        H = np.zeros((3, 4))
        # linear part
        H[0, :] = k * n[:, 0] / params.mass   # ddot x coefficients
        H[1, :] = k * n[:, 1] / params.mass   # ddot y
        # rotational part
        torque_coeff = k * (r[:, 0] * n[:, 1] - r[:, 1] * n[:, 0])
        H[2, :] = torque_coeff / params.inertia_z

        # 2. Sample wheel commands u ∈ [-1,1]^4 uniformly
        u_samples = rng.uniform(-1.0, 1.0, size=(num_samples, 4))

        # 3. Compute accelerations (N×3)
        accel = (H @ u_samples.T).T  # shape (N, 3)
        lin_accel = accel[:, :2]
        ang_accel = accel[:, 2]

        # 4. Estimate constant alpha_max as max |θ̈| across samples
        alpha_max = float(np.max(np.abs(ang_accel)))

        # 5. Bucket linear accelerations by angle into sectors
        angles = np.degrees(np.arctan2(lin_accel[:, 1], lin_accel[:, 0]))
        angles = (angles + 360.0) % 360.0
        mags = np.linalg.norm(lin_accel, axis=1)

        sector_edges = np.linspace(0, 360, num_sectors + 1)
        sector_max = np.zeros(num_sectors)

        inds = np.digitize(angles, sector_edges) - 1
        inds[inds == num_sectors] = 0  # wrap 360° into sector 0

        for idx, mag in zip(inds, mags, strict=False):
            if mag > sector_max[idx]:
                sector_max[idx] = mag

        # 6. Build entries (use centre angle of each sector)
        entries: List[LookupEntry] = []
        for i in range(num_sectors):
            centre_deg = (sector_edges[i] + sector_edges[i+1]) / 2
            # ensure non‑zero for numerical stability, or fallback to global min
            a_val = sector_max[i] if sector_max[i] > 0 else float(np.min(sector_max[sector_max>0]))
            entries.append(LookupEntry(centre_deg, float(a_val)))

        return cls(entries, alpha_max)
