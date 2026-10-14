"""Part B: physics check. TODO 3."""
from __future__ import annotations

import math

G = 9.81


def is_feasible(p: dict) -> bool:
    """TODO 3. False for scenarios no real road user could produce. Reject if ANY of:

    - the lane change needs more sideways grip than the road has: the cosine lane change of 3.5 m over
      p["cutDur"] seconds peaks at a_lat = (pi / cutDur)^2 * 3.5 / 2; reject if a_lat > mu * g;
    - the post-merge braking p["cutBrake"] exceeds mu * g;
    - the pedestrian runs: p["pedV"] > 2.5 m/s;
    - the cut-in speed p["cutV"] is not in (0, 22] m/s.
    mu is p["mu"]. A search that keeps infeasible scenarios reports failures that cannot happen.
    """
    mu = float(p.get("mu", 0.8))
    mu_g = mu * G

    cut_dur = float(p.get("cutDur", 2.4))
    if cut_dur <= 0:
        return False
    a_lat = (math.pi / cut_dur) ** 2 * 3.5 / 2.0
    if a_lat > mu_g + 1e-9:
        return False

    cut_brake = float(p.get("cutBrake", 0.0))
    if cut_brake > mu_g + 1e-9:
        return False

    ped_v = float(p.get("pedV", 1.45))
    if ped_v > 2.5 + 1e-9:
        return False

    cut_v = float(p.get("cutV", 15.0))
    if not (0.0 < cut_v <= 22.0):
        return False

    return True
