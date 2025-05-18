"""
trajectories.py

Module providing predefined planar trajectories for the 3‑RRR simulator.
All points are expressed in metres in the robot workspace, centred at (0, 0) unless stated otherwise.

Available generators
--------------------
- circle(R=0.05, N=200)
- square(side=0.08, N_per_edge=50)
- line(dx=0.1, dy=0, N=100)
- heart(scale=0.05, N=400)
- eight(R=0.05, N=400, stacked=True)

Utility helpers
---------------
- names() → list of available trajectory names.
- get(name, **kwargs) → list[(x, y)] for the chosen trajectory.

Example
-------
>>> from trajectories import get
>>> path = get("eight", R=0.04, stacked=False)
>>> for x, y in path:
...     robot.set_pose((x, y, 0))
...     # refresh Pygame, etc.
"""

import math
import numpy as np
from typing import Callable, Dict, List, Tuple

Point = Tuple[float, float]

# ---------------------------------------------------------------------------
# Individual trajectory generators
# ---------------------------------------------------------------------------

def circle(R: float = 0.05, N: int = 200) -> List[Point]:
    """Full circle of radius *R* (metres)."""
    t = np.linspace(0, 2 * np.pi, N, endpoint=False)
    return [(R * math.cos(a), R * math.sin(a)) for a in t]


def square(side: float = 0.08, N_per_edge: int = 50) -> List[Point]:
    """Axis‑aligned square whose side length is *side* (metres)."""
    s = side / 2
    pts: List[Point] = []
    lin = np.linspace(-s, s, N_per_edge, endpoint=False)
    # bottom edge
    pts += [(x, -s) for x in lin]
    # right edge
    pts += [(s, y) for y in lin]
    # top edge (reverse)
    pts += [(x, s) for x in lin[::-1]]
    # left edge (reverse)
    pts += [(-s, y) for y in lin[::-1]]
    return pts


def line(dx: float = 0.1, dy: float = 0.0, N: int = 100) -> List[Point]:
    """Straight segment from (0, 0) to (dx, dy)."""
    return [(dx * t, dy * t) for t in np.linspace(0, 1, N)]


def heart(scale: float = 0.05, N: int = 400) -> List[Point]:
    """Stylised heart curve scaled to *scale* (≈ bounding‑box diagonal)."""
    t = np.linspace(0, 2 * np.pi, N)
    x = 16 * (np.sin(t)) ** 3
    y = (
        13 * np.cos(t)
        - 5 * np.cos(2 * t)
        - 2 * np.cos(3 * t)
        - np.cos(4 * t)
    )
    x, y = x / 17 * scale, y / 17 * scale
    return list(zip(x, y))


def eight(R: float = 0.05, N: int = 400, stacked: bool = True) -> List[Point]:
    """Figure‑8 trajectory.

    Parameters
    ----------
    R : float
        Characteristic radius of each lobe.
    N : int
        Total number of points (will be split equally between the two lobes).
    stacked : bool, default True
        * True  → two **stacked** circles (vertical 8).
        * False → horizontal **lemniscate** (∞‑like) using the Bernoulli curve.
    """
    if stacked:
        half = N // 2
        upper = [
            (R * math.cos(2 * math.pi * i / half),
             R * math.sin(2 * math.pi * i / half) + R)
            for i in range(half)
        ]
        lower = [
            (R * math.cos(2 * math.pi * i / half),
             R * math.sin(2 * math.pi * i / half) - R)
            for i in range(half)
        ]
        return upper + lower
    # lemniscate of Bernoulli — centred, horizontally oriented
    t = np.linspace(0, 2 * np.pi, N)
    denom = 1 + np.sin(t) ** 2
    x = (R * np.cos(t)) / denom
    y = (R * np.sin(t) * np.cos(t)) / denom
    return list(zip(x, y))

# ---------------------------------------------------------------------------
# Registry helpers
# ---------------------------------------------------------------------------

_PREDEFINED: Dict[str, Callable[..., List[Point]]] = {
    "circle": circle,
    "square": square,
    "line": line,
    "heart": heart,
    "eight": eight,
}


def names() -> List[str]:
    """Return the list of available predefined trajectory names."""
    return list(_PREDEFINED.keys())


def get(name: str, **kwargs) -> List[Point]:
    """Return the (x, y) list corresponding to *name*.

    Parameters
    ----------
    name : str
        One of :func:`names()`.
    **kwargs : Any
        Extra keyword parameters forwarded to the underlying generator.
    """
    if name not in _PREDEFINED:
        raise KeyError(f"Trajectory '{name}' unknown. Available = {names()}")
    return _PREDEFINED[name](**kwargs)
