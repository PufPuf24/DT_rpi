"""Generic (not Rpt-bound) versions of battlib/features.py's curve-shaping
pipeline -- ICA (dQ/dV), DVA (dV/dQ) curves, and the voltage-window-crossing
time feature Phase 2 will need. Vendored and adapted rather than importing
C:\\code2\\python directly (separate project, separate drive) -- see
soh/__init__.py.

Scope note: this module computes the raw ICA/DVA CURVES from a Maintenance
cycle's own discharge trace, at whatever C-rate the bench actually ran (see
maintenance_cycle.py) -- that's the "special category" for ICA/DVA the spec
asked for. It deliberately does NOT port battlib/features.py's ica_features
/dva_features peak-position extraction: those hardcode voltage/capacity
windows and peak prominences tuned to the dissertation's own C01/C02/C05
rates. Reapplying them unexamined to a ~0.025-0.05C Maintenance discharge
would silently misassign peaks to the wrong electrochemical transition.
Peak extraction from these curves is future work once there's bench data to
retune the windows against (see APP3_0/soh/README.md).
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import PchipInterpolator

from .signal_tools import smooth_gaussian, smooth_loess

ICA_V_MIN = 2.8
ICA_V_MAX = 4.2
ICA_N_GRID = 2500
ICA_LOESS_SPAN = 30
ICA_GAUSS_SPAN = 30
ICA_CLIP_ABS = 1e6


def _monotone_xy(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sort by ``x`` and average ``y`` over duplicate ``x`` values."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    good = np.isfinite(x) & np.isfinite(y)
    x, y = x[good], y[good]
    if x.size == 0:
        return x, y

    order = np.argsort(x, kind="stable")
    x, y = x[order], y[order]

    x_u, inverse = np.unique(x, return_inverse=True)
    if x_u.size == x.size:
        return x, y

    sums = np.bincount(inverse, weights=y)
    counts = np.bincount(inverse)
    return x_u, sums / counts


def _pchip(x: np.ndarray, y: np.ndarray, grid: np.ndarray) -> np.ndarray:
    return PchipInterpolator(x, y, extrapolate=False)(grid)


def _derivative_curve(
    x: np.ndarray,
    y: np.ndarray,
    *,
    grid: np.ndarray,
    loess_span: int,
    gauss_span: int,
    clip_abs: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Smooth, resample onto ``grid``, differentiate, smooth again.

    Shared backbone of ICA (``x=V, y=Q``) and DVA (``x=Q, y=V``). Returns
    ``(grid[:-1], dy/dx)``.
    """
    if x.size < 4:
        return np.array([]), np.array([])

    y_smooth = smooth_loess(y, loess_span)
    y_grid = _pchip(x, y_smooth, grid)

    deriv = np.diff(y_grid) / np.diff(grid)
    deriv = smooth_gaussian(deriv, gauss_span)
    deriv[np.abs(deriv) > clip_abs] = np.nan

    return grid[:-1], deriv


def ica_curve(v: np.ndarray, q_mah: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Smoothed dQ/dV curve. Returns ``(V, dQ/dV)``, empty if not computable."""
    v_u, q_u = _monotone_xy(v, q_mah)
    if v_u.size < 4:
        return np.array([]), np.array([])
    grid = np.linspace(max(ICA_V_MIN, v_u[0]), min(ICA_V_MAX, v_u[-1]), ICA_N_GRID)
    return _derivative_curve(v_u, q_u, grid=grid, loess_span=ICA_LOESS_SPAN,
                              gauss_span=ICA_GAUSS_SPAN, clip_abs=ICA_CLIP_ABS)


def dva_curve(q_mah: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Smoothed dV/dQ curve. Returns ``(Q, dV/dQ)``, empty if not computable."""
    q_u, v_u = _monotone_xy(q_mah, v)
    if q_u.size < 4:
        return np.array([]), np.array([])
    grid = np.linspace(q_u[0], q_u[-1], ICA_N_GRID)
    return _derivative_curve(q_u, v_u, grid=grid, loess_span=ICA_LOESS_SPAN,
                              gauss_span=ICA_GAUSS_SPAN, clip_abs=ICA_CLIP_ABS)


def _time_at_voltage(t: np.ndarray, v: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """Interpolate the time at which the trace passes each target voltage."""
    good = np.isfinite(t) & np.isfinite(v)
    t, v = t[good], v[good]
    if t.size < 2:
        return np.full(len(targets), np.nan)

    _, first = np.unique(v, return_index=True)
    keep = np.sort(first)
    v_u, t_u = v[keep], t[keep]

    order = np.argsort(v_u)
    v_s, t_s = v_u[order], t_u[order]

    out = np.interp(targets, v_s, t_s, left=np.nan, right=np.nan)
    outside = (targets < v_s[0]) | (targets > v_s[-1])
    out[outside] = np.nan
    return out


def voltage_window_time(t: np.ndarray, v: np.ndarray, v_low: float, v_high: float) -> float:
    """Seconds spent crossing ``[v_low, v_high]``, from any voltage profile.

    Ported verbatim from battlib/features.py -- this is the exact feature the
    deployment_package's best1/best2/best3 combinations were calibrated on
    (see soh/online.py, Phase 2).
    """
    times = _time_at_voltage(np.asarray(t, float), np.asarray(v, float),
                              np.array([v_low, v_high]))
    return float(abs(times[1] - times[0]))
