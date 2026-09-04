"""Live-trace feature extraction for the short-window, native-rate field SoH
estimator -- see C:\\code2\\python\\scripts\\08_field_window_deployment.py for
how deployment_package_field.joblib was calibrated and why (fixed capacity
slice at candidate positions along a discharge, ~7 A / native ~0.1C rate,
10-30 minute windows instead of an open-ended voltage-crossing time).

A lab RPT (what the deployment package was trained on) knows its own total
discharge capacity Qtot in advance -- the discharge already ran to completion.
A live field measurement does not, since the entire point is NOT running a
full discharge just to check health. This module approximates Qtot from the
group's own last Maintenance-cycle Q_ref (see maintenance_cycle.py) -- or the
dissertation's nominal capacity as a bootstrap before any Maintenance cycle
has run for that group -- converted through the live SOC estimate (EKF,
preferred, or Coulomb counting) to know where a live discharge sits relative
to a hypothetical 100%->0% span, without ever completing one.
"""

from __future__ import annotations

import numpy as np


def voltage_at_capacity(qd_mah, v, targets_mah):
    """V(Q) interpolation -- same shape as soh/curves.py's voltage_window_time,
    x/y roles swapped: interpolates voltage as a function of discharged
    capacity instead of time as a function of voltage. NaN for a target
    outside the trace's own observed capacity range (extrapolation would
    just be a lower bound, not a measurement -- same reasoning as
    soh.curves._time_at_voltage)."""
    qd_mah = np.asarray(qd_mah, dtype=float)
    v = np.asarray(v, dtype=float)
    good = np.isfinite(qd_mah) & np.isfinite(v)
    qd_mah, v = qd_mah[good], v[good]
    if qd_mah.size < 2:
        return np.full(len(targets_mah), np.nan)

    order = np.argsort(qd_mah)
    qd_s, v_s = qd_mah[order], v[order]
    _, idx = np.unique(qd_s, return_index=True)
    qd_u, v_u = qd_s[idx], v_s[idx]

    targets = np.asarray(targets_mah, dtype=float)
    out = np.interp(targets, qd_u, v_u, left=np.nan, right=np.nan)
    outside = (targets < qd_u[0]) | (targets > qd_u[-1])
    out = np.array(out, dtype=float)
    out[outside] = np.nan
    return out


def capacity_window_dv(qd_mah, v, centre_frac, dq_mah, qtot_mah):
    """|deltaV| [V] over the capacity slice centred at `centre_frac` of
    `qtot_mah`, width `dq_mah` -- the raw feature deployment_package_field's
    dV_c<k> columns are. NaN if the live trace doesn't actually cover both
    edges (see window_capacity_coverage for the caller-side coverage check,
    same "a partial cycle can only report a lower bound" reasoning as
    soh.online.window_coverage)."""
    qc = centre_frac * qtot_mah
    edges = np.array([qc - dq_mah / 2.0, qc + dq_mah / 2.0])
    v2 = voltage_at_capacity(qd_mah, v, edges)
    if not np.all(np.isfinite(v2)):
        return float("nan")
    return float(abs(v2[1] - v2[0]))


def window_capacity_coverage(qd_mah, centre_frac, dq_mah, qtot_mah):
    """Fraction of the target capacity window actually spanned by the trace's
    own observed [min(qd), max(qd)] -- capacity-domain counterpart of
    soh.online.window_coverage."""
    qd_mah = np.asarray(qd_mah, dtype=float)
    qd_mah = qd_mah[np.isfinite(qd_mah)]
    if qd_mah.size == 0:
        return 0.0
    qc = centre_frac * qtot_mah
    qLow, qHigh = qc - dq_mah / 2.0, qc + dq_mah / 2.0
    if qHigh <= qLow:
        return 0.0
    lo = max(qLow, float(qd_mah.min()))
    hi = min(qHigh, float(qd_mah.max()))
    return max(0.0, hi - lo) / (qHigh - qLow)
