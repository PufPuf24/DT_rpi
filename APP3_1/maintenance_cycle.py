"""Guided Maintenance validation cycle: charge to full, rest, controlled
discharge to a cutoff voltage, Coulomb-count Q, derive ground-truth SOH per
series group -- the AUTHORITATIVE SoH source (Phase 1 of the SoH plan; see
soh/README.md for Phases 2-3, the opportunistic/secondary estimator).

One state machine per PACK, not per group: B01..B0n are series positions of
the same physical pack, so one discharge run carries the same current through
all of them (Q is shared) while each group's own voltage evolves
independently based on its own capacity -- one guided cycle yields a SOH for
every group at once, each against its own Q_ref.

This module only tracks state and does the numerical integration -- no
serial I/O, no Tk. BatteryMonitorGUI feeds it live samples and reacts to
phase changes (driving the IN/OUT relays); this keeps the cycle logic
independent of hardware and directly unit-testable.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

from soh.curves import dva_curve, ica_curve


class Phase(str, Enum):
    IDLE = "idle"
    CHARGING = "charging"
    RESTING = "resting"
    DISCHARGING = "discharging"
    DONE = "done"
    ABORTED = "aborted"


class MaintenanceCycle:
    """See module docstring. `nGroups` = number of series positions (B01..).

    Charge-complete detection has two independent triggers, either one ends
    the charging phase -- which one is meaningful depends on whether the
    connected charge source is a PV panel or a bench supply in CV mode:
      - pack voltage reaches/holds `charge_full_v` for `charge_hold_s`
      - pack current tapers below `charge_taper_a` while near-full voltage
    `force_advance()` is the manual override for when neither fires reliably
    for a given setup -- this is a bench, not a lab robot; the operator
    stays in the loop.
    """

    def __init__(self, n_groups, q_ref_mah=None, *,
                 rest_seconds=2 * 3600.0,
                 discharge_cutoff_v=3.35,
                 charge_full_v=4.2,
                 charge_hold_s=300.0,
                 charge_taper_a=0.5):
        self.n_groups = n_groups
        # q_ref_mah: list[float | None], one per group -- None until that
        # group's first-ever Maintenance cycle completes (own-BOL reference,
        # matching battlib.dataset.soh_table's convention).
        self.q_ref_mah = list(q_ref_mah) if q_ref_mah is not None else [None] * n_groups

        self.rest_seconds = rest_seconds
        self.discharge_cutoff_v = discharge_cutoff_v
        self.charge_full_v = charge_full_v
        self.charge_hold_s = charge_hold_s
        self.charge_taper_a = charge_taper_a

        self.phase = Phase.IDLE
        self._phase_start_t = None
        self._charge_above_since_t = None

        # discharge trace: shared t/I, per-group V and cumulative Q series
        # (Q is the same physical quantity for every group -- kept once, not
        # duplicated per group -- but ica_curve/dva_curve need it paired
        # against each group's own voltage).
        self.t_rel = []
        self.i = []
        self.v = [[] for _ in range(n_groups)]
        self.q_series_mah = []
        self._q_mah = 0.0
        self._last_t = None

        self.results = None  # filled in on completion: list of dicts, one per group
        self.discharge_rate_c = float("nan")  # recorded from the actual measured current
        self.temperature_c = float("nan")

    @property
    def q_mah(self):
        """Running Coulomb-counted charge [mAh] for the current/last discharge."""
        return self._q_mah

    # ------------------------------------------------------------------
    def start(self, t_now):
        self.phase = Phase.CHARGING
        self._phase_start_t = t_now
        self._charge_above_since_t = None
        self.t_rel, self.i = [], []
        self.v = [[] for _ in range(self.n_groups)]
        self.q_series_mah = []
        self._q_mah = 0.0
        self._last_t = None
        self.results = None

    def abort(self):
        self.phase = Phase.ABORTED

    def force_advance(self, t_now):
        if self.phase == Phase.CHARGING:
            self._begin_rest(t_now)
        elif self.phase == Phase.RESTING:
            self._begin_discharge(t_now)
        elif self.phase == Phase.DISCHARGING:
            self._finish(t_now)

    def _begin_rest(self, t_now):
        self.phase = Phase.RESTING
        self._phase_start_t = t_now

    def _begin_discharge(self, t_now):
        self.phase = Phase.DISCHARGING
        self._phase_start_t = t_now
        self.t_rel, self.i = [], []
        self.v = [[] for _ in range(self.n_groups)]
        self.q_series_mah = []
        self._q_mah = 0.0
        self._last_t = None

    def _finish(self, t_now):
        self.phase = Phase.DONE
        qDischarged = self._q_mah
        results = []
        for k in range(self.n_groups):
            qRef = self.q_ref_mah[k] if self.q_ref_mah[k] is not None else qDischarged
            isFirst = self.q_ref_mah[k] is None
            soh = 100.0 * qDischarged / qRef if qRef and qRef > 0 else float("nan")
            results.append({
                "q_mah": qDischarged,
                "q_ref_mah": qRef,
                "soh_pct": soh,
                "is_first_cycle": isFirst,
            })
            if isFirst:
                self.q_ref_mah[k] = qDischarged
        self.results = results

    # ------------------------------------------------------------------
    def on_sample(self, t_now, voltages, current, temperature_c=float("nan")):
        """One shared-current sample, `voltages`: list[float] len==n_groups
        (NaN for an unavailable channel), `current` in A (>0 = discharging,
        matching the rest of the app's I_OUT-positive convention). Returns
        True if the phase advanced as a result of this sample.

        Charge-full and discharge-cutoff are each judged from the single
        WORST cell, not an average or the pack sum: without per-cell active
        balancing, the highest cell is the one at overcharge risk, and the
        lowest is the one at over-discharge risk. Waiting for an average
        would let that cell run past its limit while the others look fine."""
        finiteV = [v for v in voltages if v == v]
        highestV = max(finiteV) if finiteV else float("nan")
        lowestV = min(finiteV) if finiteV else float("nan")

        if self.phase == Phase.CHARGING:
            if current == current and highestV == highestV:
                if highestV >= self.charge_full_v:
                    if self._charge_above_since_t is None:
                        self._charge_above_since_t = t_now
                    elif t_now - self._charge_above_since_t >= self.charge_hold_s:
                        self._begin_rest(t_now)
                        return True
                else:
                    self._charge_above_since_t = None
                if (abs(current) <= self.charge_taper_a
                        and highestV >= self.charge_full_v * 0.98):
                    self._begin_rest(t_now)
                    return True
            return False

        if self.phase == Phase.RESTING:
            if t_now - self._phase_start_t >= self.rest_seconds:
                self._begin_discharge(t_now)
                return True
            return False

        if self.phase == Phase.DISCHARGING:
            self.t_rel.append(t_now - self._phase_start_t)
            for k in range(self.n_groups):
                self.v[k].append(voltages[k])
            if self._last_t is not None and current == current:
                dtH = (t_now - self._last_t) / 3600.0
                if dtH > 0:
                    self._q_mah += current * 1000.0 * dtH  # A * 1000 * h = mAh
            self.q_series_mah.append(self._q_mah)
            self._last_t = t_now
            self.i.append(current)
            if temperature_c == temperature_c:
                self.temperature_c = temperature_c
            if lowestV == lowestV and lowestV <= self.discharge_cutoff_v:
                self._finish(t_now)
                return True
            return False

        return False

    # ------------------------------------------------------------------
    def curves(self, group):
        """ICA (V, dQ/dV) and DVA (Q, dV/dQ) curves for one group's discharge,
        from the just-completed cycle's own trace -- see soh/curves.py for why
        peak-position extraction isn't attempted here yet."""
        v = np.asarray(self.v[group], float)
        q = np.asarray(self.q_series_mah, float)
        return ica_curve(v, q), dva_curve(q, v)
