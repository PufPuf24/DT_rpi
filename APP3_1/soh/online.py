"""Opportunistic SoH estimation during normal storage operation.

The models in scripts 04 and 05 assume a reference performance test: a full
CCCV charge and a full discharge at a controlled rate and temperature. A storage
system in service almost never does that. It does partial cycles at whatever
rate the grid asks for, at whatever temperature the room happens to be.

This module bridges the two. The idea is:

1. **Watch the stream.** As current and voltage arrive, look for stretches from
   which one of the characterised features can honestly be measured -- a charge
   that crosses the whole 3.57 +/- 0.05 V window at a steady current, say, or a
   CV tail long enough to reach the 4 A threshold.

2. **Turn each into a candidate SoH.** Every feature carries a calibration
   fitted on the test cells, plus the cross-validated residual that calibration
   actually achieved. So a candidate arrives with an uncertainty attached, not
   just a number.

3. **Gate it.** A candidate is only as good as the conditions it was measured
   under. Four checks run, and each one that fails is recorded by name:

   * *conditions* -- was the rate and temperature close enough to the reference
     test the feature was characterised at?
   * *domain* -- is the feature value inside the range the calibration saw? An
     extrapolating network will happily return a confident nonsense.
   * *correlation* -- how strongly does this feature track SoH at all? A weak
     feature is not rejected, it is believed less.
   * *plausibility* -- does the candidate agree with the SoH we are already
     tracking, given both uncertainties? Capacity does not jump.

4. **Fuse what survives.** A scalar Kalman filter carries the running estimate.
   Between updates it widens by a slow random walk, since real capacity does
   fade; each accepted candidate pulls it by an amount set by the ratio of the
   two variances.

Every accepted and rejected candidate keeps its full provenance -- which
feature, measured under what conditions, with what correlation strength, and
which gate stopped it. That log is the answer to "on what basis did the number
move?".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np
import pandas as pd

from . import config as cfg
from .dataset import longest_true_run


# ---------------------------------------------------------------------------
# Confidence tiers
# ---------------------------------------------------------------------------


class Tier(str, Enum):
    """How strongly a feature tracks SoH, from the correlation analysis."""

    HIGH = "high"  # |rho| >= 0.95
    MEDIUM = "medium"  # 0.85 <= |rho| < 0.95
    LOW = "low"  # 0.70 <= |rho| < 0.85
    UNUSABLE = "unusable"  # |rho| < 0.70

    @classmethod
    def of(cls, rho: float) -> "Tier":
        magnitude = abs(rho)
        if not np.isfinite(magnitude):
            return cls.UNUSABLE
        if magnitude >= 0.95:
            return cls.HIGH
        if magnitude >= 0.85:
            return cls.MEDIUM
        if magnitude >= 0.70:
            return cls.LOW
        return cls.UNUSABLE

    @property
    def variance_inflation(self) -> float:
        """How much to widen the uncertainty of an estimate from this tier.

        A weakly-correlated feature is not thrown away -- it is allowed to
        contribute, but it moves the tracked value much less.
        """
        return {
            Tier.HIGH: 1.0,
            Tier.MEDIUM: 2.0,
            Tier.LOW: 5.0,
            Tier.UNUSABLE: np.inf,
        }[self]


# ---------------------------------------------------------------------------
# What the estimator knows about each feature
# ---------------------------------------------------------------------------


@dataclass
class FeatureCalibration:
    """One feature, its calibration to SoH, and the conditions it is valid under.

    ``predict`` maps a measured feature value to SoH. ``sigma`` is the residual
    that mapping achieved under leave-one-cell-out cross-validation -- the
    honest spread, not the training fit.
    """

    name: str
    family: str
    rho: float
    sigma: float
    training_range: tuple[float, float]
    reference_crate_charge: float
    reference_crate_discharge: float
    reference_temperature_c: float = 25.0

    # Fitted monotone calibration, filled in by ``fit_calibration``.
    _coefficients: np.ndarray | None = field(default=None, repr=False)

    @property
    def tier(self) -> Tier:
        return Tier.of(self.rho)

    def predict(self, value: float) -> float:
        if self._coefficients is None:
            raise RuntimeError(f"calibration for {self.name} has not been fitted")
        return float(np.polyval(self._coefficients, value))

    def domain_distance(self, value: float) -> float:
        """How far outside the calibrated range a value sits, in range-widths.

        0 means inside; 0.5 means half a training range beyond the edge.
        """
        lo, hi = self.training_range
        width = hi - lo
        if width <= 0:
            return np.inf
        if value < lo:
            return (lo - value) / width
        if value > hi:
            return (value - hi) / width
        return 0.0


def fit_calibration(
    train: pd.DataFrame,
    name: str,
    *,
    rho: float,
    reference_case: str,
    degree: int = 2,
    target: str = "SOH",
    group: str = "battery",
) -> FeatureCalibration:
    """Fit a single-feature SoH calibration and measure its honest residual.

    A low-order polynomial is used rather than a network: with one input and
    thirty points, anything more flexible fits noise, and a monotone-ish curve
    is what the physics suggests anyway. ``sigma`` comes from leave-one-cell-out
    residuals, so it reflects the error on a cell the calibration never saw.
    """
    block = train.dropna(subset=[name, target])
    x = block[name].to_numpy(float)
    y = block[target].to_numpy(float)

    coefficients = np.polyfit(x, y, degree)

    residuals = []
    for held_out in block[group].unique():
        inner = block[block[group] != held_out]
        outer = block[block[group] == held_out]
        if len(inner) < degree + 2 or outer.empty:
            continue
        fold = np.polyfit(inner[name].to_numpy(float), inner[target].to_numpy(float), degree)
        predicted = np.polyval(fold, outer[name].to_numpy(float))
        residuals.extend(predicted - outer[target].to_numpy(float))

    sigma = float(np.sqrt(np.mean(np.square(residuals)))) if residuals else float("nan")

    case = cfg.CASES[reference_case]
    calibration = FeatureCalibration(
        name=name,
        family=name.split("_")[0],
        rho=rho,
        sigma=sigma,
        training_range=(float(x.min()), float(x.max())),
        reference_crate_charge=case.crate_charge,
        reference_crate_discharge=case.crate_discharge,
    )
    calibration._coefficients = coefficients
    return calibration


# ---------------------------------------------------------------------------
# What arrives from the field
# ---------------------------------------------------------------------------


@dataclass
class Observation:
    """A feature measured from live operation, with the conditions it saw."""

    feature: str
    value: float
    timestamp_h: float
    equivalent_full_cycles: float
    crate_charge: float = np.nan
    crate_discharge: float = np.nan
    temperature_c: float = np.nan
    coverage: float = 1.0  # fraction of the required voltage span actually traversed
    source: str = ""  # free-text provenance, e.g. "grid charge 2026-08-27 04:12"


@dataclass
class Verdict:
    """The full record of one candidate estimate: value, trust, and reasoning."""

    observation: Observation
    soh: float
    sigma: float
    tier: Tier
    accepted: bool
    reasons: list[str] = field(default_factory=list)

    # Diagnostics that made the decision
    domain_distance: float = 0.0
    condition_penalty: float = 1.0
    disagreement_sigmas: float = np.nan

    def as_row(self) -> dict:
        return {
            "time_h": self.observation.timestamp_h,
            "efc": self.observation.equivalent_full_cycles,
            "feature": self.observation.feature,
            "value": self.observation.value,
            "soh_candidate": self.soh,
            "sigma": self.sigma,
            "tier": self.tier.value,
            "rho_tier": self.tier.value,
            "domain_distance": self.domain_distance,
            "condition_penalty": self.condition_penalty,
            "disagreement_sigmas": self.disagreement_sigmas,
            "accepted": self.accepted,
            "reasons": "; ".join(self.reasons),
            "source": self.observation.source,
        }


# ---------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------


@dataclass
class GateSettings:
    """Thresholds for accepting a candidate. All of these are policy, not physics."""

    max_domain_distance: float = 0.25  # range-widths outside the calibration
    min_coverage: float = 0.98  # the voltage window must be genuinely traversed
    # Widened from the original 0.35 (see C:\code2\python's battlib/online.py):
    # this bench can't hit the dissertation's 0.1C reference rate at all (~50 A
    # cap, realistic Maintenance-cycle discharge ~0.025-0.05C) -- a strict 35%
    # cutoff would hard-reject essentially every observation. The continuous
    # penalty below (`1.0 + 2.0 * deviation`) already scales trust down
    # proportionally to the mismatch; this just stops that from being
    # overridden by an all-or-nothing cutoff at a threshold this bench cannot
    # meet. Revisit downward once Phase 3 (soh_calibration.py) refits
    # FeatureCalibration objects at the bench's OWN actual rate.
    crate_tolerance: float = 2.5  # relative deviation from the reference rate
    temperature_tolerance_c: float = 15.0
    max_disagreement_sigmas: float = 3.0  # plausibility against the tracked value
    max_soh_rise: float = 1.0  # SoH may recover slightly, not climb
    min_tier: Tier = Tier.LOW


class AcceptanceGate:
    """Decides whether a candidate estimate is fit to be shown, and says why not."""

    def __init__(self, settings: GateSettings | None = None):
        self.settings = settings or GateSettings()

    def judge(
        self,
        observation: Observation,
        calibration: FeatureCalibration,
        *,
        tracked_soh: float | None,
        tracked_sigma: float | None,
    ) -> Verdict:
        s = self.settings
        reasons: list[str] = []

        soh = calibration.predict(observation.value)
        tier = calibration.tier

        # --- correlation strength -----------------------------------------
        if tier == Tier.UNUSABLE or _tier_rank(tier) < _tier_rank(s.min_tier):
            reasons.append(
                f"feature correlates only |rho|={abs(calibration.rho):.2f} "
                f"({tier.value}); below the {s.min_tier.value} tier required"
            )

        # --- was the feature measurable at all? ---------------------------
        if observation.coverage < s.min_coverage:
            reasons.append(
                f"voltage window only {observation.coverage:.0%} traversed "
                f"(needs {s.min_coverage:.0%}) -- the duration is a lower bound, "
                f"not a measurement"
            )

        # --- conditions ----------------------------------------------------
        penalty = 1.0
        for measured, reference, label in (
            (observation.crate_charge, calibration.reference_crate_charge, "charge C-rate"),
            (observation.crate_discharge, calibration.reference_crate_discharge, "discharge C-rate"),
        ):
            if not np.isfinite(measured) or reference <= 0:
                continue
            deviation = abs(measured - reference) / reference
            if deviation > s.crate_tolerance:
                reasons.append(
                    f"{label} {measured:.2f} C is {deviation:.0%} off the "
                    f"{reference:.2f} C the feature was characterised at"
                )
            penalty *= 1.0 + 2.0 * deviation

        if np.isfinite(observation.temperature_c):
            offset = abs(observation.temperature_c - calibration.reference_temperature_c)
            if offset > s.temperature_tolerance_c:
                reasons.append(
                    f"temperature {observation.temperature_c:.0f} C is {offset:.0f} C "
                    f"from the {calibration.reference_temperature_c:.0f} C reference"
                )
            penalty *= 1.0 + 0.04 * offset

        # --- applicability domain ------------------------------------------
        distance = calibration.domain_distance(observation.value)
        if distance > s.max_domain_distance:
            lo, hi = calibration.training_range
            reasons.append(
                f"feature value {observation.value:.4g} is {distance:.2f} range-widths "
                f"outside the calibrated span [{lo:.4g}, {hi:.4g}]"
            )
        penalty *= 1.0 + 4.0 * distance

        # --- uncertainty of this candidate ---------------------------------
        sigma = calibration.sigma * tier.variance_inflation * penalty

        # --- plausibility against what we already believe -------------------
        disagreement = np.nan
        if tracked_soh is not None and np.isfinite(tracked_soh):
            combined = np.sqrt(sigma**2 + (tracked_sigma or 0.0) ** 2)
            disagreement = abs(soh - tracked_soh) / combined if combined > 0 else np.inf
            if disagreement > s.max_disagreement_sigmas:
                reasons.append(
                    f"candidate {soh:.2f} % disagrees with the tracked {tracked_soh:.2f} % "
                    f"by {disagreement:.1f} sigma"
                )
            if soh - tracked_soh > s.max_soh_rise:
                reasons.append(
                    f"implies SoH rose {soh - tracked_soh:.2f} points; capacity does not recover "
                    f"by more than {s.max_soh_rise:.2f}"
                )

        return Verdict(
            observation=observation,
            soh=soh,
            sigma=sigma,
            tier=tier,
            accepted=not reasons,
            reasons=reasons,
            domain_distance=distance,
            condition_penalty=penalty,
            disagreement_sigmas=disagreement,
        )


def _tier_rank(tier: Tier) -> int:
    return {Tier.UNUSABLE: 0, Tier.LOW: 1, Tier.MEDIUM: 2, Tier.HIGH: 3}[tier]


# ---------------------------------------------------------------------------
# The running estimate
# ---------------------------------------------------------------------------


@dataclass
class SohTracker:
    """Carries the storage system's running SoH estimate.

    Three fusion rules are available, in decreasing order of machinery. All
    three take the same gated candidates and produce the same audit trail, so
    the choice is about how much statistical apparatus the write-up wants to
    carry, not about what the estimator can do.

    ``"kalman"``
        Scalar Kalman filter. The state is SoH in percent; between updates the
        estimate widens along a random walk in equivalent full cycles, because
        ageing continues whether or not a feature happened to be measurable.
        Each accepted candidate is folded in by inverse-variance weighting.
        This is the only rule that reports a meaningful confidence band and the
        only one that grows uncertain during a long gap in measurements.

    ``"inverse_variance"``
        Plain inverse-variance weighted mean over every accepted candidate so
        far, with no process model. One formula, no filter theory, and for a
        quantity that only drifts slowly it gives almost the same trace. The
        confidence band shrinks monotonically, which understates uncertainty
        after a long silence.

    ``"latest"``
        Show the most recent accepted candidate and nothing else. No fusion at
        all -- useful as the baseline that shows what the fusion buys.
    """

    soh: float = 100.0
    sigma: float = 2.0
    fusion: str = "kalman"
    drift_per_100_efc: float = 0.6  # process noise: how uncertain ageing makes us
    last_efc: float = 0.0
    history: list[dict] = field(default_factory=list)

    # Running sums for the inverse-variance rule.
    _weight_sum: float = 0.0
    _weighted_value_sum: float = 0.0

    def _predict_to(self, efc: float) -> None:
        """Widen the estimate to account for ageing since the last update."""
        elapsed = max(0.0, efc - self.last_efc)
        self.sigma = float(
            np.sqrt(self.sigma**2 + (self.drift_per_100_efc * elapsed / 100.0) ** 2)
        )
        self.last_efc = efc

    def update(self, verdict: Verdict) -> None:
        """Fold one judged candidate into the running estimate."""
        if self.fusion == "kalman":
            self._predict_to(verdict.observation.equivalent_full_cycles)

        usable = (
            verdict.accepted
            and np.isfinite(verdict.soh)
            and np.isfinite(verdict.sigma)
            and verdict.sigma > 0
        )
        gain = 0.0

        if usable and self.fusion == "kalman":
            gain = self.sigma**2 / (self.sigma**2 + verdict.sigma**2)
            self.soh = float(self.soh + gain * (verdict.soh - self.soh))
            self.sigma = float(np.sqrt((1 - gain) * self.sigma**2))

        elif usable and self.fusion == "inverse_variance":
            weight = 1.0 / verdict.sigma**2
            # How far this candidate moves the mean -- the same quantity the
            # Kalman gain reports, so the audit column stays comparable.
            gain = weight / (self._weight_sum + weight)
            self._weight_sum += weight
            self._weighted_value_sum += weight * verdict.soh
            self.soh = float(self._weighted_value_sum / self._weight_sum)
            self.sigma = float(np.sqrt(1.0 / self._weight_sum))

        elif usable and self.fusion == "latest":
            gain = 1.0
            self.soh = float(verdict.soh)
            self.sigma = float(verdict.sigma)

        elif not usable and self.fusion != "kalman":
            pass  # rejected candidates change nothing

        self.history.append(
            {
                **verdict.as_row(),
                "fusion": self.fusion,
                "kalman_gain": gain,
                "soh_tracked": self.soh,
                "sigma_tracked": self.sigma,
            }
        )

    def frame(self) -> pd.DataFrame:
        """The full audit trail: every candidate, accepted or not."""
        return pd.DataFrame(self.history)


# ---------------------------------------------------------------------------
# Opportunistic extraction from a live stream
# ---------------------------------------------------------------------------


@dataclass
class SegmentReport:
    """What a stretch of operation offered, and under what conditions."""

    kind: str  # "charge" | "discharge" | "cv"
    duration_s: float
    crate: float
    temperature_c: float
    v_start: float
    v_end: float
    samples: int


def scan_segments(
    t: np.ndarray,
    v: np.ndarray,
    i: np.ndarray,
    temperature: np.ndarray | None = None,
    *,
    capacity_mah: float = cfg.C_NOMINAL_MAH,
    min_duration_s: float = 300.0,
) -> list[tuple[SegmentReport, np.ndarray]]:
    """Split a stream of operation into usable charge / discharge / CV stretches.

    Returns each segment's summary together with the boolean mask selecting it,
    so a caller can run whichever feature extractor the segment supports.
    """
    t = np.asarray(t, float)
    v = np.asarray(v, float)
    i = np.asarray(i, float)
    temperature = (
        np.asarray(temperature, float) if temperature is not None else np.full_like(t, np.nan)
    )

    out: list[tuple[SegmentReport, np.ndarray]] = []

    masks = {
        "charge": (i > cfg.I_THRESHOLD_MA) & (v < cfg.CV_DECAY.v_cv_start),
        "discharge": i < -cfg.I_THRESHOLD_MA,
        "cv": (i > cfg.I_THRESHOLD_MA) & (v >= cfg.CV_DECAY.v_cv_start),
    }

    for kind, raw in masks.items():
        mask = longest_true_run(raw)
        if not mask.any():
            continue

        span = t[mask]
        duration = float(span[-1] - span[0])
        if duration < min_duration_s:
            continue

        out.append(
            (
                SegmentReport(
                    kind=kind,
                    duration_s=duration,
                    crate=float(np.nanpercentile(np.abs(i[mask]), 95) / capacity_mah),
                    temperature_c=float(np.nanmean(temperature[mask])),
                    v_start=float(v[mask][0]),
                    v_end=float(v[mask][-1]),
                    samples=int(mask.sum()),
                ),
                mask,
            )
        )

    return out


def window_coverage(v_segment: np.ndarray, v_low: float, v_high: float) -> float:
    """Fraction of the ``[v_low, v_high]`` band the segment actually crossed.

    A partial cycle that starts at 3.6 V cannot measure the time to cross a
    3.52--3.62 V window; it can only report a lower bound. This is what tells
    the gate to reject it.
    """
    if v_segment.size == 0 or v_high <= v_low:
        return 0.0
    lo = max(v_low, float(np.nanmin(v_segment)))
    hi = min(v_high, float(np.nanmax(v_segment)))
    return float(max(0.0, hi - lo) / (v_high - v_low))


# ---------------------------------------------------------------------------
# Constant-current precondition
# ---------------------------------------------------------------------------


@dataclass
class SteadyStateGate:
    """Preconditions before a voltage-window timer may start.

    The calibration RPTs are pure constant current -- no step at the moment a
    window starts. A live system's current changes on its own schedule, and
    the terminal voltage responds to a fresh step with a fast RC-relaxation
    transient that has nothing to do with capacity fade. Timing a window
    through that transient measures the cell's own settling behaviour, not
    ageing, and the result would not resemble anything the network was
    trained on -- ``scan_segments`` above finds long enough runs of same-sign
    current, but does not check any of this.

    Two conditions, BOTH required, checked at the candidate start index:

    * the current has stayed within ``stability_tolerance`` of its own recent
      mean for at least ``stability_window_s`` immediately before that index
    * at least ``cooldown_after_step_s`` has passed since the most recent
      current change bigger than ``step_threshold`` (relative)

    Both are necessary, not either: a current that has been flat for the
    stability window but had a step just before it is still relaxing even
    though it currently reads stable -- the RC transient's voltage signature
    can persist well after the current itself has settled.

    These defaults are a starting design, not validated against real field
    hardware -- the RPT data behind this study logs one sample per 60 s, too
    coarse to tune a settling time against. Calibrating them against faster,
    real logging is the first task for whoever deploys this.
    """

    stability_tolerance: float = 0.02
    stability_window_s: float = 60.0
    cooldown_after_step_s: float = 60.0
    step_threshold: float = 0.10

    def ready(
        self, t: np.ndarray, i: np.ndarray, at: int = -1
    ) -> tuple[bool, str]:
        """Whether a window timer may start at sample ``at`` (default: the last one).

        ``t`` in seconds, ``i`` in mA, both time-ordered. Returns
        ``(ready, reason)`` -- ``reason`` explains a ``False`` verdict and is
        empty when ``ready`` is ``True``.
        """
        t = np.asarray(t, float)
        i = np.asarray(i, float)
        if t.size < 2 or i.size != t.size:
            return False, "not enough samples to judge stability"

        now_idx = at if at >= 0 else t.size + at
        now_t = t[now_idx]

        # --- stability: the recent window must be flat -----------------
        recent = (t <= now_t) & (t > now_t - self.stability_window_s)
        if recent.sum() < 2:
            return False, (
                f"fewer than 2 samples in the last {self.stability_window_s:.0f} s "
                f"-- cannot judge stability yet"
            )
        i_recent = i[recent]
        baseline = np.mean(i_recent)
        if baseline == 0:
            return False, "recent current reads zero -- not in a CC segment"
        relative_spread = np.max(np.abs(i_recent - baseline)) / abs(baseline)
        if relative_spread > self.stability_tolerance:
            return False, (
                f"current varied {relative_spread:.1%} over the last "
                f"{self.stability_window_s:.0f} s (needs <= {self.stability_tolerance:.0%})"
            )

        # --- cooldown: no recent step, even before the stability window -----
        before = t <= now_t
        i_before = i[before]
        t_before = t[before]
        if i_before.size >= 2:
            steps = np.abs(np.diff(i_before)) / np.maximum(np.abs(i_before[:-1]), 1e-9)
            stepped = np.where(steps > self.step_threshold)[0]
            if stepped.size:
                last_step_t = t_before[stepped[-1] + 1]
                since = now_t - last_step_t
                if since < self.cooldown_after_step_s:
                    return False, (
                        f"current stepped >{self.step_threshold:.0%} only {since:.0f} s ago "
                        f"(needs >= {self.cooldown_after_step_s:.0f} s to relax)"
                    )

        return True, ""
