"""Trimmed vendored copy of battlib/config.py -- only the constants online.py
actually reads (RptCase/CASES for fit_calibration's reference_case lookup,
C_NOMINAL_MAH/I_THRESHOLD_MA/CV_DECAY for scan_segments). See soh/__init__.py.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RptCase:
    """One of the reference-performance-test current rates."""

    key: str
    crate_charge: float
    crate_discharge: float

    @property
    def label(self) -> str:
        return f"{self.crate_charge:.1f}C/{self.crate_discharge:.1f}C"


# The three rates the dissertation's RPTs were run at (C:\code2\python\battlib\config.py).
CASES: dict[str, RptCase] = {
    c.key: c
    for c in [
        RptCase("C01", 0.1, 0.1),
        RptCase("C02", 0.2, 0.2),
        RptCase("C05", 0.2, 0.5),
    ]
}
CASE_KEYS: list[str] = list(CASES)

# Nominal cell capacity [mAh] -- SOH_nominal reference, see soh_store.py. SoH
# itself is normalised against each group's own first Maintenance-cycle
# capacity (Q_ref), not this value.
C_NOMINAL_MAH = 78_000.0

# Current below which the cell counts as resting [mA].
I_THRESHOLD_MA = 10.0


@dataclass
class CvDecayConfig:
    v_cv_start: float = 4.20  # [V] CC -> CV transition


CV_DECAY = CvDecayConfig()
