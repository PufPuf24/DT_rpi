"""Trimmed vendored copy of C:\\code2\\python\\battlib\\models.py -- ONLY the
`FfnnConfig`/`Ffnn` classes, unchanged (field names/order matter: this is what
deployment_package.joblib's pickle was saved against, so the class shape has
to match exactly for joblib.load to reconstruct it). Training code (`fit`,
cross-validation, `SEARCH_CONFIG`/`FINAL_CONFIG`) is deliberately NOT here --
APP3_0 only ever loads and predicts with the already-fitted ensembles in
deployment_package.joblib, it never retrains this network (Phase 3's
recalibration works on FeatureCalibration objects in soh/online.py instead,
a different and much simpler model -- see soh/README.md).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler


@dataclass(frozen=True)
class FfnnConfig:
    """Architecture and training settings for one network."""

    hidden: tuple[int, ...] = (4,)
    activation: str = "tanh"
    alpha: float = 1e-2
    max_iter: int = 3000
    solver: str = "lbfgs"
    n_restarts: int = 5
    random_state: int = 0

    @property
    def name(self) -> str:
        return f"{'-'.join(map(str, self.hidden))} {self.activation}, alpha={self.alpha:g}"


@dataclass
class Ffnn:
    """A fitted ensemble of identically-configured networks."""

    conf: FfnnConfig
    features: list[str]
    x_scaler: StandardScaler
    y_scaler: StandardScaler
    members: list[MLPRegressor] = field(default_factory=list)

    def predict(self, frame: pd.DataFrame) -> np.ndarray:
        """Median prediction across the ensemble members, in SoH percent."""
        x = self.x_scaler.transform(frame[self.features].to_numpy(float))
        stacked = np.vstack([m.predict(x) for m in self.members])
        return self.y_scaler.inverse_transform(
            np.median(stacked, axis=0).reshape(-1, 1)
        ).ravel()

    def predict_spread(self, frame: pd.DataFrame) -> np.ndarray:
        """Spread across ensemble members [SoH %] -- a cheap uncertainty proxy."""
        x = self.x_scaler.transform(frame[self.features].to_numpy(float))
        stacked = np.vstack([m.predict(x) for m in self.members])
        scale = float(self.y_scaler.scale_[0])
        return np.std(stacked, axis=0) * scale
