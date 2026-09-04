"""2RC ECM + Extended Kalman Filter joint state estimator (SOC, v1, v2).

Python port of ECM_EKF_3DISS_FIG.m (MATLAB, dissertation GITT/ECM_parameters_bez_CO),
same state vector, same predict/update equations, same H = [dOCV/dSOC -
I*dR0/dSOC, -1, -1] linearisation. Reuses ecm_model.py's LUT loader/temperature
blending (_loadLut/_tempWeights/_clamp) rather than duplicating it -- same
JSON parameter files, same 2RC physics, same discharge-table-only
simplification (see ecm_model.py's module docstring for why).

This is a DIFFERENT thing from EcmModel.step(): that one is open-loop (current
in, predicted voltage out) specifically so its prediction can be compared
against the real measured voltage on the chart -- feeding it the measurement
would defeat that comparison. This class instead FUSES the measured voltage
into the state every step, correcting SOC drift the way a real BMS SOC
estimator does; it is meant to run alongside EcmModel, not replace it, same as
the MATLAB script computes and plots both soc_cc (Coulomb counting) and
soc_ekf side by side.

Current-sign convention matches ecm_model.py: I > 0 = discharge (current flows
OUT of the battery). MATLAB's I_sign=-1 flip is a data-loading detail of that
script's own .mat file, not part of the algorithm -- the app already delivers
current in this sign convention everywhere, so no flip is applied here.
"""

from __future__ import annotations

import math

import numpy as np

from ecm_model import TEMP_KEYS, _clamp, _loadLut, _tempWeights


class EcmEkfEstimator:
    """State ``x = [soc, v1, v2]``. See module docstring for what this is for.

    Tuning defaults are exactly the MATLAB script's (`sigmaV`, `qSOC`, `qV1`,
    `qV2`, `P0_soc`, `P0_v1`, `P0_v2`) -- carried over rather than re-derived,
    since they were already tuned there against real GITT/validation data.
    """

    def __init__(self, dischargeJsonPath, initialSocFraction=1.0,
                 sigmaV=0.010, qSoc=5e-7, qV1=5e-5, qV2=5e-5,
                 p0Soc=0.05 ** 2, p0V1=0.05 ** 2, p0V2=0.05 ** 2,
                 epsR=1e-9, epsC=1e-12):
        self.lut = _loadLut(dischargeJsonPath)
        self.epsR = epsR
        self.epsC = epsC

        # Analytic derivatives of the PCHIP OCV/R0 curves -- exact for the spline,
        # unlike MATLAB's gradient() on a discretised grid (see module docstring).
        self._ocvDeriv = {k: self.lut[k]["OCV"].derivative() for k in TEMP_KEYS}
        self._r0Deriv = {k: self.lut[k]["R0"].derivative() for k in TEMP_KEYS}

        self.x = np.array([_clamp(initialSocFraction, 0.0, 1.0), 0.0, 0.0], dtype=float)
        self.P = np.diag([p0Soc, p0V1, p0V2]).astype(float)
        self.Q = np.diag([qSoc, qV1, qV2]).astype(float)
        self.R = float(sigmaV) ** 2

        self.lastTemperatureC = 25.0
        self.lastVoltage = float("nan")

    @property
    def soc(self):
        return float(self.x[0])

    @property
    def v1(self):
        return float(self.x[1])

    @property
    def v2(self):
        return float(self.x[2])

    @property
    def socSigma(self):
        """Current SOC uncertainty [fraction], sqrt of P[0,0] -- the EKF's own
        confidence in its estimate, widens between measurements, narrows on update."""
        return float(math.sqrt(max(self.P[0, 0], 0.0)))

    def reset(self, initialSocFraction=1.0):
        self.x = np.array([_clamp(initialSocFraction, 0.0, 1.0), 0.0, 0.0], dtype=float)
        self.P = np.diag([self.P[0, 0], self.P[1, 1], self.P[2, 2]]).astype(float)
        self.lastVoltage = float("nan")

    def _paramsAt(self, soc, temperatureC):
        loKey, hiKey, frac = _tempWeights(temperatureC)
        lo, hi = self.lut[loKey], self.lut[hiKey]

        def blend(name):
            vLo = float(lo[name](soc))
            vHi = float(hi[name](soc))
            return vLo + frac * (vHi - vLo)

        dOcvLo, dOcvHi = float(self._ocvDeriv[loKey](soc)), float(self._ocvDeriv[hiKey](soc))
        dR0Lo, dR0Hi = float(self._r0Deriv[loKey](soc)), float(self._r0Deriv[hiKey](soc))

        qah = lo["QAh"] + frac * (hi["QAh"] - lo["QAh"])
        return {
            "OCV": blend("OCV"), "R0": max(blend("R0"), self.epsR),
            "R1": max(blend("R1"), self.epsR), "R2": max(blend("R2"), self.epsR),
            "C1": max(blend("C1"), self.epsC), "C2": max(blend("C2"), self.epsC),
            "QAh": qah,
            "dOCV": dOcvLo + frac * (dOcvHi - dOcvLo),
            "dR0": dR0Lo + frac * (dR0Hi - dR0Lo),
        }

    def step(self, currentA, dtS, temperatureC, measuredVoltageV):
        """One predict(+update) step. `currentA`: I>0=discharge. `measuredVoltageV`
        may be NaN (measurement dropout) -- the filter then just predicts forward,
        widening its uncertainty, same as a GPS outage in a nav filter. Returns
        (socFraction, predictedVoltageV) -- the voltage AFTER fusing the
        measurement (MATLAB's V_sim), not the pre-update prediction."""
        if dtS is None or dtS <= 0 or currentA != currentA:
            return self.soc, self.lastVoltage
        if temperatureC == temperatureC:
            self.lastTemperatureC = temperatureC
        temperatureC = self.lastTemperatureC

        socK = _clamp(self.x[0], 0.0, 1.0)
        p = self._paramsAt(socK, temperatureC)

        tau1 = max(p["R1"] * p["C1"], 1e-9)
        tau2 = max(p["R2"] * p["C2"], 1e-9)
        a1 = math.exp(-dtS / tau1)
        a2 = math.exp(-dtS / tau2)

        socPred = _clamp(socK - currentA * dtS / (3600.0 * p["QAh"]), 0.0, 1.0)
        v1Pred = a1 * self.x[1] + (1 - a1) * p["R1"] * currentA
        v2Pred = a2 * self.x[2] + (1 - a2) * p["R2"] * currentA
        xPred = np.array([socPred, v1Pred, v2Pred])

        F = np.array([[1.0, 0.0, 0.0], [0.0, a1, 0.0], [0.0, 0.0, a2]])
        Ppred = F @ self.P @ F.T + self.Q

        socP = _clamp(xPred[0], 0.0, 1.0)
        pP = self._paramsAt(socP, temperatureC)
        vPred = pP["OCV"] - currentA * pP["R0"] - xPred[1] - xPred[2]

        if measuredVoltageV != measuredVoltageV:
            self.x, self.P = xPred, Ppred
            self.lastVoltage = vPred
            return self.soc, vPred

        H = np.array([pP["dOCV"] - currentA * pP["dR0"], -1.0, -1.0])
        y = measuredVoltageV - vPred
        Svv = float(H @ Ppred @ H.T + self.R)
        K = (Ppred @ H) / Svv

        self.x = xPred + K * y
        self.x[0] = _clamp(self.x[0], 0.0, 1.0)

        I3 = np.eye(3)
        KH = np.outer(K, H)
        # Joseph form -- numerically stable (stays symmetric PSD) under repeated
        # updates, same guarantee the MATLAB script's (I-KH)P(I-KH)'+KRK' gives.
        self.P = (I3 - KH) @ Ppred @ (I3 - KH).T + np.outer(K, K) * self.R

        socU = _clamp(self.x[0], 0.0, 1.0)
        pU = self._paramsAt(socU, temperatureC)
        voltage = pU["OCV"] - currentA * pU["R0"] - self.x[1] - self.x[2]
        self.lastVoltage = voltage
        return self.soc, voltage
