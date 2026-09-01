"""
2RC ECM (Equivalent Circuit Model) baterie — bez Kalmanova filtru.

Simuluje napětí JEDNOHO článku ze změřeného proudu (Coulomb counting SOC + 2 RC větve),
podle ECM_val_ODE_adj_DISSERATION_FIGURE.m. Parametry (SOC, OCV, R0, R1, R2, C1, C2, QAh)
jsou pro 3 teploty (5/25/45 °C), exportované z MATLABu (P_discharge_validated.mat,
P_charge_new.mat) do JSON přes ECM/export_params.m.

Konvence znaménka proudu: I > 0 = vybíjení (proud TEČE Z baterie ven).
V appce: I_bat = I_OUT − I_IN (Load vybíjí, IN/PV nabíjí).

Zjednodušení (ověřeno na EV_val.mat): pro simulaci se používá VÝHRADNĚ vybíjecí LUT,
i když proud teče opačným směrem (nabíjení) — přepínání na samostatnou nabíjecí tabulku
(jinou kapacitu QAh) při každé změně znaménka proudu vneslo na validačních datech chybu
~6× vyšší (28.7 mV vs 4.6 mV RMSE), protože běžné krátké proudové špičky/regenerace
způsobovaly časté přepínání a s ním spojené skoky SOC. Nabíjecí tabulka (`chargeLut`) se
pro budoucí použití pořád načítá, jen se v `step()` aktivně nepoužívá.
"""

import bisect
import json
import math
import os

from scipy.interpolate import PchipInterpolator

TEMPERATURES_C = [5.0, 25.0, 45.0]
TEMP_KEYS = ["T005", "T025", "T045"]


def _loadLut(path):
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    lut = {}
    for key in TEMP_KEYS:
        entry = raw[key]
        soc = entry["SOC"]
        lut[key] = {
            "QAh": float(entry["QAh"]),
            "OCV": PchipInterpolator(soc, entry["OCV"]),
            "R0": PchipInterpolator(soc, entry["R0"]),
            "R1": PchipInterpolator(soc, entry["R1"]),
            "R2": PchipInterpolator(soc, entry["R2"]),
            "C1": PchipInterpolator(soc, entry["C1"]),
            "C2": PchipInterpolator(soc, entry["C2"]),
        }
    return lut


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _tempWeights(temperatureC):
    """Váhy pro lineární interpolaci mezi dvěma sousedními teplotními sadami."""
    t = _clamp(temperatureC, TEMPERATURES_C[0], TEMPERATURES_C[-1])
    if t <= TEMPERATURES_C[1]:
        lo, hi = TEMPERATURES_C[0], TEMPERATURES_C[1]
        loKey, hiKey = TEMP_KEYS[0], TEMP_KEYS[1]
    else:
        lo, hi = TEMPERATURES_C[1], TEMPERATURES_C[2]
        loKey, hiKey = TEMP_KEYS[1], TEMP_KEYS[2]
    frac = 0.0 if hi == lo else (t - lo) / (hi - lo)
    return loKey, hiKey, frac


class EcmModel:
    def __init__(self, dischargeJsonPath, chargeJsonPath, initialSocFraction=1.0,
                 epsR=1e-9, epsC=1e-12):
        self.dischargeLut = _loadLut(dischargeJsonPath) if os.path.exists(dischargeJsonPath) else None
        self.chargeLut = _loadLut(chargeJsonPath) if os.path.exists(chargeJsonPath) else None
        self.epsR = epsR
        self.epsC = epsC

        self.soc = _clamp(initialSocFraction, 0.0, 1.0)
        self.v1 = 0.0
        self.v2 = 0.0
        self.lastTemperatureC = 25.0
        self.lastMode = "discharge"
        self.lastVoltage = float("nan")
        self.lastPLossW = float("nan")

    def reset(self, initialSocFraction=1.0):
        self.soc = _clamp(initialSocFraction, 0.0, 1.0)
        self.v1 = 0.0
        self.v2 = 0.0
        self.lastVoltage = float("nan")
        self.lastPLossW = float("nan")

    def _paramsAt(self, lut, soc, temperatureC):
        loKey, hiKey, frac = _tempWeights(temperatureC)
        lo, hi = lut[loKey], lut[hiKey]

        def blend(name):
            vLo = float(lo[name](soc))
            vHi = float(hi[name](soc))
            return vLo + frac * (vHi - vLo)

        qah = lo["QAh"] + frac * (hi["QAh"] - lo["QAh"])
        return {
            "OCV": blend("OCV"), "R0": max(blend("R0"), self.epsR),
            "R1": max(blend("R1"), self.epsR), "R2": max(blend("R2"), self.epsR),
            "C1": max(blend("C1"), self.epsC), "C2": max(blend("C2"), self.epsC),
            "QAh": qah,
        }

    def step(self, currentA, dtS, temperatureC):
        """Jeden krok simulace. currentA: I>0 = vybíjení. Vrátí simulované napětí [V]
        nebo NaN, pokud chybí platný proud/teplota nebo nejsou dostupná data pro daný režim."""
        if currentA != currentA or dtS is None or dtS <= 0:
            return self.lastVoltage

        if temperatureC == temperatureC:  # není NaN
            self.lastTemperatureC = temperatureC
        temperatureC = self.lastTemperatureC

        # Režim se určuje jen pro informativní UI popisek ("Nabíjení"/"Vybíjení") —
        # parametry se berou vždy z vybíjecí tabulky, viz vysvětlení v docstringu modulu.
        self.lastMode = "discharge" if currentA >= 0 else "charge"

        lut = self.dischargeLut
        if lut is None:
            return float("nan")

        p = self._paramsAt(lut, self.soc, temperatureC)

        # Ztrátový (Jouleův) výkon za tento krok — pro FMU tepelný model. Používá STAV
        # PŘED aktualizací v1/v2, shodně s referenčním FMU/run_fmu_thermal.py.
        self.lastPLossW = (currentA ** 2 * p["R0"]
                            + self.v1 ** 2 / p["R1"] + self.v2 ** 2 / p["R2"])

        self.soc = _clamp(self.soc - currentA * dtS / (3600.0 * p["QAh"]), 0.0, 1.0)

        tau1 = max(p["R1"] * p["C1"], 1e-9)
        tau2 = max(p["R2"] * p["C2"], 1e-9)
        a1 = math.exp(-dtS / tau1)
        a2 = math.exp(-dtS / tau2)

        self.v1 = a1 * self.v1 + (1 - a1) * p["R1"] * currentA
        self.v2 = a2 * self.v2 + (1 - a2) * p["R2"] * currentA

        voltage = p["OCV"] - currentA * p["R0"] - self.v1 - self.v2
        self.lastVoltage = voltage
        return voltage

    def estimateSocFromRestVoltage(self, restVoltageV, temperatureC):
        """Odhadne SOC z klidového napětí (OCV) inverzní interpolací přes vybíjecí
        OCV(SOC) křivku při dané teplotě. Vrátí SOC v [0,1], nebo None bez vybíjecích dat."""
        if self.dischargeLut is None:
            return None

        n = 400
        socGrid = [i / n for i in range(n + 1)]
        ocvGrid = [self._paramsAt(self.dischargeLut, s, temperatureC)["OCV"] for s in socGrid]

        pairs = sorted(zip(ocvGrid, socGrid))
        ocvSorted = [p[0] for p in pairs]
        socSorted = [p[1] for p in pairs]

        if restVoltageV <= ocvSorted[0]:
            return socSorted[0]
        if restVoltageV >= ocvSorted[-1]:
            return socSorted[-1]

        idx = bisect.bisect_left(ocvSorted, restVoltageV)
        o0, o1 = ocvSorted[idx - 1], ocvSorted[idx]
        s0, s1 = socSorted[idx - 1], socSorted[idx]
        frac = 0.0 if o1 == o0 else (restVoltageV - o0) / (o1 - o0)
        return _clamp(s0 + frac * (s1 - s0), 0.0, 1.0)


# Meze pro rozpoznání "klidového" prvního vzorku — mimo ně je odhad SOC z OCV nespolehlivý.
RESTING_CURRENT_LIMIT_A = 1.0
OCV_PLAUSIBLE_MIN_V = 2.5
OCV_PLAUSIBLE_MAX_V = 4.3


def estimateInitialSocPct(model, avgFirstCellVoltage, firstNetCurrent, firstTemperatureC,
                           fallbackPct):
    """Odhadne počáteční SOC [%] z klidového napětí prvního vzorku, jinak vrátí fallbackPct.
    Vrátí (initialSocPct, wasEstimated, reason) — reason je popis, proč se odhad nepoužil."""
    plausible = (avgFirstCellVoltage == avgFirstCellVoltage
                 and OCV_PLAUSIBLE_MIN_V <= avgFirstCellVoltage <= OCV_PLAUSIBLE_MAX_V
                 and firstNetCurrent == firstNetCurrent
                 and abs(firstNetCurrent) <= RESTING_CURRENT_LIMIT_A)
    if plausible:
        estSoc = model.estimateSocFromRestVoltage(avgFirstCellVoltage, firstTemperatureC)
        if estSoc is not None:
            return estSoc * 100.0, True, None

    reason = (f"V={avgFirstCellVoltage:.3f} V, I={firstNetCurrent:.2f} A "
              f"mimo klidové meze ({OCV_PLAUSIBLE_MIN_V}–{OCV_PLAUSIBLE_MAX_V} V, "
              f"|I|≤{RESTING_CURRENT_LIMIT_A} A)")
    return fallbackPct, False, reason


def replaySeries(model, tData, netCurrentSeries, temperatureSeries):
    """Přehraje model přes sekvenci (stejné délky jako tData).
    Vrátí (ecmVoltageY, ecmSocPctY, pLossWattY) — pLossWattY je NaN tam, kde k reálnému
    kroku nedošlo (chybí dt), jinak Jouleův ztrátový výkon toho kroku (vstup pro FMU)."""
    ecmY, ecmSocY, pLossY, lastT = [], [], [], None
    for i in range(len(tData)):
        current = netCurrentSeries[i]
        temp = temperatureSeries[i] if temperatureSeries else float("nan")
        dt = None if lastT is None else tData[i] - lastT
        lastT = tData[i]
        if dt is not None and dt > 0:
            v = model.step(current, dt, temp)
            pLossY.append(model.lastPLossW)
        else:
            v = model.lastVoltage
            pLossY.append(float("nan"))
        ecmY.append(v)
        ecmSocY.append(model.soc * 100.0)
    return ecmY, ecmSocY, pLossY
