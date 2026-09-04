"""
Převod odporu termistoru na teplotu [°C].

Mid pack / Case: vlastní zafitovaná log-lineární rovnice (dodaná uživatelem),
Electronics box / T1 / T2 / T3: TDK/EPCOS B57164K0103K (R25 = 10 kΩ, B25/100 = 4300 K,
R/T charakteristika č. 2904) — tabulka poměrů RT/R25 z datasheetu (dsh.118-010.1.pdf),
interpolovaná lineárně v ln(poměr) vs. teplota.
"""

import math

R25_B57164K0103 = 10_000.0  # Ω

# (teplota [°C], RT/R25) — EPCOS B57164K, R/T č. 2904, B25/100 = 4300 K
TABLE_B57164K0103 = [
    (-55, 121.46), (-50, 84.439), (-45, 59.243), (-40, 41.938), (-35, 29.947),
    (-30, 21.567), (-25, 15.641), (-20, 11.466), (-15, 8.451), (-10, 6.2927),
    (-5, 4.7077), (0, 3.5563), (5, 2.7119), (10, 2.086), (15, 1.6204),
    (20, 1.2683), (25, 1.0000), (30, 0.7942), (35, 0.63268), (40, 0.5074),
    (45, 0.41026), (50, 0.33363), (55, 0.27243), (60, 0.2237), (65, 0.18459),
    (70, 0.15305), (75, 0.12755), (80, 0.10677), (85, 0.089928), (90, 0.076068),
    (95, 0.064524), (100, 0.054941), (105, 0.047003), (110, 0.040358),
    (115, 0.034743), (120, 0.030007), (125, 0.026006),
]


def _tableToCelsius(resistanceOhm, r25Ohm, table):
    if resistanceOhm != resistanceOhm or resistanceOhm <= 0:  # NaN nebo nesmysl
        return float("nan")

    ratio = resistanceOhm / r25Ohm
    try:
        lnRatio = math.log(ratio)
    except ValueError:
        return float("nan")

    lnTable = [(t, math.log(r)) for t, r in table]

    if lnRatio >= lnTable[0][1]:
        (t0, l0), (t1, l1) = lnTable[0], lnTable[1]
    elif lnRatio <= lnTable[-1][1]:
        (t0, l0), (t1, l1) = lnTable[-2], lnTable[-1]
    else:
        t0 = l0 = t1 = l1 = None
        for i in range(len(lnTable) - 1):
            (ta, la), (tb, lb) = lnTable[i], lnTable[i + 1]
            if la >= lnRatio >= lb:
                t0, l0, t1, l1 = ta, la, tb, lb
                break
        if t0 is None:
            return float("nan")

    frac = (lnRatio - l0) / (l1 - l0)
    return t0 + frac * (t1 - t0)


def b57164k0103Celsius(resistanceOhm):
    return _tableToCelsius(resistanceOhm, R25_B57164K0103, TABLE_B57164K0103)


def customNtcCelsius(resistanceOhm):
    """Mid pack / Case — dodaná rovnice: T = ln((R[kΩ] - 0.3019) / 27.5692) / -0.0424"""
    if resistanceOhm != resistanceOhm:
        return float("nan")
    rKohm = resistanceOhm / 1000.0
    arg = (rKohm - 0.3019) / 27.5692
    if arg <= 0:
        return float("nan")
    try:
        return math.log(arg) / -0.0424
    except ValueError:
        return float("nan")


CONVERTER_BY_NAME = {
    "Mid pack": customNtcCelsius,
    "Case": customNtcCelsius,
    "Electronics box": b57164k0103Celsius,
    "T1": b57164k0103Celsius,
    "T2": b57164k0103Celsius,
    "T3": b57164k0103Celsius,
}


def resistanceToCelsius(name, resistanceOhm):
    converter = CONVERTER_BY_NAME.get(name)
    if converter is None:
        return float("nan")
    return converter(resistanceOhm)
