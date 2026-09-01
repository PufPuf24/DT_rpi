"""
Textový logger pro Battery Digital Twin.
Zapisuje každý měřicí cyklus do dvou tab-oddělených souborů:
    Battery_monitor_output.txt - podrobný log, každý kanál zvlášť
    BATTERY1_out.txt           - souhrnný log (sečtené napětí, mid-pack teplota)
"""

import os


class BatteryFileLogger:
    DETAIL_FILENAME = "Battery_monitor_output.txt"
    SUMMARY_FILENAME = "BATTERY1_out.txt"

    def __init__(self, batteryLabels, tempLabels, outDir=None):
        self.outDir = outDir or os.path.dirname(os.path.abspath(__file__))
        self.batteryLabels = list(batteryLabels)
        self.tempLabels = list(tempLabels)

        self.detailPath = os.path.join(self.outDir, self.DETAIL_FILENAME)
        self.summaryPath = os.path.join(self.outDir, self.SUMMARY_FILENAME)

        self._ensureHeader(self.detailPath, self._detailHeader())
        self._ensureHeader(self.summaryPath, self._summaryHeader())

    def _detailHeader(self):
        cols = ["Datum", "Cas"]
        cols += [f"{label}_V" for label in self.batteryLabels]
        cols += ["ProudIN_A", "ProudOUT_A"]
        cols += [f"T_{label}" for label in self.tempLabels]
        cols += ["Rele_IN(Photovoltaics)", "Rele_OUT(Load)"]
        return cols

    def _summaryHeader(self):
        return ["Datum", "Cas", "Napeti_Sum_V", "ProudIN_A", "ProudOUT_A",
                "T_MidPack", "Rele_OUT(Load)"]

    @staticmethod
    def _ensureHeader(path, columns):
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\t".join(columns) + "\n")

    @staticmethod
    def _fmt(value):
        if value is None or value != value:  # None nebo NaN
            return "---"
        return f"{value:.2f}"

    @staticmethod
    def _fmtVoltage(value):
        if value is None or value != value:  # None nebo NaN
            return "---"
        return f"{value:.4f}"

    @staticmethod
    def _onOff(state):
        return "ON" if state else "OFF"

    def logCycle(self, timestamp, batteryVoltages, currentIN, currentOUT,
                 temperatures, relayIN, relayOUT):
        date = timestamp.strftime("%Y/%m/%d")
        clock = timestamp.strftime("%H:%M:%S")

        detailRow = [date, clock]
        detailRow += [self._fmtVoltage(v) for v in batteryVoltages]
        detailRow += [self._fmt(currentIN), self._fmt(currentOUT)]
        detailRow += [self._fmt(t) for t in temperatures]
        detailRow += [self._onOff(relayIN), self._onOff(relayOUT)]
        self._appendRow(self.detailPath, detailRow)

        validVoltages = [v for v in batteryVoltages if v == v]
        voltageSum = sum(validVoltages) if validVoltages else float("nan")
        midPackTemp = temperatures[0] if temperatures else float("nan")
        summaryRow = [date, clock, self._fmt(voltageSum), self._fmt(currentIN),
                      self._fmt(currentOUT), self._fmt(midPackTemp), self._onOff(relayOUT)]
        self._appendRow(self.summaryPath, summaryRow)

    @staticmethod
    def _appendRow(path, row):
        with open(path, "a", encoding="utf-8") as f:
            f.write("\t".join(row) + "\n")
