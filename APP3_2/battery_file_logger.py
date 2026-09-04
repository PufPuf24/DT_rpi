"""
Textový logger pro Battery Digital Twin.
Zapisuje každý měřicí cyklus do dvou tab-oddělených souborů:
    Battery_monitor_output.txt - podrobný log, každý kanál zvlášť
    BATTERY1_out.txt           - souhrnný log (sečtené napětí, mid-pack teplota)

Volitelně ještě do třetího souboru, stejný formát jako Battery_monitor_output.txt,
ale s mnohem kratší, pevnou periodou zápisu (Battery_monitor_output_5s.txt) --
zapíná/vypíná se samostatně (setFastLoggingEnabled), viz "Data" karta v
BatteryMonitorGUI.py.
"""

import os


class BatteryFileLogger:
    DETAIL_FILENAME = "Battery_monitor_output.txt"
    SUMMARY_FILENAME = "BATTERY1_out.txt"
    FAST_FILENAME = "Battery_monitor_output_5s.txt"

    def __init__(self, batteryLabels, tempLabels, outDir=None):
        self.outDir = outDir or os.path.dirname(os.path.abspath(__file__))
        self.batteryLabels = list(batteryLabels)
        self.tempLabels = list(tempLabels)

        self.detailPath = os.path.join(self.outDir, self.DETAIL_FILENAME)
        self.summaryPath = os.path.join(self.outDir, self.SUMMARY_FILENAME)
        self.fastPath = os.path.join(self.outDir, self.FAST_FILENAME)
        self.fastEnabled = False

        self._ensureHeader(self.detailPath, self._detailHeader())
        self._ensureHeader(self.summaryPath, self._summaryHeader())

    def setFastLoggingEnabled(self, enabled):
        """Turns the fast (short-period), detail-format log on/off. The file (and
        its header) is only created the first time it's turned on, not on every
        app startup, so it doesn't appear unless someone actually asked for it."""
        self.fastEnabled = bool(enabled)
        if self.fastEnabled:
            self._ensureHeader(self.fastPath, self._detailHeader())

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

    def _detailRow(self, timestamp, batteryVoltages, currentIN, currentOUT,
                   temperatures, relayIN, relayOUT):
        row = [timestamp.strftime("%Y/%m/%d"), timestamp.strftime("%H:%M:%S")]
        row += [self._fmtVoltage(v) for v in batteryVoltages]
        row += [self._fmt(currentIN), self._fmt(currentOUT)]
        row += [self._fmt(t) for t in temperatures]
        row += [self._onOff(relayIN), self._onOff(relayOUT)]
        return row

    def logCycle(self, timestamp, batteryVoltages, currentIN, currentOUT,
                 temperatures, relayIN, relayOUT):
        detailRow = self._detailRow(timestamp, batteryVoltages, currentIN, currentOUT,
                                     temperatures, relayIN, relayOUT)
        self._appendRow(self.detailPath, detailRow)

        validVoltages = [v for v in batteryVoltages if v == v]
        voltageSum = sum(validVoltages) if validVoltages else float("nan")
        midPackTemp = temperatures[0] if temperatures else float("nan")
        summaryRow = [detailRow[0], detailRow[1], self._fmt(voltageSum), self._fmt(currentIN),
                      self._fmt(currentOUT), self._fmt(midPackTemp), self._onOff(relayOUT)]
        self._appendRow(self.summaryPath, summaryRow)

    def logFastCycle(self, timestamp, batteryVoltages, currentIN, currentOUT,
                      temperatures, relayIN, relayOUT):
        """Same row format/content as logCycle's detail row, written to the
        separate fast-log file -- no-op unless setFastLoggingEnabled(True)."""
        if not self.fastEnabled:
            return
        row = self._detailRow(timestamp, batteryVoltages, currentIN, currentOUT,
                               temperatures, relayIN, relayOUT)
        self._appendRow(self.fastPath, row)

    @staticmethod
    def _appendRow(path, row):
        with open(path, "a", encoding="utf-8") as f:
            f.write("\t".join(row) + "\n")
