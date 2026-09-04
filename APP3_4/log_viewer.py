"""Historical log file viewer for the Battery Digital Twin."""

import os
from datetime import datetime
from tkinter import filedialog

import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from data_tools import parse_log_file
from ecm_model import EcmModel, estimateInitialSocPct, replaySeries
from theme import ACCENT, dual, tokens_for_mode


def _classify(columnName):
    if columnName in ("Datum", "Cas") or "Rele" in columnName:
        return None
    if columnName.endswith("_V"):
        return "voltage"
    if columnName.endswith("_A"):
        return "current"
    if columnName.startswith("T_"):
        return "temperature"
    return None


class LogViewerWindow(ctk.CTkToplevel):
    def __init__(self, master, initialDir=None, ecmDischargeJson=None, ecmChargeJson=None,
                 ecmParallelCount=1, ecmTempSourceName="T3"):
        super().__init__(master)
        self.title("Historical Log Viewer")
        self.geometry("1150x760")
        self.configure(fg_color=dual("app_bg"))
        self.initialDir = initialDir
        self.ecmParallelCount = max(1, int(ecmParallelCount))
        self.ecmTempSourceName = ecmTempSourceName

        self.ecm = None
        if (ecmDischargeJson and ecmChargeJson
                and os.path.exists(ecmDischargeJson) and os.path.exists(ecmChargeJson)):
            self.ecm = EcmModel(ecmDischargeJson, ecmChargeJson, initialSocFraction=1.0)

        topBar = ctk.CTkFrame(self, corner_radius=14, fg_color=dual("card_bg"),
                               border_width=1, border_color=dual("border"))
        topBar.pack(fill="x", padx=16, pady=16)
        ctk.CTkButton(topBar, text="📂  Open log file…", command=self.openFile).pack(
            side="left", padx=12, pady=12)
        self.fileLabel = ctk.CTkLabel(topBar, text="No file loaded.",
                                       text_color=dual("text_secondary"))
        self.fileLabel.pack(side="left", padx=12, pady=12)
        self.ecmStatusLabel = ctk.CTkLabel(topBar, text="", text_color=dual("text_secondary"))
        self.ecmStatusLabel.pack(side="left", padx=12, pady=12)

        chartFrame = ctk.CTkFrame(self, corner_radius=16, fg_color=dual("card_bg"),
                                   border_width=1, border_color=dual("border"))
        chartFrame.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.fig = Figure(dpi=100)
        gs = self.fig.add_gridspec(3, 1, hspace=0.25)
        self.axV = self.fig.add_subplot(gs[0])
        self.axI = self.fig.add_subplot(gs[1], sharex=self.axV)
        self.axR = self.fig.add_subplot(gs[2], sharex=self.axV)
        self.axV.set_ylabel("Voltage [V]")
        self.axI.set_ylabel("Current [A]")
        self.axR.set_ylabel("Temperature [°C]")
        self.axR.set_xlabel("Time")
        for ax in (self.axV, self.axI):
            ax.tick_params(labelbottom=False)
        self.fig.subplots_adjust(left=0.08, right=0.85, top=0.97, bottom=0.1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=chartFrame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=12)

        self.applyTheme()

    def applyTheme(self):
        t = tokens_for_mode(ctk.get_appearance_mode())
        self.fig.set_facecolor(t["card_bg"])
        for ax in (self.axV, self.axI, self.axR):
            ax.set_facecolor(t["card_bg"])
            ax.tick_params(colors=t["text_secondary"])
            ax.xaxis.label.set_color(t["text"])
            ax.yaxis.label.set_color(t["text"])
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.grid(True, color=t["separator"], linewidth=0.7, alpha=0.9)
            legend = ax.get_legend()
            if legend is not None:
                frame = legend.get_frame()
                frame.set_facecolor(t["card_bg"])
                frame.set_edgecolor(t["border"])
                for text in legend.get_texts():
                    text.set_color(t["text"])
        self.canvas.draw_idle()

    def openFile(self):
        path = filedialog.askopenfilename(
            title="Select a log file", initialdir=self.initialDir,
            filetypes=[("Text logs", "*.txt"), ("All files", "*.*")])
        if not path:
            return

        header, columns = parse_log_file(path)
        if "Datum" not in columns or "Cas" not in columns:
            self.fileLabel.configure(text="Invalid file format (missing Date/Time columns).")
            return

        timestamps = []
        for d, c in zip(columns["Datum"], columns["Cas"]):
            try:
                timestamps.append(datetime.strptime(f"{d} {c}", "%Y/%m/%d %H:%M:%S"))
            except ValueError:
                timestamps.append(None)

        axByGroup = {"voltage": self.axV, "current": self.axI, "temperature": self.axR}
        for ax in axByGroup.values():
            ax.clear()

        plottedAny = {"voltage": False, "current": False, "temperature": False}
        for name in header:
            group = _classify(name)
            if group is None:
                continue
            ax = axByGroup[group]
            values, validTs = [], []
            for ts, raw in zip(timestamps, columns[name]):
                if ts is None:
                    continue
                try:
                    values.append(float(raw))
                    validTs.append(ts)
                except ValueError:
                    continue
            if values:
                ax.plot(validTs, values, "-", linewidth=1.2, label=name)
                plottedAny[group] = True

        self._runEcm(header, columns, timestamps, plottedAny)

        self.axV.set_ylabel("Voltage [V]")
        self.axI.set_ylabel("Current [A]")
        self.axR.set_ylabel("Temperature [°C]")
        self.axR.set_xlabel("Time")
        for ax in (self.axV, self.axI):
            ax.tick_params(labelbottom=False)
        for group, ax in axByGroup.items():
            if plottedAny[group]:
                ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8,
                          borderaxespad=0.0, frameon=True)

        self.fig.autofmt_xdate()
        self.applyTheme()
        self.fileLabel.configure(text=os.path.basename(path))

    def _runEcm(self, header, columns, timestamps, plottedAny):
        self.ecmStatusLabel.configure(text="")
        if self.ecm is None:
            return
        tempCol = f"T_{self.ecmTempSourceName}"
        required = ("ProudIN_A", "ProudOUT_A", tempCol)
        if not all(c in columns for c in required):
            return

        validIdx = [i for i, ts in enumerate(timestamps) if ts is not None]
        if not validIdx:
            return

        def parseFloatOrNan(raw):
            try:
                return float(raw)
            except (TypeError, ValueError):
                return float("nan")

        t0 = timestamps[validIdx[0]]
        tSeconds = [(timestamps[i] - t0).total_seconds() for i in validIdx]
        currentIn = [parseFloatOrNan(columns["ProudIN_A"][i]) for i in validIdx]
        currentOut = [parseFloatOrNan(columns["ProudOUT_A"][i]) for i in validIdx]
        ecmTempSeries = [parseFloatOrNan(columns[tempCol][i]) for i in validIdx]
        # pack current / number of parallel branches -- the ECM model is per 1 cell/branch
        netCurrent = [((currentOut[k] - currentIn[k]) / self.ecmParallelCount)
                      if (currentIn[k] == currentIn[k] and currentOut[k] == currentOut[k])
                      else float("nan") for k in range(len(tSeconds))]

        battCols = [c for c in header if c.endswith("_V")]
        firstVoltages = [parseFloatOrNan(columns[c][validIdx[0]]) for c in battCols]
        validFirstV = [v for v in firstVoltages if v == v]
        avgFirstV = sum(validFirstV) / len(validFirstV) if validFirstV else float("nan")
        firstTemp = (ecmTempSeries[0] if ecmTempSeries and ecmTempSeries[0] == ecmTempSeries[0]
                     else 25.0)

        initialSocPct, wasEstimated, reason = estimateInitialSocPct(
            self.ecm, avgFirstV, netCurrent[0], firstTemp, fallbackPct=100.0)
        self.ecm.reset(initialSocFraction=initialSocPct / 100.0)

        ecmY, ecmSocY, _pLossY = replaySeries(self.ecm, tSeconds, netCurrent, ecmTempSeries)
        ecmTimestamps = [timestamps[i] for i in validIdx]
        self.axV.plot(ecmTimestamps, ecmY, "--", linewidth=1.6, color=ACCENT,
                       label="ECM sim (1 cell)")
        plottedAny["voltage"] = True

        socNote = (f"initial SOC estimated at {initialSocPct:.1f} % from rest voltage"
                   if wasEstimated else f"initial SOC {initialSocPct:.1f} % (default — {reason})")
        self.ecmStatusLabel.configure(
            text=f"ECM: {socNote}  ·  final SOC {self.ecm.soc*100:.1f} %")
