"""
Battery Digital Twin -- with ECM voltage simulation and a pack thermal model.
Python translation/extension of BatteryMonitorGUI.m

Dependencies:
    pip install pyserial matplotlib customtkinter scipy fmpy
"""

import bisect
import copy
import math
import os
import queue
import threading
import time
from datetime import datetime
from tkinter import filedialog, messagebox

import customtkinter as ctk
import matplotlib
import numpy as np
import serial
import serial.tools.list_ports

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config
from battery_file_logger import BatteryFileLogger
from cutoff_logic import decide_relay_action
from data_tools import export_session_csv, parse_log_file
from theme import ACCENT, DARK, GRAY, GREEN, LIGHT, ORANGE, RED, YELLOW, dual, tokens_for_mode
from thermistor import resistanceToCelsius
from ecm_model import EcmModel, estimateInitialSocPct, replaySeries
from pack_rom_thermal_model import PackRomThermalWorker as FmuThermalWorker, N_CELLS as FMU_N_CELLS
from fmu_detail_window import FmuDetailWindow
from log_window import LogWindow

ECM_DISCHARGE_JSON = "P_discharge_validated.json"
ECM_CHARGE_JSON = "P_charge_new.json"
FMU_THERMAL_FILE = "pack_rom.npz"  # Python ROM (NumPy) -- replacement for Pack_ROM_SML_ME.fmu,
                                   # see pack_rom_thermal_model.py and C:\code\ROM_pack\README.md
FMU_MIN_REFRESH_S = 1.0  # the Python ROM is cheap (~15 us/sample) -- 1 s is the worker's
                        # own hard floor (see PackRomThermalWorker._loop), not a real limit

# Absolute per-cell operating window -- damage limits, not the (tighter) automatic-cutoff
# thresholds in config.CELL_MIN_SAFE_V/CELL_MAX_SAFE_V. Drawn as red reference lines and
# used to flag when an Estimation projection would run past them.
CELL_ABS_V_MIN = 2.6
CELL_ABS_V_MAX = 4.2
CELL_ABS_T_MIN = -30.0
CELL_ABS_T_MAX = 55.0

# Battery Health page -- coloring thresholds for how far a series group's whole-session
# average voltage sits from the pack's own average (see _refreshHealthPage). Cosmetic
# only, not a safety cutoff -- picked to flag drift worth a look, not to be alarming.
BALANCE_WARN_PCT = 0.5
BALANCE_CRIT_PCT = 1.5

# Estimation ("digital twin") forward projection
ESTIMATION_HORIZONS_H = [1, 2, 4, 8, 12, 24]
ESTIMATION_DEFAULT_H = 2
ESTIMATION_DT_S = 10.0          # projection sample spacing -- smooth without being wasteful
ESTIMATION_RECOMPUTE_MS = 2000  # how often the projection refreshes while switched on

BAUD_RATE = 115200
MAX_POINTS = 1000
MAX_LOG_LINES = 200
FILE_LOG_PERIOD = 30.0  # s, how often a line gets written to the text log files

MIN_POLL_PERIOD = 0.1   # s, lower bound of Fast mode
MAX_POLL_PERIOD = 60.0  # s, upper bound of Slow mode
DEFAULT_POLL_PERIOD = 1.0
SLOW_THRESHOLD = 1.0    # s, from this step (inclusive) MEASure switches to Slow; also the
                        # exact boundary between the Fast/Slow slider ranges in the UI

# Physical ranges matching the range parameter in the command (15V / 0V15 / 200k) -- a
# reply outside this range (even with margin) is a bad reading / misidentified port,
# not a real value, so it's dropped to NaN instead of corrupting the chart.
BATTERY_RANGE_V = 15.0
CURRENT_SHUNT_RANGE_V = 0.15   # the "0V15" range on CH13/CH14 (shunt)
RESIST_RANGE_OHM = 200_000.0
RANGE_MARGIN = 1.1

# A "Slow" measurement takes longer on the instrument than "Fast" -- too short a timeout
# would make readline() cut off mid-reply. The remainder would then be read as part of
# the next query for a DIFFERENT channel, splicing two different replies into one
# nonsensical number (see the merged "321010.375" case).
FAST_READ_TIMEOUT = 0.4
SLOW_READ_TIMEOUT = 2.0

# Gap before EVERY command (including outside the automatic cycle -- relay SET, manual
# console...). The firmware (HAL_ADC.ino) switches the channel multiplexer and immediately
# starts a sigma-delta ADC conversion without discarding the first (still "contaminated" by
# the previous channel) sample. Manual queries from the console work reliably because
# there's naturally enough settling time between clicks; the automatic cycle runs at the
# firmware's minimum with no margin and can occasionally read a value still influenced by
# the previous channel (typically shows up as "channel X reads channel Y's data").
INTER_COMMAND_DELAY = 0.05


def parseNumeric(s, rangeLimit=None):
    """Parse an instrument reply to float; return None for a non-numeric, infinite, or
    physically nonsensical value (outside the expected measurement range)."""
    try:
        value = float(s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    if rangeLimit is not None and abs(value) > rangeLimit:
        return None
    return value


class MonitorApp:
    def __init__(self, root: ctk.CTk):
        self.root = root
        self.root.title("Battery Digital Twin")
        self.root.geometry("1440x840+60+40")
        self.root.minsize(1150, 680)
        self.root.configure(fg_color=dual("app_bg"))

        # -- Channel configuration (persistent, editable on the Settings page) --
        cfg = config.load_config()
        self.batteryChannels = list(cfg["battery_channels"])
        self.currentChannels = [(cfg["current_channels"]["IN"], "I_IN"),
                                 (cfg["current_channels"]["OUT"], "I_OUT")]
        self.shuntOhms = float(cfg["shunt_ohms"])
        self.resistChannels = [(ch, name) for ch, name in cfg["resist_channels"]]
        self.relayInCh = cfg["relay_channels"]["IN"]
        self.relayOutCh = cfg["relay_channels"]["OUT"]
        self.relayChannels = [(self.relayInCh, "IN (Photovoltaics)"), (self.relayOutCh, "Load")]
        self.cutoffOffV = float(cfg["cutoff_off_v"])
        self.cutoffOnV = float(cfg["cutoff_on_v"])
        self.cutoffEnabled = bool(cfg["cutoff_enabled"])
        self.cutoffInitialState = bool(cfg["cutoff_initial_state"])
        self._cutoffInitialApplied = False

        # State variables
        self.sPort = None
        self.portLock = threading.Lock()
        self.tStart = None
        self.stopEvent = None
        self.workerThread = None
        self.dataQueue = queue.Queue()
        self.autoEventQueue = queue.Queue()
        self.measuring = False
        self.relayState = {ch: False for ch, _ in self.relayChannels}
        self.pollPeriod = DEFAULT_POLL_PERIOD
        self._portLost = False

        self.tData = []
        self.battY = [[] for _ in self.batteryChannels]
        self.currY = [[] for _ in self.currentChannels]
        self.resY = [[] for _ in self.resistChannels]
        self.ecmY = []
        self.ecmSocY = []
        self.ecmPLossY = []

        self.scriptDir = os.path.dirname(os.path.abspath(__file__))
        self.ecmInitialSocPct = float(cfg.get("ecm_initial_soc_pct", 100.0))
        self.ecmParallelCount = max(1, int(cfg.get("ecm_parallel_count", 3)))
        self.ecmTempSourceName = cfg.get("ecm_temp_source", "T3")
        self._updateEcmTempSourceIndex()
        self.ecm = EcmModel(os.path.join(self.scriptDir, ECM_DISCHARGE_JSON),
                             os.path.join(self.scriptDir, ECM_CHARGE_JSON),
                             initialSocFraction=self.ecmInitialSocPct / 100.0)
        self._ecmLastTNow = None
        self._lastNetCurrent = float("nan")  # net current last fed to the ECM (per 1
                                             # cell/branch -- divided by ecmParallelCount),
                                             # from whichever source (live/loadTestFile/
                                             # loadCurrentProfile) -- see Estimation below
        self._lastDisplayCurrent = float("nan")  # same moment, but at the SAME SCALE as
                                             # currY/currLines (not divided -- or, for
                                             # loadCurrentProfile with a per-branch file,
                                             # not multiplied either). Used only so the
                                             # current-projection line holds exactly the
                                             # value the chart was already showing --
                                             # _lastNetCurrent is a different physical
                                             # quantity (the ECM's own per-branch current)
                                             # and would visibly jump on the chart.

        # -- Estimation ("what if this current keeps up") -- a forward projection of
        # voltage/temperature, off by default. See _runEstimation. --
        self.estimationOn = False
        self.estimationHorizonH = ESTIMATION_DEFAULT_H
        self._estimationAfterId = None
        self.customVMin = float(cfg.get("custom_v_min", CELL_ABS_V_MIN))
        self.customVMax = float(cfg.get("custom_v_max", CELL_ABS_V_MAX))
        self.customTMin = float(cfg.get("custom_t_min", CELL_ABS_T_MIN))
        self.customTMax = float(cfg.get("custom_t_max", CELL_ABS_T_MAX))

        # -- Pack thermal model (24 cells, computed on a background thread). Always on
        # when the model file is available -- no user switch, see _buildControlPage. --
        self.fmuResultQueue = queue.Queue()
        self.fmuThermal = FmuThermalWorker(os.path.join(self.scriptDir, FMU_THERMAL_FILE),
                                            self.fmuResultQueue, minRefreshIntervalS=FMU_MIN_REFRESH_S)
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]
        self._fmuLastStatusText = "Not started"
        self.fmuDetailWindow = None

        self.fileLogger = BatteryFileLogger(
            batteryLabels=[f"B{i+1:02d}" for i in range(len(self.batteryChannels))],
            tempLabels=[name for _, name in self.resistChannels])
        self._lastFileLogTime = 0.0

        # -- Debug log: a small always-on buffer, viewed on demand (see openLogWindow) --
        self._logLines = ["Application ready.", 'Click "Scan COM ports".']
        self.logWindow = None

        self._buildUI()
        self.root.protocol("WM_DELETE_WINDOW", self.onClose)
        self.applyPlotTheme()
        self._pollQueue()

    # ------------------------------------------------------------------
    def _buildUI(self):
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

        self._buildHeader()

        # -- Top-level page switch (see the segmented button in the header): the
        # original sidebar+chart layout is now "Monitoring", living in its own frame
        # so a second page ("Battery Health") can occupy the exact same grid cell and
        # be swapped in/out with .grid()/.grid_remove(), same pattern as the sidebar's
        # own Controls/Settings switch (_switchSidebarPage). --
        self.monitoringPage = ctk.CTkFrame(self.root, fg_color="transparent")
        self.monitoringPage.grid_columnconfigure(1, weight=1)
        self.monitoringPage.grid_rowconfigure(0, weight=1)

        self.healthPage = ctk.CTkFrame(self.root, fg_color="transparent")
        self.healthPage.grid_columnconfigure(0, weight=1)
        self.healthPage.grid_rowconfigure(1, weight=1)

        self._pages = {"Monitoring": self.monitoringPage, "Battery Health": self.healthPage}
        self.monitoringPage.grid(row=1, column=0, columnspan=2, sticky="nsew")

        self._buildChartArea()
        self._buildSidebar()
        self._buildHealthPage(self.healthPage)

    # ---- HEADER ---------------------------------------------------------
    def _buildHeader(self):
        header = ctk.CTkFrame(self.root, corner_radius=16, fg_color=dual("card_bg"),
                               border_width=1, border_color=dual("border"))
        header.grid(row=0, column=0, columnspan=2, sticky="new", padx=16, pady=(16, 8))
        header.grid_columnconfigure(2, weight=1)

        titleBox = ctk.CTkFrame(header, fg_color="transparent")
        titleBox.grid(row=0, column=0, padx=20, pady=14, sticky="w")
        ctk.CTkLabel(titleBox, text="🔋  Battery Digital Twin",
                     font=ctk.CTkFont(size=19, weight="bold"),
                     text_color=dual("text")).pack(anchor="w")
        ctk.CTkLabel(titleBox, text="Live measurement, ECM voltage and pack thermal simulation",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary")).pack(anchor="w")

        # -- Top-level page switch -- swaps the whole sidebar+chart area for the pack
        # health view, see _buildUI/_switchPage. Same widget/pattern as the sidebar's
        # own Controls/Settings switch. --
        self.pageSwitcher = ctk.CTkSegmentedButton(header, values=["Monitoring", "Battery Health"],
                                                    command=self._switchPage)
        self.pageSwitcher.set("Monitoring")
        self.pageSwitcher.grid(row=0, column=1, padx=20, pady=14)

        rightBar = ctk.CTkFrame(header, fg_color="transparent")
        rightBar.grid(row=0, column=2, sticky="e", padx=20, pady=12)

        # -- Estimation: forward projection of voltage/temperature, see _runEstimation.
        # Anchored to the LEFT edge of rightBar so it sits right after the title, ahead
        # of the always-there icons/theme/status on the far right. --
        self.estimationVar = ctk.BooleanVar(value=False)
        self.estimationSwitch = ctk.CTkSwitch(rightBar, text="Estimation",
                                              variable=self.estimationVar,
                                              onvalue=True, offvalue=False,
                                              progress_color=ORANGE, text_color=dual("text"),
                                              command=self._onEstimationToggle)
        self.estimationSwitch.pack(side="left", padx=(0, 8))

        self.estimationHorizonMenu = ctk.CTkOptionMenu(
            rightBar, values=[f"{h} h" for h in ESTIMATION_HORIZONS_H], width=68,
            command=self._onEstimationHorizonChange)
        self.estimationHorizonMenu.set(f"{self.estimationHorizonH} h")
        # only packed while estimation is on -- see _onEstimationToggle

        resolved = ctk.get_appearance_mode()
        self.appearanceSwitch = ctk.CTkSegmentedButton(
            rightBar, values=["Light", "Dark"], command=self.setAppearance)
        self.appearanceSwitch.set(resolved)
        self.appearanceSwitch.pack(side="right")

        ctk.CTkLabel(rightBar, text="Theme", font=ctk.CTkFont(size=12),
                     text_color=dual("text_secondary")).pack(side="right", padx=(10, 6))

        toolGroup = ctk.CTkFrame(rightBar, fg_color=dual("card_bg_alt"), corner_radius=8)
        toolGroup.pack(side="right", padx=(0, 14))
        for icon, tip, cmd in (
            ("⌨", "SCPI console", self.openScpiConsole),
            ("📁", "Historical log viewer", self.openLogViewer),
            ("📝", "Debug log", self.openLogWindow),
        ):
            ctk.CTkButton(toolGroup, text=icon, width=36, fg_color="transparent",
                         hover_color=dual("card_bg"), text_color=dual("text"),
                         command=cmd).pack(side="left", padx=2, pady=2)

        self.statusPill = ctk.CTkLabel(rightBar, text="Disconnected", corner_radius=12,
                                        fg_color=GRAY, text_color="white",
                                        font=ctk.CTkFont(size=12, weight="bold"),
                                        height=26, width=118)
        self.statusPill.pack(side="right", padx=(0, 10))

    # ---- SIDEBAR ----------------------------------------------------------
    def _buildSidebar(self):
        sidebarCard = ctk.CTkFrame(self.monitoringPage, width=330, corner_radius=16,
                                    fg_color=dual("card_bg"), border_width=1,
                                    border_color=dual("border"))
        sidebarCard.grid(row=0, column=0, sticky="nsw", padx=(16, 8), pady=(0, 16))
        sidebarCard.grid_propagate(False)
        sidebarCard.grid_rowconfigure(1, weight=1)
        sidebarCard.grid_columnconfigure(0, weight=1)

        switcher = ctk.CTkSegmentedButton(sidebarCard, values=["Controls", "Settings"],
                                           command=self._switchSidebarPage)
        switcher.set("Controls")
        switcher.grid(row=0, column=0, sticky="ew", padx=16, pady=16)
        self.sidebarSwitcher = switcher

        self.controlPage = ctk.CTkScrollableFrame(sidebarCard, fg_color="transparent")
        self.settingsPage = ctk.CTkScrollableFrame(sidebarCard, fg_color="transparent")
        self._sidebarPages = {"Controls": self.controlPage, "Settings": self.settingsPage}
        self.controlPage.grid(row=1, column=0, sticky="nsew", padx=(10, 6), pady=(0, 16))

        self._buildControlPage(self.controlPage)
        self._buildSettingsPage(self.settingsPage)

    def _switchSidebarPage(self, value):
        for name, page in self._sidebarPages.items():
            if name == value:
                page.grid(row=1, column=0, sticky="nsew", padx=(10, 6), pady=(0, 16))
            else:
                page.grid_remove()

    def _switchPage(self, value):
        for name, page in self._pages.items():
            if name == value:
                page.grid(row=1, column=0, columnspan=2, sticky="nsew")
            else:
                page.grid_remove()
        if value == "Battery Health":
            self._refreshHealthPage()

    def _card(self, parent, title, expanded=True, badge_text=None):
        """Collapsible card -- clicking the header (chevron/title) expands/collapses the
        body. Returns the `body` frame to pack content into, same as the old *Frame
        pattern. Keeps the sidebar short without touching each card's own content --
        just its outer wrapper. When `badge_text` is given, a small pill (e.g. the
        current value) is created in the header, available as `body.badge`."""
        outer = ctk.CTkFrame(parent, corner_radius=12, fg_color=dual("card_bg_alt"))
        outer.pack(fill="x", pady=(0, 10))

        header = ctk.CTkFrame(outer, fg_color="transparent", cursor="hand2")
        header.pack(fill="x", padx=12, pady=(10, 4 if expanded else 10))

        chevron = ctk.CTkLabel(header, text=("▾" if expanded else "▸"), width=14,
                               font=ctk.CTkFont(size=12), text_color=dual("text_secondary"))
        chevron.pack(side="left")
        titleLbl = ctk.CTkLabel(header, text=title, font=ctk.CTkFont(size=13, weight="bold"),
                                text_color=dual("text"), anchor="w")
        titleLbl.pack(side="left", padx=(4, 0), fill="x", expand=True)

        body = ctk.CTkFrame(outer, fg_color="transparent")
        if badge_text is not None:
            body.badge = ctk.CTkLabel(header, text=badge_text, corner_radius=10,
                                      fg_color=ACCENT, text_color="white",
                                      font=ctk.CTkFont(size=11, weight="bold"),
                                      width=64, height=22)
            body.badge.pack(side="right")
        if expanded:
            body.pack(fill="x")

        def toggle(_event=None):
            if body.winfo_ismapped():
                body.pack_forget()
                chevron.configure(text="▸")
                header.pack_configure(pady=(10, 10))
            else:
                body.pack(fill="x")
                chevron.configure(text="▾")
                header.pack_configure(pady=(10, 4))

        for w in (header, chevron, titleLbl):
            w.bind("<Button-1>", toggle)
        return body

    def _buildControlPage(self, parent):
        # -- Connection --
        connFrame = self._card(parent, "Connection", expanded=True)

        self.btnScan = ctk.CTkButton(connFrame, text="🔍 Scan COM ports",
                                      command=self.scanPorts)
        self.btnScan.pack(fill="x", padx=12, pady=4)

        self.ddPorts = ctk.CTkComboBox(connFrame, values=["(Scan for ports first)"],
                                        state="disabled", fg_color=dual("card_bg"),
                                        text_color=dual("text"),
                                        dropdown_fg_color=dual("card_bg"),
                                        dropdown_text_color=dual("text"),
                                        dropdown_hover_color=dual("card_bg_alt"))
        self.ddPorts.set("(Scan for ports first)")
        self.ddPorts.pack(fill="x", padx=12, pady=4)

        self.btnConnect = ctk.CTkButton(connFrame, text="🔌  Connect",
                                         state="disabled", command=self.toggleConnection)
        self.btnConnect.pack(fill="x", padx=12, pady=4)
        self._btnConnectDefaults = dict(fg_color=self.btnConnect.cget("fg_color"),
                                         border_width=0,
                                         text_color=self.btnConnect.cget("text_color"))

        self.btnStart = ctk.CTkButton(connFrame, text="▶  Start measurement",
                                       state="disabled", command=self.toggleMeasurement)
        self.btnStart.pack(fill="x", padx=12, pady=(4, 12))
        self._btnStartDefaults = dict(fg_color=self.btnStart.cget("fg_color"),
                                       border_width=0,
                                       text_color=self.btnStart.cget("text_color"))

        # -- Sampling speed --
        self._speedMode = "Slow"
        self._lastFastPeriod = 0.3
        self._lastSlowPeriod = DEFAULT_POLL_PERIOD

        speedFrame = self._card(parent, "Sampling speed", expanded=False,
                                badge_text="1.00 s")
        self.speedPill = speedFrame.badge

        self.speedModeSwitch = ctk.CTkSegmentedButton(
            speedFrame, values=["Slow (accurate)", "Fast"], command=self._onSpeedModeChange)
        self.speedModeSwitch.set("Slow (accurate)")
        self.speedModeSwitch.pack(fill="x", padx=12, pady=(2, 8))

        self.speedWarningLabel = ctk.CTkLabel(
            speedFrame,
            text="⚠ Fast mode: less time for the A/D converter to settle per channel — "
                 "readings may be less accurate.",
            font=ctk.CTkFont(size=10), text_color=ORANGE,
            wraplength=260, justify="left")
        # packed only when switching to "Fast", see _applySpeedMode

        valueRow = ctk.CTkFrame(speedFrame, fg_color="transparent")
        valueRow.pack(fill="x", padx=12, pady=(0, 2))
        ctk.CTkLabel(valueRow, text="Step:", font=ctk.CTkFont(size=12),
                     text_color=dual("text_secondary")).pack(side="left")
        ctk.CTkLabel(valueRow, text="s", font=ctk.CTkFont(size=12),
                     text_color=dual("text_secondary")).pack(side="right", padx=(2, 0))
        self.speedEntry = ctk.CTkEntry(valueRow, width=70, justify="right")
        self.speedEntry.pack(side="right")
        self.speedEntry.bind("<Return>", self._onSpeedEntryCommit)
        self.speedEntry.bind("<FocusOut>", self._onSpeedEntryCommit)

        self.speedSlider = ctk.CTkSlider(speedFrame, from_=SLOW_THRESHOLD, to=MAX_POLL_PERIOD,
                                          progress_color=ACCENT, command=self._onSpeedSliderMove)
        self.speedSlider.pack(fill="x", padx=12, pady=(4, 6))

        self.speedHintLabel = ctk.CTkLabel(
            speedFrame, text="", font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
            wraplength=260, justify="left")
        self.speedHintLabel.pack(anchor="w", padx=12, pady=(0, 10))

        self._applySpeedMode("Slow", DEFAULT_POLL_PERIOD)

        # -- Relays --
        relayFrame = self._card(parent, "Relays", expanded=False)

        self.relaySwitches = {}
        for ch, name in self.relayChannels:
            var = ctk.BooleanVar(value=False)
            sw = ctk.CTkSwitch(relayFrame, text=f"CH{ch} — {name}", variable=var,
                                onvalue=True, offvalue=False, state="disabled",
                                progress_color=GREEN, text_color=dual("text"),
                                command=lambda c=ch, v=var: self.toggleRelay(c, v))
            sw.pack(fill="x", padx=12, pady=6)
            self.relaySwitches[ch] = (sw, var)

        ctk.CTkLabel(relayFrame, text="Only active while connected.",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary"),
                     wraplength=260, justify="left").pack(anchor="w", padx=12, pady=(0, 10))

        # -- Automatic OUT (Load) cutoff --
        cutoffFrame = self._card(parent, "Automatic OUT (Load) cutoff", expanded=False)

        self.cutoffVoltageLabel = ctk.CTkLabel(cutoffFrame, text="Current voltage sum: --- V",
                                                font=ctk.CTkFont(size=13, weight="bold"),
                                                text_color=dual("text_secondary"))
        self.cutoffVoltageLabel.pack(anchor="w", padx=12, pady=(0, 8))

        self.cutoffSwitchVar = ctk.BooleanVar(value=self.cutoffEnabled)
        self.cutoffSwitch = ctk.CTkSwitch(cutoffFrame, text="Enable sequence",
                                           variable=self.cutoffSwitchVar,
                                           onvalue=True, offvalue=False,
                                           progress_color=ORANGE, text_color=dual("text"),
                                           command=self._toggleCutoff)
        self.cutoffSwitch.pack(fill="x", padx=12, pady=4)

        self.cutoffInitialVar = ctk.BooleanVar(value=self.cutoffInitialState)
        self.cutoffInitialSwitch = ctk.CTkSwitch(cutoffFrame, text="OUT on immediately when the sequence starts",
                                                  variable=self.cutoffInitialVar,
                                                  onvalue=True, offvalue=False,
                                                  progress_color=GREEN, text_color=dual("text"),
                                                  command=self._saveCutoffInitialState)
        self.cutoffInitialSwitch.pack(fill="x", padx=12, pady=(0, 4))
        ctk.CTkLabel(cutoffFrame,
                     text="OUT relay state at the moment the sequence is activated, until the "
                          "voltage first crosses one of the thresholds.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        thresholdRow = ctk.CTkFrame(cutoffFrame, fg_color="transparent")
        thresholdRow.pack(fill="x", padx=12, pady=(6, 2))
        ctk.CTkLabel(thresholdRow, text="Turn off below (V)", font=ctk.CTkFont(size=11),
                     text_color=dual("text_secondary")).grid(row=0, column=0, sticky="w")
        ctk.CTkLabel(thresholdRow, text="Turn on above (V)", font=ctk.CTkFont(size=11),
                     text_color=dual("text_secondary")).grid(row=0, column=1, sticky="w",
                                                              padx=(12, 0))
        self.cutoffOffEntry = ctk.CTkEntry(thresholdRow, width=104)
        self.cutoffOffEntry.insert(0, f"{self.cutoffOffV:.1f}")
        self.cutoffOffEntry.grid(row=1, column=0, sticky="w", pady=2)
        self.cutoffOnEntry = ctk.CTkEntry(thresholdRow, width=104)
        self.cutoffOnEntry.insert(0, f"{self.cutoffOnV:.1f}")
        self.cutoffOnEntry.grid(row=1, column=1, sticky="w", padx=(12, 0), pady=2)

        ctk.CTkButton(cutoffFrame, text="Apply thresholds", command=self._applyCutoffThresholds).pack(
            fill="x", padx=12, pady=(8, 4))

        nCells = len(self.batteryChannels)
        safeMin = nCells * config.CELL_MIN_SAFE_V
        safeMax = nCells * config.CELL_MAX_SAFE_V
        ctk.CTkLabel(cutoffFrame,
                     text=f"Safe range for {nCells} cells: {safeMin:.1f}–{safeMax:.1f} V "
                          f"({config.CELL_MIN_SAFE_V:.1f}–{config.CELL_MAX_SAFE_V:.1f} V/cell). "
                          f"IN (PV) is never cut off automatically.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        self.cutoffStatusLabel = ctk.CTkLabel(cutoffFrame, text="", font=ctk.CTkFont(size=11),
                                               text_color=dual("text_secondary"),
                                               wraplength=270, justify="left")
        self.cutoffStatusLabel.pack(anchor="w", padx=12, pady=(0, 10))

        # -- ECM model (1 cell) --
        ecmFrame = self._card(parent, "ECM model (1 cell, no Kalman filter)", expanded=False)

        self.ecmSocLabel = ctk.CTkLabel(ecmFrame, text=f"SOC: {self.ecm.soc*100:.1f} %",
                                         font=ctk.CTkFont(size=13, weight="bold"),
                                         text_color=dual("text_secondary"))
        self.ecmSocLabel.pack(anchor="w", padx=12, pady=(0, 2))
        self.ecmVoltageLabel = ctk.CTkLabel(ecmFrame, text="Simulated voltage: --- V",
                                             font=ctk.CTkFont(size=12),
                                             text_color=dual("text_secondary"))
        self.ecmVoltageLabel.pack(anchor="w", padx=12, pady=(0, 2))
        self.ecmModeLabel = ctk.CTkLabel(ecmFrame, text="Mode: —  ·  Model temperature: --- °C",
                                          font=ctk.CTkFont(size=11),
                                          text_color=dual("text_secondary"))
        self.ecmModeLabel.pack(anchor="w", padx=12, pady=(0, 8))

        ecmRow = ctk.CTkFrame(ecmFrame, fg_color="transparent")
        ecmRow.pack(fill="x", padx=12, pady=(0, 4))
        ctk.CTkLabel(ecmRow, text="Initial SOC (%)", font=ctk.CTkFont(size=11),
                     text_color=dual("text_secondary")).pack(side="left")
        self.ecmInitialSocEntry = ctk.CTkEntry(ecmRow, width=80)
        self.ecmInitialSocEntry.insert(0, f"{self.ecmInitialSocPct:.0f}")
        self.ecmInitialSocEntry.pack(side="right")

        ctk.CTkButton(ecmFrame, text="🔄  Reset ECM", fg_color="transparent",
                      border_width=1, border_color=dual("border"), text_color=dual("text"),
                      command=self.resetEcm).pack(fill="x", padx=12, pady=(4, 10))

        # -- Pack thermal model (24 cells) -- always on when the model is available;
        # no user switch needed since the Python ROM is cheap enough to just always run.
        # See the chart's own 📈 icon (top-right, _buildChartArea) for the 24-cell detail.
        fmuFrame = self._card(parent, "Thermal model (24 cells)", expanded=False)

        if not self.fmuThermal.available:
            ctk.CTkLabel(fmuFrame, text=f"Unavailable: {self.fmuThermal.unavailableReason}",
                         font=ctk.CTkFont(size=10), text_color=RED,
                         wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(4, 4))

        self.fmuStatusLabel = ctk.CTkLabel(fmuFrame, text=f"Status: {self._fmuLastStatusText}",
                                            font=ctk.CTkFont(size=11),
                                            text_color=dual("text_secondary"),
                                            wraplength=270, justify="left")
        self.fmuStatusLabel.pack(anchor="w", padx=12, pady=(4, 4))

        ctk.CTkLabel(fmuFrame,
                     text="Always on while the model is available. Runs in the background "
                          "(fractions of a second per cycle) and updates about once a second "
                          "during a live run. Hide the curves from the chart legend if you "
                          "don't want to see them.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 10))

        # -- Data --
        dataFrame = self._card(parent, "Data", expanded=False)
        ctk.CTkButton(dataFrame, text="🗑  Clear data and chart", fg_color="transparent",
                      border_width=1, border_color=dual("border"), text_color=dual("text"),
                      command=self.clearData).pack(fill="x", padx=12, pady=4)
        ctk.CTkButton(dataFrame, text="📤  Export session to CSV", fg_color="transparent",
                      border_width=1, border_color=dual("border"), text_color=dual("text"),
                      command=self.exportSessionCsv).pack(fill="x", padx=12, pady=4)
        ctk.CTkButton(dataFrame, text="🖼  Export chart as image", fg_color="transparent",
                      border_width=1, border_color=dual("border"), text_color=dual("text"),
                      command=self.exportChartImage).pack(fill="x", padx=12, pady=4)

        self.btnLoadTestFile = ctk.CTkButton(
            dataFrame, text="📂  Load test data", fg_color="transparent",
            border_width=1, border_color=dual("border"), text_color=dual("text"),
            command=self.loadTestFile)
        self.btnLoadTestFile.pack(fill="x", padx=12, pady=(4, 4))
        ctk.CTkLabel(dataFrame,
                     text="Loads a saved .txt log (Battery_monitor_output.txt format) instead "
                          "of a live run — for checking the chart and the ECM model. Only "
                          "available while no measurement is running.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 12))

        self.btnLoadCurrentProfile = ctk.CTkButton(
            dataFrame, text="⚡  Load current profile (test)", fg_color="transparent",
            border_width=1, border_color=dual("border"), text_color=dual("text"),
            command=self.loadCurrentProfile)
        self.btnLoadCurrentProfile.pack(fill="x", padx=12, pady=(4, 4))
        ctk.CTkLabel(dataFrame,
                     text="Loads just a current-over-time profile (two columns: time [s], "
                          "current [A] — same format as EV_minus.txt) and runs it through the "
                          "ECM + thermal model. Asks whether it's the WHOLE PACK current "
                          "(divided by the branch count) or already PER-CELL (e.g. "
                          "EV_minus.txt — used unchanged); mixing this up throws P_loss off by "
                          "about 9×. Cell voltages and sensor temperatures aren't shown, the "
                          "file doesn't contain them.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 12))

    def _buildSettingsPage(self, parent):
        ctk.CTkLabel(parent, text="Channel mapping requires an app restart after saving.",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=4, pady=(4, 10))

        mapFrame = ctk.CTkFrame(parent, corner_radius=12, fg_color=dual("card_bg_alt"))
        mapFrame.pack(fill="x", pady=(0, 12))
        ctk.CTkLabel(mapFrame, text="Channels", font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=12, pady=(10, 6))

        self.settingsEntries = {}

        def addRow(container, label, key, value):
            row = ctk.CTkFrame(container, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=3)
            ctk.CTkLabel(row, text=label, width=160, anchor="w",
                         text_color=dual("text_secondary"),
                         font=ctk.CTkFont(size=11)).pack(side="left")
            entry = ctk.CTkEntry(row, width=80)
            entry.insert(0, str(value))
            entry.pack(side="right")
            self.settingsEntries[key] = entry

        addRow(mapFrame, "Battery — first CH", "batt_start", self.batteryChannels[0])
        addRow(mapFrame, "Battery — cell count", "batt_count", len(self.batteryChannels))
        addRow(mapFrame, "Current IN — CH", "curr_in", self.currentChannels[0][0])
        addRow(mapFrame, "Current OUT — CH", "curr_out", self.currentChannels[1][0])
        addRow(mapFrame, "Shunt (mΩ)", "shunt_mohm", self.shuntOhms * 1000)
        ctk.CTkLabel(mapFrame, text="", height=2).pack()

        for ch, name in self.resistChannels:
            addRow(mapFrame, f"Temperature “{name}” — CH", f"temp_{ch}", ch)
        ctk.CTkLabel(mapFrame, text="", height=6).pack()

        relayMapFrame = ctk.CTkFrame(parent, corner_radius=12, fg_color=dual("card_bg_alt"))
        relayMapFrame.pack(fill="x", pady=(0, 12))
        ctk.CTkLabel(relayMapFrame, text="Relays", font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=12, pady=(10, 6))
        addRow(relayMapFrame, "IN (Photovoltaics) — CH", "relay_in", self.relayInCh)
        addRow(relayMapFrame, "OUT (Load) — CH", "relay_out", self.relayOutCh)
        ctk.CTkLabel(relayMapFrame, text="", height=6).pack()

        # -- ECM model — takes effect immediately, no restart needed --
        ecmSettingsFrame = ctk.CTkFrame(parent, corner_radius=12, fg_color=dual("card_bg_alt"))
        ecmSettingsFrame.pack(fill="x", pady=(0, 12))
        ctk.CTkLabel(ecmSettingsFrame, text="ECM model", font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=12, pady=(10, 6))

        parallelRow = ctk.CTkFrame(ecmSettingsFrame, fg_color="transparent")
        parallelRow.pack(fill="x", padx=12, pady=3)
        ctk.CTkLabel(parallelRow, text="Parallel branches (P)", width=160, anchor="w",
                     text_color=dual("text_secondary"),
                     font=ctk.CTkFont(size=11)).pack(side="left")
        self.ecmParallelEntry = ctk.CTkEntry(parallelRow, width=80)
        self.ecmParallelEntry.insert(0, str(self.ecmParallelCount))
        self.ecmParallelEntry.pack(side="right")

        ctk.CTkLabel(ecmSettingsFrame,
                     text="Current fed to the ECM model (1 cell) = measured pack current / P. "
                          "E.g. 8S3P → P = 3.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        tempSourceRow = ctk.CTkFrame(ecmSettingsFrame, fg_color="transparent")
        tempSourceRow.pack(fill="x", padx=12, pady=3)
        ctk.CTkLabel(tempSourceRow, text="Temperature source for ECM/thermal", width=160,
                     anchor="w", text_color=dual("text_secondary"),
                     font=ctk.CTkFont(size=11)).pack(side="left")
        tempSourceNames = [name for _, name in self.resistChannels]
        self.ecmTempSourceMenu = ctk.CTkOptionMenu(tempSourceRow, values=tempSourceNames, width=110)
        self.ecmTempSourceMenu.set(self.ecmTempSourceName if self.ecmTempSourceName in tempSourceNames
                                    else tempSourceNames[0])
        self.ecmTempSourceMenu.pack(side="right")

        ctk.CTkLabel(ecmSettingsFrame,
                     text="Which temperature channel is used for the ECM's OCV/R lookup and "
                          "for the thermal model's initial calibration (pick a channel that's "
                          "actually populated on your bench — Mid pack is often unconnected). "
                          "Both take effect immediately, no restart needed.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 4))

        ctk.CTkButton(ecmSettingsFrame, text="Apply", command=self.applyEcmSettings).pack(
            fill="x", padx=12, pady=(4, 4))
        self.ecmParallelStatusLabel = ctk.CTkLabel(
            ecmSettingsFrame, text="", font=ctk.CTkFont(size=11),
            text_color=dual("text_secondary"), wraplength=270, justify="left")
        self.ecmParallelStatusLabel.pack(anchor="w", padx=12, pady=(0, 10))

        # -- Custom (yellow) safety limits, shown alongside the absolute (red) ones --
        limitsFrame = ctk.CTkFrame(parent, corner_radius=12, fg_color=dual("card_bg_alt"))
        limitsFrame.pack(fill="x", pady=(0, 12))
        ctk.CTkLabel(limitsFrame, text="Custom safety limits (per cell)",
                     font=ctk.CTkFont(size=13, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=12, pady=(10, 4))
        ctk.CTkLabel(limitsFrame,
                     text="Shown as yellow lines on the chart, inside the absolute red "
                          f"limits ({CELL_ABS_V_MIN:g}–{CELL_ABS_V_MAX:g} V, "
                          f"{CELL_ABS_T_MIN:g}–{CELL_ABS_T_MAX:g} °C). The defaults "
                          "below are generic -- set them to your cell's actual datasheet "
                          "limits.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 8))

        def limitRow(label, value):
            row = ctk.CTkFrame(limitsFrame, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=3)
            ctk.CTkLabel(row, text=label, width=160, anchor="w",
                         text_color=dual("text_secondary"),
                         font=ctk.CTkFont(size=11)).pack(side="left")
            entry = ctk.CTkEntry(row, width=80)
            entry.insert(0, f"{value:g}")
            entry.pack(side="right")
            return entry

        self.customVMinEntry = limitRow("Min voltage (V)", self.customVMin)
        self.customVMaxEntry = limitRow("Max voltage (V)", self.customVMax)
        self.customTMinEntry = limitRow("Min temperature (°C)", self.customTMin)
        self.customTMaxEntry = limitRow("Max temperature (°C)", self.customTMax)

        ctk.CTkButton(limitsFrame, text="Apply", command=self._applyCustomLimits).pack(
            fill="x", padx=12, pady=(6, 4))
        self.customLimitsStatusLabel = ctk.CTkLabel(
            limitsFrame, text="", font=ctk.CTkFont(size=11),
            text_color=dual("text_secondary"), wraplength=270, justify="left")
        self.customLimitsStatusLabel.pack(anchor="w", padx=12, pady=(0, 10))

        ctk.CTkButton(parent, text="💾  Save settings", command=self.saveSettings).pack(
            fill="x", pady=(0, 4))
        self.settingsStatusLabel = ctk.CTkLabel(parent, text="", font=ctk.CTkFont(size=11),
                                                 text_color=dual("text_secondary"),
                                                 wraplength=270, justify="left")
        self.settingsStatusLabel.pack(anchor="w", pady=(6, 4))

    # ---- BATTERY HEALTH PAGE -----------------------------------------------
    def _buildHealthPage(self, parent):
        """Pack layout view: one column per series group (B01..B0n), each drawn as
        `self.ecmParallelCount` pouch cells side by side -- the parallel group really is
        wired/measured as one battery (a single voltage channel), the extra pouches are
        purely a physical-layout illustration of the Ns*Np pack. SoH is a placeholder
        (no estimation algorithm exists yet -- see _refreshHealthPage); the balance
        figure is real, computed from the actual logged data."""
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=1)

        summaryCard = ctk.CTkFrame(parent, corner_radius=16, fg_color=dual("card_bg"),
                                    border_width=1, border_color=dual("border"))
        summaryCard.grid(row=0, column=0, sticky="new", padx=16, pady=(0, 8))
        ctk.CTkLabel(summaryCard, text="Pack health", font=ctk.CTkFont(size=16, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=20, pady=(16, 2))
        self.healthSummaryLabel = ctk.CTkLabel(summaryCard, text="No voltage data yet.",
                                                font=ctk.CTkFont(size=13), text_color=dual("text"))
        self.healthSummaryLabel.pack(anchor="w", padx=20, pady=(0, 4))
        ctk.CTkLabel(summaryCard,
                     text="Balance % = how far each series group's whole-session average voltage "
                          "sits from the pack's own average -- not an instantaneous reading. Each "
                          "group's parallel cells share one voltage channel, so all of a group's "
                          "pouches are colored the same.",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary"),
                     wraplength=900, justify="left").pack(anchor="w", padx=20, pady=(0, 16))

        gridCard = ctk.CTkFrame(parent, corner_radius=16, fg_color=dual("card_bg"),
                                 border_width=1, border_color=dual("border"))
        gridCard.grid(row=1, column=0, sticky="nsew", padx=16, pady=(0, 16))

        n = len(self.batteryChannels)
        for k in range(n):
            gridCard.grid_columnconfigure(k, weight=1)

        self.healthSohLabels = []
        self.healthPouches = []
        self.healthAvgLabels = []
        self.healthDevLabels = []
        for k in range(n):
            col = ctk.CTkFrame(gridCard, fg_color=dual("card_bg_alt"), corner_radius=10)
            col.grid(row=0, column=k, sticky="nsew", padx=6, pady=16)

            ctk.CTkLabel(col, text=f"B{k+1:02d}", font=ctk.CTkFont(size=13, weight="bold"),
                         text_color=dual("text")).pack(pady=(12, 2))
            # SoH estimation isn't implemented yet -- placeholder slot, see class docstring.
            sohLabel = ctk.CTkLabel(col, text="SoH: —", font=ctk.CTkFont(size=11),
                                     text_color=dual("text_secondary"))
            sohLabel.pack(pady=(0, 8))
            self.healthSohLabels.append(sohLabel)

            pouchRow = ctk.CTkFrame(col, fg_color="transparent")
            pouchRow.pack(pady=(0, 8))
            pouches = []
            for _ in range(self.ecmParallelCount):
                pouch = ctk.CTkFrame(pouchRow, width=26, height=72, corner_radius=5,
                                      fg_color=GRAY, border_width=1, border_color=dual("border"))
                pouch.pack_propagate(False)
                pouch.pack(side="left", padx=2)
                pouches.append(pouch)
            self.healthPouches.append(pouches)

            avgLabel = ctk.CTkLabel(col, text="— V", font=ctk.CTkFont(size=12, weight="bold"),
                                     text_color=dual("text"))
            avgLabel.pack(pady=(4, 0))
            self.healthAvgLabels.append(avgLabel)
            devLabel = ctk.CTkLabel(col, text="Δ —", font=ctk.CTkFont(size=11),
                                     text_color=dual("text_secondary"))
            devLabel.pack(pady=(0, 12))
            self.healthDevLabels.append(devLabel)

    def _refreshHealthPage(self):
        """Recompute the whole-session per-group average voltage and each group's
        percentage deviation from the pack's own average -- the balance metric the
        Battery Health page shows. Always over the FULL history in self.battY, per
        spec, not a rolling window; cheap enough (n=8 short lists) to just redo from
        scratch on every call."""
        n = len(self.batteryChannels)
        avgV = []
        for k in range(n):
            vals = [v for v in self.battY[k] if v == v]
            avgV.append(sum(vals) / len(vals) if vals else float("nan"))
        validAvg = [v for v in avgV if v == v]
        center = sum(validAvg) / len(validAvg) if validAvg else float("nan")

        for k in range(n):
            v = avgV[k]
            if v == v and center == center and center != 0:
                devPct = (v - center) / center * 100.0
                absDev = abs(devPct)
                color = (GREEN if absDev <= BALANCE_WARN_PCT
                         else ORANGE if absDev <= BALANCE_CRIT_PCT else RED)
                self.healthAvgLabels[k].configure(text=f"{v:.3f} V")
                self.healthDevLabels[k].configure(text=f"Δ {devPct:+.2f} %", text_color=color)
                for pouch in self.healthPouches[k]:
                    pouch.configure(fg_color=color)
            else:
                self.healthAvgLabels[k].configure(text="— V")
                self.healthDevLabels[k].configure(text="Δ —", text_color=dual("text_secondary"))
                for pouch in self.healthPouches[k]:
                    pouch.configure(fg_color=GRAY)

        if center == center:
            spreadPct = (max(abs(v - center) for v in validAvg) / center * 100.0
                         if validAvg else 0.0)
            self.healthSummaryLabel.configure(
                text=f"Pack average voltage: {center:.3f} V   ·   "
                     f"balance spread (max deviation): ±{spreadPct:.2f} %   ·   "
                     f"{len(self.tData)} samples over the session")
        else:
            self.healthSummaryLabel.configure(
                text="No voltage data yet — start a measurement or load a test file.")

    # ---- CHART AREA -------------------------------------------------------
    def _buildChartArea(self):
        chartFrame = ctk.CTkFrame(self.monitoringPage, corner_radius=16, fg_color=dual("card_bg"),
                                   border_width=1, border_color=dual("border"))
        chartFrame.grid(row=0, column=1, sticky="nsew", padx=(8, 16), pady=(0, 16))
        chartFrame.grid_rowconfigure(0, weight=1)
        chartFrame.grid_columnconfigure(0, weight=1)

        self.fig = Figure(dpi=100)
        gs = self.fig.add_gridspec(3, 1, hspace=0.2)
        self.axV = self.fig.add_subplot(gs[0])
        self.axI = self.fig.add_subplot(gs[1], sharex=self.axV)
        self.axR = self.fig.add_subplot(gs[2], sharex=self.axV)

        self.axV.set_ylabel("Voltage [V]")
        self.axI.set_ylabel("Current [A]")
        self.axR.set_ylabel("Temperature [°C]")
        self.axR.set_xlabel("Time [s]")
        for ax in (self.axV, self.axI):
            ax.tick_params(labelbottom=False)

        colorCycle = matplotlib.rcParams["axes.prop_cycle"].by_key()["color"]

        self.battLines = []
        for k, ch in enumerate(self.batteryChannels):
            (line,) = self.axV.plot([], [], "-", linewidth=1.4,
                                     color=colorCycle[k % len(colorCycle)],
                                     label=f"B{k+1:02d} (CH{ch})")
            self.battLines.append(line)

        self.currLines = []
        currColors = [GREEN, RED]
        for k, (ch, name) in enumerate(self.currentChannels):
            (line,) = self.axI.plot([], [], "-", linewidth=1.6, color=currColors[k % 2],
                                     label=f"{name} (CH{ch})")
            self.currLines.append(line)

        self.resLines = []
        for k, (ch, name) in enumerate(self.resistChannels):
            (line,) = self.axR.plot([], [], "-", linewidth=1.4,
                                     color=colorCycle[k % len(colorCycle)],
                                     label=f"{name} (CH{ch})")
            self.resLines.append(line)

        (ecmLine,) = self.axV.plot([], [], "--", linewidth=1.6, color=ACCENT,
                                    label="ECM sim (1 cell)")
        self.ecmLines = [ecmLine]

        # 24 thin, semi-transparent lines -- the pack thermal model, not 24 separate
        # legend entries (only the first carries a label, to keep the legend readable).
        self.fmuLines = []
        for k in range(FMU_N_CELLS):
            (line,) = self.axR.plot([], [], "-", linewidth=0.6, color=ORANGE, alpha=0.35,
                                     visible=self.fmuThermal.available,
                                     label="Thermal sim (24 cells)" if k == 0 else "_nolegend_")
            self.fmuLines.append(line)

        # -- Estimation projection curves: same axes, dashed, invisible until a forward
        # projection actually runs (see _runEstimation / _clearEstimation). --
        (self.ecmProjLine,) = self.axV.plot([], [], "--", linewidth=1.8, color=ACCENT,
                                            alpha=0.55, label="_nolegend_")
        # Color is reassigned each run (see _runEstimation) to match whichever of
        # I_IN/I_OUT the held current falls on -- RED here is just a placeholder.
        (self.currProjLine,) = self.axI.plot([], [], "--", linewidth=1.8, color=RED,
                                             alpha=0.55, label="_nolegend_")
        self.fmuProjLines = []
        for k in range(FMU_N_CELLS):
            (line,) = self.axR.plot([], [], "--", linewidth=0.7, color=ORANGE, alpha=0.5,
                                     visible=False, label="_nolegend_")
            self.fmuProjLines.append(line)
        self._estimationMarkers = []

        # -- vertical marker at "now" -- where real (measured/replayed) data ends and the
        # projection starts, so the two visually can't be mistaken for one continuous
        # measured curve. Hidden until a projection actually runs. --
        self._estimationNowLines = {
            ax: ax.axvline(x=0, color=GRAY, linewidth=1.2, linestyle="-", alpha=0.6,
                           visible=False)
            for ax in (self.axV, self.axI, self.axR)
        }

        # -- Absolute (red) and custom (yellow) per-cell safety window -- always drawn,
        # excluded from the Y-autoscale by _autoscaleCharts (an axhline would otherwise
        # stretch a +/-1 degC bench swing across the full -30..55 degC axis and hide it).
        refKw = dict(linewidth=1.1, linestyle=(0, (6, 3)), alpha=0.85, zorder=0)
        self.absVLines = [self.axV.axhline(CELL_ABS_V_MAX, color=RED, **refKw),
                          self.axV.axhline(CELL_ABS_V_MIN, color=RED, **refKw)]
        self.absTLines = [self.axR.axhline(CELL_ABS_T_MAX, color=RED, **refKw),
                          self.axR.axhline(CELL_ABS_T_MIN, color=RED, **refKw)]
        self.customVLines = [self.axV.axhline(self.customVMax, color=YELLOW, **refKw),
                             self.axV.axhline(self.customVMin, color=YELLOW, **refKw)]
        self.customTLines = [self.axR.axhline(self.customTMax, color=YELLOW, **refKw),
                             self.axR.axhline(self.customTMin, color=YELLOW, **refKw)]

        for ax in (self.axV, self.axI, self.axR):
            ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8,
                      borderaxespad=0.0, frameon=True)

        self.fig.subplots_adjust(left=0.07, right=0.85, top=0.97, bottom=0.08)

        # -- Crosshair with value readout --
        self._vlines = {}
        self._readouts = {}
        for ax in (self.axV, self.axI, self.axR):
            vline = ax.axvline(x=0, color=GRAY, linewidth=0.8, linestyle="--", visible=False)
            text = ax.text(0.012, 0.96, "", transform=ax.transAxes, va="top", ha="left",
                            fontsize=8, visible=False,
                            bbox=dict(boxstyle="round,pad=0.4", alpha=0.95))
            self._vlines[ax] = vline
            self._readouts[ax] = text

        self.canvas = FigureCanvasTkAgg(self.fig, master=chartFrame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=12, pady=12)
        self.canvas.mpl_connect("motion_notify_event", self._onChartHover)
        self.canvas.mpl_connect("figure_leave_event", self._hideCrosshair)
        self.canvas.mpl_connect("pick_event", self._onLegendPick)
        self.canvas.draw()

        # Floating icon over the chart -- per-cell thermal detail, kept right next to
        # what it's a detail OF instead of buried in a sidebar card.
        self.btnFmuDetail = ctk.CTkButton(chartFrame, text="📈", width=34, height=30,
                                          corner_radius=8, fg_color=dual("card_bg_alt"),
                                          hover_color=dual("border"), text_color=dual("text"),
                                          command=self.openFmuDetailWindow)
        self.btnFmuDetail.place(relx=1.0, x=-14, y=14, anchor="ne")

        # Plain colored dashes instead of colored emoji -- Tk's default font renders
        # 🔴/🟡 as flat monochrome glyphs, which defeats the point of a color legend.
        hintRow = ctk.CTkFrame(chartFrame, fg_color="transparent")
        hintRow.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 8))
        swatchFont = ctk.CTkFont(size=14, weight="bold")
        hintFont = ctk.CTkFont(size=10)
        ctk.CTkLabel(hintRow, text="—", text_color=RED, font=swatchFont).pack(side="left")
        ctk.CTkLabel(hintRow, text=f" absolute cell limit ({CELL_ABS_V_MIN:g}–{CELL_ABS_V_MAX:g} V, "
                                   f"{CELL_ABS_T_MIN:g}–{CELL_ABS_T_MAX:g} °C)    ",
                    font=hintFont, text_color=dual("text_secondary")).pack(side="left")
        ctk.CTkLabel(hintRow, text="—", text_color=YELLOW, font=swatchFont).pack(side="left")
        ctk.CTkLabel(hintRow, text=" custom limit (Settings)",
                    font=hintFont, text_color=dual("text_secondary")).pack(side="left")

    # ------------------------------------------------------------------
    # CHART CROSSHAIR / VALUE READOUT
    def _onChartHover(self, event):
        if not self.tData or event.inaxes not in self._vlines or event.xdata is None:
            self._hideCrosshair()
            return

        idx = bisect.bisect_left(self.tData, event.xdata)
        if idx >= len(self.tData):
            idx = len(self.tData) - 1
        elif idx > 0 and abs(self.tData[idx - 1] - event.xdata) < abs(self.tData[idx] - event.xdata):
            idx -= 1
        t = self.tData[idx]

        def fmt(lines, series, unit, prec):
            parts = []
            for line, arr in zip(lines, series):
                v = arr[idx]
                label = line.get_label().split(" (")[0]
                parts.append(f"{label}: {v:.{prec}f}{unit}" if v == v else f"{label}: ---")
            return "\n".join(parts)

        header = f"t = {t:.1f} s\n"
        self._readouts[self.axV].set_text(
            header + fmt(self.battLines + self.ecmLines, self.battY + [self.ecmY], " V", 3))
        self._readouts[self.axI].set_text(header + fmt(self.currLines, self.currY, " A", 3))

        tempText = header + fmt(self.resLines, self.resY, " °C", 1)
        if self.fmuThermal.available and self.fmuTData:
            fIdx = bisect.bisect_left(self.fmuTData, event.xdata)
            if fIdx >= len(self.fmuTData):
                fIdx = len(self.fmuTData) - 1
            elif (fIdx > 0 and abs(self.fmuTData[fIdx - 1] - event.xdata)
                  < abs(self.fmuTData[fIdx] - event.xdata)):
                fIdx -= 1
            cellVals = [y[fIdx] for y in self.fmuY if fIdx < len(y) and y[fIdx] == y[fIdx]]
            if cellVals:
                label = self.fmuLines[0].get_label().split(" (")[0]
                tempText += (f"\n{label}: {min(cellVals):.2f}–{max(cellVals):.2f} °C"
                            if len(cellVals) > 1 else f"\n{label}: {cellVals[0]:.2f} °C")
        self._readouts[self.axR].set_text(tempText)

        for ax, vline in self._vlines.items():
            vline.set_xdata([t, t])
            vline.set_visible(True)
        for text in self._readouts.values():
            text.set_visible(True)

        self.canvas.draw_idle()

    def _hideCrosshair(self, event=None):
        changed = False
        for vline in self._vlines.values():
            if vline.get_visible():
                vline.set_visible(False)
                changed = True
        for text in self._readouts.values():
            if text.get_visible():
                text.set_visible(False)
                changed = True
        if changed:
            self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def _rebuildLegend(self, ax, lines):
        """The legend only shows curves with at least one valid point (a channel that's
        permanently '---' across the whole file doesn't waste space) -- INCLUDING
        currently hidden ones, just dimmed, because the legend now doubles as a control:
        clicking an entry shows/hides that curve (see _onLegendPick). Several lines
        sharing one label (the 24 thin thermal-model curves share a single label)
        collapse into one entry; a click moves all of them together."""
        from matplotlib.lines import Line2D

        groups, order = {}, []
        for ln in lines:
            label = ln.get_label()
            if label.startswith("_") or not any(v == v for v in ln.get_ydata()):
                continue
            groups.setdefault(label, []).append(ln)
            if label not in order:
                order.append(label)

        existing = ax.get_legend()
        if existing is not None:
            existing.remove()
        if not order:
            return

        handles = []
        for label in order:
            members = groups[label]
            ref = members[0]
            isVisible = ref.get_visible()
            # a dedicated, fully opaque proxy artist -- unlike passing `ref` straight into
            # legend(), this doesn't inherit ref.get_alpha() (0.35 for the 24 overlapping
            # thermal-model lines), so the legend swatch shows the curve's real color
            # instead of a washed-out one
            proxy = Line2D([], [], color=ref.get_color(), linestyle=ref.get_linestyle(),
                           linewidth=max(ref.get_linewidth(), 1.6),
                           alpha=1.0 if isVisible else 0.35, label=label)
            proxy._toggleLines = members
            handles.append(proxy)

        legend = ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.01, 1.0),
                           fontsize=8, borderaxespad=0.0, frameon=True)
        for legLine, legText, handle in zip(legend.get_lines(), legend.get_texts(), handles):
            legLine.set_picker(8)
            legLine._toggleLines = handle._toggleLines
            legText.set_picker(True)
            legText._toggleLines = handle._toggleLines
            legText.set_alpha(handle.get_alpha())

    def _onLegendPick(self, event):
        """Clicking a legend entry shows/hides that curve -- or all 24 thermal-model
        curves at once, for the merged entry. Purely a display toggle: the thermal
        model itself always keeps computing in the background (see _startFmuThermal),
        so hiding it here doesn't stop or restart anything -- replaces the old
        dedicated checkbox card in Controls."""
        lines = getattr(event.artist, "_toggleLines", None)
        if not lines:
            return

        newVisible = not lines[0].get_visible()
        for ln in lines:
            ln.set_visible(newVisible)
        if lines[0] in self.fmuLines:
            # keep the (still-hidden-until-a-projection-runs) projection curves in sync
            # with their live counterpart, cell for cell
            for k, ln in enumerate(self.fmuLines):
                if ln in lines:
                    self.fmuProjLines[k].set_visible(newVisible and bool(self.fmuProjLines[k].get_xdata().size))

        for ax, allLines in ((self.axV, self.battLines + self.ecmLines),
                             (self.axI, self.currLines),
                             (self.axR, self.resLines + self.fmuLines)):
            if lines[0] in allLines:
                self._rebuildLegend(ax, allLines)
                break
        self._autoscaleCharts()
        self.applyPlotTheme()
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # CHART AUTOSCALE
    def _dataYRange(self, lines):
        """Y-range spanning only the given VISIBLE, finite-valued lines. Used instead of
        a blanket ax.relim() so the always-on red/yellow safety references (drawn with
        axhline, which -- unintuitively -- DOES count towards relim()) can't stretch a
        +/-1 degC bench swing across the full -30..55 degC axis and flatten it."""
        los, his = [], []
        for line in lines:
            if not line.get_visible():
                continue
            y = np.asarray(line.get_ydata(), dtype=float)
            y = y[np.isfinite(y)]
            if y.size:
                los.append(float(y.min()))
                his.append(float(y.max()))
        return (min(los), max(his)) if los else None

    def _autoscaleCharts(self):
        """Central autoscale for the three shared-x charts, called after any edit to
        their data. X is driven off the real (non-projection) series and shared across
        all three via sharex; Y is computed per axis from _dataYRange so the reference
        lines never distort it. While Estimation is on, _runEstimation manages xlim
        itself (extending it to cover the projection horizon) instead of this method."""
        if not self.estimationOn:
            xs = [v for line in (self.battLines + self.currLines + self.resLines)
                  for v in line.get_xdata() if v == v]
            if xs:
                self.axV.set_xlim(min(xs), max(xs))

        self.axI.relim()
        self.axI.autoscale_view(scalex=False)

        rng = self._dataYRange(self.battLines + self.ecmLines + [self.ecmProjLine])
        if rng:
            lo, hi = rng
            pad = max(0.06 * (hi - lo), 0.03)
            self.axV.set_ylim(lo - pad, hi + pad)

        rng = self._dataYRange(self.resLines + self.fmuLines + self.fmuProjLines)
        if rng:
            lo, hi = rng
            pad = max(0.06 * (hi - lo), 0.3)
            self.axR.set_ylim(lo - pad, hi + pad)

    # ------------------------------------------------------------------
    # ESTIMATION (forward projection of voltage/temperature)
    def _onEstimationToggle(self):
        self.estimationOn = bool(self.estimationVar.get())
        if self.estimationOn:
            self.estimationHorizonMenu.pack(side="left", padx=(0, 14))
            self.logMsg(f"Estimation on -- projecting {self.estimationHorizonH} h ahead "
                        "at the current current.")
            self._runEstimation()
            self._scheduleEstimationTick()
        else:
            if self._estimationAfterId is not None:
                self.root.after_cancel(self._estimationAfterId)
                self._estimationAfterId = None
            self.estimationHorizonMenu.pack_forget()
            self.logMsg("Estimation off.")
            self._clearEstimation()

    def _onEstimationHorizonChange(self, value):
        try:
            self.estimationHorizonH = int(value.split()[0])
        except (ValueError, IndexError):
            return
        if self.estimationOn:
            self._runEstimation()

    def _scheduleEstimationTick(self):
        self._estimationAfterId = self.root.after(ESTIMATION_RECOMPUTE_MS, self._estimationTick)

    def _estimationTick(self):
        if not self.estimationOn:
            return
        self._runEstimation()
        self._scheduleEstimationTick()

    def _runEstimation(self):
        """Clone the live ECM state and project it forward at the CURRENT net current
        held constant -- "if this load keeps up, where does voltage/temperature go?"
        The thermal side reuses the live ROM object itself: ThermalROM.temperatures()
        is a pure function of its z0 argument, so calling it with a hypothetical future
        P_loss never touches the state the live worker is stepping (see
        PackRomThermalWorker.snapshotState). Both are `copy`/snapshot-based specifically
        so this can run every couple of seconds without disturbing the real simulation."""
        if not self.tData or self._lastNetCurrent != self._lastNetCurrent:
            self._clearEstimation()
            return

        tNow = self.tData[-1]
        dt = ESTIMATION_DT_S
        n = max(2, int(self.estimationHorizonH * 3600.0 // dt))
        tRel = np.arange(1, n + 1) * dt
        tAbs = tNow + tRel

        ecmClone = copy.deepcopy(self.ecm)
        heldCurrent = self._lastNetCurrent
        heldTemp = self.ecm.lastTemperatureC
        vProj = np.empty(n)
        pLossProj = np.empty(n)
        for i in range(n):
            vProj[i] = ecmClone.step(heldCurrent, dt, heldTemp)
            pLossProj[i] = ecmClone.lastPLossW
        self.ecmProjLine.set_data(tAbs, vProj)

        # -- current projection: held constant, drawn on whichever of I_IN/I_OUT the
        # sign matches (I >= 0 = discharge/I_OUT red, I < 0 = charge/I_IN green). Uses
        # _lastDisplayCurrent, NOT heldCurrent -- heldCurrent is the ECM's own per-branch
        # current (divided by ecmParallelCount), a different scale than what currLines
        # actually shows, so plotting heldCurrent here would visibly jump away from the
        # real I_IN/I_OUT curve right where the projection starts. --
        displayCurrent = self._lastDisplayCurrent
        self.currProjLine.set_color(RED if displayCurrent >= 0 else GREEN)
        self.currProjLine.set_data(tAbs, np.full(n, abs(displayCurrent)))

        for line in self._estimationNowLines.values():
            line.set_xdata([tNow, tNow])
            line.set_visible(True)

        tempsProj = None
        if self.fmuThermal.available:
            z, tRef = self.fmuThermal.snapshotState()
            if z is not None:
                tGrid = np.concatenate(([0.0], tRel))
                pGrid = np.concatenate(([pLossProj[0]], pLossProj))
                tempsProj = self.fmuThermal.rom.temperatures(tGrid, pGrid, T_amb=tRef, z0=z)
                tempsProj = tempsProj[1:] - 273.15
                for k, line in enumerate(self.fmuProjLines):
                    line.set_data(tAbs, tempsProj[:, k])
                    line.set_visible(self.fmuLines[k].get_visible())

        xMin = self.tData[0]
        for ax in (self.axV, self.axI, self.axR):
            ax.set_xlim(xMin, tAbs[-1])

        self._drawEstimationMarkers(tAbs, vProj, tempsProj)
        self._autoscaleAfterEstimation(vProj, tempsProj)
        self.canvas.draw_idle()

    def _autoscaleAfterEstimation(self, vProj, tempsProj):
        """Same idea as _autoscaleCharts's Y handling, but also folds in the projection
        curves (which _dataYRange already does via self.ecmProjLine/fmuProjLines) --
        xlim is left alone here since _runEstimation just set it to the horizon."""
        rng = self._dataYRange(self.battLines + self.ecmLines + [self.ecmProjLine])
        if rng:
            lo, hi = rng
            pad = max(0.06 * (hi - lo), 0.03)
            self.axV.set_ylim(lo - pad, hi + pad)
        rng = self._dataYRange(self.resLines + self.fmuLines + self.fmuProjLines)
        if rng:
            lo, hi = rng
            pad = max(0.06 * (hi - lo), 0.3)
            self.axR.set_ylim(lo - pad, hi + pad)
        self.axI.relim()
        self.axI.autoscale_view(scalex=False)

    def _clearEstimation(self):
        self.ecmProjLine.set_data([], [])
        self.currProjLine.set_data([], [])
        for line in self.fmuProjLines:
            line.set_data([], [])
            line.set_visible(False)
        for line in self._estimationNowLines.values():
            line.set_visible(False)
        for artist in self._estimationMarkers:
            artist.remove()
        self._estimationMarkers = []
        self._autoscaleCharts()
        self.canvas.draw_idle()

    def _drawEstimationMarkers(self, tAbs, vProj, tempsProj):
        """Mark the first point (if any) where the projection reaches each safety
        line -- lets it run straight through rather than stopping there, per spec."""
        for artist in self._estimationMarkers:
            artist.remove()
        self._estimationMarkers = []
        tNow = self.tData[-1]

        def firstCrossIdx(arr, limit, mode):
            cond = arr >= limit if mode == "above" else arr <= limit
            idx = np.flatnonzero(cond)
            return int(idx[0]) if idx.size else None

        def mark(ax, t, y, color):
            m, = ax.plot([t], [y], marker="x", markersize=9, markeredgewidth=2.2,
                        color=color, zorder=6)
            eta = t - tNow
            label = f"+{eta / 60:.0f} min" if eta < 3600 else f"+{eta / 3600:.1f} h"
            a = ax.annotate(label, xy=(t, y), xytext=(6, 8), textcoords="offset points",
                            fontsize=8, color=color, fontweight="bold", zorder=6)
            self._estimationMarkers += [m, a]

        for limit, mode, color in ((CELL_ABS_V_MAX, "above", RED), (CELL_ABS_V_MIN, "below", RED),
                                   (self.customVMax, "above", YELLOW), (self.customVMin, "below", YELLOW)):
            idx = firstCrossIdx(vProj, limit, mode)
            if idx is not None:
                mark(self.axV, tAbs[idx], vProj[idx], color)

        if tempsProj is not None:
            hottest, coldest = tempsProj.max(axis=1), tempsProj.min(axis=1)
            for limit, mode, color, arr in (
                (CELL_ABS_T_MAX, "above", RED, hottest), (CELL_ABS_T_MIN, "below", RED, coldest),
                (self.customTMax, "above", YELLOW, hottest), (self.customTMin, "below", YELLOW, coldest),
            ):
                idx = firstCrossIdx(arr, limit, mode)
                if idx is not None:
                    mark(self.axR, tAbs[idx], arr[idx], color)

    def _applyCustomLimits(self):
        try:
            vMin = float(self.customVMinEntry.get().replace(",", "."))
            vMax = float(self.customVMaxEntry.get().replace(",", "."))
            tMin = float(self.customTMinEntry.get().replace(",", "."))
            tMax = float(self.customTMaxEntry.get().replace(",", "."))
        except ValueError:
            self.customLimitsStatusLabel.configure(text="Invalid value.", text_color=RED)
            return
        if not (vMin < vMax and tMin < tMax):
            self.customLimitsStatusLabel.configure(text="Min must be less than Max.", text_color=RED)
            return

        self.customVMin, self.customVMax = vMin, vMax
        self.customTMin, self.customTMax = tMin, tMax
        self.customVLines[0].set_ydata([vMax, vMax])
        self.customVLines[1].set_ydata([vMin, vMin])
        self.customTLines[0].set_ydata([tMax, tMax])
        self.customTLines[1].set_ydata([tMin, tMin])

        cfg = config.load_config()
        cfg["custom_v_min"], cfg["custom_v_max"] = vMin, vMax
        cfg["custom_t_min"], cfg["custom_t_max"] = tMin, tMax
        config.save_config(cfg)

        self.customLimitsStatusLabel.configure(text="Saved and applied.", text_color=GREEN)
        self.logMsg(f"Custom safety limits set: {vMin:.2f}-{vMax:.2f} V, {tMin:.1f}-{tMax:.1f} °C.")
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # APPEARANCE / THEME
    def setAppearance(self, value):
        ctk.set_appearance_mode(value)
        self.applyPlotTheme()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.applyTheme()

    def applyPlotTheme(self):
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
                frame.set_alpha(1.0)
                for text in legend.get_texts():
                    text.set_color(t["text"])

            self._vlines[ax].set_color(t["text_secondary"])
            readout = self._readouts[ax]
            readout.set_color(t["text"])
            patch = readout.get_bbox_patch()
            if patch is not None:
                patch.set_facecolor(t["card_bg"])
                patch.set_edgecolor(t["border"])

        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # COM PORT SCANNING
    def scanPorts(self):
        self.logMsg("Scanning COM ports for a device...")
        self._setStatus("Scanning...", ORANGE)
        self.root.update()

        avail = [p.device for p in serial.tools.list_ports.comports()]
        if not avail:
            self.logMsg("[!] No free COM ports detected.")
            self._setStatus("No COM ports", GRAY)
            return

        validPorts = []
        probeCh = self.batteryChannels[0]
        for p in avail:
            try:
                testSerial = serial.Serial(p, BAUD_RATE, timeout=0.4)
                testSerial.reset_input_buffer()
                testSerial.reset_output_buffer()

                testSerial.write(f"MEASure:VOLTage{probeCh}? 15V,Fast\r\n".encode("ascii"))
                resp = testSerial.readline().decode("ascii", errors="ignore").strip()
                testSerial.close()

                value = parseNumeric(resp, rangeLimit=BATTERY_RANGE_V * RANGE_MARGIN)
                if value is not None:
                    validPorts.append(p)
                    self.logMsg(f"  -> {p} replied: {value:.3f} V")
            except Exception:
                pass  # port didn't reply or is busy

        if validPorts:
            self.ddPorts.configure(values=validPorts, state="readonly")
            self.ddPorts.set(validPorts[0])
            self.btnConnect.configure(state="normal")
            self._setStatus(f"Found: {', '.join(validPorts)}", GREEN)
            self.logMsg("[OK] Select a port and click Connect.")
        else:
            self.ddPorts.configure(values=["(No matching device found)"], state="disabled")
            self.ddPorts.set("(No matching device found)")
            self.btnConnect.configure(state="disabled")
            self._setStatus("No device found", RED)
            self.logMsg("[!] No port replied to the query.")

    # ------------------------------------------------------------------
    # PORT CONNECTION (separate from starting a measurement)
    def toggleConnection(self):
        if self.sPort is None:
            targetPort = self.ddPorts.get()
            try:
                self.sPort = serial.Serial(targetPort, BAUD_RATE, timeout=FAST_READ_TIMEOUT)
                self.sPort.reset_input_buffer()
                self.sPort.reset_output_buffer()
            except Exception as ex:
                self.logMsg(f"[ERROR] Could not open {targetPort}: {ex}")
                self.sPort = None
                return

            self._portLost = False
            self.btnConnect.configure(text="🔌  Disconnect", fg_color="transparent",
                                       border_width=1, border_color=RED, text_color=RED)
            self.btnScan.configure(state="disabled")
            self.ddPorts.configure(state="disabled")
            self.btnStart.configure(state="normal")
            self._setStatus(f"Connected ({targetPort})", GREEN)
            self.logMsg(f"Connected to {targetPort}.")

            for sw, var in self.relaySwitches.values():
                sw.configure(state="normal")
            self._refreshRelayAvailability()

            if self.tStart is None:
                self.tStart = time.perf_counter()
        else:
            if self.measuring:
                return  # button is locked while measuring -- stop the measurement first
            self.disconnect()

    def disconnect(self):
        if self.measuring:
            self.stopMeasurement()

        if self.sPort is not None:
            try:
                self.sPort.close()
            except Exception:
                pass
            self.sPort = None

        self.btnConnect.configure(text="🔌  Connect", **self._btnConnectDefaults)
        self.btnStart.configure(state="disabled")
        self.btnScan.configure(state="normal")
        self.ddPorts.configure(state="readonly")
        self._setStatus("Disconnected", GRAY)
        self.logMsg("Disconnected.")

        for ch, (sw, var) in self.relaySwitches.items():
            sw.configure(state="disabled")
            var.set(False)
            self.relayState[ch] = False

        self._cutoffInitialApplied = False
        self.cutoffVoltageLabel.configure(text="Current voltage sum: --- V")

    # ------------------------------------------------------------------
    # START / STOP MEASUREMENT (requires an active connection)
    def toggleMeasurement(self):
        if not self.measuring:
            if self.sPort is None:
                self.logMsg("[!] Connect to a port first.")
                return

            self.measuring = True
            self._portLost = False
            self.btnStart.configure(text="⏹  Stop measurement", fg_color="transparent",
                                     border_width=1, border_color=RED, text_color=RED)
            self.btnConnect.configure(state="disabled")
            self.btnLoadTestFile.configure(state="disabled")
            self.btnLoadCurrentProfile.configure(state="disabled")
            self._setStatus(f"Measuring ({self.sPort.port})", GREEN)
            self._refreshRelayAvailability()

            if self.cutoffEnabled and not self._cutoffInitialApplied:
                initialState = bool(self.cutoffInitialVar.get())
                self._cutoffInitialApplied = self._setRelayImmediate(
                    self.relayOutCh, initialState,
                    f"measurement start with sequence — initial state {'ON' if initialState else 'OFF'}")

            if self.tStart is None:
                self.tStart = time.perf_counter()

            self.logMsg("Starting data collection...")

            if self.fmuThermal.available and not self.fmuThermal.isRunning():
                self._startFmuThermal()

            self.stopEvent = threading.Event()
            self.workerThread = threading.Thread(target=self._workerLoop, daemon=True)
            self.workerThread.start()
        else:
            self.stopMeasurement()
            self.logMsg("Measurement stopped by user.")

    # ------------------------------------------------------------------
    def _setStatus(self, text, color):
        self.statusPill.configure(text=text, fg_color=color)

    # ------------------------------------------------------------------
    # SAMPLING SPEED (Fast/Slow)
    def _onSpeedModeChange(self, value):
        """Switching to Fast requires confirming the warning (see the card) --
        switching back to Slow doesn't, since that's always the safe/more accurate
        direction."""
        if value == "Fast":
            proceed = messagebox.askyesno(
                "Fast sampling mode",
                "In fast mode the instrument has less time to settle its A/D converter "
                "on each channel — readings may be less accurate than in slow mode.\n\n"
                "Switch to fast mode anyway?")
            if not proceed:
                self.speedModeSwitch.set("Slow (accurate)")
                return
            self._applySpeedMode("Fast", self._lastFastPeriod)
        else:
            self._applySpeedMode("Slow", self._lastSlowPeriod)

    def _applySpeedMode(self, mode, period):
        self._speedMode = mode
        if mode == "Fast":
            self.speedSlider.configure(from_=MIN_POLL_PERIOD, to=SLOW_THRESHOLD,
                                       number_of_steps=round((SLOW_THRESHOLD - MIN_POLL_PERIOD) / 0.05))
            self.speedWarningLabel.pack(anchor="w", padx=12, pady=(0, 6), before=self.speedHintLabel)
            self.speedHintLabel.configure(
                text=f"Range {MIN_POLL_PERIOD:.2f}–{SLOW_THRESHOLD:.2f} s. Faster, but "
                     "less accurate readings.")
            self.speedPill.configure(fg_color=ORANGE)
        else:
            self.speedWarningLabel.pack_forget()
            self.speedSlider.configure(from_=SLOW_THRESHOLD, to=MAX_POLL_PERIOD,
                                       number_of_steps=round(MAX_POLL_PERIOD - SLOW_THRESHOLD))
            self.speedHintLabel.configure(
                text=f"Range {SLOW_THRESHOLD:.0f}–{MAX_POLL_PERIOD:.0f} s. More accurate, "
                     "the recommended mode for regular measurements.")
            self.speedPill.configure(fg_color=ACCENT)
        self.setPollPeriod(period, fromSlider=False)

    def _onSpeedSliderMove(self, value):
        self.setPollPeriod(value, fromSlider=True)

    def _onSpeedEntryCommit(self, event=None):
        lo, hi = ((MIN_POLL_PERIOD, SLOW_THRESHOLD) if self._speedMode == "Fast"
                 else (SLOW_THRESHOLD, MAX_POLL_PERIOD))
        try:
            value = float(self.speedEntry.get().replace(",", "."))
        except ValueError:
            value = self.pollPeriod
        value = min(max(value, lo), hi)
        self.setPollPeriod(value, fromSlider=False)

    def setPollPeriod(self, value, fromSlider=True):
        self.pollPeriod = float(value)
        if not fromSlider:
            self.speedSlider.set(self.pollPeriod)
        self.speedEntry.delete(0, "end")
        self.speedEntry.insert(0, f"{self.pollPeriod:.2f}")
        self.speedPill.configure(text=f"{self.pollPeriod:.2f} s")
        if self._speedMode == "Fast":
            self._lastFastPeriod = self.pollPeriod
        else:
            self._lastSlowPeriod = self.pollPeriod

    # ------------------------------------------------------------------
    # AUTOMATIC OUT (Load) CUTOFF
    def _toggleCutoff(self):
        self.cutoffEnabled = bool(self.cutoffSwitchVar.get())
        cfg = config.load_config()
        cfg["cutoff_enabled"] = self.cutoffEnabled
        config.save_config(cfg)
        self._refreshRelayAvailability()
        state = "enabled" if self.cutoffEnabled else "disabled"
        self.logMsg(f"Automatic OUT cutoff sequence {state}.")

        if self.cutoffEnabled:
            initialState = bool(self.cutoffInitialVar.get())
            applied = self._setRelayImmediate(
                self.relayOutCh, initialState,
                f"sequence start — initial state {'ON' if initialState else 'OFF'}")
            self._cutoffInitialApplied = applied
        else:
            self._cutoffInitialApplied = False

    def _saveCutoffInitialState(self):
        self.cutoffInitialState = bool(self.cutoffInitialVar.get())
        cfg = config.load_config()
        cfg["cutoff_initial_state"] = self.cutoffInitialState
        config.save_config(cfg)

    def _setRelayImmediate(self, ch, newState, reason):
        """Sends SET:OUTput directly from the main (GUI) thread and reflects the state
        in the UI. Returns True if the command was actually sent."""
        if self.sPort is None:
            self.logMsg(f"[!] Can't set relay CH{ch} — port not connected.")
            return False
        cmd = f"SET:OUTput{ch} {1 if newState else 0}"
        with self.portLock:
            try:
                time.sleep(INTER_COMMAND_DELAY)
                self.sPort.write((cmd + "\r\n").encode("ascii"))
            except Exception as ex:
                self.logMsg(f"[ERROR] Could not send relay command CH{ch}: {ex}")
                return False

        self.relayState[ch] = newState
        if ch in self.relaySwitches:
            _, var = self.relaySwitches[ch]
            var.set(newState)
        name = dict(self.relayChannels)[ch]
        self.logMsg(f"Relay CH{ch} ({name}) -> {'ON' if newState else 'OFF'} ({reason})")
        return True

    def _applyCutoffThresholds(self):
        try:
            offV = float(self.cutoffOffEntry.get().replace(",", "."))
            onV = float(self.cutoffOnEntry.get().replace(",", "."))
        except ValueError:
            self.cutoffStatusLabel.configure(text="Enter valid numbers.", text_color=RED)
            return

        nCells = len(self.batteryChannels)
        safeMin = nCells * config.CELL_MIN_SAFE_V
        safeMax = nCells * config.CELL_MAX_SAFE_V

        if not (safeMin <= offV < onV <= safeMax):
            self.cutoffStatusLabel.configure(
                text=f"Invalid thresholds — must satisfy {safeMin:.1f} ≤ off < on ≤ {safeMax:.1f} V.",
                text_color=RED)
            return

        self.cutoffOffV = offV
        self.cutoffOnV = onV
        cfg = config.load_config()
        cfg["cutoff_off_v"] = offV
        cfg["cutoff_on_v"] = onV
        config.save_config(cfg)
        self.cutoffStatusLabel.configure(
            text=f"Thresholds saved: off {offV:.1f} V / on {onV:.1f} V.",
            text_color=GREEN)
        self.logMsg(f"Automatic cutoff thresholds set: off {offV:.1f} V / on {onV:.1f} V.")

    # ------------------------------------------------------------------
    # ECM MODEL
    def resetEcm(self):
        try:
            pct = float(self.ecmInitialSocEntry.get().replace(",", "."))
        except ValueError:
            self.logMsg("[!] ECM reset: invalid initial SOC value.")
            return
        pct = min(max(pct, 0.0), 100.0)

        self.ecm.reset(initialSocFraction=pct / 100.0)
        self._ecmLastTNow = None
        self.ecmY = [float("nan")] * len(self.tData)
        self.ecmSocY = [float("nan")] * len(self.tData)
        if self.ecmLines:
            self.ecmLines[0].set_data(self.tData, self.ecmY)
            self.canvas.draw_idle()

        self.ecmInitialSocPct = pct
        cfg = config.load_config()
        cfg["ecm_initial_soc_pct"] = pct
        config.save_config(cfg)

        self.ecmSocLabel.configure(text=f"SOC: {self.ecm.soc*100:.1f} %")
        self.ecmVoltageLabel.configure(text="Simulated voltage: --- V")
        self.ecmModeLabel.configure(text="Mode: —  ·  Model temperature: --- °C")
        self.logMsg(f"ECM model reset to SOC {pct:.1f} %.")

    # ------------------------------------------------------------------
    # PACK THERMAL MODEL (always on when available -- see _buildControlPage)
    def openFmuDetailWindow(self):
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.focus()
            return
        self.fmuDetailWindow = FmuDetailWindow(self.root, FMU_N_CELLS)
        self.fmuDetailWindow.set_on_close(lambda: setattr(self, "fmuDetailWindow", None))
        self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY, self.fmuTData, self.fmuY)

    def _startFmuThermal(self):
        initialTempC = 25.0
        srcY = self.resY[self.ecmTempSourceIndex] if self.resY else []
        if srcY and srcY[-1] == srcY[-1]:
            initialTempC = srcY[-1]
        self.fmuThermal.setInitialTemperature(initialTempC)
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]
        self.fmuThermal.start()
        self._fmuLastStatusText = f"running (initial temperature {initialTempC:.1f} °C)"
        self.fmuStatusLabel.configure(text=f"Status: {self._fmuLastStatusText}")
        self.logMsg(f"Thermal model started, initial temperature {initialTempC:.1f} °C.")

    def _refreshRelayAvailability(self):
        if self.relayOutCh not in self.relaySwitches:
            return
        sw, _ = self.relaySwitches[self.relayOutCh]
        if self.sPort is None:
            sw.configure(state="disabled")
        elif self.cutoffEnabled:
            sw.configure(state="disabled")
        else:
            sw.configure(state="normal")

    # ------------------------------------------------------------------
    # CHANNEL SETTINGS
    def _updateEcmTempSourceIndex(self):
        names = [name for _, name in self.resistChannels]
        self.ecmTempSourceIndex = names.index(self.ecmTempSourceName) if self.ecmTempSourceName in names else 0

    def applyEcmSettings(self):
        try:
            p = int(float(self.ecmParallelEntry.get().replace(",", ".")))
        except ValueError:
            self.ecmParallelStatusLabel.configure(text="Invalid value.", text_color=RED)
            return
        if p < 1:
            self.ecmParallelStatusLabel.configure(text="Branch count must be ≥ 1.", text_color=RED)
            return

        self.ecmParallelCount = p
        self.ecmTempSourceName = self.ecmTempSourceMenu.get()
        self._updateEcmTempSourceIndex()

        cfg = config.load_config()
        cfg["ecm_parallel_count"] = p
        cfg["ecm_temp_source"] = self.ecmTempSourceName
        config.save_config(cfg)
        self.ecmParallelStatusLabel.configure(
            text=f"Saved and applied: P = {p}, temperature from “{self.ecmTempSourceName}”.",
            text_color=GREEN)
        self.logMsg(f"ECM: pack current divided by {p} (parallel branches), "
                    f"temperature taken from “{self.ecmTempSourceName}”.")

    def saveSettings(self):
        try:
            battStart = int(self.settingsEntries["batt_start"].get())
            battCount = int(self.settingsEntries["batt_count"].get())
            currIn = int(self.settingsEntries["curr_in"].get())
            currOut = int(self.settingsEntries["curr_out"].get())
            shuntMohm = float(self.settingsEntries["shunt_mohm"].get().replace(",", "."))
            relayIn = int(self.settingsEntries["relay_in"].get())
            relayOut = int(self.settingsEntries["relay_out"].get())
            resistChannels = []
            for ch, name in self.resistChannels:
                newCh = int(self.settingsEntries[f"temp_{ch}"].get())
                resistChannels.append([newCh, name])
        except (ValueError, KeyError):
            self.settingsStatusLabel.configure(
                text="Invalid value — check the channel numbers.", text_color=RED)
            return

        if battCount < 1 or shuntMohm <= 0:
            self.settingsStatusLabel.configure(
                text="Cell count must be ≥ 1 and shunt > 0 Ω.", text_color=RED)
            return

        resistChNumbers = [ch for ch, _ in resistChannels]
        if len(set(resistChNumbers)) != len(resistChNumbers):
            self.settingsStatusLabel.configure(
                text="Two temperature sensors share the same channel — fix that "
                     "(otherwise the same data would show up under two names).",
                text_color=RED)
            return

        cfg = config.load_config()
        cfg["battery_channels"] = list(range(battStart, battStart + battCount))
        cfg["current_channels"] = {"IN": currIn, "OUT": currOut}
        cfg["shunt_ohms"] = shuntMohm / 1000.0
        cfg["resist_channels"] = resistChannels
        cfg["relay_channels"] = {"IN": relayIn, "OUT": relayOut}
        config.save_config(cfg)

        self.settingsStatusLabel.configure(
            text="Saved. Restart the app for the new channel mapping to take effect.",
            text_color=GREEN)
        self.logMsg("Channel settings saved — restart the app for it to take effect.")

    # ------------------------------------------------------------------
    # DATA EXPORT
    def exportSessionCsv(self):
        if not self.tData:
            self.logMsg("[!] Nothing to export — no data in the current session.")
            return
        path = filedialog.asksaveasfilename(
            title="Export session to CSV", defaultextension=".csv",
            filetypes=[("CSV file", "*.csv")],
            initialfile=f"battery_session_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return

        header = ["Time_s"]
        header += [f"B{i+1:02d}_V" for i in range(len(self.batteryChannels))]
        header += [f"{name}_A" for _, name in self.currentChannels]
        header += [f"{name}_C" for _, name in self.resistChannels]
        header += ["ECM_V", "ECM_SOC_pct"]

        rows = []
        for i, t in enumerate(self.tData):
            row = [f"{t:.2f}"]
            row += [f"{self.battY[k][i]:.4f}" if self.battY[k][i] == self.battY[k][i] else ""
                    for k in range(len(self.batteryChannels))]
            row += [f"{self.currY[k][i]:.3f}" if self.currY[k][i] == self.currY[k][i] else ""
                    for k in range(len(self.currentChannels))]
            row += [f"{self.resY[k][i]:.1f}" if self.resY[k][i] == self.resY[k][i] else ""
                    for k in range(len(self.resistChannels))]
            row += [f"{self.ecmY[i]:.4f}" if self.ecmY[i] == self.ecmY[i] else ""]
            row += [f"{self.ecmSocY[i]:.2f}" if self.ecmSocY[i] == self.ecmSocY[i] else ""]
            rows.append(row)

        export_session_csv(path, header, rows)
        self.logMsg(f"[OK] CSV export: {path}")

    def exportChartImage(self):
        path = filedialog.asksaveasfilename(
            title="Export chart as image", defaultextension=".png",
            filetypes=[("PNG image", "*.png"), ("PDF document", "*.pdf")],
            initialfile=f"battery_chart_{datetime.now():%Y%m%d_%H%M%S}.png")
        if not path:
            return
        self.fig.savefig(path, dpi=150, facecolor=self.fig.get_facecolor())
        self.logMsg(f"[OK] Chart exported: {path}")

    # ------------------------------------------------------------------
    # LOAD TEST DATA (replaces a live run -- only while not measuring)
    def loadTestFile(self):
        if self.measuring:
            self.logMsg("[!] Can't load test data while a measurement is running.")
            return

        if self.tData:
            proceed = messagebox.askyesno(
                "Overwrite current data?",
                "The current data in the chart will be replaced with the file's contents. "
                "Continue?")
            if not proceed:
                return

        path = filedialog.askopenfilename(
            title="Load test data", initialdir=self.fileLogger.outDir,
            filetypes=[("Text logs", "*.txt"), ("All files", "*.*")])
        if not path:
            return

        header, columns = parse_log_file(path)
        required = (["Datum", "Cas"]
                    + [f"B{i+1:02d}_V" for i in range(len(self.batteryChannels))]
                    + ["ProudIN_A", "ProudOUT_A"]
                    + [f"T_{name}" for _, name in self.resistChannels])
        missing = [c for c in required if c not in columns]
        if missing:
            self.logMsg(f"[ERROR] File is missing expected columns: {', '.join(missing)}")
            return
        if not columns["Datum"]:
            self.logMsg("[!] File contains no data.")
            return

        # Same physical plausibility check as the live measurement (see parseNumeric
        # above) -- old saved files can contain contaminated/spliced values (see the
        # comment on INTER_COMMAND_DELAY) that live parsing drops but raw .txt logs
        # still carry. Without this filter, such a glitch (e.g. hundreds of amps) would
        # go straight into the ECM/thermal computation and skew it (P_loss ~ I²).
        currentRangeA = CURRENT_SHUNT_RANGE_V * RANGE_MARGIN / self.shuntOhms

        def parseFloat(raw, rangeLimit=None):
            try:
                v = float(raw)
            except (TypeError, ValueError):
                return float("nan")
            if not math.isfinite(v) or (rangeLimit is not None and abs(v) > rangeLimit):
                return float("nan")
            return v

        timestamps = []
        for d, c in zip(columns["Datum"], columns["Cas"]):
            try:
                timestamps.append(datetime.strptime(f"{d} {c}", "%Y/%m/%d %H:%M:%S"))
            except ValueError:
                timestamps.append(None)

        validIdx = [i for i, ts in enumerate(timestamps) if ts is not None]
        if not validIdx:
            self.logMsg("[ERROR] Could not parse any date/time in the file.")
            return
        t0 = timestamps[validIdx[0]]

        tData, battY = [], [[] for _ in self.batteryChannels]
        currY, resY = [[] for _ in self.currentChannels], [[] for _ in self.resistChannels]

        for i in validIdx:
            tData.append((timestamps[i] - t0).total_seconds())
            for k in range(len(self.batteryChannels)):
                battY[k].append(parseFloat(columns[f"B{k+1:02d}_V"][i],
                                            rangeLimit=BATTERY_RANGE_V * RANGE_MARGIN))
            currY[0].append(parseFloat(columns["ProudIN_A"][i], rangeLimit=currentRangeA))
            currY[1].append(parseFloat(columns["ProudOUT_A"][i], rangeLimit=currentRangeA))
            for k, (_, name) in enumerate(self.resistChannels):
                resY[k].append(parseFloat(columns[f"T_{name}"][i]))

        # -- initial SOC from the rest voltage on the first row (see the ECM card) --
        firstBatt = [battY[k][0] for k in range(len(self.batteryChannels))]
        validFirstBatt = [v for v in firstBatt if v == v]
        avgFirstV = sum(validFirstBatt) / len(validFirstBatt) if validFirstBatt else float("nan")
        firstIn, firstOut = currY[0][0], currY[1][0]
        firstPackCurrent = (firstOut - firstIn) if (firstIn == firstIn and firstOut == firstOut) else float("nan")
        firstNetCurrent = (firstPackCurrent / self.ecmParallelCount
                           if firstPackCurrent == firstPackCurrent else float("nan"))
        tempSourceY = resY[self.ecmTempSourceIndex] if resY else []
        firstTemp = tempSourceY[0] if tempSourceY and tempSourceY[0] == tempSourceY[0] else 25.0

        try:
            fallbackPct = float(self.ecmInitialSocEntry.get().replace(",", "."))
        except ValueError:
            fallbackPct = 100.0

        initialSocPct, wasEstimated, reason = estimateInitialSocPct(
            self.ecm, avgFirstV, firstNetCurrent, firstTemp, fallbackPct)

        if wasEstimated:
            self.logMsg(f"Initial SOC estimated from the rest voltage on the first row "
                        f"({avgFirstV:.3f} V @ {firstTemp:.1f} °C): {initialSocPct:.1f} %.")
        else:
            self.logMsg(f"[!] The first row doesn't look like a battery rest voltage ({reason}) — "
                        f"using the manual initial SOC {initialSocPct:.1f} % "
                        f"(can be changed in the ECM model card).")

        self.ecmInitialSocEntry.delete(0, "end")
        self.ecmInitialSocEntry.insert(0, f"{initialSocPct:.1f}")
        self.ecmInitialSocPct = initialSocPct
        self.ecm.reset(initialSocFraction=initialSocPct / 100.0)

        # -- replay the ECM over the whole loaded current profile (current divided by
        # the number of parallel branches -- the model is per 1 cell/branch) --
        netCurrentSeries = [((currY[1][i] - currY[0][i]) / self.ecmParallelCount)
                             if (currY[0][i] == currY[0][i] and currY[1][i] == currY[1][i])
                             else float("nan") for i in range(len(tData))]
        ecmY, ecmSocY, pLossY = replaySeries(self.ecm, tData, netCurrentSeries,
                                              tempSourceY if tempSourceY else None)
        self._lastNetCurrent = netCurrentSeries[-1] if netCurrentSeries else float("nan")
        self._lastDisplayCurrent = ((currY[1][-1] - currY[0][-1])
                                     if (currY[0] and currY[0][-1] == currY[0][-1]
                                         and currY[1][-1] == currY[1][-1]) else float("nan"))

        self.tData, self.battY, self.currY, self.resY = tData, battY, currY, resY
        self.ecmY, self.ecmSocY, self.ecmPLossY = ecmY, ecmSocY, pLossY
        self._ecmLastTNow = tData[-1] if tData else None

        for k, line in enumerate(self.battLines):
            line.set_data(self.tData, self.battY[k])
        for k, line in enumerate(self.currLines):
            line.set_data(self.tData, self.currY[k])
        for k, line in enumerate(self.resLines):
            line.set_data(self.tData, self.resY[k])
        self.ecmLines[0].set_data(self.tData, self.ecmY)

        # -- thermal batch recompute over the whole P_loss profile from the same ECM run --
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]
        for line in self.fmuLines:
            line.set_data([], [])
        if self.fmuThermal.available:
            validTP = [(tData[i], pLossY[i]) for i in range(len(tData)) if pLossY[i] == pLossY[i]]
            if len(validTP) >= 2:
                validT = [x[0] for x in validTP]
                validP = [x[1] for x in validTP]
                self._fmuLastStatusText = "computing batch…"
                self.fmuStatusLabel.configure(text=f"Status: {self._fmuLastStatusText}")
                self.fmuThermal.startBatch(validT, validP, firstTemp)
                self.logMsg("[FMU] Batch history recompute started in the background…")
            else:
                self.logMsg("[FMU] Not enough valid P_loss samples for a batch recompute.")

        self._hideCrosshair()
        self._rebuildLegend(self.axV, self.battLines + self.ecmLines)
        self._rebuildLegend(self.axI, self.currLines)
        self._rebuildLegend(self.axR, self.resLines + self.fmuLines)
        self._autoscaleCharts()
        self.applyPlotTheme()
        self.canvas.draw_idle()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY, self.fmuTData, self.fmuY)

        self.ecmSocLabel.configure(text=f"SOC: {self.ecm.soc*100:.1f} %")
        self.ecmVoltageLabel.configure(
            text=f"Simulated voltage: {self.ecm.lastVoltage:.3f} V"
            if self.ecm.lastVoltage == self.ecm.lastVoltage else "Simulated voltage: --- V")
        ecmModeText = "Discharging" if self.ecm.lastMode == "discharge" else "Charging"
        self.ecmModeLabel.configure(
            text=f"Mode: {ecmModeText}  ·  Model temperature: {self.ecm.lastTemperatureC:.1f} °C")

        self.logMsg(f"[OK] Loaded {len(tData)} rows from {os.path.basename(path)} "
                    f"(duration {tData[-1] - tData[0]:.0f} s). ECM replayed over the whole current profile.")
        self._refreshHealthPage()

    # ------------------------------------------------------------------
    def loadCurrentProfile(self):
        """Loads just a current profile (two columns: time [s], current [A] -- same
        format as ROM_pack/FMU/EV_minus.txt, no header) and runs it through the ECM +
        thermal model, without needing a real log with all the sensors. Sign convention:
        I > 0 = discharge.

        The current convention is asked explicitly because it differs from file to file:
        - EV_minus.txt (ROM_pack) is a validated current PER 1 CELL/BRANCH -- goes into
          the ECM unchanged.
        - A real measured/estimated current for the WHOLE PACK (terminal) must be
          divided by `self.ecmParallelCount`, same as the live measurement and
          loadTestFile.
        Mixing this up means computing P_loss ~9x off (P ~ I^2, and the division error
        is a factor of (ecmParallelCount)^2 = 9 at N=3) -- exactly what a user reported
        on 2026-08-30 after testing with EV_minus.txt selected as "pack current".

        Cell voltages and sensor temperatures aren't shown in the chart -- the file
        doesn't contain them. I_IN/I_OUT show the sign-split magnitude of the input
        current (the charging branch on I_IN, discharging on I_OUT) -- matching how
        the bench's real (one-directional) shunts measure, unlike a single signed
        channel."""
        if self.measuring:
            self.logMsg("[!] Can't load a test profile while a measurement is running.")
            return

        if self.tData:
            proceed = messagebox.askyesno(
                "Overwrite current data?",
                "The current data in the chart will be replaced with the file's contents. "
                "Continue?")
            if not proceed:
                return

        path = filedialog.askopenfilename(
            title="Load test current profile (time, pack current)",
            initialdir=self.fileLogger.outDir,
            filetypes=[("Text profiles", "*.txt"), ("All files", "*.*")])
        if not path:
            return

        tData, fileCurrentA = [], []
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    parts = line.replace(",", ".").split()
                    if len(parts) < 2:
                        continue
                    try:
                        tv, iv = float(parts[0]), float(parts[1])
                    except ValueError:
                        continue  # header or comment line -- skip
                    tData.append(tv)
                    fileCurrentA.append(iv)
        except OSError as ex:
            self.logMsg(f"[ERROR] Could not open the file: {ex}")
            return

        if len(tData) < 2:
            self.logMsg("[ERROR] The file contains no valid data (expected two columns "
                        "separated by space/tab: time [s], current [A]).")
            return

        isPackCurrent = messagebox.askyesno(
            "Current convention in the file",
            "Is the current in the file the current of the WHOLE PACK (at the terminals)?\n\n"
            f"Yes — the current is divided by the number of parallel branches (currently "
            f"{self.ecmParallelCount}), same as the live measurement.\n\n"
            "No — the current in the file is already per 1 cell/branch "
            "(e.g. EV_minus.txt from ROM_pack) and goes into the ECM unchanged.")

        ambientDialog = ctk.CTkInputDialog(
            text="Ambient / initial pack temperature [°C]:\n(the profile has no temperature sensors)",
            title="Test current profile")
        ambientStr = ambientDialog.get_input()
        if ambientStr is None:
            self.logMsg("Loading the test current profile was cancelled.")
            return
        try:
            ambientC = float(ambientStr.replace(",", "."))
        except ValueError:
            ambientC = 25.0
            self.logMsg(f"[!] Invalid temperature “{ambientStr}”, using {ambientC:.1f} °C.")

        try:
            initialSocPct = float(self.ecmInitialSocEntry.get().replace(",", "."))
        except ValueError:
            initialSocPct = 100.0
        self.ecm.reset(initialSocFraction=initialSocPct / 100.0)

        # -- I_IN/I_OUT for the chart: sign-split magnitude (one-directional shunts,
        # same as the real HW) -- I > 0 (discharge) on OUT, I < 0 (charge) on IN.
        currY = [[max(-v, 0.0) for v in fileCurrentA], [max(v, 0.0) for v in fileCurrentA]]
        battY = [[float("nan")] * len(tData) for _ in self.batteryChannels]
        resY = [[float("nan")] * len(tData) for _ in self.resistChannels]

        # -- current fed to the ECM: divided by ecmParallelCount exactly once per the
        # convention picked above, or unchanged if the file is already per 1 cell/branch --
        netCurrentSeries = ([v / self.ecmParallelCount for v in fileCurrentA] if isPackCurrent
                            else list(fileCurrentA))
        self._lastNetCurrent = netCurrentSeries[-1] if netCurrentSeries else float("nan")
        self._lastDisplayCurrent = fileCurrentA[-1] if fileCurrentA else float("nan")
        tempSourceY = [ambientC] * len(tData)
        ecmY, ecmSocY, pLossY = replaySeries(self.ecm, tData, netCurrentSeries, tempSourceY)

        self.tData, self.battY, self.currY, self.resY = tData, battY, currY, resY
        self.ecmY, self.ecmSocY, self.ecmPLossY = ecmY, ecmSocY, pLossY
        self._ecmLastTNow = tData[-1] if tData else None

        for k, line in enumerate(self.battLines):
            line.set_data([], [])
        for k, line in enumerate(self.currLines):
            line.set_data(self.tData, self.currY[k])
        for k, line in enumerate(self.resLines):
            line.set_data([], [])
        self.ecmLines[0].set_data(self.tData, self.ecmY)

        # -- thermal batch recompute over the whole P_loss profile from the same ECM run --
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]
        for line in self.fmuLines:
            line.set_data([], [])
        if self.fmuThermal.available:
            validTP = [(tData[i], pLossY[i]) for i in range(len(tData)) if pLossY[i] == pLossY[i]]
            if len(validTP) >= 2:
                validT = [x[0] for x in validTP]
                validP = [x[1] for x in validTP]
                self._fmuLastStatusText = "computing batch…"
                self.fmuStatusLabel.configure(text=f"Status: {self._fmuLastStatusText}")
                self.fmuThermal.startBatch(validT, validP, ambientC)
                self.logMsg("[FMU] Batch recompute of the test profile started in the background…")
            else:
                self.logMsg("[FMU] Not enough valid P_loss samples for a batch recompute.")

        self._hideCrosshair()
        self._rebuildLegend(self.axV, self.battLines + self.ecmLines)
        self._rebuildLegend(self.axI, self.currLines)
        self._rebuildLegend(self.axR, self.resLines + self.fmuLines)
        self._autoscaleCharts()
        self.applyPlotTheme()
        self.canvas.draw_idle()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY, self.fmuTData, self.fmuY)

        self.ecmSocLabel.configure(text=f"SOC: {self.ecm.soc*100:.1f} %")
        self.ecmVoltageLabel.configure(
            text=f"Simulated voltage: {self.ecm.lastVoltage:.3f} V"
            if self.ecm.lastVoltage == self.ecm.lastVoltage else "Simulated voltage: --- V")
        ecmModeText = "Discharging" if self.ecm.lastMode == "discharge" else "Charging"
        self.ecmModeLabel.configure(
            text=f"Mode: {ecmModeText}  ·  Model temperature: {self.ecm.lastTemperatureC:.1f} °C")

        convText = (f"PACK current, divided by {self.ecmParallelCount}" if isPackCurrent
                   else "already per 1 cell/branch, unchanged")
        self.logMsg(f"[OK] Loaded current profile {os.path.basename(path)}: {len(tData)} samples, "
                    f"duration {tData[-1] - tData[0]:.0f} s, current in file "
                    f"{min(fileCurrentA):.2f}..{max(fileCurrentA):.2f} A ({convText}), "
                    f"fed to ECM {min(netCurrentSeries):.2f}..{max(netCurrentSeries):.2f} A, "
                    f"ambient temperature {ambientC:.1f} °C, initial SOC {initialSocPct:.1f} %. "
                    f"Voltages and sensor temperatures aren't in the chart (not in the file).")
        self._refreshHealthPage()

    # ------------------------------------------------------------------
    # HISTORICAL LOG VIEWER / SCPI CONSOLE / DEBUG LOG
    def openLogViewer(self):
        from log_viewer import LogViewerWindow
        LogViewerWindow(self.root, initialDir=self.fileLogger.outDir,
                         ecmDischargeJson=os.path.join(self.scriptDir, ECM_DISCHARGE_JSON),
                         ecmChargeJson=os.path.join(self.scriptDir, ECM_CHARGE_JSON),
                         ecmParallelCount=self.ecmParallelCount,
                         ecmTempSourceName=self.ecmTempSourceName)

    def openScpiConsole(self):
        if self.measuring:
            proceed = messagebox.askyesno(
                "Stop measurement?",
                "Opening the SCPI console will stop the running measurement. Continue?")
            if not proceed:
                return
            self.stopMeasurement()
            self.logMsg("Measurement stopped to open the SCPI console.")

        from scpi_console import ScpiConsoleWindow
        ScpiConsoleWindow(self.root, self.sendRawCommand)

    def openLogWindow(self):
        """Debug log -- kept out of the way on purpose (see MAX_LOG_LINES buffer in
        logMsg): day-to-day operation doesn't need it, so it's a window opened on
        demand instead of a permanent sidebar tab."""
        if self.logWindow is not None and self.logWindow.winfo_exists():
            self.logWindow.focus()
            return
        self.logWindow = LogWindow(self.root, self._logLines,
                                   on_close=lambda: setattr(self, "logWindow", None))

    def sendRawCommand(self, cmd):
        if self.sPort is None:
            return None
        with self.portLock:
            try:
                time.sleep(INTER_COMMAND_DELAY)
                self.sPort.reset_input_buffer()
                self.sPort.write((cmd + "\r\n").encode("ascii"))
                line = self.sPort.readline().decode("ascii", errors="ignore").strip()
                return line if line else "(empty reply / timeout)"
            except Exception as ex:
                return f"(error: {ex})"

    # ------------------------------------------------------------------
    # QUERY A SINGLE CHANNEL (called from any thread, protected by the lock)
    def _query(self, cmd, rangeLimit=None):
        with self.portLock:
            try:
                # Give the instrument time to settle the multiplexer/ADC after the
                # previous command (see the comment on INTER_COMMAND_DELAY) and discard
                # anything left unread from the previous (possibly timed-out) query.
                time.sleep(INTER_COMMAND_DELAY)
                self.sPort.reset_input_buffer()
                self.sPort.write((cmd + "\r\n").encode("ascii"))
                line = self.sPort.readline().decode("ascii", errors="ignore").strip()
                value = parseNumeric(line, rangeLimit)
                if value is not None:
                    return value
            except (serial.SerialException, OSError):
                self._portLost = True
            except Exception:
                pass
        return float("nan")

    def _autoSetRelay(self, ch, newState, reason):
        cmd = f"SET:OUTput{ch} {1 if newState else 0}"
        with self.portLock:
            try:
                time.sleep(INTER_COMMAND_DELAY)
                self.sPort.write((cmd + "\r\n").encode("ascii"))
            except Exception:
                return
        self.relayState[ch] = newState
        self.autoEventQueue.put(("relay", ch, newState, reason))

    def _attemptReconnect(self):
        portName = self.sPort.port if self.sPort is not None else None
        if not portName:
            return False
        try:
            with self.portLock:
                try:
                    self.sPort.close()
                except Exception:
                    pass
                self.sPort = serial.Serial(portName, BAUD_RATE, timeout=0.4)
                self.sPort.reset_input_buffer()
                self.sPort.reset_output_buffer()
            self._portLost = False
            self.autoEventQueue.put(("reconnect", None, True, f"Port {portName} reconnected."))
            return True
        except Exception as ex:
            self.autoEventQueue.put(
                ("reconnect", None, False, f"Port {portName} unavailable, retrying… ({ex})"))
            return False

    # ------------------------------------------------------------------
    # BACKGROUND THREAD: ONE MEASUREMENT CYCLE OVER ALL CHANNEL GROUPS
    def _workerLoop(self):
        while not self.stopEvent.is_set():
            if self._portLost:
                if not self._attemptReconnect():
                    time.sleep(1.0)
                    continue

            cycleStart = time.time()
            period = self.pollPeriod
            speed = "Fast" if period < SLOW_THRESHOLD else "Slow"

            with self.portLock:
                try:
                    self.sPort.timeout = FAST_READ_TIMEOUT if speed == "Fast" else SLOW_READ_TIMEOUT
                except Exception:
                    pass

            battery = []
            for ch in self.batteryChannels:
                if self.stopEvent.is_set():
                    return
                battery.append(self._query(f"MEASure:VOLTage{ch}? 15V,{speed}",
                                            rangeLimit=BATTERY_RANGE_V * RANGE_MARGIN))

            currents = []
            for ch, _ in self.currentChannels:
                if self.stopEvent.is_set():
                    return
                v = self._query(f"MEASure:VOLTage{ch}? 0V15,{speed}",
                                 rangeLimit=CURRENT_SHUNT_RANGE_V * RANGE_MARGIN)
                currents.append(v / self.shuntOhms if v == v else float("nan"))

            resistances = []
            for ch, name in self.resistChannels:
                if self.stopEvent.is_set():
                    return
                rawOhm = self._query(f"MEASure:RESistance{ch}? 200k,{speed}",
                                      rangeLimit=RESIST_RANGE_OHM * RANGE_MARGIN)
                resistances.append(resistanceToCelsius(name, rawOhm))

            if self.cutoffEnabled and not self._portLost:
                validV = [v for v in battery if v == v]
                voltageSum = sum(validV) if validV else float("nan")
                isOutOn = self.relayState.get(self.relayOutCh, False)
                action = decide_relay_action(voltageSum, isOutOn, self.cutoffOffV, self.cutoffOnV)
                if action is False:
                    self._autoSetRelay(self.relayOutCh, False,
                                        f"voltage sum {voltageSum:.2f} V ≤ {self.cutoffOffV:.2f} V")
                elif action is True:
                    self._autoSetRelay(self.relayOutCh, True,
                                        f"voltage sum {voltageSum:.2f} V ≥ {self.cutoffOnV:.2f} V")

            tNow = time.perf_counter() - self.tStart
            self.dataQueue.put((tNow, battery, currents, resistances))

            elapsed = time.time() - cycleStart
            if not self.stopEvent.is_set():
                time.sleep(max(0.0, period - elapsed))

    # ------------------------------------------------------------------
    # QUEUE PROCESSING ON THE MAIN (GUI) THREAD
    def _pollQueue(self):
        updated = False
        while True:
            try:
                tNow, battery, currents, resistances = self.dataQueue.get_nowait()
            except queue.Empty:
                break
            self._applyMeasurement(tNow, battery, currents, resistances)
            updated = True

        while True:
            try:
                tag, ch, state, reason = self.autoEventQueue.get_nowait()
            except queue.Empty:
                break
            if tag == "reconnect":
                if state:
                    self.logMsg(f"✅ {reason}")
                    self._setStatus("Measuring", GREEN)
                else:
                    self.logMsg(f"⚠️ {reason}")
                    self._setStatus("Port lost, recovering…", ORANGE)
            elif tag == "relay":
                self.logMsg(f"[AUTO] CH{ch} -> {'ON' if state else 'OFF'} — {reason}")
                if ch in self.relaySwitches:
                    _, var = self.relaySwitches[ch]
                    var.set(state)

        fmuUpdated = False
        fmuLegendRebuildNeeded = False
        while True:
            try:
                tag, tLast, temps, extra = self.fmuResultQueue.get_nowait()
            except queue.Empty:
                break
            if tag == "ok":
                self.fmuTData.append(tLast)
                for k in range(FMU_N_CELLS):
                    self.fmuY[k].append(temps[k])
                self._fmuLastStatusText = f"updated (computed in {extra:.1f} s)"
                self.logMsg(f"[FMU] recompute done ({extra:.1f} s), t={tLast:.0f} s, "
                            f"cell 1 = {temps[0]:.2f} °C")
            elif tag == "error":
                self._fmuLastStatusText = f"recompute error — retrying ({extra})"
                self.logMsg(f"[FMU] [!] recompute failed, retrying: {extra}")
            elif tag == "batch_ok":
                tOut, allTemps = tLast, temps  # batch: tLast=[times], temps=[[24 temps] per time]
                self.fmuTData = list(tOut)
                self.fmuY = [[row[k] for row in allTemps] for k in range(FMU_N_CELLS)]
                flatVals = [v for row in allTemps for v in row if v == v]
                if flatVals:
                    tMin, tMax = min(flatVals), max(flatVals)
                    rangeTxt = f", temperature {tMin:.2f}–{tMax:.2f} °C (Δ={tMax - tMin:.3f} °C)"
                else:
                    rangeTxt = ""
                self._fmuLastStatusText = f"batch done ({len(tOut)} samples){rangeTxt}"
                self.logMsg(f"[FMU] Batch history recompute done ({len(tOut)} samples){rangeTxt}.")
                if flatVals and (tMax - tMin) < 0.05:
                    self.logMsg("[FMU] [i] The temperature range is very small — for this current "
                                "profile the power loss is only tens of mW, so the actual heating "
                                "is negligible (the chart curve will look almost flat, not like "
                                "nothing is being computed).")
                fmuLegendRebuildNeeded = True
            elif tag == "batch_error":
                self._fmuLastStatusText = f"batch failed ({extra})"
                self.logMsg(f"[FMU] [!] Batch history recompute failed: {extra}")
            self.fmuStatusLabel.configure(text=f"Status: {self._fmuLastStatusText}")
            fmuUpdated = True

        if updated:
            for k, line in enumerate(self.battLines):
                line.set_data(self.tData, self.battY[k])
            for k, line in enumerate(self.currLines):
                line.set_data(self.tData, self.currY[k])
            for k, line in enumerate(self.resLines):
                line.set_data(self.tData, self.resY[k])
            self.ecmLines[0].set_data(self.tData, self.ecmY)
            if self.healthPage.winfo_ismapped():
                self._refreshHealthPage()

        if fmuUpdated:
            for k, line in enumerate(self.fmuLines):
                line.set_data(self.fmuTData, self.fmuY[k])
            if fmuLegendRebuildNeeded:
                self._rebuildLegend(self.axR, self.resLines + self.fmuLines)
                self.applyPlotTheme()
            if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
                self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY,
                                                  self.fmuTData, self.fmuY)

        if updated or fmuUpdated:
            self._autoscaleCharts()
            self.canvas.draw_idle()

        self.root.after(50, self._pollQueue)

    def _applyMeasurement(self, tNow, battery, currents, resistances):
        self.tData.append(tNow)
        for k, v in enumerate(battery):
            self.battY[k].append(v)
        for k, v in enumerate(currents):
            self.currY[k].append(v)
        for k, v in enumerate(resistances):
            self.resY[k].append(v)

        # -- ECM simulation (1 cell) -- I_bat = I_OUT - I_IN, I>0 = discharge.
        # The pack has self.ecmParallelCount parallel branches, the model is per
        # 1 cell/branch, so the current is divided by the branch count (assuming
        # even current sharing between them). --
        packCurrent = currents[1] - currents[0] if len(currents) > 1 else float("nan")
        netCurrent = packCurrent / self.ecmParallelCount if packCurrent == packCurrent else float("nan")
        self._lastNetCurrent = netCurrent
        self._lastDisplayCurrent = packCurrent
        ecmTemp = resistances[self.ecmTempSourceIndex] if resistances else float("nan")

        ecmDt = None if self._ecmLastTNow is None else tNow - self._ecmLastTNow
        self._ecmLastTNow = tNow
        if ecmDt is not None and ecmDt > 0:
            ecmVoltage = self.ecm.step(netCurrent, ecmDt, ecmTemp)
            if self.fmuThermal.isRunning():
                self.fmuThermal.addSample(tNow, self.ecm.lastPLossW)
        else:
            ecmVoltage = self.ecm.lastVoltage
        self.ecmY.append(ecmVoltage)
        self.ecmSocY.append(self.ecm.soc * 100.0)
        self.ecmPLossY.append(self.ecm.lastPLossW)

        if ecmVoltage == ecmVoltage:
            self.ecmVoltageLabel.configure(text=f"Simulated voltage: {ecmVoltage:.3f} V")
        else:
            self.ecmVoltageLabel.configure(text="Simulated voltage: --- V")
        ecmModeText = "Discharging" if self.ecm.lastMode == "discharge" else "Charging"
        self.ecmModeLabel.configure(
            text=f"Mode: {ecmModeText}  ·  Model temperature: {self.ecm.lastTemperatureC:.1f} °C")
        self.ecmSocLabel.configure(text=f"SOC: {self.ecm.soc*100:.1f} %")

        if len(self.tData) > MAX_POINTS:
            self.tData = self.tData[-MAX_POINTS:]
            self.battY = [y[-MAX_POINTS:] for y in self.battY]
            self.currY = [y[-MAX_POINTS:] for y in self.currY]
            self.resY = [y[-MAX_POINTS:] for y in self.resY]
            self.ecmY = self.ecmY[-MAX_POINTS:]
            self.ecmSocY = self.ecmSocY[-MAX_POINTS:]
            self.ecmPLossY = self.ecmPLossY[-MAX_POINTS:]

        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY,
                                              self.fmuTData, self.fmuY)

        validV = [v for v in battery if v == v]
        voltageSum = sum(validV) if validV else float("nan")
        if voltageSum == voltageSum:
            self.cutoffVoltageLabel.configure(text=f"Current voltage sum: {voltageSum:.2f} V")
        else:
            self.cutoffVoltageLabel.configure(text="Current voltage sum: --- V")

        battParts = [f"B{i+1:02d}:{v:6.3f}V" if v == v else f"B{i+1:02d}: ---"
                     for i, v in enumerate(battery)]
        currParts = [f"{name}:{v:7.3f}A" if v == v else f"{name}: ---"
                     for (ch, name), v in zip(self.currentChannels, currents)]
        resParts = [f"{name}:{v:6.1f}°C" if v == v else f"{name}: ---"
                    for (ch, name), v in zip(self.resistChannels, resistances)]

        self.logMsg(f"[{tNow:06.1f}s] " + " | ".join(battParts) +
                    " || " + " | ".join(currParts) +
                    " || " + " | ".join(resParts))

        now = time.time()
        if now - self._lastFileLogTime >= FILE_LOG_PERIOD:
            self._lastFileLogTime = now
            currentIN = currents[0] if len(currents) > 0 else float("nan")
            currentOUT = currents[1] if len(currents) > 1 else float("nan")
            self.fileLogger.logCycle(datetime.now(), battery, currentIN, currentOUT,
                                      resistances, self.relayState.get(self.relayInCh, False),
                                      self.relayState.get(self.relayOutCh, False))

    # ------------------------------------------------------------------
    # RELAYS (manual control)
    def toggleRelay(self, ch, var):
        if self.sPort is None:
            self.logMsg("[!] Can't switch a relay without an active connection.")
            var.set(self.relayState[ch])
            return

        newState = bool(var.get())
        cmd = f"SET:OUTput{ch} {1 if newState else 0}"
        with self.portLock:
            try:
                time.sleep(INTER_COMMAND_DELAY)
                self.sPort.write((cmd + "\r\n").encode("ascii"))
            except Exception as ex:
                self.logMsg(f"[ERROR] Could not send relay command CH{ch}: {ex}")
                var.set(self.relayState[ch])
                return

        self.relayState[ch] = newState
        name = dict(self.relayChannels)[ch]
        self.logMsg(f"Relay CH{ch} ({name}) -> {'ON' if newState else 'OFF'} ({cmd})")

    # ------------------------------------------------------------------
    # STOP MEASUREMENT (the port stays connected -- see disconnect() to fully disconnect)
    def stopMeasurement(self):
        if self.stopEvent is not None:
            self.stopEvent.set()
        if self.workerThread is not None and self.workerThread.is_alive():
            self.workerThread.join(timeout=1.0)
        self.workerThread = None
        self.stopEvent = None
        self._portLost = False

        if self.fmuThermal.isRunning():
            self.fmuThermal.stop()
            self._fmuLastStatusText = "Stopped (measurement ended)"
            self.fmuStatusLabel.configure(text=f"Status: {self._fmuLastStatusText}")

        self.measuring = False
        self.btnStart.configure(text="▶  Start measurement", **self._btnStartDefaults)
        self.btnLoadTestFile.configure(state="normal")
        self.btnLoadCurrentProfile.configure(state="normal")
        if self.sPort is not None:
            self.btnConnect.configure(state="normal")
            self._setStatus(f"Connected ({self.sPort.port})", GREEN)
        else:
            self._setStatus("Disconnected", GRAY)
        self._refreshRelayAvailability()

    # ------------------------------------------------------------------
    # CLEAR DATA
    def clearData(self):
        self.tStart = time.perf_counter()
        self.tData = []
        self.battY = [[] for _ in self.batteryChannels]
        self.currY = [[] for _ in self.currentChannels]
        self.resY = [[] for _ in self.resistChannels]
        self.ecmY = []
        self.ecmSocY = []
        self.ecmPLossY = []
        self._ecmLastTNow = None  # ECM state (SOC/v1/v2) isn't cleared, just the chart history
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]  # thermal P_loss history isn't cleared either

        for line in self.battLines + self.currLines + self.resLines + self.ecmLines + self.fmuLines:
            line.set_data([], [])
        self._hideCrosshair()
        self._autoscaleCharts()
        self.canvas.draw_idle()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY, self.fmuTData, self.fmuY)
        self._setLogLines(["Data cleared."])
        self._refreshHealthPage()

    # ------------------------------------------------------------------
    # DEBUG LOG
    # Message history lives in a small capped buffer, independent of whether the log
    # window (see openLogWindow) is currently open -- it's day-to-day noise for normal
    # operation, so it isn't shown by default, but nothing is lost while it's closed.
    def logMsg(self, msg):
        self._logLines.append(msg)
        if len(self._logLines) > MAX_LOG_LINES:
            del self._logLines[:len(self._logLines) - MAX_LOG_LINES]
        if self.logWindow is not None and self.logWindow.winfo_exists():
            self.logWindow.append(msg)

    def _setLogLines(self, lines):
        self._logLines = list(lines)
        if self.logWindow is not None and self.logWindow.winfo_exists():
            self.logWindow.set_lines(self._logLines)

    # ------------------------------------------------------------------
    # WINDOW CLOSE
    def onClose(self):
        self.disconnect()
        if self._estimationAfterId is not None:
            self.root.after_cancel(self._estimationAfterId)
        if self.fmuThermal.isRunning():
            self.fmuThermal.stop()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.destroy()
        if self.logWindow is not None and self.logWindow.winfo_exists():
            self.logWindow.destroy()
        self.root.destroy()


def main():
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    root = ctk.CTk()
    MonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
