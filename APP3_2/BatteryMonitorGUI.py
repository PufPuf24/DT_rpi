"""
Battery Digital Twin -- with ECM voltage simulation and a pack thermal model.
Python translation/extension of BatteryMonitorGUI.m

Dependencies:
    pip install pyserial matplotlib customtkinter scipy numpy
    pip install scikit-learn joblib pandas   # SoH: soh/ (opportunistic FFNN estimator)
    (fmpy/Ansys are no longer needed -- the thermal model is pack_rom_thermal_model.py,
    pure NumPy, see thermal_rom/. fmu_thermal_model.py is kept only for reference and
    is not imported by this file.)

SoH degradation estimation (Battery Health page): see soh/README.md for the full
design -- ground-truth Maintenance validation cycle (Coulomb counting, this is the
trusted number) vs. two opportunistic, lower-trust estimators that share one
per-group SohTracker: the tV-window FFNN from C:\\code2\\python, and a "quick field"
short-window (10-30 min) FFNN calibrated directly at the bench's own ~7 A native
rate (ported from C:\\code2\\MTB's Peukert/field-calibration study). All wired in
and tested.

SOC estimation (ECM model card): two independent estimates run side by side --
ecm_model.EcmModel (self.ecm, open-loop Coulomb counting, kept unfused on purpose so
its "Simulated voltage" stays a fair comparison against the real measurement) and
ecm_ekf.EcmEkfEstimator (self.ecmEkf, a 2RC ECM + EKF ported from the dissertation's
ECM_EKF_3DISS_FIG.m, fusing the real measured voltage to correct SOC drift -- live
measurement only, see the "(EKF)" label).
"""

import bisect
import copy
import csv
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
import matplotlib.dates as mdates
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import joblib
import pandas as pd

import config
import soh_store
from battery_file_logger import BatteryFileLogger
from cutoff_logic import decide_relay_action
from data_tools import export_session_csv, parse_log_file
from maintenance_cycle import MaintenanceCycle
from maintenance_cycle import Phase as MaintPhase
from soh import config as sohConfig
from soh.curves import voltage_window_time
from soh.field_window import capacity_window_dv, window_capacity_coverage
from soh.online import GateSettings, Observation, SohTracker, SteadyStateGate, Tier, Verdict, scan_segments, window_coverage
from theme import ACCENT, DARK, GRAY, GREEN, LIGHT, ORANGE, RED, YELLOW, dual, tokens_for_mode
from thermistor import resistanceToCelsius
from ecm_model import EcmModel, estimateInitialSocPct, replaySeries
from ecm_ekf import EcmEkfEstimator
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

# Estimation ("digital twin") forward projection. Horizon is chosen on a slider (plus a
# manual minutes entry) instead of a fixed dropdown -- 1-minute steps below 1 h (where the
# resolution actually matters), 30-minute steps from 1 h up to the 24 h ceiling.
ESTIMATION_MINUTES_STEPS = list(range(1, 61)) + list(range(90, 24 * 60 + 1, 30))
ESTIMATION_DEFAULT_MIN = 120  # 2 h, same default as before
ESTIMATION_DT_S = 10.0          # projection sample spacing -- smooth without being wasteful
ESTIMATION_RECOMPUTE_MS = 2000  # how often the projection refreshes while switched on

# Opportunistic SoH (Phase 2, see soh/README.md) -- how often to rescan the live
# history for a usable voltage-window segment. scan_segments/window_coverage over
# up to MAX_POINTS samples is cheap, but there is no reason to redo it every 50 ms
# poll tick -- a genuinely new CC segment doesn't appear that fast.
SOH_OPPORTUNISTIC_SCAN_S = 30.0
# An opportunistic observation this old is logged but excluded from fusion --
# belt-and-braces safety cap alongside SohTracker's own Kalman drift widening
# (see soh/README.md's Phase 3 section for why this is a cap, not the main
# mechanism). The user was unsure whether a week or a month was right and ruled
# out longer than a month; 30 days is the stated upper bound, not a guess.
SOH_MAX_OBSERVATION_AGE_DAYS = 30.0
# A combination's windows must all come from segments within this long of each
# other -- otherwise a charge window from an hour ago and a discharge window
# from just now could get combined as if they were one coherent measurement,
# exactly the "don't combine features across too different conditions/time"
# concern raised for this estimator.
SOH_COMBO_MAX_TIME_GAP_S = 3600.0
# Every current deployment_package.joblib combination is characterised at the
# dissertation's C01 case (0.1C/0.1C, see soh/README.md) -- this bench cannot
# reliably run at that rate, which is exactly why GateSettings.crate_tolerance
# is widened rather than left at its 35% default (see soh/online.py).
SOH_REFERENCE_CRATE = 0.1
# FFNN combinations don't have a single-feature correlation (rho) the way
# soh.online.Tier was designed around -- bridge via the package's own static
# accuracy band instead (see _judgeAndFuseFfnnObservation's docstring).
SOH_FFNN_BAND_TIER = {"accept": Tier.HIGH, "marginal": Tier.MEDIUM}

# Safety-net relay-off verification (see _measureRelayCurrent / _scheduleRelayOffVerification):
# SET:OUTput is fire-and-forget over SCPI, so this is the only way the app can notice a relay
# that didn't actually respond (stuck/welded contacts, dead coil, ...) instead of silently
# trusting its own last command. Not a safety interlock by itself -- just makes a failure loud
# instead of invisible. Delay must clear typical relay/contact-bounce and let any residual
# inductive-load current decay before judging it a real fault, not settling noise.
RELAY_OFF_VERIFY_DELAY_S = 1.5
RELAY_OFF_VERIFY_CURRENT_A = 0.5

BAUD_RATE = 115200
MAX_POINTS = 1000
# Separate, much larger retention for the quick-field SoH scan (see
# _scanFieldWindowSoh) -- MAX_POINTS exists for chart responsiveness and would
# throw away the early part of a multi-hour discharge long before the 10-30 min
# window at a LATE capacity-fraction position (e.g. 70%) is even reached,
# discovered exactly this way in testing: a real 8h discharge left only the
# final ~80 min under MAX_POINTS, so the 10%/40% windows never had data left to
# find, only the 70% one. This buffer is small per-sample (5 floats/group) so a
# much longer retention costs little.
SOH_FIELD_HISTORY_MAX_POINTS = 20000
MAX_LOG_LINES = 200
FILE_LOG_PERIOD = 30.0  # s, how often a line gets written to the text log files
FILE_LOG_PERIOD_FAST = 5.0  # s, period of the optional Battery_monitor_output_5s.txt

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


def _formatHorizonMinutes(minutes):
    """Compact label for the Estimation horizon slider/log messages, e.g. 45 -> '45 min',
    120 -> '2 h', 150 -> '2h30'."""
    if minutes < 60:
        return f"{minutes} min"
    h, m = divmod(minutes, 60)
    return f"{h} h" if m == 0 else f"{h}h{m:02d}"


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
        self.fastFileLogEnabled = bool(cfg["fast_file_log_enabled"])

        # State variables
        self.sPort = None
        self.portLock = threading.Lock()
        self.tStart = None
        # Separate from self.tStart on purpose -- toggleConnection() also sets tStart
        # (as a t=0 reference for anything timestamped before Start is clicked, e.g. a
        # manual relay toggle), which used to silently defeat toggleMeasurement()'s own
        # "is this the actual first start" check further down (both tested the SAME
        # `self.tStart is None`, so by the time Start was clicked it was never None
        # anymore -- the rest-voltage initial-SOC estimate never ran for a live
        # connect+start session, only for loadTestFile(), which doesn't use this guard
        # at all). Reset on disconnect so a later reconnect gets a fresh estimate.
        self._socEstimatedThisSession = False
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
        self.ecmEkfSocY = []  # NaN where the EKF wasn't fed a real measurement (see _applyMeasurement)
        self.ecmPLossY = []

        self.scriptDir = os.path.dirname(os.path.abspath(__file__))
        self.ecmInitialSocPct = float(cfg.get("ecm_initial_soc_pct", 100.0))
        self.ecmParallelCount = max(1, int(cfg.get("ecm_parallel_count", 3)))
        self.ecmTempSourceName = cfg.get("ecm_temp_source", "T3")
        self._updateEcmTempSourceIndex()
        self.ecm = EcmModel(os.path.join(self.scriptDir, ECM_DISCHARGE_JSON),
                             os.path.join(self.scriptDir, ECM_CHARGE_JSON),
                             initialSocFraction=self.ecmInitialSocPct / 100.0)
        # SOC via EKF (voltage-corrected, see ecm_ekf.py) -- runs alongside self.ecm,
        # not instead of it: self.ecm stays open-loop on purpose (its "Simulated
        # voltage" is compared AGAINST the real measurement on the chart; feeding it
        # that same measurement would defeat the comparison). Only meaningful live
        # (needs a real measured voltage each step), not during the Estimation
        # projection, which has no future measurement to correct against.
        self.ecmEkf = EcmEkfEstimator(os.path.join(self.scriptDir, ECM_DISCHARGE_JSON),
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
        self.estimationHorizonMin = ESTIMATION_DEFAULT_MIN
        self._estimationAfterId = None
        self.customVMin = float(cfg.get("custom_v_min", CELL_ABS_V_MIN))
        self.customVMax = float(cfg.get("custom_v_max", CELL_ABS_V_MAX))
        self.customTMin = float(cfg.get("custom_t_min", CELL_ABS_T_MIN))
        self.customTMax = float(cfg.get("custom_t_max", CELL_ABS_T_MAX))

        # -- SoH: Maintenance validation cycle (ground-truth, Coulomb-counted) --
        # see maintenance_cycle.py / soh/README.md. Loaded here so a prior session's
        # SoH survives a restart, as specified.
        self.maintenanceDischargeCutoffV = float(cfg.get("maintenance_discharge_cutoff_v", 3.35))
        self.maintenanceChargeFullV = float(cfg.get("maintenance_charge_full_v", 4.2))
        self.sohState = soh_store.load_state(self.scriptDir)  # {"0": {...}, "1": {...}, ...}
        self.maintenanceCycle = None
        self._maintenanceAfterId = None

        # -- SoH: opportunistic live estimator (Phase 2, secondary/interim to the
        # Maintenance cycle above -- see soh/README.md). Loads the pretrained
        # deployment package once; missing/unloadable just turns the feature off,
        # same degrade-gracefully pattern as self.fmuThermal.available. --
        self.sohDeployment = None
        self.sohDeploymentError = None
        try:
            self.sohDeployment = joblib.load(
                os.path.join(self.scriptDir, "soh", "deployment_package.joblib"))
        except Exception as ex:
            self.sohDeploymentError = str(ex)
        self.sohSteadyGate = self.sohDeployment["cc_gate"] if self.sohDeployment else SteadyStateGate()
        self.sohGateSettings = GateSettings()  # widened crate_tolerance -- see soh/online.py
        n = len(self.batteryChannels)
        self.sohTrackers = []
        for k in range(n):
            saved = self.sohState.get(str(k), {}).get("tracker")
            if saved:
                self.sohTrackers.append(SohTracker(soh=saved["soh"], sigma=saved["sigma"],
                                                    last_efc=saved.get("last_efc", 0.0)))
            else:
                self.sohTrackers.append(SohTracker())
        # Session-only cumulative |Ah| throughput (all groups share one series
        # current, so one counter covers all of them) -- EFC proxy, see
        # _estimateEfc's docstring for the cross-session caveat.
        self._sohAhThroughputAh = 0.0
        self._sohLastScanT = None

        # -- SoH: "quick field" estimator -- short (10-30 min), FIXED-capacity-slice
        # window at this bench's own native ~7A rate, no Peukert extrapolation needed
        # (unlike the MTB e14/e15 0.02C study this is ported from). Feeds the SAME
        # per-group SohTracker as the tV-window estimator above, tagged distinctly and
        # deliberately cautious -- see _scanFieldWindowSoh / soh/README.md. The user's
        # own framing: "rychlá data, ne referenční" (quick data, not reference).
        self.sohFieldDeployment = None
        self.sohFieldDeploymentError = None
        try:
            self.sohFieldDeployment = joblib.load(
                os.path.join(self.scriptDir, "soh", "deployment_package_field.joblib"))
        except Exception as ex:
            self.sohFieldDeploymentError = str(ex)
        # -- dense/early grid: single-feature calibrations at many more (and much
        # earlier) window positions than the coarse combos above, for whatever a
        # live 15-20 min pulse happens to cover "regardless of what" (the user's own
        # framing) -- see 09_field_window_early_grid.py and soh/README.md. Optional:
        # the coarse package above still works fine on its own if this one is missing.
        self.sohFieldGridDeployment = None
        self.sohFieldGridDeploymentError = None
        try:
            self.sohFieldGridDeployment = joblib.load(
                os.path.join(self.scriptDir, "soh", "deployment_package_field_grid.joblib"))
        except Exception as ex:
            self.sohFieldGridDeploymentError = str(ex)
        self.sohFieldSteadyGate = (self.sohFieldDeployment["cc_gate"]
                                    if self.sohFieldDeployment else SteadyStateGate())
        self._sohFieldLastScanT = None
        # Dedicated history for the quick-field scan -- see SOH_FIELD_HISTORY_MAX_POINTS's
        # comment for why this can't just reuse self.tData/self.battY.
        n = len(self.batteryChannels)
        self._sohFieldT = []
        self._sohFieldNetCurrentA = []
        self._sohFieldSocPct = []
        self._sohFieldTempC = []
        self._sohFieldV = [[] for _ in range(n)]

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
        self.fileLogger.setFastLoggingEnabled(self.fastFileLogEnabled)
        self._lastFileLogTime = 0.0
        self._lastFastFileLogTime = 0.0

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

        # CTkScrollableFrame, not a plain CTkFrame -- the page stacks 4 cards
        # (pack health, the B01-Bn grid, Maintenance cycle, SoH history) with no
        # chart pane to absorb a short window the way Monitoring's does, so on a
        # window shorter than all 4 combined the B01-Bn grid used to get squeezed
        # into whatever sliver of row 1 was left over, its labels unreadable, with
        # no way to reach the rest -- reported directly by the user. Same widget
        # already used for the Monitoring sidebar's Controls/Settings pages.
        self.healthPage = ctk.CTkScrollableFrame(self.root, fg_color="transparent")
        self.healthPage.grid_columnconfigure(0, weight=1)

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

        # Horizon control: slider (1 min steps up to 1 h, 30 min steps up to 24 h) plus a
        # manual entry for an exact value the slider's grid can't land on. Both live in one
        # frame so _onEstimationToggle can pack/pack_forget them as a single unit, same as
        # the old dropdown.
        self.estimationHorizonFrame = ctk.CTkFrame(rightBar, fg_color="transparent")

        self.estimationHorizonValueLabel = ctk.CTkLabel(
            self.estimationHorizonFrame, text=_formatHorizonMinutes(self.estimationHorizonMin),
            width=44, font=ctk.CTkFont(size=12), text_color=dual("text"))
        self.estimationHorizonValueLabel.pack(side="left", padx=(0, 4))

        self.estimationHorizonSlider = ctk.CTkSlider(
            self.estimationHorizonFrame, from_=0, to=len(ESTIMATION_MINUTES_STEPS) - 1,
            number_of_steps=len(ESTIMATION_MINUTES_STEPS) - 1, width=130,
            command=self._onEstimationHorizonSlide)
        self.estimationHorizonSlider.set(ESTIMATION_MINUTES_STEPS.index(self.estimationHorizonMin))
        self.estimationHorizonSlider.pack(side="left", padx=(0, 6))

        self.estimationHorizonEntry = ctk.CTkEntry(self.estimationHorizonFrame, width=46)
        self.estimationHorizonEntry.insert(0, str(self.estimationHorizonMin))
        self.estimationHorizonEntry.bind("<Return>", self._onEstimationHorizonEntryChange)
        self.estimationHorizonEntry.bind("<FocusOut>", self._onEstimationHorizonEntryChange)
        self.estimationHorizonEntry.pack(side="left", padx=(0, 4))
        ctk.CTkLabel(self.estimationHorizonFrame, text="min", font=ctk.CTkFont(size=11),
                     text_color=dual("text_secondary")).pack(side="left")
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
        ecmFrame = self._card(parent, "ECM model", expanded=False)

        self.ecmSocLabel = ctk.CTkLabel(ecmFrame, text=f"SOC (Coulomb counting): {self.ecm.soc*100:.1f} %",
                                         font=ctk.CTkFont(size=13, weight="bold"),
                                         text_color=dual("text_secondary"))
        self.ecmSocLabel.pack(anchor="w", padx=12, pady=(0, 2))
        # EKF SOC (see ecm_ekf.py) -- voltage-corrected, live only (needs a real
        # measured voltage each step, see _applyMeasurement); stays "--" for
        # loadTestFile/loadCurrentProfile replay and the Estimation projection.
        self.ecmEkfSocLabel = ctk.CTkLabel(
            ecmFrame, text=f"SOC (EKF): {self.ecmEkf.soc*100:.1f} % ± {self.ecmEkf.socSigma*100:.1f} pp",
            font=ctk.CTkFont(size=13, weight="bold"), text_color=dual("text_secondary"))
        self.ecmEkfSocLabel.pack(anchor="w", padx=12, pady=(0, 2))
        self.ecmVoltageLabel = ctk.CTkLabel(ecmFrame, text="Simulated voltage: --- V",
                                             font=ctk.CTkFont(size=12),
                                             text_color=dual("text_secondary"))
        self.ecmVoltageLabel.pack(anchor="w", padx=12, pady=(0, 2))
        self.ecmModeLabel = ctk.CTkLabel(ecmFrame, text="Mode: —  ·  Model temperature: --- °C",
                                          font=ctk.CTkFont(size=11),
                                          text_color=dual("text_secondary"))
        self.ecmModeLabel.pack(anchor="w", padx=12, pady=(0, 4))

        # -- small SOC sparkline: Coulomb counting vs EKF over this session -- --
        self.ecmSocFig = Figure(dpi=100, figsize=(2.8, 1.3))
        self.ecmSocAx = self.ecmSocFig.add_subplot(111)
        self.ecmSocAx.set_ylabel("SOC [%]", fontsize=8)
        self.ecmSocAx.tick_params(labelsize=7)
        (self.ecmSocLine,) = self.ecmSocAx.plot([], [], "-", linewidth=1.1, color=GRAY,
                                                  label="CC")
        (self.ecmEkfSocLine,) = self.ecmSocAx.plot([], [], "-", linewidth=1.1, color=ACCENT,
                                                     label="EKF")
        self.ecmSocAx.legend(fontsize=7, loc="lower left", frameon=False)
        self.ecmSocFig.subplots_adjust(left=0.24, right=0.96, top=0.94, bottom=0.24)
        self.ecmSocCanvas = FigureCanvasTkAgg(self.ecmSocFig, master=ecmFrame)
        self.ecmSocCanvas.get_tk_widget().pack(fill="x", padx=12, pady=(0, 8))

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

        self.fastFileLogVar = ctk.BooleanVar(value=self.fastFileLogEnabled)
        self.fastFileLogSwitch = ctk.CTkSwitch(
            dataFrame, text=f"Also log every {FILE_LOG_PERIOD_FAST:.0f} s",
            variable=self.fastFileLogVar, onvalue=True, offvalue=False,
            progress_color=GREEN, text_color=dual("text"),
            command=self._toggleFastFileLog)
        self.fastFileLogSwitch.pack(fill="x", padx=12, pady=(4, 4))
        ctk.CTkLabel(dataFrame,
                     text="Same format as Battery_monitor_output.txt, just a separate file "
                          f"written every {FILE_LOG_PERIOD_FAST:.0f} s instead of every "
                          f"{FILE_LOG_PERIOD:.0f} s -- for analysis that needs finer time "
                          "resolution than the normal log. The normal log keeps writing "
                          "either way.",
                     font=ctk.CTkFont(size=10), text_color=dual("text_secondary"),
                     wraplength=270, justify="left").pack(anchor="w", padx=12, pady=(0, 12))

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
        purely a physical-layout illustration of the Ns*Np pack. SoH comes from the
        Maintenance validation cycle below (ground truth, Coulomb-counted -- see
        maintenance_cycle.py); the balance figure is computed from the actual logged
        data, independent of SoH."""
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
        self.healthOpportunisticLabels = []
        self.healthPouches = []
        self.healthAvgLabels = []
        self.healthDevLabels = []
        for k in range(n):
            col = ctk.CTkFrame(gridCard, fg_color=dual("card_bg_alt"), corner_radius=10)
            col.grid(row=0, column=k, sticky="nsew", padx=6, pady=16)

            ctk.CTkLabel(col, text=f"B{k+1:02d}", font=ctk.CTkFont(size=13, weight="bold"),
                         text_color=dual("text")).pack(pady=(12, 2))
            # Filled from self.sohState by _updateSohLabels() -- last Maintenance result,
            # or "—" if this group has never completed one.
            sohLabel = ctk.CTkLabel(col, text="SoH: —", font=ctk.CTkFont(size=11),
                                     text_color=dual("text_secondary"))
            sohLabel.pack(pady=(0, 2))
            self.healthSohLabels.append(sohLabel)
            # Secondary, lower-trust figure -- opportunistic live estimate between
            # Maintenance runs (Phase 2, see soh/README.md). Never overwrites the
            # Maintenance SoH above; blank until at least one candidate is fused.
            oppLabel = ctk.CTkLabel(col, text="", font=ctk.CTkFont(size=10),
                                     text_color=dual("text_secondary"))
            oppLabel.pack(pady=(0, 8))
            self.healthOpportunisticLabels.append(oppLabel)

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

        self._buildMaintenanceCard(parent)
        self._buildSohHistoryCard(parent)
        self._readSohHistoryFromLog()
        self._refreshSohHistoryChart()
        self._updateSohLabels()

    def _buildMaintenanceCard(self, parent):
        """Guided validation cycle: charge to full -> rest -> constant-current
        discharge to a cutoff -> Coulomb-counted Q -> SOH = 100*Q/Q_ref (own first-ever
        cycle, per group). This is the authoritative/ground-truth SoH source -- see
        maintenance_cycle.py. Requires an active live measurement (reuses the same
        serial stream _applyMeasurement already processes, see _maintenanceOnSample)."""
        card = ctk.CTkFrame(parent, corner_radius=16, fg_color=dual("card_bg"),
                             border_width=1, border_color=dual("border"))
        card.grid(row=2, column=0, sticky="new", padx=16, pady=(0, 16))

        ctk.CTkLabel(card, text="Maintenance cycle", font=ctk.CTkFont(size=16, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=20, pady=(16, 2))
        ctk.CTkLabel(card,
                     text="Charge to full -> rest 2 h -> constant-current discharge to "
                          f"{self.maintenanceDischargeCutoffV:.2f} V/cell. Tracks Coulomb-counted "
                          "Q and reports ground-truth SOH per group when it finishes. Charge/"
                          "discharge end is judged from the single weakest/strongest cell, not "
                          "an average, so one weak group can't be over-discharged while the "
                          "others still look fine. Runs alongside a normal live measurement.",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary"),
                     wraplength=900, justify="left").pack(anchor="w", padx=20, pady=(0, 10))

        row = ctk.CTkFrame(card, fg_color="transparent")
        row.pack(fill="x", padx=20, pady=(0, 6))
        self.btnMaintenanceStart = ctk.CTkButton(row, text="▶  Start Maintenance cycle",
                                                  command=self._startMaintenanceCycle)
        self.btnMaintenanceStart.pack(side="left", padx=(0, 8))
        self.btnMaintenanceAdvance = ctk.CTkButton(
            row, text="Force advance phase", fg_color="transparent",
            border_width=1, border_color=dual("border"), text_color=dual("text"),
            state="disabled", command=self._forceAdvanceMaintenanceCycle)
        self.btnMaintenanceAdvance.pack(side="left", padx=(0, 8))
        self.btnMaintenanceAbort = ctk.CTkButton(
            row, text="Abort", fg_color="transparent", border_width=1, border_color=RED,
            text_color=RED, state="disabled", command=self._abortMaintenanceCycle)
        self.btnMaintenanceAbort.pack(side="left")

        self.maintenanceStatusLabel = ctk.CTkLabel(
            card, text="Idle -- start a live measurement, then start the cycle.",
            font=ctk.CTkFont(size=13, weight="bold"), text_color=dual("text"))
        self.maintenanceStatusLabel.pack(anchor="w", padx=20, pady=(6, 2))
        self.maintenanceLiveLabel = ctk.CTkLabel(
            card, text="", font=ctk.CTkFont(size=12), text_color=dual("text_secondary"))
        self.maintenanceLiveLabel.pack(anchor="w", padx=20, pady=(0, 16))

    def _buildSohHistoryCard(self, parent):
        """SoH over calendar time, per group -- a real date axis (not session-relative
        like the main chart), since a meaningful trend spans days/weeks/months across
        many sessions. Line = opportunistic tracked estimate (secondary); diamond
        markers = Maintenance-cycle ground truth (authoritative). Backed by
        soh_log.csv, read once at build time (_readSohHistoryFromLog) and appended to
        in-memory from then on (_onMaintenanceComplete / _judgeAndFuseFfnnObservation)
        rather than re-reading the file on every update."""
        card = ctk.CTkFrame(parent, corner_radius=16, fg_color=dual("card_bg"),
                             border_width=1, border_color=dual("border"))
        card.grid(row=3, column=0, sticky="new", padx=16, pady=(0, 16))

        ctk.CTkLabel(card, text="SoH history", font=ctk.CTkFont(size=16, weight="bold"),
                     text_color=dual("text")).pack(anchor="w", padx=20, pady=(16, 2))
        ctk.CTkLabel(card,
                     text="Lines are the opportunistic live estimate (secondary); diamond "
                          "markers are Maintenance-cycle ground truth (authoritative). "
                          "X axis is real calendar time -- this spans sessions, not just "
                          "the current one.",
                     font=ctk.CTkFont(size=11), text_color=dual("text_secondary"),
                     wraplength=900, justify="left").pack(anchor="w", padx=20, pady=(0, 8))

        self.sohFig = Figure(dpi=100, figsize=(6, 2.4))
        self.sohAx = self.sohFig.add_subplot(111)
        self.sohAx.set_ylabel("SoH [%]")
        self.sohAx.xaxis_date()

        colorCycle = matplotlib.rcParams["axes.prop_cycle"].by_key()["color"]
        self.sohLines = []
        self.sohMaintScatters = []
        for k in range(len(self.batteryChannels)):
            color = colorCycle[k % len(colorCycle)]
            (line,) = self.sohAx.plot([], [], "-", linewidth=1.3, color=color,
                                       label=f"B{k + 1:02d}")
            self.sohLines.append(line)
            scatter = self.sohAx.scatter([], [], marker="D", s=36, color=color,
                                          edgecolor="white", linewidth=0.6, zorder=5)
            self.sohMaintScatters.append(scatter)
        self.sohAx.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8,
                           borderaxespad=0.0, frameon=True)
        self.sohFig.subplots_adjust(left=0.08, right=0.85, top=0.95, bottom=0.22)
        self.sohFig.autofmt_xdate(rotation=20, ha="right")

        self.sohCanvas = FigureCanvasTkAgg(self.sohFig, master=card)
        self.sohCanvas.get_tk_widget().pack(fill="both", expand=True, padx=20, pady=(0, 16))

    def _readSohHistoryFromLog(self):
        """(Re)builds the in-memory SoH-over-time series per group from soh_log.csv --
        called once at page build; new points are appended directly from then on
        (see _onMaintenanceComplete / _judgeAndFuseFfnnObservation), not re-read."""
        n = len(self.batteryChannels)
        self.sohHistoryT = [[] for _ in range(n)]
        self.sohHistorySoh = [[] for _ in range(n)]
        self.sohHistoryKind = [[] for _ in range(n)]
        path = os.path.join(self.scriptDir, soh_store.LOG_FILENAME)
        if not os.path.exists(path):
            return
        groupIndex = {f"B{k + 1:02d}": k for k in range(n)}
        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                for row in csv.DictReader(f):
                    k = groupIndex.get(row.get("group"))
                    if k is None or row.get("run_type") == "opportunistic" and row.get("accepted") != "True":
                        continue
                    try:
                        t = datetime.fromisoformat(row["time_iso"])
                        soh = float(row["soh_pct"])
                    except (ValueError, KeyError, TypeError):
                        continue
                    self.sohHistoryT[k].append(t)
                    self.sohHistorySoh[k].append(soh)
                    self.sohHistoryKind[k].append(row.get("run_type", ""))
        except Exception as ex:
            self.logMsg(f"[!] Could not read soh_log.csv for the SoH history chart: {ex}")

    def _appendSohHistory(self, k, t, soh, kind):
        self.sohHistoryT[k].append(t)
        self.sohHistorySoh[k].append(soh)
        self.sohHistoryKind[k].append(kind)

    def _refreshSohHistoryChart(self):
        allT, allSoh = [], []
        for k in range(len(self.batteryChannels)):
            oppT = [mdates.date2num(t) for t, kind in zip(self.sohHistoryT[k], self.sohHistoryKind[k])
                    if kind == "opportunistic"]
            oppSoh = [s for s, kind in zip(self.sohHistorySoh[k], self.sohHistoryKind[k])
                      if kind == "opportunistic"]
            self.sohLines[k].set_data(oppT, oppSoh)

            maintT = [mdates.date2num(t) for t, kind in zip(self.sohHistoryT[k], self.sohHistoryKind[k])
                      if kind == "maintenance"]
            maintSoh = [s for s, kind in zip(self.sohHistorySoh[k], self.sohHistoryKind[k])
                        if kind == "maintenance"]
            if maintT:
                self.sohMaintScatters[k].set_offsets(np.column_stack([maintT, maintSoh]))
            else:
                self.sohMaintScatters[k].set_offsets(np.empty((0, 2)))

            allT.extend(oppT)
            allT.extend(maintT)
            allSoh.extend(oppSoh)
            allSoh.extend(maintSoh)

        # A degenerate Maintenance result (e.g. Force-advanced through Discharging
        # with no samples yet -- Q=0/0) can produce a NaN/inf SOH; matplotlib raises
        # on a non-finite axis limit, which must not take down the whole app over a
        # chart refresh. Drop non-finite pairs rather than let that propagate.
        finitePairs = [(t, s) for t, s in zip(allT, allSoh) if np.isfinite(t) and np.isfinite(s)]
        if finitePairs:
            finiteT, finiteSoh = zip(*finitePairs)
            tLo, tHi = min(finiteT), max(finiteT)
            tPad = max(0.03 * (tHi - tLo), 0.02)
            self.sohAx.set_xlim(tLo - tPad, tHi + tPad)
            sLo, sHi = min(finiteSoh), max(finiteSoh)
            sPad = max(0.05 * (sHi - sLo), 1.0)
            self.sohAx.set_ylim(sLo - sPad, sHi + sPad)
        self.sohCanvas.draw_idle()

    def _updateSohLabels(self):
        """Reflect self.sohState (persisted Maintenance results, authoritative) and
        self.sohTrackers (opportunistic, secondary -- Phase 2) onto the pack grid.
        Called on page build, after every completed/aborted Maintenance cycle, and
        after every fused opportunistic candidate."""
        for k in range(len(self.batteryChannels)):
            entry = self.sohState.get(str(k))
            if entry is None or entry.get("last_soh_pct") is None:
                self.healthSohLabels[k].configure(text="SoH: —", text_color=dual("text_secondary"))
            else:
                soh = entry["last_soh_pct"]
                color = GREEN if soh >= 90.0 else ORANGE if soh >= 80.0 else RED
                self.healthSohLabels[k].configure(text=f"SoH: {soh:.1f} %", text_color=color)

            tracker = self.sohTrackers[k]
            if tracker.history:
                self.healthOpportunisticLabels[k].configure(
                    text=f"live est.: {tracker.soh:.1f} % ± {tracker.sigma:.1f}")
            else:
                self.healthOpportunisticLabels[k].configure(text="")

    # ------------------------------------------------------------------
    # MAINTENANCE CYCLE (ground-truth SoH -- see maintenance_cycle.py)
    def _startMaintenanceCycle(self):
        if not self.measuring:
            self.logMsg("[!] Start a live measurement before starting the Maintenance cycle.")
            return
        if self.maintenanceCycle is not None and self.maintenanceCycle.phase not in (
                MaintPhase.IDLE, MaintPhase.DONE, MaintPhase.ABORTED):
            return

        n = len(self.batteryChannels)
        qRef = [self.sohState.get(str(k), {}).get("q_ref_mah") for k in range(n)]
        self.maintenanceCycle = MaintenanceCycle(
            n, q_ref_mah=qRef,
            discharge_cutoff_v=self.maintenanceDischargeCutoffV,
            charge_full_v=self.maintenanceChargeFullV)
        self.maintenanceCycle.start(self.tData[-1] if self.tData else 0.0)

        # Charging: source (IN) connected, load (OUT) disconnected.
        self._setRelayImmediate(self.relayInCh, True, "Maintenance cycle: charging")
        self._setRelayImmediate(self.relayOutCh, False, "Maintenance cycle: charging")

        self.btnMaintenanceStart.configure(state="disabled")
        self.btnMaintenanceAdvance.configure(state="normal")
        self.btnMaintenanceAbort.configure(state="normal")
        self.logMsg("[Maintenance] Cycle started -- charging.")
        self._refreshMaintenanceStatus()

    def _abortMaintenanceCycle(self):
        if self.maintenanceCycle is None:
            return
        self.maintenanceCycle.abort()
        self._setRelayImmediate(self.relayInCh, False, "Maintenance cycle aborted")
        self._setRelayImmediate(self.relayOutCh, False, "Maintenance cycle aborted")
        self.logMsg("[Maintenance] Cycle aborted by user.")
        self._onMaintenanceCycleEnded()

    def _forceAdvanceMaintenanceCycle(self):
        """Manual override for when the automatic charge-complete/rest-elapsed
        detection doesn't fire reliably for this bench's actual charge source --
        see MaintenanceCycle's own docstring. Forcing past DISCHARGING ends the
        cycle with whatever Q has accumulated so far, same as reaching the cutoff
        would -- a deliberate early stop, not an abort."""
        if self.maintenanceCycle is None or not self.tData:
            return
        oldPhase = self.maintenanceCycle.phase
        self.maintenanceCycle.force_advance(self.tData[-1])
        if self.maintenanceCycle.phase != oldPhase:
            self._onMaintenancePhaseChanged(oldPhase)

    def _maintenanceOnSample(self, tNow, battery, packCurrent, temperatureC):
        """Hooked from _applyMeasurement -- the Maintenance cycle rides the same
        live serial stream as everything else, no separate polling loop."""
        cyc = self.maintenanceCycle
        if cyc is None or cyc.phase in (MaintPhase.IDLE, MaintPhase.DONE, MaintPhase.ABORTED):
            return
        oldPhase = cyc.phase
        cyc.on_sample(tNow, battery, packCurrent, temperatureC)
        if cyc.phase != oldPhase:
            self._onMaintenancePhaseChanged(oldPhase)
        self._refreshMaintenanceStatus()

    def _onMaintenancePhaseChanged(self, oldPhase):
        cyc = self.maintenanceCycle
        if cyc.phase == MaintPhase.RESTING:
            self._setRelayImmediate(self.relayInCh, False, "Maintenance cycle: resting")
            self._setRelayImmediate(self.relayOutCh, False, "Maintenance cycle: resting")
            self.logMsg("[Maintenance] Charging done -- resting.")
        elif cyc.phase == MaintPhase.DISCHARGING:
            self._setRelayImmediate(self.relayOutCh, True, "Maintenance cycle: discharging")
            self.logMsg("[Maintenance] Rest done -- discharging.")
        elif cyc.phase == MaintPhase.DONE:
            self._setRelayImmediate(self.relayOutCh, False, "Maintenance cycle: done")
            self._onMaintenanceComplete()
        self._refreshMaintenanceStatus()

    def _onMaintenanceComplete(self):
        cyc = self.maintenanceCycle
        if not cyc.results or not all(np.isfinite(r["soh_pct"]) and r["q_mah"] > 0 for r in cyc.results):
            # Degenerate result -- e.g. "Force advance phase" clicked through
            # Discharging before any real sample arrived (Q=0/0 -> NaN). Don't
            # persist/log/chart a result that would look like a real measurement
            # but isn't; see maintenance_cycle.MaintenanceCycle._finish.
            self.logMsg("[Maintenance] [!] Cycle ended with no usable discharge data "
                        "(0 mAh measured) -- not recording a SOH result.")
            self._onMaintenanceCycleEnded()
            return
        now = datetime.now()
        nowIso = now.isoformat(timespec="seconds")
        for k, result in enumerate(cyc.results):
            entry = self.sohState.setdefault(str(k), {})
            entry["q_ref_mah"] = result["q_ref_mah"]
            entry["last_soh_pct"] = result["soh_pct"]
            entry["last_cycle_at"] = nowIso
            entry["last_q_mah"] = result["q_mah"]
            soh_store.append_log({
                "time_iso": nowIso, "group": f"B{k+1:02d}", "run_type": "maintenance",
                "soh_pct": result["soh_pct"], "q_mah": result["q_mah"],
                "q_ref_mah": result["q_ref_mah"], "temperature_c": cyc.temperature_c,
                "cutoff_v": self.maintenanceDischargeCutoffV,
            }, self.scriptDir)
            self._appendSohHistory(k, now, result["soh_pct"], "maintenance")
        soh_store.save_state(self.sohState, self.scriptDir)
        self._updateSohLabels()
        self._refreshSohHistoryChart()
        self.logMsg("[Maintenance] Cycle complete. SOH: " +
                    ", ".join(f"B{k+1:02d}={r['soh_pct']:.1f}%" for k, r in enumerate(cyc.results)))
        self._onMaintenanceCycleEnded()

    def _onMaintenanceCycleEnded(self):
        self.btnMaintenanceStart.configure(state="normal")
        self.btnMaintenanceAdvance.configure(state="disabled")
        self.btnMaintenanceAbort.configure(state="disabled")
        self._refreshMaintenanceStatus()

    def _refreshMaintenanceStatus(self):
        cyc = self.maintenanceCycle
        if cyc is None or cyc.phase == MaintPhase.IDLE:
            self.maintenanceStatusLabel.configure(
                text="Idle -- start a live measurement, then start the cycle.")
            self.maintenanceLiveLabel.configure(text="")
            return
        phaseText = {
            MaintPhase.CHARGING: "Charging", MaintPhase.RESTING: "Resting",
            MaintPhase.DISCHARGING: "Discharging", MaintPhase.DONE: "Done",
            MaintPhase.ABORTED: "Aborted",
        }[cyc.phase]
        self.maintenanceStatusLabel.configure(text=f"Phase: {phaseText}")
        if cyc.phase == MaintPhase.DISCHARGING and cyc.t_rel:
            self.maintenanceLiveLabel.configure(
                text=f"Elapsed {cyc.t_rel[-1] / 60.0:.1f} min  ·  Q so far: {cyc.q_mah:.1f} mAh"
                     + (f"  ·  I = {cyc.i[-1]:.2f} A" if cyc.i else ""))
        elif cyc.phase == MaintPhase.DONE and cyc.results:
            self.maintenanceLiveLabel.configure(
                text="  ·  ".join(f"B{k+1:02d}: {r['soh_pct']:.1f}%"
                                   for k, r in enumerate(cyc.results)))
        else:
            self.maintenanceLiveLabel.configure(text="")

    # ------------------------------------------------------------------
    # OPPORTUNISTIC SoH (Phase 2, secondary/interim -- see soh/README.md)
    def _estimateEfc(self, k):
        """Equivalent full cycles for group k's SohTracker (Kalman drift widening).
        Approximated from cumulative |Ah| throughput THIS SESSION ONLY -- there is
        no persisted lifetime throughput counter yet, so a freshly restarted
        session under-counts EFC and the tracker's uncertainty grows slower across
        a restart than it truly should. Falls back to the dissertation's nominal
        capacity if this group has no Maintenance-cycle Q_ref yet."""
        qRefMah = self.sohState.get(str(k), {}).get("q_ref_mah")
        qRefAh = (qRefMah / 1000.0) if qRefMah else (sohConfig.C_NOMINAL_MAH / 1000.0)
        if qRefAh <= 0:
            return 0.0
        return self._sohAhThroughputAh / (2.0 * qRefAh)

    def _buildLiveCurrentSeries(self):
        """(t, iMa, tempSeries) for the whole live session so far, shared by both
        opportunistic scanners (_scanOpportunisticSoh / _scanFieldWindowSoh) --
        one series current for the whole pack, no reason to rebuild it twice.
        Returns (None, None, None) if the arrays aren't aligned yet."""
        t = np.asarray(self.tData, dtype=float)
        if len(self.currY) < 2 or len(self.currY[0]) != t.size or len(self.currY[1]) != t.size:
            return None, None, None
        netA = np.array([
            (self.currY[1][j] - self.currY[0][j]) / self.ecmParallelCount
            if (self.currY[0][j] == self.currY[0][j] and self.currY[1][j] == self.currY[1][j])
            else float("nan")
            for j in range(t.size)
        ])
        # Sign flip: this app's convention is I>0=discharge (matches ecm_model.py,
        # cutoff_logic.py, everywhere else in BatteryMonitorGUI.py); soh.online's
        # scan_segments (vendored from the dissertation's battlib) uses the opposite,
        # I>0=charge -- its own masks are literally `i > threshold` for "charge" and
        # `i < -threshold` for "discharge". Feeding it unflipped silently swaps every
        # charge/discharge classification instead of erroring, which is exactly what
        # happened during testing (a real discharge segment came back labeled
        # "charge" and never matched any discharge-side window).
        iMa = -netA * 1000.0
        tempSeries = None
        if self.resY and len(self.resY[self.ecmTempSourceIndex]) == t.size:
            tempSeries = np.asarray(self.resY[self.ecmTempSourceIndex], dtype=float)
        return t, iMa, tempSeries

    def _scanOpportunisticSoh(self, tNow):
        """Look for a usable constant-current segment in the recent live history and
        time it against the pretrained deployment package's voltage windows -- the
        opportunistic, secondary SoH source (Maintenance cycle stays authoritative).
        Throttled to SOH_OPPORTUNISTIC_SCAN_S: a new qualifying segment doesn't
        appear every poll tick, rescanning that often would just be wasted work."""
        if self.sohDeployment is not None and self.tData and (
                self._sohLastScanT is None or tNow - self._sohLastScanT >= SOH_OPPORTUNISTIC_SCAN_S):
            self._sohLastScanT = tNow
            t, iMa, tempSeries = self._buildLiveCurrentSeries()
            if t is not None:
                ready, reason = self.sohSteadyGate.ready(t, iMa)
                if ready:
                    for k in range(len(self.batteryChannels)):
                        v = np.asarray(self.battY[k], dtype=float)
                        if v.size == t.size:
                            self._scanGroupForFfnnObservation(k, t, v, iMa, tempSeries, tNow)

        self._scanFieldWindowSoh(tNow)

    def _scanFieldWindowSoh(self, tNow):
        """'Quick field' SoH -- see soh/field_window.py and
        08_field_window_deployment.py. A discharge-only, fixed-capacity-slice
        sibling of _scanOpportunisticSoh: same steady-state precondition, same
        throttle, different feature (delta-V over a 10-30 minute window at this
        bench's own native ~7A rate, instead of timing a voltage crossing).
        Deliberately the lowest-trust source -- see soh/README.md -- but feeds the
        SAME per-group SohTracker as the tV-window estimator, so whichever source
        is more accurate at the moment naturally outweighs the other; no separate
        trust-tier bookkeeping needed.

        Capacity positioning: a live partial window can't know its own total
        discharge capacity the way a completed lab RPT can (that is the entire
        point of not running a full discharge) -- this uses the group's own last
        Maintenance-cycle Q_ref (nominal capacity as a bootstrap before any
        Maintenance cycle has run) together with the live SOC estimate (EKF,
        preferred; Coulomb counting as a fallback) to know where a live discharge
        sits relative to a hypothetical 100%->0% span, without completing one.

        Two data sources, same as _scanOpportunisticSoh's own retroactive-scan
        support for loadTestFile(): live streaming fills the dedicated
        _sohField* buffers (see _applyMeasurement) because self.tData/self.battY
        get MAX_POINTS-trimmed and would silently drop early-discharge history;
        a loaded test file instead sets self.tData/self.battY ONCE to the WHOLE,
        uncapped file and never touches _sohField*, so this falls back to those
        plus the open-loop self.ecmSocY (self.ecmEkfSocY is deliberately all-NaN
        for a replayed file -- the EKF only runs live, see loadTestFile)."""
        if self.sohFieldDeployment is None:
            return
        if (self._sohFieldLastScanT is not None
                and tNow - self._sohFieldLastScanT < SOH_OPPORTUNISTIC_SCAN_S):
            return

        if self._sohFieldT:
            t = np.asarray(self._sohFieldT, dtype=float)
            iMa = -np.asarray(self._sohFieldNetCurrentA, dtype=float) * 1000.0
            socPct = np.asarray(self._sohFieldSocPct, dtype=float)
            tempSeries = np.asarray(self._sohFieldTempC, dtype=float)
            vSeries = self._sohFieldV
        else:
            t, iMa, tempSeries = self._buildLiveCurrentSeries()
            if t is None:
                return
            ekfSoc = (np.asarray(self.ecmEkfSocY, dtype=float)
                      if len(self.ecmEkfSocY) == t.size else np.full(t.size, np.nan))
            olSoc = (np.asarray(self.ecmSocY, dtype=float)
                     if len(self.ecmSocY) == t.size else np.full(t.size, np.nan))
            if not np.isfinite(ekfSoc).any() and not np.isfinite(olSoc).any():
                return
            socPct = np.where(np.isfinite(ekfSoc), ekfSoc, olSoc)
            vSeries = self.battY
        self._sohFieldLastScanT = tNow

        # Sign flip to soh.online's convention (I>0=charge) -- see
        # _buildLiveCurrentSeries's comment for why this matters.
        ready, reason = self.sohFieldSteadyGate.ready(t, iMa)
        if not ready:
            return

        for k in range(len(self.batteryChannels)):
            v = np.asarray(vSeries[k], dtype=float)
            if v.size != t.size:
                continue
            qRefMah = self.sohState.get(str(k), {}).get("q_ref_mah") or sohConfig.C_NOMINAL_MAH
            qdMah = (1.0 - socPct / 100.0) * qRefMah
            self._scanGroupForFieldWindowObservation(k, t, v, iMa, qdMah, qRefMah, tempSeries, tNow)

    def _scanGroupForFieldWindowObservation(self, k, t, v, iMa, qdMah, qRefMah, tempSeries, tNow):
        deployment = self.sohFieldDeployment
        discharging = iMa < -abs(sohConfig.I_THRESHOLD_MA)
        if not discharging.any():
            return
        temperatureC = float(np.nanmean(tempSeries[discharging])) if tempSeries is not None else float("nan")
        crateDischarge = float(np.nanpercentile(np.abs(iMa[discharging]), 95) / sohConfig.C_NOMINAL_MAH)

        # Longest duration first -- "cim del tim lepe", per spec -- falling back to
        # shorter tiers only if the pack hasn't held a steady discharge long enough
        # yet for the longer one's windows to be fully covered.
        for durationEntry in sorted(deployment["meta"]["durations"], key=lambda d: -d["minutes"]):
            minutes = durationEntry["minutes"]
            dqMah = durationEntry["dQ_mAh"]
            models_ = deployment["models"][minutes]

            for comboMeta in durationEntry["combinations"]:  # already ordered best3, best2, best1
                model = models_.get(comboMeta["name"])
                if model is None:
                    continue

                featureValues = {}
                ok = True
                for win in comboMeta["windows"]:
                    coverage = window_capacity_coverage(qdMah[discharging], win["centre_frac"],
                                                         dqMah, qRefMah)
                    if coverage < 0.98:
                        ok = False
                        break
                    dv = capacity_window_dv(qdMah[discharging], v[discharging],
                                             win["centre_frac"], dqMah, qRefMah)
                    if not np.isfinite(dv):
                        ok = False
                        break
                    featureValues[win["raw_name"]] = dv
                if not ok:
                    continue

                frame = pd.DataFrame([featureValues])[model.features]
                predictedSoh = float(model.predict(frame)[0])
                self._judgeAndFuseFfnnObservation(
                    k, comboMeta, predictedSoh, tNow,
                    float("nan"), crateDischarge, temperatureC,
                    runType="quick_field", sourceLabel=f"quick field {minutes}min")
                return  # longest/best available combo for this group this scan

        # None of the coarse (10%-90% step 10%) multi-feature combos are covered
        # yet -- fall back to the dense/early single-feature grid (09_field_window_
        # early_grid.py), sorted best-accuracy-first, and use whichever position the
        # discharge has ACTUALLY reached so far. This is what makes the estimator
        # usable "regardless of what" (the user's own framing) instead of requiring
        # a specific pre-planned window: most single-feature grid entries near the
        # start of a discharge are reachable within 10-30 minutes total, unlike the
        # coarse combos above (their easiest position alone needs 70+ minutes).
        gridDeployment = self.sohFieldGridDeployment
        if gridDeployment is not None:
            for entry in gridDeployment["meta"]["entries"]:  # pre-sorted, best val_rmse first
                win = entry["window"]
                coverage = window_capacity_coverage(qdMah[discharging], win["centre_frac"],
                                                     win["dQ_mAh"], qRefMah)
                if coverage < 0.98:
                    continue
                dv = capacity_window_dv(qdMah[discharging], v[discharging],
                                         win["centre_frac"], win["dQ_mAh"], qRefMah)
                if not np.isfinite(dv):
                    continue
                model = gridDeployment["models"].get(entry["key"])
                if model is None:
                    continue
                frame = pd.DataFrame([{win["raw_name"]: dv}])[model.features]
                predictedSoh = float(model.predict(frame)[0])
                comboMeta = {"name": entry["key"], "band": entry["band"],
                             "val_rmse_percent": entry["val_rmse_percent"]}
                self._judgeAndFuseFfnnObservation(
                    k, comboMeta, predictedSoh, tNow,
                    float("nan"), crateDischarge, temperatureC, runType="quick_field",
                    sourceLabel=(f"quick field grid {entry['duration_min']}min"
                                 f"@{win['centre_frac']:.0%}"))
                return  # best-accuracy grid entry currently satisfiable

    def _scanGroupForFfnnObservation(self, k, t, v, iMa, tempSeries, tNow):
        segments = scan_segments(t, v, iMa, tempSeries)
        if not segments:
            return

        meta = self.sohDeployment["meta"]
        for comboMeta in meta["combinations"]:  # already ordered best3, best2, best1
            model = self.sohDeployment["models"].get(comboMeta["name"])
            if model is None:
                continue

            featureValues = {}
            windowTimes = []  # (segStart, segEnd, crateCharge, crateDischarge) per window used
            ok = True
            for win in comboMeta["windows"]:
                vLow, vHigh = win["centre_v"] - win["width_v"] / 2.0, win["centre_v"] + win["width_v"] / 2.0
                wantKind = "charge" if win["mode"] == "ch" else "discharge"
                found = False
                for seg, mask in segments:
                    if seg.kind != wantKind:
                        continue
                    if window_coverage(v[mask], vLow, vHigh) < self.sohGateSettings.min_coverage:
                        continue
                    dur = voltage_window_time(t[mask], v[mask], vLow, vHigh)
                    if not np.isfinite(dur):
                        continue
                    featureValues[win["raw_name"]] = dur
                    segSpan = t[mask]
                    windowTimes.append((float(segSpan[0]), float(segSpan[-1]), seg))
                    found = True
                    break
                if not found:
                    ok = False
                    break
            if not ok:
                continue

            # All windows must come from segments close in time to each other --
            # otherwise a charge window from an hour ago and a discharge window from
            # just now could get combined as if measured under one coherent
            # condition (see SOH_COMBO_MAX_TIME_GAP_S).
            spanStart = min(w[0] for w in windowTimes)
            spanEnd = max(w[1] for w in windowTimes)
            if spanEnd - spanStart > SOH_COMBO_MAX_TIME_GAP_S:
                continue

            usedSegs = [w[2] for w in windowTimes]
            crateCharge = next((s.crate for s in usedSegs if s.kind == "charge"), float("nan"))
            crateDischarge = next((s.crate for s in usedSegs if s.kind == "discharge"), float("nan"))
            temperatureC = float(np.nanmean([s.temperature_c for s in usedSegs]))

            frame = pd.DataFrame([featureValues])[model.features]
            predictedSoh = float(model.predict(frame)[0])
            self._judgeAndFuseFfnnObservation(k, comboMeta, predictedSoh, tNow,
                                               crateCharge, crateDischarge, temperatureC)
            return  # best available combo for this group this scan -- don't also log weaker ones

    def _judgeAndFuseFfnnObservation(self, k, comboMeta, predictedSoh, tNow,
                                      crateCharge, crateDischarge, temperatureC,
                                      runType="opportunistic", sourceLabel="live opportunistic"):
        """Bridges a deployment package's multi-feature FFNN combinations into
        soh.online's Observation/Verdict/SohTracker machinery. NOT the same path
        as soh.online.AcceptanceGate.judge() -- that class is built around a
        single-feature FeatureCalibration (one rho, one domain range), which an
        FFNN combination fusing 1-3 windows doesn't have. Verdict/Observation are
        still real soh.online objects, so SohTracker.update() and soh_store's
        logging schema work unchanged; only the acceptance judgement itself
        (condition penalty, plausibility) is reimplemented here for this combo
        shape, reusing GateSettings' tuned thresholds (crate_tolerance etc.) as
        the source of truth for what counts as "too far off".

        Shared by both opportunistic estimators (tV-window, native rate; and the
        "quick field" short fixed-capacity-window one, see _scanFieldWindowSoh) --
        `runType`/`sourceLabel` are what tell them apart in soh_log.csv and the
        debug log; both feed the SAME per-group SohTracker, so a more accurate
        estimate (smaller sigma, from either source) naturally pulls the tracked
        value harder than a less accurate one -- exactly the "quick data, replaced
        by anything more precise" behaviour the quick-field estimator needs,
        without a separate trust-tier mechanism."""
        s = self.sohGateSettings
        reasons = []

        tier = SOH_FFNN_BAND_TIER.get(comboMeta["band"], Tier.LOW)
        baseSigma = comboMeta["val_rmse_percent"]

        penalty = 1.0
        for measured, label in ((crateCharge, "charge C-rate"), (crateDischarge, "discharge C-rate")):
            if not np.isfinite(measured) or measured <= 0:
                continue
            deviation = abs(measured - SOH_REFERENCE_CRATE) / SOH_REFERENCE_CRATE
            if deviation > s.crate_tolerance:
                reasons.append(f"{label} {measured:.3f} C is {deviation:.0%} off the "
                                f"{SOH_REFERENCE_CRATE:.2f} C reference")
            penalty *= 1.0 + 2.0 * deviation
        if temperatureC == temperatureC:
            offset = abs(temperatureC - 25.0)
            if offset > s.temperature_tolerance_c:
                reasons.append(f"temperature {temperatureC:.0f} C is {offset:.0f} C from "
                                "the 25 C reference")
            penalty *= 1.0 + 0.04 * offset

        sigma = baseSigma * tier.variance_inflation * penalty

        tracker = self.sohTrackers[k]
        if tracker.history:
            combined = math.sqrt(sigma ** 2 + tracker.sigma ** 2)
            disagreement = abs(predictedSoh - tracker.soh) / combined if combined > 0 else float("inf")
            if disagreement > s.max_disagreement_sigmas:
                reasons.append(f"candidate {predictedSoh:.2f} % disagrees with tracked "
                                f"{tracker.soh:.2f} % by {disagreement:.1f} sigma")
            if predictedSoh - tracker.soh > s.max_soh_rise:
                reasons.append(f"implies SOH rose {predictedSoh - tracker.soh:.2f} points")

        obs = Observation(
            feature=comboMeta["name"], value=predictedSoh, timestamp_h=tNow / 3600.0,
            equivalent_full_cycles=self._estimateEfc(k),
            crate_charge=crateCharge, crate_discharge=crateDischarge,
            temperature_c=temperatureC, coverage=1.0,
            source=f"{sourceLabel} @ t={tNow:.0f} s")
        verdict = Verdict(observation=obs, soh=predictedSoh, sigma=sigma, tier=tier,
                           accepted=not reasons, reasons=reasons)

        ageDays = 0.0  # live observations are always "now" -- the age cap matters
                        # once opportunistic history can be replayed/backfilled
        if ageDays <= SOH_MAX_OBSERVATION_AGE_DAYS:
            tracker.update(verdict)

        row = verdict.as_row()
        row["time_iso"] = datetime.now().isoformat(timespec="seconds")
        row["group"] = f"B{k + 1:02d}"
        row["run_type"] = runType
        row["soh_pct"] = row.pop("soh_candidate")
        row["sigma_pct"] = row.pop("sigma")
        row["temperature_c"] = temperatureC
        row["rate_c"] = crateDischarge if crateDischarge == crateDischarge else crateCharge
        soh_store.append_log(row, self.scriptDir)

        if verdict.accepted:
            self.logMsg(f"[SoH] B{k + 1:02d} {runType} ({comboMeta['name']}): "
                        f"{predictedSoh:.1f} % -> tracked {tracker.soh:.1f} % ± {tracker.sigma:.1f}")
            # Chart the FUSED tracker value, not the raw candidate -- matches the
            # "live est." label and gives a smooth trend instead of noisy points.
            self._appendSohHistory(k, datetime.now(), tracker.soh, "opportunistic")
            self._refreshSohHistoryChart()
        else:
            self.logMsg(f"[SoH] B{k + 1:02d} {runType} candidate rejected: "
                        + "; ".join(reasons))
        self._updateSohLabels()

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
                                    label="ECM sim")
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
            self.estimationHorizonFrame.pack(side="left", padx=(0, 14))
            self.logMsg(f"Estimation on -- projecting {_formatHorizonMinutes(self.estimationHorizonMin)} "
                        "ahead at the current current.")
            self._runEstimation()
            self._scheduleEstimationTick()
        else:
            if self._estimationAfterId is not None:
                self.root.after_cancel(self._estimationAfterId)
                self._estimationAfterId = None
            self.estimationHorizonFrame.pack_forget()
            self.logMsg("Estimation off.")
            self._clearEstimation()

    def _onEstimationHorizonSlide(self, value):
        idx = max(0, min(int(round(float(value))), len(ESTIMATION_MINUTES_STEPS) - 1))
        self._setEstimationHorizonMinutes(ESTIMATION_MINUTES_STEPS[idx], moveSlider=False)

    def _onEstimationHorizonEntryChange(self, event=None):
        try:
            minutes = int(round(float(self.estimationHorizonEntry.get().replace(",", "."))))
        except ValueError:
            self.estimationHorizonEntry.delete(0, "end")
            self.estimationHorizonEntry.insert(0, str(self.estimationHorizonMin))
            return
        minutes = max(1, min(minutes, 24 * 60))
        self._setEstimationHorizonMinutes(minutes, moveSlider=True)

    def _setEstimationHorizonMinutes(self, minutes, moveSlider):
        """Common path for both the slider (already-quantized values) and the manual entry
        (any exact minute count) -- the entry can land off the slider's grid on purpose, so
        the slider is only ever moved to the NEAREST grid step, never used to override it."""
        self.estimationHorizonMin = minutes
        if moveSlider:
            nearestIdx = min(range(len(ESTIMATION_MINUTES_STEPS)),
                              key=lambda i: abs(ESTIMATION_MINUTES_STEPS[i] - minutes))
            self.estimationHorizonSlider.set(nearestIdx)
        self.estimationHorizonValueLabel.configure(text=_formatHorizonMinutes(minutes))
        entryText = str(minutes)
        if self.estimationHorizonEntry.get() != entryText:
            self.estimationHorizonEntry.delete(0, "end")
            self.estimationHorizonEntry.insert(0, entryText)
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
        n = max(2, int(self.estimationHorizonMin * 60.0 // dt))
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

        # -- small embedded figures (ECM SOC sparkline, SoH history) -- sit on a
        # card body, which is transparent over card_bg_alt, not card_bg. --
        for fig, ax, canvas in ((self.ecmSocFig, self.ecmSocAx, self.ecmSocCanvas),
                                 (self.sohFig, self.sohAx, self.sohCanvas)):
            fig.set_facecolor(t["card_bg_alt"])
            ax.set_facecolor(t["card_bg_alt"])
            ax.tick_params(colors=t["text_secondary"])
            ax.yaxis.label.set_color(t["text"])
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.grid(True, color=t["separator"], linewidth=0.5, alpha=0.7)
            legend = ax.get_legend()
            if legend is not None:
                for text in legend.get_texts():
                    text.set_color(t["text_secondary"])
            canvas.draw_idle()

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

    def _shutdownRelaysAndVerify(self):
        """Best-effort safety net before the port actually closes (manual Disconnect, or
        the app exiting -- see onClose): command every relay OFF, then take one fresh
        direct current reading per relay to confirm it actually happened. Runs
        synchronously (unlike the async _scheduleRelayOffVerification path used elsewhere)
        because by design relays must be off, and confirmed off, BEFORE the rest of
        teardown proceeds -- not sometime after, on whatever the async delay happens to be."""
        for ch, _ in self.relayChannels:
            self._setRelayImmediate(ch, False, "disconnecting", verify=False)
        time.sleep(RELAY_OFF_VERIFY_DELAY_S)
        for ch, _ in self.relayChannels:
            measuredA = self._measureRelayCurrent(ch)
            if measuredA == measuredA and measuredA > RELAY_OFF_VERIFY_CURRENT_A:
                self._showRelayStillOnWarning(ch, "disconnecting", measuredA)

    def disconnect(self):
        if self.measuring:
            self.stopMeasurement()

        if self.sPort is not None:
            self._shutdownRelaysAndVerify()
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
        self._socEstimatedThisSession = False

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

            if not self._socEstimatedThisSession:
                # Only on the actual first start of a session (not a Stop/Start resume,
                # which must keep the ECM's accumulated SOC/state) -- same rest-voltage SOC
                # estimate loadTestFile already does from a log's first row, just taken
                # live instead of read from a file. Deliberately NOT gated on
                # `self.tStart is None` -- toggleConnection() already sets tStart on
                # Connect for unrelated reasons, which used to make this never fire at all.
                self._estimateInitialSocFromLiveReading()
                self._socEstimatedThisSession = True

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

    def _estimateInitialSocFromLiveReading(self):
        """Rest-voltage initial-SOC estimate for a fresh live measurement -- the live-start
        counterpart to loadTestFile's identical estimate from a log's first row (see
        estimateInitialSocPct). Takes one direct reading of every channel synchronously
        (before the worker thread starts streaming), same queries _workerLoop makes per
        cycle. Safe to call under load too: estimateInitialSocPct just falls back to the
        manually-entered SOC with a logged reason when the reading doesn't look like rest."""
        speed = "Fast" if self.pollPeriod < SLOW_THRESHOLD else "Slow"

        battV = [self._query(f"MEASure:VOLTage{ch}? 15V,{speed}",
                              rangeLimit=BATTERY_RANGE_V * RANGE_MARGIN)
                 for ch in self.batteryChannels]
        validV = [v for v in battV if v == v]
        avgV = sum(validV) / len(validV) if validV else float("nan")

        inCh, _ = self.currentChannels[0]
        outCh, _ = self.currentChannels[1]
        iIn = self._query(f"MEASure:VOLTage{inCh}? 0V15,{speed}",
                           rangeLimit=CURRENT_SHUNT_RANGE_V * RANGE_MARGIN)
        iOut = self._query(f"MEASure:VOLTage{outCh}? 0V15,{speed}",
                            rangeLimit=CURRENT_SHUNT_RANGE_V * RANGE_MARGIN)
        packCurrent = ((iOut - iIn) / self.shuntOhms
                       if (iIn == iIn and iOut == iOut) else float("nan"))
        netCurrent = packCurrent / self.ecmParallelCount if packCurrent == packCurrent else float("nan")

        tempCh, tempName = self.resistChannels[self.ecmTempSourceIndex]
        tempRawOhm = self._query(f"MEASure:RESistance{tempCh}? 200k,{speed}",
                                  rangeLimit=RESIST_RANGE_OHM * RANGE_MARGIN)
        temp = resistanceToCelsius(tempName, tempRawOhm)
        if temp != temp:
            temp = 25.0

        try:
            fallbackPct = float(self.ecmInitialSocEntry.get().replace(",", "."))
        except ValueError:
            fallbackPct = 100.0

        initialSocPct, wasEstimated, reason = estimateInitialSocPct(
            self.ecm, avgV, netCurrent, temp, fallbackPct)

        if wasEstimated:
            self.logMsg(f"Initial SOC estimated from a rest-voltage reading at measurement "
                        f"start ({avgV:.3f} V @ {temp:.1f} °C): {initialSocPct:.1f} %.")
        else:
            self.logMsg(f"[!] Measurement start doesn't look like a battery rest voltage "
                        f"({reason}) — using the manual initial SOC {initialSocPct:.1f} %.")

        self.ecmInitialSocEntry.delete(0, "end")
        self.ecmInitialSocEntry.insert(0, f"{initialSocPct:.1f}")
        self.ecmInitialSocPct = initialSocPct
        self.ecm.reset(initialSocFraction=initialSocPct / 100.0)
        self.ecmEkf.reset(initialSocFraction=initialSocPct / 100.0)

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
            # Disabling the sequence hands manual control of the Load relay back to the
            # user (see _refreshRelayAvailability) -- default that handoff to OFF rather
            # than leaving it energized with no automatic protection watching it anymore.
            self._setRelayImmediate(self.relayOutCh, False, "sequence disabled")

    def _saveCutoffInitialState(self):
        self.cutoffInitialState = bool(self.cutoffInitialVar.get())
        cfg = config.load_config()
        cfg["cutoff_initial_state"] = self.cutoffInitialState
        config.save_config(cfg)

    # ------------------------------------------------------------------
    # DATA -- optional fast (short-period) text log
    def _toggleFastFileLog(self):
        self.fastFileLogEnabled = bool(self.fastFileLogVar.get())
        self.fileLogger.setFastLoggingEnabled(self.fastFileLogEnabled)
        cfg = config.load_config()
        cfg["fast_file_log_enabled"] = self.fastFileLogEnabled
        config.save_config(cfg)
        state = "enabled" if self.fastFileLogEnabled else "disabled"
        self.logMsg(f"Fast ({FILE_LOG_PERIOD_FAST:.0f} s) text log {state} "
                    f"({BatteryFileLogger.FAST_FILENAME}).")

    def _setRelayImmediate(self, ch, newState, reason, verify=True):
        """Sends SET:OUTput directly from the main (GUI) thread and reflects the state
        in the UI. Returns True if the command was actually sent. `verify=False` skips
        scheduling the current-based off-verification (see _scheduleRelayOffVerification)
        for callers that already do their own, e.g. the disconnect/shutdown sequence."""
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
        if verify and not newState:
            self._scheduleRelayOffVerification(ch, reason)
        return True

    def _measureRelayCurrent(self, ch):
        """One direct, synchronous current reading [A] for whichever shunt channel the
        given relay is expected to gate (IN relay -> I_IN, OUT/Load relay -> I_OUT) --
        same command/range as the worker loop's own per-cycle reading. Returns NaN on a
        bad/missing reply. Safe to call whether or not a measurement is currently running
        (goes through the same self.portLock as everything else touching the port)."""
        if self.sPort is None:
            return float("nan")
        idx = 0 if ch == self.relayInCh else 1
        currCh, _ = self.currentChannels[idx]
        speed = "Fast" if self.pollPeriod < SLOW_THRESHOLD else "Slow"
        v = self._query(f"MEASure:VOLTage{currCh}? 0V15,{speed}",
                         rangeLimit=CURRENT_SHUNT_RANGE_V * RANGE_MARGIN)
        return abs(v / self.shuntOhms) if v == v else float("nan")

    def _scheduleRelayOffVerification(self, ch, reason):
        """Must be called from the GUI thread (root.after is not thread-safe) -- the
        worker thread posts to autoEventQueue instead, see _autoSetRelay/_pollQueue."""
        self.root.after(int(RELAY_OFF_VERIFY_DELAY_S * 1000),
                         lambda: self._verifyRelayOffAsync(ch, reason))

    def _verifyRelayOffAsync(self, ch, reason):
        if self.sPort is None:
            return  # disconnected in the meantime -- nothing left to check
        measuredA = self._measureRelayCurrent(ch)
        if measuredA == measuredA and measuredA > RELAY_OFF_VERIFY_CURRENT_A:
            self._showRelayStillOnWarning(ch, reason, measuredA)

    def _showRelayStillOnWarning(self, ch, reason, measuredA):
        """Blocking modal -- see the 2026-09-01 welded-contact incident that prompted this:
        SET:OUTput has no readback, so a relay that silently stopped responding to commands
        would otherwise look identical (in this app) to one that's genuinely open. The user
        must explicitly acknowledge before continuing; this is deliberately NOT a passive
        banner or an audible alarm, per spec."""
        name = dict(self.relayChannels).get(ch, str(ch))
        self.logMsg(f"[!] CH{ch} ({name}) commanded OFF ({reason}) but current sensing "
                    f"still reads {measuredA:.2f} A (> {RELAY_OFF_VERIFY_CURRENT_A:.1f} A).")
        messagebox.showwarning(
            "Relay may still be energized",
            f"CH{ch} ({name}) was just commanded OFF ({reason}), but current sensing "
            f"still reads {measuredA:.2f} A -- above the {RELAY_OFF_VERIFY_CURRENT_A:.1f} A "
            "threshold for calling it off.\n\n"
            "The relay may not be responding (e.g. welded/stuck contacts from switching "
            "an inductive load) -- check it physically before relying on it again.")

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
    def _refreshEcmSocChart(self):
        """Small sparkline in the ECM card: Coulomb-counting vs EKF SOC over this
        session. Cheap (two line updates on an already-small figure) so it's just
        called every _applyMeasurement tick rather than throttled."""
        self.ecmSocLine.set_data(self.tData, self.ecmSocY)
        self.ecmEkfSocLine.set_data(self.tData, self.ecmEkfSocY)
        self.ecmSocAx.relim()
        self.ecmSocAx.autoscale_view()
        self.ecmSocCanvas.draw_idle()

    def resetEcm(self):
        try:
            pct = float(self.ecmInitialSocEntry.get().replace(",", "."))
        except ValueError:
            self.logMsg("[!] ECM reset: invalid initial SOC value.")
            return
        pct = min(max(pct, 0.0), 100.0)

        self.ecm.reset(initialSocFraction=pct / 100.0)
        self.ecmEkf.reset(initialSocFraction=pct / 100.0)
        self._ecmLastTNow = None
        self.ecmY = [float("nan")] * len(self.tData)
        self.ecmSocY = [float("nan")] * len(self.tData)
        self.ecmEkfSocY = [float("nan")] * len(self.tData)
        if self.ecmLines:
            self.ecmLines[0].set_data(self.tData, self.ecmY)
            self.canvas.draw_idle()
        self._refreshEcmSocChart()

        self.ecmInitialSocPct = pct
        cfg = config.load_config()
        cfg["ecm_initial_soc_pct"] = pct
        config.save_config(cfg)

        self.ecmSocLabel.configure(text=f"SOC (Coulomb counting): {self.ecm.soc*100:.1f} %")
        self.ecmEkfSocLabel.configure(
            text=f"SOC (EKF): {self.ecmEkf.soc*100:.1f} % ± {self.ecmEkf.socSigma*100:.1f} pp")
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
        self.ecmEkfSocY = [float("nan")] * len(tData)  # EKF only runs live, see ecm_ekf.py
        self._ecmLastTNow = tData[-1] if tData else None
        self._refreshEcmSocChart()

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

        self.ecmSocLabel.configure(text=f"SOC (Coulomb counting): {self.ecm.soc*100:.1f} %")
        self.ecmEkfSocLabel.configure(text="SOC (EKF): -- (live measurement only)")
        self.ecmVoltageLabel.configure(
            text=f"Simulated voltage: {self.ecm.lastVoltage:.3f} V"
            if self.ecm.lastVoltage == self.ecm.lastVoltage else "Simulated voltage: --- V")
        ecmModeText = "Discharging" if self.ecm.lastMode == "discharge" else "Charging"
        self.ecmModeLabel.configure(
            text=f"Mode: {ecmModeText}  ·  Model temperature: {self.ecm.lastTemperatureC:.1f} °C")

        self.logMsg(f"[OK] Loaded {len(tData)} rows from {os.path.basename(path)} "
                    f"(duration {tData[-1] - tData[0]:.0f} s). ECM replayed over the whole current profile.")

        # Opportunistic SoH (Phase 2) otherwise only ever sees live data via
        # _applyMeasurement's hook -- a loaded file has real per-group voltages too,
        # so it deserves the same retroactive scan over the whole loaded history.
        # Bypass the throttle: a fresh load should always get one immediate attempt.
        # Also reset the quick-field estimator's dedicated live buffer (Phase 3, see
        # soh/README.md) -- it's only ever filled by _applyMeasurement, so it would
        # otherwise still hold a stale earlier live session's data and shadow this
        # file's own self.tData/self.battY/self.ecmSocY in _scanFieldWindowSoh.
        self._sohLastScanT = None
        self._sohFieldLastScanT = None
        self._sohFieldT = []
        self._sohFieldNetCurrentA = []
        self._sohFieldSocPct = []
        self._sohFieldTempC = []
        self._sohFieldV = [[] for _ in range(len(self.batteryChannels))]
        self._scanOpportunisticSoh(tData[-1])
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
        self.ecmEkfSocY = [float("nan")] * len(tData)  # EKF only runs live, see ecm_ekf.py
        self._ecmLastTNow = tData[-1] if tData else None
        self._refreshEcmSocChart()

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

        self.ecmSocLabel.configure(text=f"SOC (Coulomb counting): {self.ecm.soc*100:.1f} %")
        self.ecmEkfSocLabel.configure(text="SOC (EKF): -- (live measurement only)")
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
        if not newState:
            # off-verification needs root.after -- not thread-safe from here, hand it to
            # _pollQueue (GUI thread) via the same event, tagged so it isn't logged twice
            self.autoEventQueue.put(("verifyRelayOff", ch, None, reason))

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
            elif tag == "verifyRelayOff":
                self._scheduleRelayOffVerification(ch, reason)

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

        self._maintenanceOnSample(tNow, battery, packCurrent, ecmTemp)

        ecmDt = None if self._ecmLastTNow is None else tNow - self._ecmLastTNow
        self._ecmLastTNow = tNow
        if ecmDt is not None and ecmDt > 0 and netCurrent == netCurrent:
            self._sohAhThroughputAh += abs(netCurrent) * ecmDt / 3600.0
        if ecmDt is not None and ecmDt > 0:
            ecmVoltage = self.ecm.step(netCurrent, ecmDt, ecmTemp)
            if self.fmuThermal.isRunning():
                self.fmuThermal.addSample(tNow, self.ecm.lastPLossW)
        else:
            ecmVoltage = self.ecm.lastVoltage
        self.ecmY.append(ecmVoltage)
        self.ecmSocY.append(self.ecm.soc * 100.0)
        self.ecmPLossY.append(self.ecm.lastPLossW)

        # -- EKF SOC (see ecm_ekf.py) -- fuses the REAL measured cell voltage (average
        # across series groups, same convention as estimateInitialSocPct's rest-voltage
        # reading) to correct SOC drift; only meaningful live, where that measurement
        # exists every step -- see the "-- (live measurement only)" label elsewhere. --
        validBattV = [v for v in battery if v == v]
        avgMeasuredV = sum(validBattV) / len(validBattV) if validBattV else float("nan")
        if ecmDt is not None and ecmDt > 0:
            self.ecmEkf.step(netCurrent, ecmDt, ecmTemp, avgMeasuredV)
            self.ecmEkfSocY.append(self.ecmEkf.soc * 100.0)
        else:
            self.ecmEkfSocY.append(float("nan"))

        # -- quick-field SoH history buffer + scan -- AFTER the EKF step, so the
        # freshest SOC is what gets recorded/scanned (see SOH_FIELD_HISTORY_MAX_POINTS). --
        self._sohFieldT.append(tNow)
        self._sohFieldNetCurrentA.append(netCurrent)
        self._sohFieldSocPct.append(self.ecmEkfSocY[-1] if self.ecmEkfSocY[-1] == self.ecmEkfSocY[-1]
                                     else self.ecm.soc * 100.0)
        self._sohFieldTempC.append(ecmTemp)
        for gk, gv in enumerate(battery):
            self._sohFieldV[gk].append(gv)
        if len(self._sohFieldT) > SOH_FIELD_HISTORY_MAX_POINTS:
            self._sohFieldT = self._sohFieldT[-SOH_FIELD_HISTORY_MAX_POINTS:]
            self._sohFieldNetCurrentA = self._sohFieldNetCurrentA[-SOH_FIELD_HISTORY_MAX_POINTS:]
            self._sohFieldSocPct = self._sohFieldSocPct[-SOH_FIELD_HISTORY_MAX_POINTS:]
            self._sohFieldTempC = self._sohFieldTempC[-SOH_FIELD_HISTORY_MAX_POINTS:]
            self._sohFieldV = [col[-SOH_FIELD_HISTORY_MAX_POINTS:] for col in self._sohFieldV]

        self._scanOpportunisticSoh(tNow)

        if ecmVoltage == ecmVoltage:
            self.ecmVoltageLabel.configure(text=f"Simulated voltage: {ecmVoltage:.3f} V")
        else:
            self.ecmVoltageLabel.configure(text="Simulated voltage: --- V")
        ecmModeText = "Discharging" if self.ecm.lastMode == "discharge" else "Charging"
        self.ecmModeLabel.configure(
            text=f"Mode: {ecmModeText}  ·  Model temperature: {self.ecm.lastTemperatureC:.1f} °C")
        self.ecmSocLabel.configure(text=f"SOC (Coulomb counting): {self.ecm.soc*100:.1f} %")
        self.ecmEkfSocLabel.configure(
            text=f"SOC (EKF): {self.ecmEkf.soc*100:.1f} % ± {self.ecmEkf.socSigma*100:.1f} pp")

        if len(self.tData) > MAX_POINTS:
            self.tData = self.tData[-MAX_POINTS:]
            self.battY = [y[-MAX_POINTS:] for y in self.battY]
            self.currY = [y[-MAX_POINTS:] for y in self.currY]
            self.resY = [y[-MAX_POINTS:] for y in self.resY]
            self.ecmY = self.ecmY[-MAX_POINTS:]
            self.ecmSocY = self.ecmSocY[-MAX_POINTS:]
            self.ecmEkfSocY = self.ecmEkfSocY[-MAX_POINTS:]
            self.ecmPLossY = self.ecmPLossY[-MAX_POINTS:]

        self._refreshEcmSocChart()

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
        currentIN = currents[0] if len(currents) > 0 else float("nan")
        currentOUT = currents[1] if len(currents) > 1 else float("nan")
        socCcPct = self.ecmSocY[-1] if self.ecmSocY else float("nan")
        socEkfPct = self.ecmEkfSocY[-1] if self.ecmEkfSocY else float("nan")
        if now - self._lastFileLogTime >= FILE_LOG_PERIOD:
            self._lastFileLogTime = now
            self.fileLogger.logCycle(datetime.now(), battery, currentIN, currentOUT,
                                      resistances, self.relayState.get(self.relayInCh, False),
                                      self.relayState.get(self.relayOutCh, False),
                                      socCcPct, socEkfPct)
        if self.fastFileLogEnabled and now - self._lastFastFileLogTime >= FILE_LOG_PERIOD_FAST:
            self._lastFastFileLogTime = now
            self.fileLogger.logFastCycle(datetime.now(), battery, currentIN, currentOUT,
                                          resistances, self.relayState.get(self.relayInCh, False),
                                          self.relayState.get(self.relayOutCh, False),
                                          socCcPct, socEkfPct)

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
        if not newState:
            self._scheduleRelayOffVerification(ch, "manual switch")

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
        self.ecmEkfSocY = []  # NaN where the EKF wasn't fed a real measurement (see _applyMeasurement)
        self.ecmPLossY = []
        self._ecmLastTNow = None  # ECM state (SOC/v1/v2) isn't cleared, just the chart history
        self.fmuTData = []
        self.fmuY = [[] for _ in range(FMU_N_CELLS)]  # thermal P_loss history isn't cleared either

        # self.tStart resets below, which restarts what tNow means -- the quick-field
        # buffer MUST reset with it (unlike ECM/tracker state) or SteadyStateGate.ready()
        # would compare a post-reset "now" against pre-reset timestamps in the same array.
        self._sohFieldT = []
        self._sohFieldNetCurrentA = []
        self._sohFieldSocPct = []
        self._sohFieldTempC = []
        self._sohFieldV = [[] for _ in self.batteryChannels]
        self._sohFieldLastScanT = None

        for line in self.battLines + self.currLines + self.resLines + self.ecmLines + self.fmuLines:
            line.set_data([], [])
        self._hideCrosshair()
        self._autoscaleCharts()
        self.canvas.draw_idle()
        if self.fmuDetailWindow is not None and self.fmuDetailWindow.winfo_exists():
            self.fmuDetailWindow.update_data(self.tData, self.ecmPLossY, self.fmuTData, self.fmuY)
        self._setLogLines(["Data cleared."])
        self._refreshEcmSocChart()
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
