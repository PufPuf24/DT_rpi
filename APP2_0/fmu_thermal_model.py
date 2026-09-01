"""
Reálný časový obal okolo Pack_ROM_SML_ME.fmu (Ansys Simplorer, Model Exchange, 24 článků).

Heat generation pro všech 24 vstupů FMU = P_loss z ECM modelu (I²·R0 + v1²/R1 + v2²/R2),
stejný profil pro všechny (ECM reprezentuje jeden agregovaný článek, viz FMU/run_fmu_thermal.py).
Výstup: 24 objemově průměrovaných teplot na článek.

Dvě věci ověřené EMPIRICKY (nejde je poznat z dokumentace), obě zásadní pro spolehlivost:

1) fmpy/CVode (nativní Sundials knihovna) NENÍ bezpečné volat z jiného vlákna než hlavního —
   identické volání simulate_fmu(), které v hlavním vlákně čerstvého procesu spolehlivě
   projde, v threading.Thread nepředvídatelně padá (CVode Jacobian/RHS chyby). Proto se
   samotný FMU výpočet spouští v SAMOSTATNÉM PROCESU (multiprocessing) — Python vlákno se
   používá jen pro orchestraci (čekání na nová data, spouštění procesů), ne pro výpočet.

2) Inkrementální stepování (checkpoint stavu → jen navázat) je přes fmpy vysokoúrovňové API
   nespolehlivé (pád na fmi2SetupExperiment při opakovaném použití fmu_instance/fmu_state).
   Místo toho se periodicky přesimuluje CELÁ dosavadní historie P_loss(t) — pokud předchozí
   přepočet ještě neskončil, další kolo se přeskočí, takže se appka na pomalejším HW sama
   zpomalí místo pádu. Historie roste bez omezení, čas přepočtu s délkou testu poroste.

POZOR: .fmu obsahuje jen binaries/win64 — na Linux/ARM (RPi 5) se nenačte, dokud nebude
doplněná odpovídající platformní binárka (export ze Simplorer). Tenhle modul je platformně
neutrální (fmpy si binárku vybírá podle běžícího OS automaticky).
"""

import contextlib
import io
import multiprocessing
import os
import queue as stdlib_queue
import re
import threading
import time
import zipfile

import numpy as np

N_CELLS = 24
STOP_MARGIN = 10.0  # CVode "chokes", když má poslední adaptivní krok přistát přesně na
                     # posledním vzorku vstupu — ověřeno empiricky: 2 s nestačí (zvlášť při
                     # konstantním, neměnícím se vstupu — baterie v klidu), 10 s je bezpečné.


def _fmuIoMaps(fmuPath):
    xml = zipfile.ZipFile(fmuPath).read("modelDescription.xml").decode("utf-8")
    ins = re.findall(
        r'<ScalarVariable name="(Input\d+_Input\d+_volheat_batt(\d+))" valueReference="\d+" causality="input"',
        xml)
    outs = re.findall(
        r'<ScalarVariable name="(Output\d+_Output\d+_volave_batt(\d+))" valueReference="\d+" causality="output"',
        xml)
    inMap = {int(b): full for full, b in ins}
    outMap = {int(b): full for full, b in outs}
    if len(inMap) != N_CELLS or len(outMap) != N_CELLS:
        raise ValueError(f"FMU neobsahuje očekávaných {N_CELLS} vstupů/výstupů "
                          f"(nalezeno {len(inMap)} vstupů, {len(outMap)} výstupů).")
    return inMap, outMap


def _simulateOneCycle(fmuPath, inMap, outMap, outNames, tList, pList, resultQueue):
    """Běží v SAMOSTATNÉM PROCESU (viz modul docstring). Vrací nekalibrovaná data —
    kalibraci offsetu na reálnou počáteční teplotu dělá orchestrátor (má k tomu stav)."""
    try:
        tArr = np.asarray(tList, dtype=np.float64)
        pArr = np.asarray(pList, dtype=np.float64)

        dtype = [("time", np.float64)] + [(inMap[i], np.float64) for i in range(1, N_CELLS + 1)]
        fmuInput = np.zeros(len(tArr), dtype=dtype)
        fmuInput["time"] = tArr
        for i in range(1, N_CELLS + 1):
            fmuInput[inMap[i]] = pArr

        duration = float(tArr[-1] - tArr[0])
        margin = min(STOP_MARGIN, duration / 2.0)
        simStop = float(tArr[-1]) - margin
        outputInterval = max(20.0, duration / 100.0)

        from fmpy import simulate_fmu
        noise = io.StringIO()
        with contextlib.redirect_stderr(noise):
            result = simulate_fmu(
                fmuPath, start_time=float(tArr[0]), stop_time=simStop,
                output_interval=outputInterval, input=fmuInput, output=outNames,
                fmi_type="ModelExchange")

        rawEndC = [float(result[outMap[i]][-1]) - 273.15 for i in range(1, N_CELLS + 1)]
        rawStartC = [float(result[outMap[i]][0]) - 273.15 for i in range(1, N_CELLS + 1)]
        resultQueue.put(("ok", float(tArr[-1]), rawEndC, rawStartC))

    except Exception as ex:
        resultQueue.put(("error", None, None, str(ex)))


def _simulateFullHistory(fmuPath, inMap, outMap, outNames, tList, pList, initialTemperatureC,
                          resultQueue):
    """Běží v SAMOSTATNÉM PROCESU. Jednorázový dávkový běh přes CELOU historii najednou
    (pro přehrání uloženého souboru) — na rozdíl od _simulateOneCycle vrací celou časovou
    řadu, ne jen poslední bod, a kalibraci offsetu (na initialTemperatureC) dělá rovnou zde,
    protože se (na rozdíl od živého režimu) nic dalšího mezi voláními neděje."""
    try:
        tArr = np.asarray(tList, dtype=np.float64)
        pArr = np.asarray(pList, dtype=np.float64)

        dtype = [("time", np.float64)] + [(inMap[i], np.float64) for i in range(1, N_CELLS + 1)]
        fmuInput = np.zeros(len(tArr), dtype=dtype)
        fmuInput["time"] = tArr
        for i in range(1, N_CELLS + 1):
            fmuInput[inMap[i]] = pArr

        duration = float(tArr[-1] - tArr[0])
        margin = min(STOP_MARGIN, duration / 2.0)
        simStop = float(tArr[-1]) - margin
        outputInterval = max(20.0, duration / 100.0)

        from fmpy import simulate_fmu
        noise = io.StringIO()
        with contextlib.redirect_stderr(noise):
            result = simulate_fmu(
                fmuPath, start_time=float(tArr[0]), stop_time=simStop,
                output_interval=outputInterval, input=fmuInput, output=outNames,
                fmi_type="ModelExchange")

        tOut = result["time"].tolist()
        rawC = np.column_stack([result[outMap[i]] for i in range(1, N_CELLS + 1)]) - 273.15
        offset = np.asarray(initialTemperatureC) - rawC[0, :]
        calibratedC = rawC + offset
        resultQueue.put(("ok", tOut, calibratedC.tolist()))

    except Exception as ex:
        resultQueue.put(("error", None, str(ex)))


class FmuThermalWorker:
    """Orchestrátor na pozadí (Python vlákno): drží rostoucí historii P_loss(t) a periodicky
    spouští _simulateOneCycle v samostatném procesu. Výsledek posílá do resultQueue jako:
        ("ok", tLastS, [24 kalibrovaných teplot v °C], vypocetniDobaS)
        ("error", None, None, popisChyby)
    """

    def __init__(self, fmuPath, resultQueue, minRefreshIntervalS=20.0, subprocessTimeoutS=120.0):
        self.fmuPath = fmuPath
        self.resultQueue = resultQueue
        self.minRefreshIntervalS = minRefreshIntervalS
        self.subprocessTimeoutS = subprocessTimeoutS

        self._lock = threading.Lock()
        self._historyT = []
        self._historyP = []
        self._offsetC = [0.0] * N_CELLS
        self._pendingTargetC = None

        self._stopEvent = None
        self._thread = None

        self.available = False
        self.unavailableReason = None
        self.inMap = None
        self.outMap = None
        self._outNames = None

        if not os.path.exists(fmuPath):
            self.unavailableReason = f"soubor {fmuPath} nenalezen"
            return
        try:
            self.inMap, self.outMap = _fmuIoMaps(fmuPath)
            self._outNames = [self.outMap[i] for i in range(1, N_CELLS + 1)]
            self.available = True
        except Exception as ex:
            self.unavailableReason = str(ex)

    # ------------------------------------------------------------------
    def start(self):
        if not self.available or self._thread is not None:
            return
        with self._lock:
            self._historyT = []
            self._historyP = []
            self._pendingTargetC = None
        self._stopEvent = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        if self._stopEvent is not None:
            self._stopEvent.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stopEvent = None

    def isRunning(self):
        return self._thread is not None and self._thread.is_alive()

    def addSample(self, tSeconds, pLossW):
        if pLossW != pLossW:  # NaN — nemá smysl krmit model chybějícím vzorkem
            return
        with self._lock:
            self._historyT.append(tSeconds)
            self._historyP.append(pLossW)

    def setInitialTemperature(self, temperatureC):
        """Zkalibruje offset tak, aby FMU výstup na začátku odpovídal reálně naměřené teplotě
        (offset se dopočítá při příštím úspěšném přepočtu)."""
        with self._lock:
            self._pendingTargetC = temperatureC

    def startBatch(self, tList, pList, initialTemperatureC):
        """Jednorázový dávkový přepočet CELÉ zadané historie najednou (pro přehrání
        uloženého souboru) — běží asynchronně na pozadí, výsledek přijde do resultQueue
        jako ('batch_ok', [časy], [[24 teplot] na každý čas], None) nebo
        ('batch_error', None, None, popisChyby). Nezávislé na start()/stop() (živý režim)."""
        if not self.available or len(tList) < 2:
            self.resultQueue.put(("batch_error", None, None,
                                   "nedostatek dat" if self.available else self.unavailableReason))
            return

        def _run():
            mpQueue = multiprocessing.Queue()
            proc = multiprocessing.Process(
                target=_simulateFullHistory,
                args=(self.fmuPath, self.inMap, self.outMap, self._outNames,
                      tList, pList, initialTemperatureC, mpQueue),
                daemon=True)
            proc.start()
            try:
                tag, tOut, payload = mpQueue.get(timeout=300.0)
            except stdlib_queue.Empty:
                proc.terminate()
                proc.join(timeout=2.0)
                self.resultQueue.put(("batch_error", None, None, "výpočet vypršel (> 300 s)"))
                return
            finally:
                proc.join(timeout=5.0)
                if proc.is_alive():
                    proc.terminate()
                mpQueue.close()

            if tag == "ok":
                self.resultQueue.put(("batch_ok", tOut, payload, None))
            else:
                self.resultQueue.put(("batch_error", None, None, payload))

        threading.Thread(target=_run, daemon=True).start()

    # ------------------------------------------------------------------
    def _loop(self):
        while not self._stopEvent.is_set():
            with self._lock:
                t = list(self._historyT)
                p = list(self._historyP)

            if len(t) < 2:
                if self._stopEvent.wait(1.0):
                    return
                continue

            cycleStart = time.time()
            mpQueue = multiprocessing.Queue()
            proc = multiprocessing.Process(
                target=_simulateOneCycle,
                args=(self.fmuPath, self.inMap, self.outMap, self._outNames, t, p, mpQueue),
                daemon=True)
            proc.start()

            try:
                tag, tLast, rawEndC, rawStartC = mpQueue.get(timeout=self.subprocessTimeoutS)
            except stdlib_queue.Empty:
                proc.terminate()
                proc.join(timeout=2.0)
                self.resultQueue.put(("error", None, None,
                                       f"FMU výpočet vypršel (> {self.subprocessTimeoutS:.0f} s)"))
                if self._stopEvent.wait(5.0):
                    return
                continue
            finally:
                proc.join(timeout=5.0)
                if proc.is_alive():
                    proc.terminate()
                mpQueue.close()

            if tag == "error":
                self.resultQueue.put(("error", None, None, rawStartC))  # rawStartC nese chybu
            else:
                with self._lock:
                    if self._pendingTargetC is not None:
                        self._offsetC = [self._pendingTargetC - b for b in rawStartC]
                        self._pendingTargetC = None
                    offset = list(self._offsetC)

                calibratedC = [raw + off for raw, off in zip(rawEndC, offset)]
                elapsed = time.time() - cycleStart
                self.resultQueue.put(("ok", tLast, calibratedC, elapsed))

            waitS = max(self.minRefreshIntervalS - (time.time() - cycleStart), 1.0)
            if self._stopEvent.wait(waitS):
                return
