r"""
Reálný časový obal okolo pack_rom.npz (Python ROM, čisté NumPy) — náhrada za
Pack_ROM_SML_ME.fmu (Ansys Twin Builder). Veřejné API (available/start/stop/
isRunning/addSample/setInitialTemperature/startBatch, tvar zpráv v resultQueue)
je BEZE ZMĚNY vůči fmu_thermal_model.FmuThermalWorker — jde o čistou záměnu
implementace pod stejným rozhraním, BatteryMonitorGUI.py se mění jen o import
a cestu k souboru modelu (viz FMU_THERMAL_FILE).

Heat generation pro všech 24 vstupů ROMu = P_loss z ECM modelu (I²·R0 + v1²/R1 +
v2²/R2), stejný profil pro všechny (ECM reprezentuje jeden agregovaný článek,
stejně jako u FMU verze — viz ROM_pack/run_pack_thermal.py).

Proč tahle verze nepotřebuje multiprocessing ani STOP_MARGIN, na rozdíl od FMU:
ThermalROM.step()/.temperatures() jsou čisté NumPy funkce (diagonální stavový
prostor, analytický exp() na pól, žádný nativní iterační solver) — bezpečné
volat z libovolného vlákna a nemají žádnou "poslední vzorek" fragilitu, kterou
má CVode u Model-Exchange FMU (viz fmu_thermal_model.py, docstring nahoře).

Živý režim navíc nemusí přepočítávat CELOU historii na každém refreshi: krok
ROMu je O(1) (624 stavů), takže se místo toho jen průběžně střádá stav
(`rom.step(...)`) od místa, kam se dostal minule — cena jednoho refreshe je
úměrná POČTU NOVÝCH vzorků, ne délce dosavadního testu. Naměřeno v
ROM_pack/benchmark.py: ~16 µs na nový vzorek, oproti ~1,7 ms/vzorek u FMU
(Model Exchange + CVode, 2280 stavů) — floor faktoru ~110×, a s FMU navíc
rostoucí s délkou testu (celá historie se tam musí přesimulovat od nuly).
"""

from __future__ import annotations

import os
import threading
import time

import numpy as np

from thermal_rom import ThermalROM

N_CELLS = 24


class PackRomThermalWorker:
    """Stejné veřejné API jako fmu_thermal_model.FmuThermalWorker — viz tam pro
    kontrakt zpráv ve resultQueue: ("ok", tLast, [24 °C], elapsedS),
    ("error", None, None, popis), ("batch_ok", [časy], [[24 °C] na čas], None),
    ("batch_error", None, None, popis)."""

    def __init__(self, romPath, resultQueue, minRefreshIntervalS=20.0, subprocessTimeoutS=None):
        self.resultQueue = resultQueue
        self.minRefreshIntervalS = minRefreshIntervalS
        # subprocessTimeoutS je jen kvůli shodné signatuře s FMU verzí — ROM
        # se nepouští v samostatném procesu, takže se nikde nepoužívá.

        self._lock = threading.Lock()
        self._historyT = []
        self._historyP = []
        self._appliedIdx = 0            # kolik vzorků historie je už promítnutých do rom.z
        self._pendingTargetC = None

        self._stopEvent = None
        self._thread = None

        self.available = False
        self.unavailableReason = None
        self.rom = None

        if not os.path.exists(romPath):
            self.unavailableReason = f"soubor {romPath} nenalezen"
            return
        try:
            rom = ThermalROM.load(romPath)
            if rom.n_in != N_CELLS or rom.n_out != N_CELLS:
                raise ValueError(f"ROM neobsahuje očekávaných {N_CELLS} vstupů/výstupů "
                                  f"(má {rom.n_in} vstupů, {rom.n_out} výstupů).")
            self.rom = rom
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
            self._appliedIdx = 0
            targetC = self._pendingTargetC
            self._pendingTargetC = None
        self.rom.reset(T_amb=(targetC + 273.15) if targetC is not None else None)
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
        """Nastaví okolní/počáteční teplotu ROMu na reálně naměřenou hodnotu.
        Na rozdíl od FMU verze (dopočet offsetu ex post z prvního výsledku) se tu
        aplikuje přímo jako T_ambient — model je lineární kolem libovolné
        rovnovážné teploty, takže jde o matematicky stejnou operaci, jen o krok
        dřív a bez zaokrouhlovací chyby offsetu."""
        with self._lock:
            self._pendingTargetC = temperatureC
            if self._thread is not None:
                self.rom.T_ref = temperatureC + 273.15
                self._pendingTargetC = None

    def startBatch(self, tList, pList, initialTemperatureC):
        """Jednorázový dávkový přepočet CELÉ zadané historie najednou (pro přehrání
        uloženého souboru). Běží na pozadí ve vlákně (ne v procesu — ROM je čistá
        NumPy funkce, žádná CVode fragilita, kterou by bylo třeba izolovat).

        Na konci se živý `self.rom.z`/`T_ref` přepíše na stav na KONCI přehrané
        historie (přes `temperatures_with_state`, ne obyčejné `temperatures`) --
        jinak by živý model dál "myslel", že je na výchozí (typicky nulové) teplotě,
        i když graf ukazuje konec souboru. Bez týhle synchronizace by Estimation
        (BatteryMonitorGUI._runEstimation) po loadTestFile/loadCurrentProfile
        projektovala z fyzikálně špatného výchozího bodu -- přesně tenhle případ
        nahlásil uživatel 2026-08-31 (skok teploty na začátku odhadu)."""
        if not self.available or len(tList) < 2:
            self.resultQueue.put(("batch_error", None, None,
                                   "nedostatek dat" if self.available else self.unavailableReason))
            return

        def _run():
            try:
                t = np.asarray(tList, dtype=np.float64)
                p = np.asarray(pList, dtype=np.float64)
                tRef = initialTemperatureC + 273.15
                T, zEnd = self.rom.temperatures_with_state(t, p, T_amb=tRef)
                with self._lock:
                    self.rom.z = zEnd
                    self.rom.T_ref = tRef
                self.resultQueue.put(("batch_ok", t.tolist(), (T - 273.15).tolist(), None))
            except Exception as ex:
                self.resultQueue.put(("batch_error", None, None, str(ex)))

        threading.Thread(target=_run, daemon=True).start()

    def snapshotState(self):
        """Thread-safe (z, T_ref) snapshot of the live thermal state, for a forward
        *projection* that must not disturb it. `ThermalROM.temperatures(..., z0=...)`
        is a pure function of its arguments -- it never touches `self.rom.z` -- so the
        caller can run `rom.temperatures(tRel, pLossAssumed, z0=z, T_amb=tRef)` on this
        same rom object with a hypothetical future P_loss, entirely independent of
        whatever the live worker does with it next. Returns (None, None) if the model
        isn't available."""
        if self.rom is None:
            return None, None
        with self._lock:
            return self.rom.z.copy(), self.rom.T_ref

    # ------------------------------------------------------------------
    def _loop(self):
        while not self._stopEvent.is_set():
            with self._lock:
                t = list(self._historyT)
                p = list(self._historyP)
                appliedIdx = self._appliedIdx

            if len(t) < 2 or appliedIdx >= len(t) - 1:
                if self._stopEvent.wait(1.0):
                    return
                continue

            cycleStart = time.time()
            # first-order hold mezi po sobě jdoucími vzorky -- stejná rekonstrukce
            # vstupu (po částech lineární), jakou FMI dělá u spojitého vstupu
            for k in range(appliedIdx, len(t) - 1):
                dt = t[k + 1] - t[k]
                if dt > 0:
                    self.rom.step(p[k], dt, u_next=p[k + 1])
            temps = (self.rom.outputs() - 273.15).tolist()

            with self._lock:
                self._appliedIdx = len(t) - 1

            elapsed = time.time() - cycleStart
            self.resultQueue.put(("ok", t[-1], temps, elapsed))

            waitS = max(self.minRefreshIntervalS - (time.time() - cycleStart), 1.0)
            if self._stopEvent.wait(waitS):
                return
