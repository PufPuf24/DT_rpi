"""
Perzistentní konfigurace Battery Digital Twin (mapování kanálů, prahy odpojení).
Ukládá se do battery_monitor_config.json vedle skriptu.
"""

import json
import os

CONFIG_FILENAME = "battery_monitor_config.json"

DEFAULTS = {
    "battery_channels": list(range(5, 13)),          # B01 = CH5 ... B08 = CH12
    "current_channels": {"IN": 13, "OUT": 14},
    "shunt_ohms": 0.001,
    "resist_channels": [
        [1, "Mid pack"],
        [2, "Case"],
        [3, "Electronics box"],
        [4, "T1"],
        [5, "T2"],
        [6, "T3"],
    ],
    "relay_channels": {"IN": 1, "OUT": 2},
    "cutoff_off_v": 27.5,
    "cutoff_on_v": 32.0,
    "cutoff_enabled": False,
    "cutoff_initial_state": False,
    "ecm_initial_soc_pct": 100.0,
    "ecm_parallel_count": 3,  # počet paralelních větví v packu (např. 8S3P = 3)
    "ecm_temp_source": "T3",  # název teplotního kanálu použitý pro ECM/FMU (Mid pack bývá nezapojený)
    # Vlastní (žlutá) bezpečnostní okna pro odhad -- uvnitř absolutních (červených) mezí
    # 2.5-4.2 V / -30-55 °C, viz BatteryMonitorGUI.CELL_ABS_*. Obecné výchozí hodnoty,
    # uživatel by je měl nastavit podle datasheetu svého článku.
    "custom_v_min": 2.8,
    "custom_v_max": 4.15,
    "custom_t_min": -10.0,
    "custom_t_max": 45.0,
}

# Bezpečný rozsah napětí na jeden článek — používá se k ohraničení prahů
# automatického odpojení, nikdy se sem samo nezasahuje na úrovni jednotlivých článků.
CELL_MIN_SAFE_V = 3.0
CELL_MAX_SAFE_V = 4.2


def _path(outDir=None):
    outDir = outDir or os.path.dirname(os.path.abspath(__file__))
    return os.path.join(outDir, CONFIG_FILENAME)


def load_config(outDir=None):
    path = _path(outDir)
    cfg = json.loads(json.dumps(DEFAULTS))  # hluboká kopie výchozích hodnot
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            cfg.update(loaded)
        except Exception:
            pass
    return cfg


def save_config(cfg, outDir=None):
    path = _path(outDir)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
