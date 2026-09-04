"""Persistence for SoH state -- battery_monitor_config.json's sibling for the
Battery Health page. Two files, same convention as config.py:

    soh_state.json   current state per series group (B01..B0n): Q_ref from the
                     first-ever Maintenance cycle, the last Maintenance result,
                     and the opportunistic SohTracker's fields (Phase 2) --
                     loaded on startup so SoH survives a restart, as specified.

    soh_log.csv      append-only, one row per Maintenance cycle result or
                     opportunistic Verdict (Phase 2). This is the ML-training
                     dataset the spec asked to keep growing, additive to (not
                     replacing) C:\\code2's own dissertation dataset.
"""

from __future__ import annotations

import csv
import json
import os

STATE_FILENAME = "soh_state.json"
LOG_FILENAME = "soh_log.csv"

# Column order for soh_log.csv. Not every row fills every column (a maintenance
# row has no `feature`/`tier`; an opportunistic row has no `q_mah`) -- blank is
# fine in a CSV, keeping one shared schema is what makes later ML use easy.
LOG_COLUMNS = [
    "time_iso", "group", "run_type",
    "soh_pct", "sigma_pct",
    "q_mah", "q_ref_mah",
    "rate_c", "temperature_c", "cutoff_v",
    "feature", "value", "tier", "accepted", "reasons", "source",
]


def _path(outDir, filename):
    return os.path.join(outDir, filename)


def load_state(outDir):
    """Returns {} if no state has ever been saved -- callers treat a missing
    group as "no Maintenance cycle yet", not an error."""
    path = _path(outDir, STATE_FILENAME)
    if not os.path.exists(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_state(state, outDir):
    path = _path(outDir, STATE_FILENAME)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(state, f, indent=2, ensure_ascii=False)


def append_log(row, outDir):
    """Appends one row to soh_log.csv, writing the header if the file is new.
    `row` may omit columns -- missing ones are written blank."""
    path = _path(outDir, LOG_FILENAME)
    isNew = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_COLUMNS, extrasaction="ignore")
        if isNew:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in LOG_COLUMNS})
