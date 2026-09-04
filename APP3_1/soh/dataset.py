"""Trimmed vendored copy of battlib/dataset.py -- only longest_true_run, which
online.py's scan_segments uses to isolate one contiguous charge/discharge/CV
run from short blips at the boundaries. See soh/__init__.py.
"""

from __future__ import annotations

import numpy as np


def longest_true_run(mask: np.ndarray) -> np.ndarray:
    """Keep only the longest contiguous run of ``True``."""
    mask = np.asarray(mask, dtype=bool).ravel()
    out = np.zeros_like(mask)
    if not mask.any():
        return out

    edges = np.diff(np.concatenate(([0], mask.view(np.int8), [0])))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)  # exclusive

    best = int(np.argmax(ends - starts))
    out[starts[best]: ends[best]] = True
    return out
