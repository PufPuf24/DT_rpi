"""Vendored subset of C:\\code2\\python\\battlib -- SoH degradation estimation.

Only what BatteryMonitorGUI.py's Battery Health page needs at runtime:
online.py (gating/fusion/provenance), features.py (voltage_window_time,
ICA/DVA), models.py (Ffnn, to unpickle deployment_package.joblib), and the
trimmed config.py/dataset.py those depend on. The full research pipeline
(correlation search, FFNN training, figures) stays in C:\\code2\\python --
this package only carries what runs live, same convention as thermal_rom/
being vendored from C:\\code\\ROM_pack.
"""
