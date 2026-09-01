r"""Runtime-only vendored copy of thermal_rom for APP2_0 -- just the ThermalROM class.

The identification tooling (thermal_rom.identify, .stepres, .physics, .fmu_export)
lives in C:\code\ROM_pack; only the compiled model (pack_rom.npz) and the pure-NumPy
runtime that plays it back are needed here. See C:\code\ROM_pack\README.md for how
pack_rom.npz was built and validated against the Twin Builder FMU.
"""

from .rom import ThermalROM

__all__ = ["ThermalROM"]
