# SoH degradation estimation -- status and next steps

Three SoH sources, by design trust order (set directly by the user). All three
opportunistic-type sources (2 and 3) feed the SAME per-group `SohTracker`, so
whichever is more accurate at a given moment (smaller sigma) naturally
outweighs the other -- there is no separate trust-tier mechanism to maintain,
just each source's own honestly-computed accuracy.

1. **Maintenance validation cycle** (authoritative, ground truth) --
   `maintenance_cycle.py` + `soh_store.py`, wired into
   `BatteryMonitorGUI.py`'s Battery Health page. **Done and tested** (see
   `_startMaintenanceCycle`/`_maintenanceOnSample`/`_onMaintenanceComplete`
   and the end-to-end test that drove it through the real GUI with a fake
   serial port -- relay orchestration, Coulomb counting, per-group own-BOL
   `SOH = 100*Q/Q_ref`, persistence to `soh_state.json`/`soh_log.csv`,
   restart survival, second-cycle fade detection, abort).

2. **Opportunistic live estimation** (secondary/interim, between Maintenance
   runs) -- the pretrained FFNN from `C:\code2\python` (`deployment_package
   .joblib`, cell identity confirmed with the user to be the same cell the
   dissertation calibrated against). Vendored (`soh/online.py`,
   `soh/curves.py`, `soh/config.py`, `soh/dataset.py`, `soh/
   signal_tools.py`, `battlib/` -- the last only to satisfy the joblib
   pickle's import path, see `battlib/__init__.py`). **Done and tested**
   (see `_scanOpportunisticSoh`/`_scanGroupForFfnnObservation`/
   `_judgeAndFuseFfnnObservation` in `BatteryMonitorGUI.py`, and the
   end-to-end test that fed a synthetic steady discharge through the real
   `_applyMeasurement` and confirmed a candidate got fused, logged, and
   shown in the Battery Health page's secondary "live est." label).

### How the live wiring works

- `SteadyStateGate.ready()` (loaded from the package's own `cc_gate`) gates
  whether the live current tail is settled enough before `scan_segments` +
  `window_coverage` + `soh.curves.voltage_window_time` are even attempted --
  throttled to `SOH_OPPORTUNISTIC_SCAN_S` (30 s), not run every poll tick.
- Tries `best3` -> `best2` -> `best1` in order; a combination's windows must
  all come from segments within `SOH_COMBO_MAX_TIME_GAP_S` (1 h) of each
  other, so a charge window from an hour ago can't get silently combined
  with a discharge window from just now as if measured under one condition
  (the "don't combine features across too different conditions/time" concern
  raised when this was speced).
- **Bridging note**: `soh.online.AcceptanceGate.judge()` is built around a
  single-feature `FeatureCalibration` (one correlation `rho`, one domain
  range) -- an FFNN combination fusing 1-3 windows doesn't have either, so
  `judge()` isn't called for this path. `_judgeAndFuseFfnnObservation`
  reimplements the same condition-penalty/plausibility logic against real
  `Observation`/`Verdict` objects instead (so `SohTracker.update()` and the
  CSV log schema are untouched), using the package's own static accept/
  marginal band as the tier and `GateSettings` for the tolerance numbers.
- **C-rate ("smaller currents")**: every current combination is
  characterised at the dissertation's C01 (0.1C/0.1C); this bench mostly
  can't hit that. Per the user's answer, `GateSettings.crate_tolerance` is
  widened from the original 0.35 to 2.5 (see `soh/online.py`'s comment) so
  a slow bench segment gets a continuously scaled-up uncertainty penalty
  instead of a hard reject -- verified against a real widened-tolerance
  test case (see the end-to-end test), not just reasoned from the plan.
- **EFC caveat**: `SohTracker`'s Kalman drift needs "equivalent full
  cycles"; there is no persisted lifetime-throughput counter yet, so
  `_estimateEfc` approximates it from cumulative |Ah| THIS SESSION ONLY
  (`self._sohAhThroughputAh`, reset on every restart). A freshly restarted
  session therefore under-counts EFC and the tracker's uncertainty grows
  slower than it truly should across a restart -- worth a real persisted
  counter if this ends up mattering in practice.
- The `SOH_MAX_OBSERVATION_AGE_DAYS` (30) cap exists in the code but is a
  no-op for now (`ageDays = 0.0`, live observations are always "now") --
  it starts to matter once opportunistic history can be replayed/backfilled
  from `soh_log.csv` rather than only ever fused live.

3. **"Quick field" short-window estimation** (secondary, LOWEST trust of the
   three -- the user's own framing: "rychlá data, ne referenční", quick data,
   not reference, replaced whenever something more accurate turns up). Ported
   from `C:\code2\MTB\e09-e15` (Peukert rate-extrapolation study) and a new
   companion `C:\code2\python\scripts\08_field_window_deployment.py`. **Done
   and tested** (see `_scanFieldWindowSoh`/`_scanGroupForFieldWindowObservation`
   in `BatteryMonitorGUI.py`, and the end-to-end test that fed a realistic,
   physics-consistent ~7A discharge -- generated from the same ECM LUT the app
   itself uses -- through the real GUI and confirmed a `quick_field`-tagged
   candidate fused into the same tracker as the tV-window path).

### Why this exists alongside the tV-window estimator (2)

MTB's e14/e15 built a field-deployable SoH estimate for a storage system that
can ONLY run at 0.02C -- 5-10x below the dissertation's own reference rate,
needing Peukert-law rate extrapolation (e09-e12) to justify at all. This
bench's own described operating current (~7 A) is close to that reference
rate already (~0.1C for a ~75-78 Ah cell) -- so the SAME feature family
(delta-V over a FIXED, KNOWN capacity slice, not an open-ended voltage
crossing) was recalibrated directly at 7 A instead, with NO extrapolation
needed at all. Investigation script + real numbers: `scratchpad/
field_window_native_rate.py` (session-local, not part of the app) found
comparable-or-better validation RMSE than the ORIGINAL tV-window catalog,
using a fixed 10-30 minute window instead of an open-ended crossing time --
see `08_field_window_deployment.py`'s own header for the final, DEPLOY_CONFIG
numbers that actually shipped (all `best3` combos land in the "accept" band,
<0.5 pp; `best1` lands "marginal", 0.58-0.73 pp).

### How the live wiring works (and the bug it took to get there)

- Same `SteadyStateGate` precondition and `SOH_OPPORTUNISTIC_SCAN_S` throttle
  as the tV-window estimator; discharge-only (`iMa < -I_THRESHOLD_MA`), same
  sign-flip caveat.
- **Capacity positioning problem**: a completed lab RPT knows its own total
  discharge capacity `Qtot` in advance; a live PARTIAL window cannot, since
  the entire point is not running a full discharge. Solved by approximating
  `Qtot` as the group's own last Maintenance-cycle `Q_ref` (nominal capacity
  as a bootstrap before any Maintenance cycle has run), converted through the
  live SOC estimate (EKF, `self.ecmEkfSocY`) to know where a live discharge
  sits relative to a hypothetical 100%->0% span -- see `soh/field_window.py`.
- **The `MAX_POINTS` bug, found during testing**: the app's chart-history
  arrays (`self.tData`/`self.battY`/...) are capped at `MAX_POINTS` (1000)
  for chart responsiveness. The winning window combinations span capacity
  positions from 10% to 70% of a full discharge, which at bench rates can be
  many hours apart -- a real end-to-end test showed an 8 h discharge left
  only its final ~80 minutes inside `MAX_POINTS`, so the 10%/40% windows had
  no data left to find, only the late 70% one, and NOTHING ever fused. Fixed
  with a dedicated, much larger, independent buffer
  (`SOH_FIELD_HISTORY_MAX_POINTS = 20000`, `self._sohFieldT` and siblings),
  populated every `_applyMeasurement` tick regardless of the chart trim, and
  reset only where it has to be (`clearData()`, because `self.tStart`
  resetting there would otherwise mix pre- and post-reset timestamps in the
  same array -- NOT on a Maintenance cycle or anything else, matching the
  existing "ECM state isn't cleared, just the chart display" precedent).
- Tries duration tiers **longest-first** (30 -> 10 min), and within each,
  `best3 -> best2 -> best1` -- "delší je lepší", but gracefully falls back to
  whatever the accumulated discharge has actually covered so far. Confirmed
  in testing: early in a discharge (~11% of capacity covered), only a
  single-feature `best1` at the 10%-position window from ANY tier can
  possibly have full coverage yet -- the scan correctly found and used that,
  well before any 2-3 feature combo became available.
- Shares `_judgeAndFuseFfnnObservation` with the tV-window path (see that
  method's docstring) via a `runType`/`sourceLabel` parameter, so
  `soh_log.csv` and the debug log tell the two apart (`quick_field` vs
  `opportunistic`) while both still fuse into the one per-group `SohTracker`.

### `loadTestFile()` support (found and fixed after live testing looked fine)

`loadTestFile()` ("Load test data") already retroactively ran the tV-window
scan (`_scanOpportunisticSoh`) once over a whole loaded log -- see that
function's own comment -- but the quick-field scan hung off it too
(`_scanOpportunisticSoh` calls `_scanFieldWindowSoh` unconditionally) while
still hard-requiring the LIVE-only dedicated buffer (`self._sohFieldT` and
siblings, only ever filled by `_applyMeasurement`), so it silently did
nothing for a loaded file -- reported by the user testing exactly this way.
Fixed: `_scanFieldWindowSoh` now falls back to `self.tData`/`self.battY`
(the whole, uncapped loaded file -- `loadTestFile` never trims it the way
live `_applyMeasurement` does) plus `self.ecmSocY`, the open-loop
Coulomb-counted SOC `loadTestFile` itself already batch-computes (the EKF
SOC, `self.ecmEkfSocY`, is deliberately all-NaN for a replayed file -- the
EKF only runs live, see `loadTestFile`'s own comment). `loadTestFile` also
now resets the dedicated live buffer on every load, so a stale earlier live
session can't shadow the freshly loaded file's own data.

**Important caveat this surfaced, independent of the bug above**: the coarse
`centre_frac` window positions (0.1, 0.2, ... -- see `deployment_package_
field.joblib`) are positions along a full 0%->100% discharge, so even the
EASIEST usable coarse window (`centre_frac=0.1`, appearing e.g. in the 15-min
`best1`) needs the pack to have ALREADY discharged to roughly 11% of its
Q_ref/nominal-capacity bootstrap before its window's far edge is even
reachable -- at this bench's 7 A native rate against the ~78 Ah bootstrap,
that is **about 70-75 minutes from a fresh 100% SOC**, not 10-30. The 10-30
minute figure is the window's own WIDTH (how long the ΔV measurement itself
takes once positioned), not the total wait from a full-SOC start. Resolved
by the dense/early grid tier below -- see that section for why this was a
fixable gap (the coarse search simply never looked earlier than 10% depth)
rather than a hard physical limit.

### Dense/early grid sub-tier (any position, single feature, "regardless of what")

Reported by the user directly: they want a **single, ~15-20 minute pulse "děj
se co děj"** (come what may) to produce a quick-field estimate -- they don't
know in advance whether it will land on one of the 9 coarse `centre_frac`
positions above, and don't want to have to plan for that. The coarse grid's
own search (`08_field_window_deployment.py`) never explored anywhere below
10% depth-of-discharge, so it had no way to answer "what if the pulse only
gets partway there".

**Investigation, not assumption**: extended the SAME single-feature search
(`scratchpad/field_window_dense_grid_search.py`, not part of the app) down to
a step-0.02 grid from 2% depth-of-discharge, still at this bench's native 7 A.
Finding was the OPPOSITE of the intuitive worry that early-discharge features
would be noisier: positions from 2% to about 14% depth are consistently AS
GOOD OR BETTER than the original 10%-90% grid (val_rmse 0.4-0.8 pp there vs
0.28-0.73 pp for the coarse grid's own picks), and they are reachable in as
little as **13-24 minutes total from a fresh 100% SOC** -- squarely inside the
user's stated 10-30 minute, never-under-10 budget. There is also a genuinely
bad, unstable band around 34-46% depth (val_rmse up to 10 pp under the light
search config) -- not packaged.

**Deployment**: `C:\code2\python\scripts\09_field_window_early_grid.py`, a
companion to `08_field_window_deployment.py` -- same `deltaVCapacityWindowTable`
feature family, same `DEPLOY_CONFIG`/leave-one-cell-out rigor, but SINGLE
feature only (one measured window, matching "tady mám jedno nové okno
naměřené" -- one window measured, not a combo needing several aligned ones)
across 47 candidate positions (2%-94% step 2%) x 5 durations (10-30 min),
cheaply pre-filtered then fully refit. **126 entries survived** (accept/
marginal band), packaged sorted best-`val_rmse`-first as `deployment_package_
field_grid.{json,joblib}`. Best entry overall: `25min_c17` (34% depth,
val_rmse 0.393 pp) -- found only because the FULL `DEPLOY_CONFIG` refit was
run on every pre-filter survivor rather than trusting the cheap pre-filter's
own (much noisier) numbers for that specific position.

**Live wiring**: `_scanGroupForFieldWindowObservation` tries the coarse
combos first (unchanged, best3->best2->best1, longest duration first --
still more accurate when reachable, more features beats one), and only if
NONE of those are covered yet, falls back to the dense grid: iterates the
pre-sorted (best accuracy first) entry list, uses the FIRST one whose
`window_capacity_coverage` is already satisfied by the discharge so far. This
is "children of the moment" by design -- it doesn't matter which exact
position the live pulse happens to reach, whichever pretrained window is
BOTH already covered AND most accurate wins, with no separate "did I hit the
right window" logic needed. Tagged `sourceLabel=f"quick field grid {minutes}
min@{centre_frac:.0%}"` so `soh_log.csv` shows exactly which grid point fired.
Same `self.sohFieldGridDeployment`/`Error` optional-load pattern as the
coarse package -- the coarse-only path still works fine on its own if this
file is ever missing.

**Verified end-to-end**: a 25-minute, ~3.9%-depth simulated discharge (using
the SAME physics-consistent ECM voltage generator as the other end-to-end
tests) that the COARSE combos cannot reach at all produced 14 fused grid
candidates per group over that window (first one at ~18 minutes, `10min_c1`,
2% depth), converging the tracked SOH to +/-0.23 pp -- zero coarse `quick_
field` rows, confirming the two tiers stay cleanly separated in the log. The
existing coarse-only regression test now reaches its first candidate in 18
minutes instead of the previous 72 (it stops at the first fused candidate,
whichever tier produces it first) -- a direct, measured improvement, not
just a design intention.

## Phase 3 (later): refit calibrations from the bench's own data

Once enough Maintenance-cycle (feature, ground-truth SOH) pairs exist per
group, fit a `FeatureCalibration` (see `soh/online.py`'s `fit_calibration`
pattern -- would need porting/adapting, it currently assumes multiple cells
via leave-one-*cell*-out; here it's leave-one-*cycle*-out, one physical cell
per group) at the bench's OWN actual discharge rate (~0.025-0.05C, not the
dissertation's 0.1C), and swap it in over the pretrained network once its
cross-validated sigma is smaller. This is what actually resolves the C-rate
mismatch properly, rather than just tolerating it via a wide gate.

## Deliberately not implemented yet

- **ICA/DVA peak-position features** (`ica_features`/`dva_features` in
  `C:\code2\python\battlib\features.py`) -- their windows/prominences are
  tuned to the dissertation's own C01/C02/C05 rates; reusing them unexamined
  on a ~0.025-0.05C Maintenance discharge risks misassigned peaks. `soh/
  curves.py` computes the raw ICA/DVA CURVES (which is rate-agnostic and
  already useful/storable), stopping short of peak extraction until there's
  bench data to retune against.
- **Settings-page fields** for `maintenance_charge_full_v`/`charge_hold_s`/
  `charge_taper_a`/`rest_seconds` -- currently constructor defaults in
  `maintenance_cycle.py` plus the two persisted in `config.py`
  (`maintenance_discharge_cutoff_v`, `maintenance_charge_full_v`). Exposing
  the rest as Settings fields is straightforward, just not done yet.
