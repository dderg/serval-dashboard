# Servo calibration reference

Command reference for tuning an A6-EC servo axis (EtherCAT). The
`[servo_calibration]` extension registers the `SERVO_*` console commands. Each
experiment command writes a run directory under `captures_root` (a
`manifest.json`, one `step_<name>.scap.zst` per step — zstd-compressed inline
by kalico's capture writer at write time — optional accelerometer CSVs)
and then invokes the `servo-cal` Rust binary — `analyze` writes `results.json`
with a typed verdict, `fit` writes a dynamics profile. Drive-parameter access
comes from the kalico repo: `klippy/extras/servo_param.py` and
`klippy/extras/servo_capture.py`. For the run-directory and `results.json`
schemas see [servo-cal-contracts.md](servo-cal-contracts.md); for the
inertia/feedforward fit theory see the kalico repo:
`docs/rewrite/servo-feedforward.md`; for the capture format see the kalico
repo: `docs/rewrite/servo-telemetry-capture.md`.

## Enabling

Add a bare section to `printer.cfg` and put the values that do not change
between runs in it:

```ini
[servo_calibration]
servos: motor_a, motor_b        # drives excited by the CoreXY grid (name one for a single-motor axis)
rated_torque_nm: 0.32           # motor datasheet rated torque, N*m
rotor_inertia_kgm2: 0.0000033   # rotor inertia, kg*m^2 (datasheet 1e-4 value x 1e-4)
x_start: 30                     # safe stroke window, mm
x_end: 220
y_start: 30
y_end: 220
accels: 5000, 10000, 20000      # excitation grid, mm/s^2
speeds: 100, 400                # excitation grid, mm/s
iterations: 3
dwell_ms: 700
```

Every option is optional and every command accepts a matching per-run override
(`SERVO_CALIBRATE_GAINS AXIS=Y START=40`). `rated_torque_nm`/
`rotor_inertia_kgm2` have no default — a command that needs them errors unless
they are configured or passed.

| Option | Default | Used by |
|---|---|---|
| `servos` | `stepper_x, stepper_y` | single-drive default for `SERVO=`/`AXIS=`-less commands; on `coupled_xy` kinematics the measure/fit commands derive their drives from the kinematics instead (`SERVOS=` overrides) |
| `rated_torque_nm` | — | inertia-ratio commands (`TORQUE_NM=`) |
| `rotor_inertia_kgm2` | — | inertia-ratio commands (`INERTIA_KGM2=`) |
| `x_start` / `x_end` | `20` / `200` | X strokes (`START`/`END`, `X_START`/`X_END`) |
| `y_start` / `y_end` | `20` / `200` | Y strokes (`Y_START`/`Y_END`) |
| `accels` | `5000, 10000, 20000` | excitation grid (`ACCELS=`) |
| `speeds` | `100, 400` | excitation grid (`SPEEDS=`) |
| `iterations` | `3` | strokes per grid point (`ITERATIONS=`) |
| `dwell_ms` | `700` | settle between strokes (`DWELL_MS=`) |
| `travel_speed` | `100` | CoreXY centering moves between grid points |
| `compliance_amplitude` | `0.02` | default buzz amplitude (mm) for the compliance identification sweep (`SERVO_MEASURE_COMPLIANCE AMPLITUDE=`, `SERVO_TUNE_PIN MEASURE_AMPLITUDE=`) |
| `pin_sweep_amplitude` | `0.01` | default dwell-tone amplitude (mm) for the pin staircases (`SERVO_SWEEP_PIN`/`SERVO_TUNE_PIN` `AMPLITUDE=`) |
| `accel_chip` | — | accelerometer section name (e.g. `adxl345`); when set, `SERVO_CALIBRATE_GAINS` records vibration per step, the pin staircases (`SERVO_SWEEP_PIN`/`SERVO_TUNE_PIN`) score the toolhead accel at the tone, and it is the default (and required) chip for `SERVO_COMPARE_PIN` (`ACCEL_CHIP=`) |
| `captures_root` | `~/printer_data/logs/servo_captures` | parent directory for experiment run directories |
| `journal_params` | — | comma list of drive SDO addresses (`addr[:type]`, e.g. `0x2001.0x31:u16`) read back from every captured drive at run start and recorded under `ambient.journal_params` in the manifest — the campaign's varied registers (notch mode, etc.) |
| `servo_cal_binary` | `target/snapshot/servo-cal` | path to the `servo-cal` analysis binary |

Prerequisites: the EtherCAT servo stack in the kalico repo
(`klippy/extras/servo_param.py`, `klippy/extras/servo_capture.py`) must be
configured, and the `servo-cal` binary must be built once on the host with
`cargo build --profile snapshot -p servo-ident` from this repo root.

## Tuning order

1. **Enable feedforward** — set `velocity_ff: True` on each `[motor]` so the
   tuning runs measure the loop as it will actually be driven (see the kalico
   repo: `docs/rewrite/servo-feedforward.md`).
2. **`SERVO_CALIBRATE_INERTIA_RATIO`** — identify the load inertia and
   set the base C00.06, before touching the loop gains. On `coupled_xy`
   kinematics this runs the coupled X+Y grid and fits both belt directions;
   `SERVO_SWEEP_INERTIA` empirically verifies / refines C00.06 later, at the
   tuned gains.
3. **`SERVO_APPLY_GAINS`** then **`SERVO_CALIBRATE_GAINS`** — find the loop
   gains.
4. **`SERVO_FIT_DYNAMICS`** — identify the dynamic profile at the final gains
   (iteratively, with the candidate feedforward live in the loop on
   `coupled_xy`) and point `dynamics_profile` at it to enable torque
   feedforward.
5. **`SERVO_TUNE_DYNAMICS`** — empirically tune the fitted profile on the
   running endpoint (closed-loop coordinate descent scored by the
   transient-window ferr rms; `TERMS=MASS,VISCOUS,COULOMB,LEAD` by default)
   when the regression fit varies with the excitation grid; point
   `dynamics_profile` at the tuned TOML it writes.

**`SERVO_MEASURE_TRACKING`** is the before/after check for any single change.
**`SERVO_AUTOTUNE`** packages this exact order into one command — see
[SERVO_AUTOTUNE](#servo_autotune) below — for a bench that has already built
trust in the verdicts; the manual sequence above remains the way to run any
one step in isolation or to diagnose a step SERVO_AUTOTUNE aborted on.

Every stroke is paced `M400 / G4 / M400` so it replans from idle, and the
stroke engine refuses any `(speed, accel)` pair whose `v²/a` exceeds the stroke
span — that pair cannot reach the target speed within the travel and would not
produce the intended excitation.

## Measurement commands

Every capture reads each drive's EtherCAT sync loss counter (C13.04) before
and after the strokes. The drive silently tolerates up to C13.02 (default 8)
consecutive lost/late sync events before faulting, so a tolerated loss shows
up nowhere except this counter — but it makes the drive's internal position
demand skip and double-step, injecting a following-error transient of
exactly one cycle of travel. When the counter moved during a capture the
command prints a WARNING naming the drives and deltas (and emits a
`calibration/sync_loss` event): that step's tracking metrics are
contaminated and must not be compared or scored. `SERVO_SHOW_TUNING` also
reads C13.02/C13.04 for manual checks.

#### SERVO_MEASURE_TRACKING
Single accel/speed stroke run with capture, then prints per-move following
error, overshoot and settling — the before/after check for any tuning change.
Params: `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000) `ITERATIONS` (3)
`DWELL_MS` `NAME` (track). Writes a run directory and runs `servo-cal analyze`.

#### SERVO_MEASURE_DIFFERENTIAL
Anti-phase position chirp on one AWD belt pair via the engine-resident buzz
generator: the two drives of the belt are commanded in opposite directions,
so the carriage holds (nominally) still while the drives strain the belt
against each other. The capture therefore isolates the differential
(rotor-vs-rotor) dynamics — the modes excited when paired drives fight —
and the analysis reports each detected mode's frequency, closed-loop peak
gain, half-power damping ratio and coherence, plus a differential FRF
(magnitude, phase, coherence, differential-torque spectrum) the dashboard
renders. Belt strain between the pair is **twice** `AMPLITUDE`; the command
caps `AMPLITUDE` at 0.5 mm. Needs two drives per belt. Params: `BELT` (A)
`FREQ_START` (20) `FREQ_END` (250) `HZ_PER_SEC` (5) `DURATION`
(band/`HZ_PER_SEC`) `AMPLITUDE` (0.05 mm) `RAMP` `DWELL_MS` `NAME` (diff).
Writes a run directory and runs `servo-cal analyze`.

#### SERVO_DIFF_DAMPER
Arms (or disarms) the engine-resident differential belt-pair damper on an
AWD machine. Every EtherCAT cycle the endpoint differentiates the pair's
raw encoder positions, low-passes the differential velocity and streams an
**antisymmetric** torque offset (60B2h) to the pair — a virtual dashpot
connected between the two rotors. Because the torques are equal and
opposite through the belt, the carriage sees no net force, and on
synchronized motion the differential velocity is zero, so the damper costs
no torque during printing. Unlike a notch filter it is frequency-agnostic:
it damps the inter-motor belt mode wherever toolhead position has moved it.
`GAIN` is in units of 0.1% rated torque per mm/s of differential velocity;
`GAIN=0` disarms the belt's damper. The injected torque is clamped to
`CLAMP` (0.1% rated torque, command ceiling 300) and the velocity is
low-passed at `LPF_HZ`. Velocity comes from host-side position
differencing, NOT the drive's 606Ch estimate — the drive's estimator lag
pushes delayed velocity feedback past 90° in the very band being damped,
which pumps the mode instead; `LEAD_US` (first-order lead, microseconds)
compensates the remaining EtherCAT transport and drive torque-path lag and
is tuned empirically: if the A/B sweep shows the peak *sharpening* above
some frequency, add lead until the whole band damps. State lives in the
running endpoint — re-arm after a firmware restart. Verify with an A/B
`SERVO_MEASURE_DIFFERENTIAL` sweep: the pair mode's damping ratio should
rise with the damper on. Params: `BELT` (AB) `GAIN` (required) `CLAMP`
(50) `LPF_HZ` (300) `LEAD_US` (0).

#### SERVO_DIFF_TRIM
Arms (or disarms) the engine-resident differential belt-pair **trim** —
standstill zeroing of the pair fight, config section `[servo_diff_trim]`.
Servo sync and homing leave a run-to-run differential preload between the
two drives of a belt (each enable seeds at a slightly different relax
point), so the same strain-comp map can show a different peak differential
torque on every home+sync cycle. Whenever the pair sits at **commanded
standstill** the endpoint low-passes its mechanical-frame differential
torque and integrates it into a small flat **antisymmetric position
offset** on top of the streamed targets and the strain-comp map: the pair
unwinds against itself while the carriage never moves. During motion the
loop freezes entirely (filter and integrator) — an in-motion differential
torque is legitimate (commanded feedforward, direction- and
toolhead-position-dependent inner-loop load) and must not be nulled. A
slot counts as quiescent only when its piece ring is empty (pieces land at
least the feedforward lead before their start, so an empty ring also
proves no lead-window torque is being commanded), no buzz is running, the
strain-comp ramp has settled, and the pair has been still for `SETTLE_MS`
(torque relax + telemetry lag after a decel). The offset resets on a pair
sync or torque-gate drop — the `SERVO_SYNC` release is the new zero.
`GAIN` is mm/s of offset slew per 1% differential torque — start around
`0.0001` and raise it until the offset converges within your typical
dwells; too much gain (with the loop crossover approaching the LPF
corner) oscillates. `GAIN=0` freezes the loop with the learned offset
held. Retuning any knob updates the running pair in place — the learned
offset and filter state carry over — and `REMOVE=1` drops the pair (and
its offset) entirely. `MAX_OFFSET_UM` (µm, ceiling 500) bounds the
offset — the trim's total authority; hitting it logs a
`diff_trim_clamped` warning: residual fight beyond what the trim may
absorb. Torque LPF at `LPF_HZ` (floor 0.1). Config options (`gain`,
`max_offset_um`, `lpf_hz`, `settle_ms`) arm the trim at startup when
`gain` is non-zero; the command overrides them live for tuning and
`SAVE=1` stages the current values for `SAVE_CONFIG`. Params: `BELT`
(AB) `GAIN` `MAX_OFFSET_UM` (150) `LPF_HZ` (2) `SETTLE_MS` (300)
`REMOVE` (0) `SAVE` (0).

#### SERVO_MEASURE_STRAIN_MAP
The measurement half of the belt strain map (CoreXY only). Rasters the bed
with slow constant-speed strokes — serpentine X sweeps stepped along Y by
`LINE_SPACING`, then Y sweeps stepped along X — recording one capture per
line, each stroked forward and back so the direction-dependent (friction)
half of the differential torque averages out of the analysis. The
per-belt differential pair torque as a function of (x, y) is the raw
material for separating trapped preload (DC), pulley/idler runout
(periodic in travel at each element's circumference — 40 mm motor
pulleys) and geometry/squareness (smooth 2D) — and eventually for the
feedforward strain-compensation map. `LINE_SPACING` must stay under half
the shortest period of interest (10 mm covers the 40 mm and ~13 mm
elements). Before rastering the carriage parks at the region center and
`SERVO_SYNC` releases the trapped preload, so the DC of every map is
measured from the same zero (`SYNC=0` skips, and the manifest records
`zero_sync`); without `[servo_sync]` configured the command errors. The
run directory is charted by the dashboard's strain tab. Params: `SPEED`
(50) `ACCEL` (1000) `LINE_SPACING` (10) `X_START` `X_END` `Y_START`
`Y_END` `DWELL_MS` `TAG` (strain) `SYNC` (1).

#### Strain compensation (SERVO_STRAIN_COMP_TUNE / SERVO_MEASURE_STRAIN_RESPONSE / SERVO_MEASURE_PAIR_STIFFNESS / SERVO_STRAIN_COMP_BUILD / SERVO_STRAIN_COMP_FIT / SERVO_STRAIN_COMP)
The application half of the strain map, config section
`[servo_strain_comp]`. The endpoint carries a per-belt 2D lookup table of
**antisymmetric position offsets** keyed on the commanded carriage
position: every cycle it reconstructs (x, y) from the streamed lane
positions, bilinearly interpolates each belt's grid, and offsets the
pair's two drives by equal and opposite amounts — the rotors absorb the
position-dependent tension variation (belt thickness lumps, pitch
nonuniformity, frame geometry) instead of fighting through the belt,
while the carriage never moves. Offsets ride outside the command anchors
(like the differential trim's), are clamped to ±500 µm (the grid's span
too, since re-anchoring can apply the full span) and slew-limited to
1 mm/s, so enabling, replacing, or clearing a map can never yank the
targets.

**Re-anchoring.** The map's DC follows the mechanics. Whenever torque
drops — SERVO_SYNC, M84, idle timeout — the free rotors relax the pair's
differential strain at wherever the carriage sits, including a hand-move
while unpowered, so the position where torque returns is the new
physical zero. The endpoint re-anchors the map there automatically: it
samples the grid at the re-engage position and applies everything
relative to that value, so a freshly relaxed gantry is never re-racked
and jogging after an idle timeout just works. The accepted limitation:
the map's residual error is then measured from the re-engage position
instead of the calibrated zero, so an anchor at a field extreme can
roughly double the worst-case residual. When accuracy matters — before a
print, before measuring a residual strain map (`MERGE=1`) — run
SERVO_SYNC at the map's zero point to restore the calibrated anchor;
nothing does this for you. The live anchor bias is visible in the
`strain_comp_state` event (`anchor_bias_um`).

**The stiffness is a matrix.** The two belts share the gantry, so an
antisymmetric offset on one pair also strains the other — on the Trident
bench the cross term is ~25% of the direct term, symmetric (reciprocity),
and same-signed with the racking direction. Dividing each belt's field by
its own scalar stiffness therefore copies every single-belt feature into
the other belt at the coupling ratio (the diagonal "ghost" in a
verification map). The build instead solves the 2×2 system per grid node,
`offsets = -inv(K) @ strain`: each belt gets its own correction plus a
partial same-sign helper offset for the other belt's field. A
near-singular matrix (cross terms rivaling the direct terms) fails
loudly.

**Measure the matrix rolling, not parked.** A parked belt reads
stiffer than one rolling over the pulleys and idlers — on the bench
the parked probe (`SERVO_MEASURE_PAIR_STIFFNESS`) reads ~428/−122
while every rolling measurement lands at ~353/−89, consistently across
independent excitations. The map operates while moving, so calibrate
in that regime; the parked probe supplies the starting values (its
SHAPE — the cross/direct ratio — carries over; only the overall scale
is off), and the tune loop converges the scale against reality.

`SERVO_STRAIN_COMP_TUNE RUN=<baseline raster>` is that loop: rebuild
the FULL map from the run at the trial matrix (never merging), enable
it, sweep an X **and** a Y verification line, and refit every belt's
direct and cross stiffness from the measured response to the applied
offsets — the two sweeps swap the belts' roles, so all four matrix
elements are measured independently (a single X line cannot separate
them: the own- and cross-corrections it applies are near-collinear,
and the old row-scale loop silently froze the cross:direct ratio at
whatever was passed in). Repeat until the measured matrix reproduces
the applied one — per element, within `TOL` of the row's direct
stiffness. Each pass costs two line sweeps, not a raster, and one
correction step usually lands it. When it converges the tuned
full-bed map is already on disk and enabled — it is the same map that
was being verified — and the matrix is stored for future builds and
recorded in the map (`stiffness_pct_per_mm`, `cross_pct_per_mm` per
pair). Running out of `MAX_ITERS`, lines the map doesn't vary along,
collinear own/cross corrections, or a measured direct stiffness
outside (0.2, 5)× the applied one all fail loudly. Know what the
numbers cover: the verification metric is the **smooth elastic
field** (forward/backward passes are averaged, so direction-dependent
friction asymmetry cancels out of it, and sub-20 mm ripple is below
the field model's bandwidth) — raw differential measurements keep
both, so they read higher than the tune's residual and no
position-keyed map can close that gap. Open-loop alternates that measure the matrix directly:
`SERVO_MEASURE_STRAIN_RESPONSE` steps a constant antisymmetric offset
through each pair's bank (0, ±STEP_UM, ±2·STEP_UM) while stroking one
line — the line's field cancels out of the offset-response slope, no
map or baseline needed; and `SERVO_STRAIN_COMP_FIT
BASELINE=<uncompensated run> RUN=<compensated run>` regresses the
field change between an existing run pair against the offsets the map
applied. The exact scale is in any case not critical for `MERGE=1`
convergence: a fractional matrix error leaves the same fraction of
the field behind, and each merge pass shrinks it by that factor
again.

The workflow: (1) `SERVO_MEASURE_STRAIN_MAP` rasters the baseline;
`SERVO_MEASURE_PAIR_STIFFNESS` (parked) supplies starting values.
(2) `SERVO_STRAIN_COMP_TUNE RUN=<raster> SPACING=5` converges the
matrix and leaves the tuned map enabled. Its rebuilds are the same
build as `SERVO_STRAIN_COMP_BUILD RUN=<dir>`, which fits each belt's
dense line
samples with a structured field model — 1D components at 2 mm knots
along each belt phase (x+y, x−y; CoreXY only) and along each axis, plus
a smooth 2D remainder — evaluates the model at the output grid nodes,
zeroes the maps at the region center (SERVO_SYNC's zero point), solves
the per-node 2×2 system, and writes `map_file` (default
`~/printer_data/config/strain_comp.json`). The model matters:
point-sampling the raster at grid nodes aliases everything shorter than
twice the node pitch, and the dominant fine structure is belt-phase
diagonal at the 40 mm pulley period — on the bench it left a ~35%
diagonal residue that the model build removes because diagonals stay
diagonal between the raster lines. Pass `SPACING=5` on CoreXY so the
40 mm harmonics also survive the endpoint's bilinear lookup (57×55
stays within the 64/4096 grid caps on a 300 mm bed; the build fails
loudly beyond them).
(3) `SERVO_STRAIN_COMP ENABLE=1` resolves the map's motor names to
slots/lanes on the live topology and uploads it (the tune already
leaves it enabled); `ENABLE=0` ramps the compensation back out. Verify
with a full raster when the whole-bed picture matters — the residual
field should collapse. (4) Fold later residuals straight into the map:
`SERVO_STRAIN_COMP_BUILD RUN=<verification run> MERGE=1` (no stiffness
params needed: the recorded matrix is reused), then `ENABLE=1`.
Params: tune `RUN` (required) `SPACING` `TOL` (0.05) `MAX_ITERS` (5)
`Y` (map zero) `SPEED` (50) `ACCEL` (1000) `SETTLE` (0.8) `DWELL_MS`
`TAG` `SYNC` plus the build's matrix overrides; response `SPEED` (50)
`ACCEL` (1000) `STEP_UM` (50) `SETTLE` (0.8) `Y` (area center)
`X_START`/`X_END` `DWELL_MS` `TAG` `SYNC`;
parked stiffness `STEP_UM` (50) `SETTLE` (0.8) `AXIS`; build `RUN`
(required) `STIFFNESS_A`/`STIFFNESS_B` with `CROSS_AB`/`CROSS_BA`
(%/mm matrix override; `CROSS_AB` is belt A's response to a belt B
offset, 0 disables the cross term) `SPACING` (run's line spacing);
fit `BASELINE` and `RUN` (both required).

#### SERVO_MEASURE_INERTIA
Records the excitation grid for the inertia/friction fit (no report — it is the
capture building block behind the fit commands). The active kinematics decides
the shape of the grid:

- **`coupled_xy` kinematics** (CoreXY): one capture of **every** belt drive
  with X and Y strokes at every grid point (`SERVOS=` overrides; the default
  is every motor the kinematics says drives the belts), so the fit sees each
  Cartesian mode excited on its own (X strokes excite only the x mode, Y
  strokes only the y mode). Before each stroke set the
  toolhead moves (at `travel_speed`) to the active axis' start with the idle
  axis centered in its range, so both belt runs are near-equal length during
  the measurement. Bounds come from `X_START`/`X_END`/`Y_START`/`Y_END`.
- **cartesian kinematics**: captures every motor that moves `AXIS` (every
  drive of an AWD rail), bounded by `START`/`END`. `SERVOS`, `X_START`,
  `X_END`, `Y_START`, `Y_END` only apply to `coupled_xy` kinematics and are
  rejected with an error otherwise.

Params: `AXIS` (X) `START` `END` `X_START` `X_END` `Y_START` `Y_END` `ACCELS`
`SPEEDS` `ITERATIONS` `DWELL_MS` `NAME` (ident) `SERVOS`.

## Fit / inertia-ratio commands

#### SERVO_FIT_DYNAMICS
Identifies one mass/viscous/coulomb triple per Cartesian mode (see the model
in the kalico repo: `docs/rewrite/servo-feedforward.md`) and writes a timestamped
feedforward profile. Optional `TORQUE_NM` + `INERTIA_KGM2` also print the
recommended C00.06. The active kinematics decides both the frame and the
identification strategy:

- **`coupled_xy` kinematics**: iterative closed-loop identification. Each
  round runs the `TEST_SPEED`-style XY pattern (always — there is no
  `PATTERN` option) at half `MAX_ACCEL` with speeds at half and full
  `MAX_SPEED`, fits the mode-space model through the frame built from the
  kinematics' slot order and invert flags, then **streams the fitted model
  into the running endpoint** and re-captures with the feedforward active:
  with FF in the loop the drives actually track the command, so regressing
  measured torque against *commanded* kinematics loses its bias, and each
  round's fit is cleaner than the last. Rounds stop when the parameters move
  less than `TOL` between fits — measured as a **torque-weighted** change
  (`|Δm|·a + |Δb|·v + |Δc|` against the model's total feedforward at the
  excitation ceiling), so a physically negligible flap of a near-zero term
  cannot block convergence. The converged model is then re-identified once
  at full `MAX_ACCEL`: parameters that shift more than `DRIFT` there were an
  artifact of the low-accel operating point, not physics, and the command
  aborts with both parameter sets. There is no `ACCELS`×`SPEEDS` matrix —
  give the calibration envelope as `MAX_ACCEL`/`MAX_SPEED` limits (e.g.
  capped below ringing; they default to the config grid maxima). The live
  model is restored to the configured `dynamics_profile` afterwards, also
  on failure; without one the last fitted model stays live until RESTART.
  The written profile comes from the `MAX_ACCEL` verification fit and goes
  on `[ethercat_node] dynamics_profile` (node-level, coupled).

  **`TERMS` — identify rich, apply minimal.** All three terms are always
  *regressed* (the friction columns are nuisance regressors that keep the
  mass estimate unbiased — drop them from the model and friction torque
  leaks into mass), but only the `TERMS` list (default `MASS`) is
  *applied* and written; the rest are zeroed in the profile and recorded
  as `fitted_viscous`/`fitted_coulomb` provenance keys. Mass-only is the
  default because acceleration torque is the one demand the loops cannot
  supply without position error, while with `velocity_ff` on the drive the
  speed-loop integrator already covers viscous and coulomb torque at all
  but reversal transients — and an over- or mis-fitted friction FF (full
  coulomb applied through the near-zero-velocity regime the fit never
  measures) actively injects error at every stop. Enable
  `TERMS=MASS,COULOMB` (or the full triple) only when the recorded
  fitted values are consistently large *and* tracking shows a reversal
  signature that the mass-only model leaves behind. Convergence and the
  `DRIFT` gate are evaluated on the applied model only, so a wandering
  nuisance term cannot block an otherwise settled identification.
  Params: `TERMS` (MASS) `MAX_ACCEL` `MAX_SPEED` `TOL` (0.05) `DRIFT`
  (0.15) `MAX_ROUNDS` (4) `ITERATIONS` `DWELL_MS` `BOUND` `SMALL_SIZE`
  `NAME` (ident) `SERVOS` `TORQUE_NM` `INERTIA_KGM2`.
- **cartesian kinematics**: single-shot fit of one mode with an identity
  frame over the `SERVO_MEASURE_INERTIA` grid (a per-motor candidate cannot
  be streamed into a multi-drive node, so the closed-loop iteration does
  not apply). On a multi-drive (AWD) axis `DRIVE=` picks which drive the
  scalar fit describes — required there, since the capture records every
  drive. Params: as `SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2`
  `NAME` `DRIVE`.

Both paths capture into a run directory and run
`servo-cal fit --capture <step>.scap`; the profile lands in
`~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` and a new
fit never overwrites an existing profile.

#### SERVO_TUNE_DYNAMICS
Empirical closed-loop tuner for an existing dynamics profile
(`coupled_xy` only), for when the `SERVO_FIT_DYNAMICS` regression
differs run-to-run with the excitation grid. Where the retired
golden-section refine scaled one term against a fitted correlation,
the tuner is a coordinate descent that **measures** tracking error:
each round streams the trial model into the *running* endpoint (no
restart), captures one XY pattern run at `MAX_ACCEL`/`MAX_SPEED`, and
scores each mode by the **transient-window rms** of its following
error — the excursion in the short window right after each commanded
transition, where feedforward has authority before the inner servo
loop corrects it (whole-capture rms dilutes these transients ~10×).
The ferr/accel regression is still fitted and reported per round, but
only as a direction hint and diagnostic.

Terms tune one at a time in `TERMS` order, both modes per capture,
each as a 1-D line search. The mass probe's first direction follows
the **onset bias** (mean `sign(accel)·ferr` right after each accel
step — only the first excursion when torque lands carries clean
command-path sign; positive = under-fed); other terms follow their
regression coefficient's sign. A failed first probe flips once, the
step grows while the rms clears a 2-sigma deadband measured from
per-window scatter (relative change capped at 40% per probe), and the
first non-improving probe triggers one parabolic refine through the
bracket; ties go to the best measured value. Viscous/coulomb are
floored at zero (a zero-valued term probes up by a fixed floor step),
mass at 10% of its baseline.

`TERMS=LEAD` tunes the feedforward **lead time** as one shared
node-global value (seconds, continuous — the endpoint peeks the
command ring at an arbitrary future nanosecond, so it is not
quantized to whole cycles): scored on the mean of both modes'
decel-to-stop window rms (corner exits, where timing error integrates
into a direction-locked overshoot lobe), first direction from the
summed onset bias (positive = FF lands late = probe up), floored at
zero with a half-cycle floor step. The tuned lead stays live until
`RESTART`; the written dynamics TOML always carries `ff_lead_us`
(tuned when `LEAD` is in `TERMS`, else the baseline value passes
through). `TERMS=DIRECTION_SPLIT` tunes the additive signed per-pair
split on AWD with the same sign convention as the profile frame
(`slots = [first, second]`, differential `tau_first −
lambda·tau_second`).

Passes over the terms repeat until a full pass improves nothing —
there is no round budget; the search runs until it converges (kill it
if it overstays). Then the best model is written as a dynamics TOML
under `~/printer_data/config/servo_dynamics/` (never overwriting) and
left **live** — point `[ethercat_node] dynamics_profile` at it and
`RESTART` to keep it. `torque_saturated` aborts, restores the
baseline and configured lead, and writes nothing; `resonance_detected`
only warns. The baseline is `PROFILE=`, else the model left live by
the previous tune this session, else the node-level `[ethercat_node]
dynamics_profile` (per-motor profiles are not supported) — chained
tunes refine each other's output, not the configured profile.

A crashed or killed tune can be **resumed** without repeating its
captures: `RESUME=<old run dir>` replays each round from the old run's
`ferr_r<i>.json` fit instead of capturing (the coordinate descent is
deterministic, so round *i* reproduces the same trial), then picks up
with real captures at the first round the old run is missing. It
requires the identical command line (the old run's `stroke_plan` is
checked and mismatches abort) and the same live baseline model — do
not change the profile, gains, or geometry between the crash and the
resume. Params: `MAX_ACCEL` `MAX_SPEED` `STEP` (0.15) `TERMS`
(MASS,VISCOUS,COULOMB,LEAD) `NAME` (tune) `PROFILE` `RESUME` `SERVOS`
`BOUND` `SMALL_SIZE`.

#### SERVO_SET_COMPLIANCE
Writes the per-mode **belt-compliance** term `1/ω_b²` into the
dynamics profile and streams it live (no restart). Compliance is
**identification data**: it records the locked-rotor belt frequency
`f_b` per Cartesian mode and is the **pin-rotor's `ω_b` source**. The
endpoint no longer applies a command-path geometry correction from it
— that "mode B" position/velocity/torque lead has been retired.
Geometry inversion — commanding
`x + (2ζ/ω)·ẋ + (1/ω²)·ẍ` so the toolhead follows the nominal path —
now lives in the **planner's `mode_inverse` post-processor stage**
(see [`mode_inverse` config](#planner-mode_inverse-config) below), fed
by these identified numbers. Because the endpoint no longer leads the
command path, the old **double-correction hazard** (endpoint lead
stacking with a planner inversion) is gone by construction.

`X_FREQ`/`Y_FREQ` are the **locked-rotor** belt frequencies in Hz —
the frequency the carriage rings at when the rotor holds still. This
sits *above* the coupled frequency a plain `SERVO_MEASURE_RINGDOWN`
reports (there the rotor recoils on the position-loop spring in
series with the belt, which reads low). `0` clears a mode; an omitted
mode keeps its current value. On a coupled node the per-mode terms
compose through the frame (`G = F⁺·diag(c)·F`), so per-axis
frequencies map correctly onto CoreXY motors.

`PIN=XY|X|Y|0` switches the **named** mode(s) to **pin-rotor** (mode A)
and touches nothing else: `PIN=X` then `PIN=Y` compose without
unpinning each other. To unpin one mode, clear all pins with `PIN=0`
and re-pin what should remain (e.g. `PIN=0`, then `PIN=Y …`). In mode
A the endpoint holds the rotor on the planner path and cancels the
belt reaction with a predictive torque, so the toolhead rings at the
locked-rotor `f_b` where a standard input shaper applies. Pin needs
the mode's compliance as its frequency source, so set `X_FREQ`/`Y_FREQ`
in the same call (or apply it to an existing profile). Each pinned mode
also needs its FRF **peak** frequency — `X_PEAK`/`Y_PEAK` in Hz,
reported per mode by `SERVO_MEASURE_COMPLIANCE`. With the mode's notch
`f_b` and peak `f_peak` the per-mode load fraction is `1 − (f_b/f_peak)²`
and the pinned inertia is `pin_mass = mass·(1 − (f_b/f_peak)²)`. This
replaces the old `RATIO`/C00.06 source: C00.06 is a per-drive
gain-scheduling number, not per-mode physics, whereas the IV FRF's
peak/notch ratio recovers the open-loop plant and gives the load
fraction per mode. `X_ZETA`/`Y_ZETA` (default `0.02`) set the per-mode
belt damping ratio for the predictor decay — `ZETA` is the shared
fallback for a pinned mode that omits its own — and `PIN_LEAD_US`
(default `0`) the pin torque's phase lead in microseconds; because the
pin term lives at `f_b`, the lead is tuned by minimizing the mode's
line in the rotor following-error PSD (or the pin residual telemetry).

The pin-rotor path (A) holds the rotor and makes the correction
*measurable at the rotor encoder* — the belt reaction is cancelled at
the source and what remains rings at `f_b`, which a shaper then
handles. Toolhead-follows-the-planner behaviour below `f_b` is the job
of the planner's `mode_inverse` stage, not the endpoint. The pin and
`mode_inverse` are complementary: the pin damps residual excitation the
command didn't cause (cogging, reversals, model error), while
`mode_inverse` inverts the belt geometry for commanded motion.

Baseline resolution matches `SERVO_TUNE_DYNAMICS` (`PROFILE=`, else
the live-tuned model, else the configured node profile); the result
is written as a new timestamped v7 TOML (v8 when a `PIN` mode is set;
never overwriting) and left live until `RESTART`. Requires the matching
kalico build on both sides (profile / wire schema v7, v8 for pin).
Params: `X_FREQ` `Y_FREQ` (Hz, ≥ 20; 0 disables) `PIN` (0)
`X_PEAK` `Y_PEAK` (Hz, FRF peak, required per pinned mode) `X_ZETA`
`Y_ZETA` (per-mode damping) `ZETA` (0.02, shared fallback)
`PIN_LEAD_US` (0) `NAME` (compliance) `PROFILE` `SERVOS`.

#### Planner mode_inverse config
Geometry inversion — following the nominal path by commanding
`x + (2ζ/ω)·ẋ + (1/ω²)·ẍ` — is a **planner** post-processor stage
([`trajectory::algos::ModeInverse`]), not an endpoint correction. Enable
it per axis with a `[post_processor]` section of `type: mode_inverse`
and reference it from the axis, preceded by a short smoothing kernel
(the inverse amplifies high frequencies via the `ẍ` term, so bandlimit
its input):

```
[post_processor slew]
type: smooth_bell
smooth_time: 0.0015

[post_processor belt_x]
type: mode_inverse
frequency_hz: 131.0
damping_ratio: 0.05

[axis x]
post_processors: slew, belt_x
```

`frequency_hz` is the measured locked-rotor belt notch `f_b`
(`SERVO_MEASURE_COMPLIANCE`) and `damping_ratio` is the mode's belt
damping ratio ζ (the fine-ladder winner from `SERVO_TUNE_PIN`).
`SERVO_MEASURE_COMPLIANCE` and `SERVO_TUNE_PIN` both print a
ready-to-paste snippet per mode. Runtime-tunable via
`SET_POST_PROCESSOR`; kernel and inversion are both LTI so config order
does not change the math. Adding zero endpoint lead alongside the
planner stage means there is no double-correction hazard.

#### SERVO_MEASURE_COMPLIANCE
Measures the locked-rotor belt frequency `f_b` per Cartesian mode —
the exact number `SERVO_SET_COMPLIANCE` wants — with the machine at
standstill. For each selected mode it runs the engine's swept
**position buzz** in the mode's frame pattern (in-phase for X,
anti-phase for Y on CoreXY, invert signs unfolded automatically) and
captures per-cycle command, encoder position, and measured torque
(6077h). The analysis is an **instrumental-variable FRF** from
measured torque to rotor position with the commanded buzz as the
instrument (immune to the closed-loop bias a direct estimate picks
up): its anti-resonance notch is exactly `sqrt(k_belt/m_load)/2π` —
at that frequency the load is a perfectly tuned absorber and no
applied torque can move the rotor. Plant zeros are invariant under
feedback, so the position loop fighting the excitation doesn't shift
the notch; the loop *is* the torque generator. `f_b` lands above the
familiar coupled ringdown frequency and below the plant's two-mass
peak, which is reported alongside as a sanity anchor.

Re-measuring with the machine already tuned stays honest: the endpoint
applies no command-path lead, and a live pin (A) cannot bias the notch
— the FRF is measured-torque →
position, and plant zeros don't care who generated the torque. Pinned
modes do run their predictor through buzz cycles, so accelerometer
resonance tests (`TEST_RESONANCES`) measure the *pinned* machine —
tune the input shaper from those with the pin in its print-time state.
Keep the two roles distinct: `SERVO_MEASURE_COMPLIANCE` is the
*identification* tool (it recovers `f_b`/`f_peak` and changes nothing),
while `TEST_RESONANCES` with the pin active is the *print-facing
verification* that the shaped, pinned machine actually rings where the
model predicts.

The estimator is validated in CI against a simulated closed-loop
two-mass plant (`servo-ident/tests/compliance_frf.rs`): it recovers
the analytic `f_b` within 2 Hz and refuses to be dragged onto the
coupled peak. Quality gates surface as step flags:
`compliance_notch_shallow` (< 6 dB — raise `AMPLITUDE` or narrow the
band), `compliance_flanks_incoherent`, and
`compliance_peak_below_notch` (model violation — don't apply).

Measurement only — it changes nothing on the drives. The verdict
carries `f_b`, `f_peak` and the implied compliance per mode, and the
command prints the ready-to-run
`SERVO_SET_COMPLIANCE X_FREQ=… X_PEAK=… Y_FREQ=… Y_PEAK=…` line (the
peaks make it pin-complete, with the persistence reminder). It also
prints a ready-to-paste planner
[`mode_inverse` snippet](#planner-mode_inverse-config) per mode
(`frequency_hz=f_b`); `damping_ratio` is a placeholder here — the belt
ζ comes from `SERVO_SWEEP_PIN`/`SERVO_TUNE_PIN`. When any
step is flagged it prints a
re-measure warning instead of a recommendation. Params: `MODE=XY|X|Y`
`FREQ_START` (60) `FREQ_END` (320) `HZ_PER_SEC` (1) `DURATION`
`AMPLITUDE` (0.02 mm; config `compliance_amplitude`) `RAMP` `DWELL_MS` `NAME` (compliance).
A live pin is cleared for the sweep's duration and restored afterwards -
identification must measure the raw plant (an active pin cancels torque
exactly around f_b and biases the notch estimate).

#### SERVO_SWEEP_PIN
Staircase-tunes one pin-rotor parameter (`ZETA` or `LEAD`, i.e.
`PIN_LEAD_US`) for a single already-pinned mode by dwelling the engine
buzz as a **constant tone** — `freq_start == freq_end = FREQ`, typically
the mode's notch `f_b` — in that mode's frame pattern, then re-streaming
the dynamics model live at each step. The pin runs *through* the tone:
streaming a new model mid-buzz rebuilds the endpoint's pin state, so
every step's residual demodulator restarts cleanly and settles inside
the dwell while the buzz forcing keeps integrating on the fresh damping.
The un-swept pin parameter stays at its current baseline value.

Scoring is **magnitude, not phase**. After the capture stops, the host
reads the settled pin-residual magnitude `|pin_res|` at the tone that the
analyzer already produces per step
(`drives[*].metrics.pin_residual_mm`, the 0.25 s low-passed
`pin_res_re/pin_res_im` phasor, so it is fully settled by the ≥ 1 s
dwell) and reports a `value → residual (µm)` table with the minimum
marked. The residual **phase** walks 0 → 180° across the notch naturally
and is *not* a tuning target — the well-damped hold is the one that
leaves the smallest settled residual magnitude at the tone, so the
minimum of the table wins. The mode's residual rides the drive block of
the same index, so the score is the max `pin_residual_mm` over the
step's captured drives.

Measurement only — nothing is left applied. The pre-sweep model is
restored at the end (also on any failure mid-sweep, the same
restore discipline `SERVO_CALIBRATE_GAINS` uses for drive params), and
the command prints the ready-to-run
`SERVO_SET_COMPLIANCE PIN=… …_PEAK=… …_ZETA=… PIN_LEAD_US=…` line with the
winning value substituted (measure prints, [`SERVO_SET_COMPLIANCE`](#servo_set_compliance)
applies; the peak is reconstructed from the baseline pin so the line is
complete, and the un-swept parameter is carried through unchanged). If
the capture carries no pin channels or every step reads ~0 it errors —
the swept mode must be actively pinned (`pin_mass > 0`; pin it first with
`SERVO_SET_COMPLIANCE PIN=`) and the kalico endpoint build must be
current. Params: `MODE=X|Y` `FREQ` (Hz) `PARAM` (`ZETA`|`LEAD`, default
`ZETA`) `VALUES` (comma list, one or more, each validated by the
`SERVO_SET_COMPLIANCE` `ZETA`/`PIN_LEAD_US` rules) `DWELL` (s, default 3,
min 1) `AMPLITUDE` (mm, 0.01; config `pin_sweep_amplitude`) `NAME` (pin_sweep) `ACCEL_CHIP` `PROFILE`.

**Toolhead accelerometer scoring (optional).** With `ACCEL_CHIP=` (or the
`[servo_calibration] accel_chip` config default; omit both to leave it off)
each step also runs an accelerometer capture over the same scored dwell
window and reports an extra `mm/s²` column: the single-bin accel amplitude
at the tone frequency (a direct DFT bin over the settled tail, windowed the
same way as the pin-residual scorer so the two columns are comparable), the
three axes combined as vector magnitude. The pin-residual verdict is
unchanged — it still picks the applied value — but the command additionally
prints the accel-minimum step and, when it disagrees with the residual
minimum, says so explicitly. The residual is what the drive *thinks* it left
behind; the accelerometer measures the real toolhead, so this scores the
physical spike directly. Suggested use: set `FREQ` to the old coupled peak
(or the mode's `f_b`) and pick the `ZETA`/`LEAD` that minimizes the measured
toolhead accel there. Steps whose capture yields no samples report `n/a`,
never a fake zero (the same honesty rule as the residual column).

#### SERVO_COMPARE_PIN
Sweeps one pin-rotor parameter (`ZETA` or `LEAD`) across a list of values
and compares the **toolhead accelerometer response curve** each value
produces, so a bench operator can overlay them and see which pin setting
flattens the resonance. Unlike [`SERVO_SWEEP_PIN`](#servo_sweep_pin) — which
dwells a single constant tone and scores one settled residual per value —
this runs a full **swept-sine buzz (chirp)** `FREQ_START → FREQ_END` in the
selected mode's frame pattern once per value, re-streaming the dynamics
model live before each sweep (only the swept `PARAM` changes; the other pin
parameter keeps its baseline value) and capturing the accelerometer over the
whole sweep window.

Each capture is reduced to an accel-vs-frequency curve. The chirp is linear,
so sample time maps to instantaneous frequency
(`f = FREQ_START + HZ_PER_SEC·t`); samples fall into ~1 Hz bins and each
bin's mean 3-axis vector magnitude is the raw accel (`accel_mm_s2`). Because
the buzz holds **constant displacement**, the raw accel grows like `f²`, so a
normalized `response_ratio = accel / ((2π·f)²·amplitude_mm)` is stored
alongside the raw column — that ratio divides out the geometric `f²` growth
and leaves the actual mechanical transfer shape (peaks at the resonances).
Per value the command reports the peak-response frequency and its ratio.

Results are written to a comparison manifest at
`<captures_root>/pin_compare/<NAME>/manifest.json`
(`{name, created_utc, mode, param, freq_start, freq_end, baseline_profile,
sweeps:[{value, hz_per_sec, amplitude_mm, curve_hz, accel_mm_s2,
response_ratio}]}`). Re-invoking with the same `NAME` **appends** its sweeps
to the existing manifest (build a comparison incrementally across runs); a
`mode`/`param` mismatch on append errors rather than mixing unlike curves.
The dashboard reads it over `GET /api/pin-compare` (list) and
`GET /api/pin-compare/<name>` (full manifest) to overlay the curves.

Measurement only — the pre-sweep model is restored at the end (also on any
failure mid-sweep), and a failed run persists nothing. Params: `MODE=X|Y`
(required, exactly one mode) `PARAM=ZETA|LEAD` (required) `VALUES` (comma
list, nonempty, each validated by the `SERVO_SET_COMPLIANCE`
`ZETA`/`PIN_LEAD_US` rules) `FREQ_START` `FREQ_END` (Hz, required,
hard-limit validated) `HZ_PER_SEC` (default 5.0) `AMPLITUDE` (mm; config
`compliance_amplitude`) `RAMP` `DWELL` (s between sweeps, default 3)
`ACCEL_CHIP` (**required** — pass it or set `[servo_calibration] accel_chip`;
the comparison is the accelerometer) `NAME` (default `compare`) `PROFILE`.

#### SERVO_TUNE_PIN
The full measured pin-rotor tuning campaign, chaining the identification
and staircase primitives into one command so a bench operator gets a
ready-to-keep profile in a single run. For each mode in `MODES` (XY|X|Y):

1. **Identify** — unless `X_FREQ`/`X_PEAK` (resp. `Y_FREQ`/`Y_PEAK`) are
   supplied, it runs the [`SERVO_MEASURE_COMPLIANCE`](#servo_measure_compliance)
   machinery to get the locked-rotor notch `f_b` and the FRF peak
   `f_peak` for that mode; pass both override params to skip the (slow)
   measurement for a mode whose numbers you already trust.
2. **Pin** — applies the same math as
   [`SERVO_SET_COMPLIANCE`](#servo_set_compliance) `PIN=`
   (`pin_mass = mass·(1 − (f_b/f_peak)²)`), seeding the mode's damping
   at the first `ZETA_COARSE` value.
3. **Coarse `ZETA` staircase** — dwells a constant tone at `f_b` and
   steps `ZETA_COARSE` via the [`SERVO_SWEEP_PIN`](#servo_sweep_pin)
   machinery, picking the settled pin-residual minimum.
4. **Fine `ZETA` staircase** — 5 log-spaced values spanning
   `winner/1.6 … winner·1.6` refine it; the fine winner is applied to
   that mode's `pin_zeta`.

After every mode a **single `LEAD` staircase** (`LEAD_VALUES`) runs on
the **lowest-frequency tuned mode** and its winner is applied globally:
`pin_lead_us` is a whole-model scalar, and the slowest mode advances the
most degrees per microsecond, so it resolves the phase lead the finest.

The tuned model is written as a fresh timestamped profile (the same
writer [`SERVO_SET_COMPLIANCE`](#servo_set_compliance) uses) and left
live until `RESTART`; any failure restores the pre-tune model and
reports the partial results. The summary prints a per-mode table (`f_b`,
`f_peak`, `zeta`, residual µm), the lead and its residual, and the
ready-to-run `SERVO_SET_COMPLIANCE … X_ZETA=… Y_ZETA=… PIN_LEAD_US=…`
line (per-mode `X_ZETA`/`Y_ZETA` spelling) for the pin, a ready-to-paste
planner [`mode_inverse` snippet](#planner-mode_inverse-config) per mode
(`frequency_hz=f_b`, `damping_ratio=` the fine-ladder belt ζ), plus the
reminder to point
`[ethercat_node] dynamics_profile` at the written TOML to keep it. Params:
`MODES` (XY|X|Y) `DWELL` (s, 3) `AMPLITUDE` (mm, ladder tone, 0.01; config `pin_sweep_amplitude`) `MEASURE_AMPLITUDE` (mm, identification sweep, 0.02; config `compliance_amplitude`) `LEAD_VALUES`
(`0,150,300,450,600`) `ZETA_COARSE` (`0.02,0.035,0.05,0.08,0.12,0.2,0.3`)
`X_FREQ` `Y_FREQ` `X_PEAK` `Y_PEAK` (Hz, skip a mode's measurement)
`NAME` (pin_tune) `PROFILE`.

#### SERVO_CALIBRATE_INERTIA_RATIO
Step 2 of tuning: identify the load inertia and print the recommended C00.06.
`TORQUE_NM` and `INERTIA_KGM2` are **required** (config or param). On
`coupled_xy` kinematics this runs the X+Y grid over every belt drive, fits the
per-mode masses, and prints C00.06 for both directions (per drive on AWD);
the drive takes one scalar, so start from the light-direction number and
confirm with `SERVO_SWEEP_INERTIA` (both motors must be the same model). On
cartesian kinematics it fits the single axis named by `AXIS`. Params: as
`SERVO_MEASURE_INERTIA` plus `TORQUE_NM` `INERTIA_KGM2` `NAME` (inertia).
Apply the printed number with `SERVO_SET_INERTIA_RATIO`.

## Drive-parameter / gain commands

`SERVO` selects the drive; it defaults to the sole configured servo when
`servos` names exactly one, otherwise it is required.

#### SERVO_SHOW_TUNING
Reads back tuning mode (C00.04), stiffness level (C00.05), load inertia ratio
(C00.06), gain set 1 (C01.00–02), and the velocity/torque feedforward params
(C01.13–18). Param: `SERVO`.

#### SERVO_SET_INERTIA_RATIO
Writes C00.06 load inertia ratio in percent. Params: `RATIO` (0..12000) `SERVO`.

#### SERVO_APPLY_GAINS
Switches the drive to manual tuning (C00.04=0), writes gain set 1, and prints
the readback. `POS_GAIN` is 0.1 rad/s, `SPEED_GAIN` 0.1 Hz, `INTEGRAL` 0.01 ms;
defaults are the factory Low preset. `TORQUE_FILTER` (C01.18 torque
feedforward filter cutoff, Hz, 5–16000) is only written when given. Params:
`POS_GAIN` (400) `SPEED_GAIN` (250) `INTEGRAL` (3184) `TORQUE_FILTER` `SERVO`.

#### SERVO_CALIBRATE_GAINS
Sweep of exactly one drive gain, shaper-calibrate style: give one of
`POS_GAINS=` (0.1 rad/s units), `SPEED_GAINS=` (0.1 Hz units), `INTEGRALS=`
(0.01 ms units) or `TORQUE_FILTERS=` (C01.18 torque feedforward filter cutoff,
Hz) as a comma list — the other params stay at their current
drive values, so each one is tuned individually (the swept drives must agree
on their current gains, else a command error tells you to align them first).
It records
one capture per step into the run directory, then `servo-cal analyze` writes
`results.json` whose verdict names the highest gain step without resonance or a
torque rail. Always **restores the gains that were active before the sweep**
when it finishes — also on failure — so the machine is never left on a tested
value by accident; keeping a result is always an explicit act (`APPLY=1` or
`SERVO_APPLY_GAINS`). With an accelerometer
(`accel_chip` config option or `ACCEL_CHIP=`) each step also records vibration
data (`step_<name>_accel.csv` next to the `.scap`). `APPLY=1` (default 0,
report-only) writes the verdict's recommended gains *after* the restore,
reads them back (a mismatch is a command error, nothing left half-applied),
and runs one `SERVO_MEASURE_TRACKING` to report before/after following-error
peak and overshoot; a null verdict (every step flagged) makes `APPLY=1` a
command error naming the reason instead of writing anything. `SERVO=` (comma
list) restricts the sweep to a subset of the axis servos; adding
`BASE_GAIN=` then pins the swept gain on every non-swept axis servo at that
value (their other gains untouched, recorded as `base_gains` in the manifest)
for the whole
sweep — the asymmetric-gain experiment: hold one belt pair soft while sweeping
the other pair higher; those servos are restored to their prior gains too.
Params:
`POS_GAINS` `SPEED_GAINS` (500,650,800,1000 when none given) `INTEGRALS`
`TORQUE_FILTERS` `AXIS` (X) `START` `END`
`SPEED` (100) `ACCEL` (3000) `ITERATIONS` (2) `DWELL_MS` `TAG` (cal)
`ACCEL_CHIP` `APPLY` `SERVO` `BASE_GAIN`.

#### SERVO_SWEEP_INERTIA
Empirical inertia sweep: apply the tuned gains first, then this writes each
C00.06 ratio in `RATIOS`, records one capture per step, and runs
`servo-cal analyze` (`results.json` reports per-step metrics; no automated pick
— read the overshoot trend to choose the ratio). Reverts to the lowest ratio
afterwards. Because there is no automated pick, `APPLY=1` always errors here
(nothing to apply) — choose a ratio from the report and write it with
`SERVO_SET_INERTIA_RATIO`. Params: `RATIOS`
(40,70,100,130) `AXIS` (X) `START` `END` `SPEED` (100) `ACCEL` (3000)
`ITERATIONS` (2) `DWELL_MS` `TAG` (inertia) `APPLY` `SERVO`.

#### SERVO_AUTOTUNE
Packaged tuning sequence, the manual order above run as one state machine:
baseline `SERVO_MEASURE_TRACKING` → `SERVO_CALIBRATE_INERTIA_RATIO` (identify
only) → apply the recommended C00.06 (`SERVO_SET_INERTIA_RATIO`-equivalent) →
coarse gains (`SERVO_APPLY_GAINS` factory defaults) → `SERVO_CALIBRATE_GAINS`
sweep (apply the winner) → `SERVO_FIT_DYNAMICS` → a final `SERVO_MEASURE_TRACKING` against the
baseline. Each stage transition is logged
(`calibration.autotune_stage`: `stage`, `run_dir`, `outcome`) so the dashboard
can show the sequence as it runs.

`APPLY` defaults to 0: a dry run that still measures the baseline and
identifies the inertia ratio (both read-only), then walks every remaining
stage reporting what it *would* write instead of touching the drive.
`APPLY=1` performs every stage for real and aborts loudly — naming the stage
and run directory — on any of:

- a `torque_saturated` or `resonance_detected` flag on the chosen step of any
  sweep stage (checked whether or not that stage's write ends up gated by
  `APPLY`, since continuing past a flagged step is unsafe regardless);
- a null recommendation (no clean step) on a stage that needs to promote one;
- the final verification's following-error peak regressing more than 20%
  against the baseline.

`APPLY=1` requires `rated_torque_nm`/`rotor_inertia_kgm2` (config or
`TORQUE_NM=`/`INERTIA_KGM2=`) up front — it errors before the first stroke
rather than mid-sequence. The C00.06 recommendation is recovered from the
`servo-cal fit` console output (the same "recommended C00.06 (light
direction): N%" line `SERVO_CALIBRATE_INERTIA_RATIO` already prints) — there
is no separate machine-readable field for it, since `fit` writes a profile
TOML, not a `results.json`. `SERVO_FIT_DYNAMICS` never edits `printer.cfg`;
it prints the `dynamics_profile` line to paste, exactly as it does standalone.
A successful `APPLY=1` run never persists anything to a tuning profile by
itself — run `SERVO_SAVE_TUNING SERVO=... NAME=...` afterwards to keep it.
Params: `AXIS` (X) `APPLY` (0) `TORQUE_NM` `INERTIA_KGM2` `SPEED_GAINS`
`DWELL_MS`.

## Command → output

Every experiment command writes a run directory
`<captures_root>/<tag>_<YYYYmmdd_HHMMSS>/` holding `manifest.json`, one
`step_<name>.scap` per step, optional `step_<name>_accel.csv` recordings, and
(for the analyze commands) `results.json` + `plot_series.json`. The command
prints a one-line verdict plus the run-directory path; the metrics table
streams from `servo-cal` in the interim before the dashboard (Part 3) lands.
Schemas: [servo-cal-contracts.md](servo-cal-contracts.md).

| Command | Invokes | Output |
|---|---|---|
| `SERVO_MEASURE_TRACKING` | `servo-cal analyze` | run dir + `results.json` (per-motor + combined tracking metrics; records every motor driving the axis — both lanes on CoreXY) |
| `SERVO_MEASURE_DIFFERENTIAL` | `servo-cal analyze` | run dir + `results.json` (differential FRF modes: frequency, peak gain, damping, coherence; dashboard renders the FRF) |
| `SERVO_DIFF_DAMPER` | — | no run dir; reconfigures the running endpoint |
| `SERVO_DIFF_TRIM` | — | no run dir; reconfigures the running endpoint |
| `SERVO_MEASURE_STRAIN_MAP` | dashboard `/api/runs/<name>/strain` | run dir with one capture per raster line; charted by the dashboard's strain tab |
| `SERVO_MEASURE_STRAIN_RESPONSE` | in-klippy fit | run dir with one capture per offset step; reports + stores the rolling stiffness matrix |
| `SERVO_STRAIN_COMP_TUNE` | in-klippy loop | run dir with one capture per iteration; converges the matrix, leaves the tuned map written + enabled |
| `SERVO_CALIBRATE_GAINS` | `servo-cal analyze` | run dir + `results.json` verdict (highest clean gain step); `APPLY=1` also writes + verifies |
| `SERVO_SWEEP_INERTIA` | `servo-cal analyze` | run dir + `results.json` (no automated pick, so `APPLY=1` always errors) |
| `SERVO_SWEEP_ACCEL` | `servo-cal analyze` | run dir + `results.json` verdict (max non-railing accel); `APPLY=1` verifies at the recommended accel (no SDO write) |
| `SERVO_SWEEP_PIN` | `servo-cal analyze` | run dir + `results.json` (per-step settled pin-residual magnitude; prints the `value → µm` table + winning `SERVO_SET_COMPLIANCE` line; nothing applied) |
| `SERVO_COMPARE_PIN` | host-side chirp reduction (no `servo-cal`) | `<captures_root>/pin_compare/<NAME>/manifest.json` — one accel-vs-frequency curve (raw `accel_mm_s2` + normalized `response_ratio`) per swept value; same `NAME` appends; dashboard overlays via `/api/pin-compare`; nothing applied |
| `SERVO_TUNE_PIN` | `servo-cal analyze` (per staircase) | run dir(s) + tuned `dynamics_<name>_<stamp>.toml` (per-mode coarse→fine `ZETA` + shared `LEAD` staircases; model stays live until RESTART; restores pre-tune model on failure) |
| `SERVO_FIT_DYNAMICS`, `SERVO_CALIBRATE_INERTIA_RATIO` | `servo-cal fit` | run dir + `~/printer_data/config/servo_dynamics/dynamics_<name>_<stamp>.toml` + C00.06 |
| `SERVO_TUNE_DYNAMICS` | `servo-cal fit --response ferr` (per capture) | run dir + tuned `dynamics_<name>_<stamp>.toml` when a pass beats the baseline (search is host-side; tuned model stays live until RESTART) |
| `SERVO_MEASURE_INERTIA` | — | run dir + `.scap` capture only (the building block behind the fit commands) |
| `SERVO_AUTOTUNE` | all of the above, in sequence | one run dir per stage; `APPLY=0` (default) is a dry rehearsal, `APPLY=1` runs and applies for real |

## The manual capture analyzer

The kalico repo's `scripts/servo_capture.py` remains the standalone single-file `.scap` analyzer
for ad-hoc inspection (`--help` for the full option list): following-error,
overshoot/settling, torque-saturation metrics per drive; `--fft` prints
resonance peaks, `--plot` opens a time-series dashboard, `--png` saves one
headless, `--combine-corexy A[:s],B[:s]` with `--axis` renders the CoreXY
dashboard; `--drive` restricts to one drive in a multi-drive capture, `--csv`
exports samples. The four gain/inertia/refine/accel sweep-report scripts and
the fit-dynamics wrapper script were deleted — their metrics and verdict logic
moved into `servo-cal`.

## Capture retention / disk budget

Run directories accumulate under `<captures_root>`
(`~/printer_data/logs/servo_captures` on the bench). Captures are now
zstd-compressed **at write time** — kalico's capture writer streams each
`step_<name>.scap.zst` through a zstd encoder as it is recorded, so no bulk
back-compression pass runs over fresh runs (the old compress-later batch job
saturated the memory bus and tripped RT frame faults). `scripts/servo-capture-prune`
keeps the tree within a byte budget and back-compresses only any **legacy raw
`*.scap`** left from before inline compression; `install.sh` installs it as
`servo-capture-prune.service` (oneshot, `Nice=19` + `IOSchedulingClass=idle`)
driven by `servo-capture-prune.timer` (`OnCalendar=daily`, `OnBootSec=15min`,
`Persistent=true`).

Policy, applied in order every run:

1. **Back-compress legacy raw payloads.** New captures are already
   `*.scap.zst`, so this step only touches legacy raw `*.scap` (already-`.zst`
   files are skipped). For any run dir whose newest file is older than
   `--cold-age-hours` (default **48h**), each remaining raw `*.scap` is
   compressed in place with `zstd -6 --rm` (bus-throttled; see the unit) to
   `*.scap.zst`. Small analysis artifacts (`manifest.json`, `results.json`,
   `plot_series.json`, …) are left readable so the dashboard's run list and
   plots keep working for compressed runs.
2. **Budget prune.** While the total exceeds `--budget-gib` (default **8 GiB**),
   the oldest whole run dir (ordered by newest-file mtime) is deleted,
   oldest-first. `pin_compare/<name>` dirs are last-resort: they are only
   touched after every ordinary run has been exhausted.
3. **Min-keep floor.** Nothing newer than `--min-keep-hours` (default **24h**)
   is ever deleted. If the budget cannot be met without deleting a dir inside
   that window, the prune pass refuses — it deletes **nothing** and exits
   nonzero with a loud message (so the timer run is marked failed and the
   captures are left intact for you to clear space manually).

`--dry-run` prints the full plan (compressions + deletions) and modifies
nothing.

### Changing the budget

Edit the `ExecStart` line in the installed unit
(`~/servo-cal/servo-capture-prune.service`) to append the flag, e.g.
`--budget-gib 20`, then `sudo systemctl daemon-reload`. To make it permanent,
edit `service/servo-capture-prune.service` in this repo and re-run
`./install.sh`. Run it by hand any time with:

```sh
~/servo-cal/servo-capture-prune --root ~/printer_data/logs/servo_captures --dry-run
```

### Re-analyzing a compressed capture

No decompression step is needed. `servo-cal` (and the dashboard it serves)
detect zstd by the frame magic and decode `.scap.zst` transparently, so
`analyze`, `fit`, and the strain re-fit read compressed captures directly:

```sh
servo-cal analyze run_dir            # reads step_*.scap.zst in place
```

If you want a raw `.scap` for the standalone `scripts/servo_capture.py`
inspector (which still expects raw bytes), decompress a copy by hand:

```sh
zstd -d run_dir/step_foo.scap.zst    # -> run_dir/step_foo.scap
```
