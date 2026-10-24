# ORP — Open Reentry Platform

ORP is a Python **atmospheric reentry simulator** for the descent phase of flight.
It integrates a vehicle's equations of motion **forward in time** from a set of entry
conditions and a *replayed* bank-angle schedule, across multiple planetary
environments (Earth and Mars from day one).

## Quickstart

### Install

```text
pip install "orp[gui]"
```

The `[gui]` extra pulls PyQt6 and matplotlib for the desktop interface and the run
figures. The base install runs simulations and writes the data outputs
(trajectory.csv, session.yaml, provenance.txt); the figures need the `plot` or `gui`
extra, and a run without matplotlib says so in one line and still exits 0. Note
for the validation gates: `orp gates` replays digitized flight datasets that live in
`data/flights/` at the repository root and are not shipped in the package — running
the flight-replay gates requires a source checkout (clone the repo and run from its
root; an installed package says so honestly and exits with a distinct code).

### Run

Run one forward reentry (Apollo capsule, Earth, a constant 60° bank command) from the
repo root:

```text
$ python -m orp.cli run --vehicle apollo --planet earth --frame planet-relative --bank-deg 60 --out out/demo
Run complete: 4378 samples, terminated by GROUND_HIT at t=437.700 s.
Peak deceleration: 11.2043 g
Peak heat rate: 870448 W/m^2
Final state: altitude -5.5 m, velocity 75.95 m/s, latitude -1.23649 deg, longitude 9.62559 deg, flight-path angle -79.2518 deg, heading 242.1849 deg
Run provenance (weakest link): NOT_VALIDATED
Outputs written to out/demo
```

The output directory holds `trajectory.csv` (every flight-data channel), the five
standard figures, `session.yaml` (a reproducible record of the run's inputs), and
`provenance.txt`. Every run states its **weakest-link provenance** because a trajectory
is only as trustworthy as the least-validated input that produced it — quoting one
strong component would launder the weak ones. The line above says `NOT_VALIDATED`
because a hand-typed `--bank-deg` constant is an unsourced control history, and the run
says so rather than rounding up.

`orp vehicles` lists the vehicle library with per-property provenance;
`orp gates` runs the validation gates and reports their statuses exactly as the gates
state them (today: scaffolded / honest-FAIL, none validated).

The desktop interface (PyQt6) launches with either:

```text
$ orp gui
$ python -m orp.gui.main_window
```

It needs the optional GUI dependencies — install them with `pip install orp[gui]`
(without them, `orp gui` refuses with that hint while `run`, `vehicles`, and `gates`
keep working).

### Replaying a recorded bank history (`--bank-csv` / `BankSchedule.from_csv`)

`from_csv` loads a two-column CSV — time in seconds, commanded bank angle in degrees —
with an optional header row, `#` comment lines, strictly increasing times, and no blank
cells, NaNs, or gap markers (it refuses rather than repairs; angles may use either the
−180..180 or 0..360 convention, detected and recorded in the provenance notes):

```csv
time_s,bank_cmd_deg
0.0,15.0
115.5,-15.0
390.5,15.0
```

The worked example in this repo is the Artemis I commanded-bank history,
`data/flights/artemis1_bank_commanded.csv`, digitized from AAS 24-174 Figure 12(a). It
carries its dataset tag verbatim:

> DATASET TAG: MACHINE-DIGITIZED (pixel extraction from a published figure; not flight
> telemetry)

and its sign caveat verbatim:

> Sign convention: signed bank EXACTLY as plotted in Figure 12(a). The paper text
> defines no bank sign convention (pages 3-7 searched verbatim); the physical meaning
> of the sign is locked only by the Gate-3 crossrange comparison (see docs/gates/).
> Flagged for OpenReentry [APPROX-ROTATION].

That file uses a four-column flagged format (transition/occlusion samples are
deliberately blank) and is loaded by its dedicated loader,
`orp.gates.gate3_artemis_replay.load_digitized_schedule`; `from_csv` deliberately
refuses it, so a two-column hold-last extract is the way to replay it through
`--bank-csv` (see `orp/tests/test_bank_schedule_from_csv.py` for the exact recipe).

Session format 2 records the built-in aerodynamic calculator and its parameters,
including constant-coefficient provenance, so CLI sessions restore the constant
vehicle coefficients instead of falling back to the engine default. Custom
calculator strategies are refused rather than silently substituted.

Format-1 sessions do not identify the aerodynamic model and are refused. Re-export
from the original setup; changing the version number alone cannot recover the
missing model. Replay tests cover the built-in Earth/Apollo and Mars/MSL paths in
the same environment. Sessions do not serialize custom planet/vehicle/integrator
implementations or guarantee replay across code versions. This is numerical
reproducibility, not flight validation.

## Three non-negotiable principles

1. **Forward simulation only.**
   The simulator takes entry conditions plus a pre-recorded bank-angle schedule and
   integrates forward. **No function in this codebase accepts a desired landing point
   and returns a bank schedule.** ORP models what *did* / *would* happen given a control
   history; it never solves the inverse guidance/targeting problem. This is a hard,
   permanent architectural wall — see `orp/core/bank_schedule/schedule.py` and
   `orp/core/simulation/engine.py`. If a proposed feature is unsure whether it crosses
   this line, it does: raise, don't compute.

2. **Provenance on everything.**
   Every vehicle property and every simulation output carries a validation tag —
   `VERIFIED_FLIGHT`, `VERIFIED_CFD`, `ASSERTED`, or `NOT_VALIDATED` — and every vehicle
   property carries a source citation string. Provenance propagates: a trajectory is only
   as trustworthy as the weakest input that produced it. This is the product's core
   differentiator, not an afterthought. See `orp/core/provenance/`.

3. **Multi-planet.**
   The `Vehicle` + `Planet` abstraction is present from the start. A `Planet` bundles an
   atmosphere model, a gravity model, a mean radius, and a rotation rate. Earth and Mars
   ship in `orp/core/planet/registry.py`.

## Status

This is an **architectural skeleton**. The contracts (base classes, method signatures,
type hints, docstrings, the provenance system, the planet/vehicle/model wiring) are
complete. The *flight physics* — aerodynamic forces, equation-of-motion derivatives,
altitude-dependent atmosphere and gravity variation — are placeholders that return
zeros or planet reference constants, clearly marked with `# --- PHYSICS SEAM ---`, so
real implementations drop into named seams without reshaping the architecture. See
[`docs` in code]; the convention is documented in
`orp/core/__init__.py`.

## Architecture & credit

ORP's architecture deliberately mirrors **[OpenRocket](https://openrocket.info/)**, the
open-source model-rocketry simulator, adapted from ascent to the descent/reentry regime:

- an **Engine / Stepper** split (orchestrator vs. pluggable physics integrator — Strategy
  pattern),
- a mutable per-instant `SimulationStatus` vs. an immutable `SimulationConditions`
  dependency-injection container,
- a `FlightData` / `FlightDataBranch` column-store for trajectory output,
- pluggable `AtmosphericModel` / `GravityModel` / `AerodynamicCalculator` strategies, and
- a momentary `FlightConditions` input object feeding the aerodynamics.

ORP reuses these **design patterns only**. No OpenRocket source code is copied. OpenRocket
is gratefully acknowledged as architectural inspiration.

## License

ORP is licensed under the **GNU General Public License, version 3 or later
(GPL-3.0-or-later)**. OpenRocket is also GPL-licensed; mirroring its design patterns in a
GPL project is consistent with that license. See the license headers in source files.

Copyright © Charles W. Dowd Jr.

## Layout

```
orp/
  core/
    simulation/    engine, stepper (RK4), status, conditions, flight_data
    vehicles/      EntryVehicle base + YAML library loader
    aerodynamics/  AerodynamicCalculator interface, Modified Newtonian, FlightConditions
    atmosphere/    AtmosphericModel interface, Earth ISA, Mars
    gravity/       GravityModel interface, Earth WGS84, Mars
    bank_schedule/ forward-only bank-angle replay
    provenance/    ValidationLevel, ProvenanceTag, TaggedValue
    planet/        Planet bundle + EARTH/MARS registry
  data/vehicles/   apollo.yaml, msl.yaml
  gui/             (placeholder)
  tests/           smoke test over the import graph
```
