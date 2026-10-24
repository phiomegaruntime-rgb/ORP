# ORP — Open Reentry Platform
# Copyright (C) Charles W. Dowd Jr.
# SPDX-License-Identifier: GPL-3.0-or-later
"""Reproducible-run sessions: :func:`save_session` and :func:`load_session`.

A *session* is a single human-readable YAML file that captures everything needed to
reproduce one forward reentry run exactly:

* the vehicle (name + a SHA-256 content hash of its YAML definition file),
* the planet (name),
* the full entry state, every field explicit, with a mandatory ``frame`` tag,
* the bank schedule's *source* (a constant value, explicit arrays, or a CSV path) plus a
  SHA-256 hash of the schedule's numerical content (independent of how it was sourced),
* the built-in aerodynamic model and its parameters (custom strategies refused),
* the integrator settings (time step, max simulation time, and the two termination
  bounds),
* the provenance level of every component, recorded at save time.

FORWARD-ONLY WALL
=================
A session is a *recording of inputs to a forward run*, nothing more. It stores a vehicle,
a planet, an entry state, and a **replayed** bank schedule — exactly the things
:class:`~orp.core.simulation.conditions.SimulationConditions` already accepts. There is no
field anywhere in the session schema whose semantics are a desired landing point or any
other terminal target, and there is a test
(:class:`~orp.tests.test_session.TestForwardOnlyWall`) that inspects the schema
programmatically to keep it that way. Bank schedules are inputs; crossrange/downrange are
outputs of replaying them; this module never turns a target into a control.

REFUSAL OVER REPAIR
===================
Loading is strict. Any drift between what was saved and what is on disk now is an error,
never something to paper over:

* a SHA-256 mismatch on the vehicle YAML, or on the schedule's numerical content, raises
  :class:`SessionIntegrityError` naming exactly which component changed and showing both
  the saved and the recomputed hash;
* a missing ``frame`` tag, or an unrecognised one, raises :class:`SessionFormatError`;
* a ``frame`` of ``inertial`` is refused with its own message: session files record the
  planet-relative state the engine actually consumed, inertial entry states are converted
  to planet-relative at the CLI boundary via :mod:`orp.core.frames`, and the path is
  convert first, then save. Loading never performs that conversion itself;
* an unknown vehicle or planet name raises (``FileNotFoundError`` / ``KeyError``,
  surfaced by the underlying registries).

Nothing is silently substituted, regenerated, or upgraded. Saving is *pure recording*:
:func:`save_session` reads provenance levels as they already are and writes them down; it
never mutates, re-tags, or upgrades any provenance level on the objects it is handed.
"""

from __future__ import annotations

import hashlib
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from orp.core.bank_schedule.schedule import BankSchedule
from orp.core.planet import registry as planet_registry
from orp.core.planet.planet import Planet
from orp.core.provenance.tags import ProvenanceTag, ValidationLevel
from orp.core.simulation.conditions import SimulationConditions
from orp.core.vehicles.base import EntryVehicle
from orp.core.vehicles.library import VehicleLibrary

__all__ = [
    "SessionError",
    "SessionFormatError",
    "SessionIntegrityError",
    "VALID_FRAMES",
    "save_session",
    "load_session",
]

#: The session-file format version (bump on incompatible schema changes).
SESSION_FORMAT_VERSION = 2

#: The only two legal values for the entry-state ``frame`` tag.
FRAME_INERTIAL = "inertial"
FRAME_PLANET_RELATIVE = "planet-relative"
VALID_FRAMES: tuple[str, ...] = (FRAME_INERTIAL, FRAME_PLANET_RELATIVE)

#: The three legal bank-schedule source kinds.
SOURCE_CONSTANT = "constant"
SOURCE_ARRAYS = "arrays"
SOURCE_CSV = "csv"
_VALID_SOURCES: tuple[str, ...] = (SOURCE_CONSTANT, SOURCE_ARRAYS, SOURCE_CSV)


# ---------------------------------------------------------------------------
# Errors
# ---------------------------------------------------------------------------


class SessionError(Exception):
    """Base class for all session save/load failures."""


class SessionFormatError(SessionError):
    """The session file is structurally invalid (missing/!invalid keys, bad frame tag)."""


class SessionIntegrityError(SessionError):
    """A content hash recorded at save time no longer matches what is on disk now.

    The message names exactly which component changed and shows both the saved hash and the
    hash recomputed at load time.
    """


# ---------------------------------------------------------------------------
# Hashing helpers
# ---------------------------------------------------------------------------


def _sha256_bytes(data: bytes) -> str:
    """Return the lowercase hex SHA-256 digest of ``data``."""
    return hashlib.sha256(data).hexdigest()


def _sha256_file(path: Path) -> str:
    """Return the SHA-256 of a file's exact bytes (no normalisation)."""
    return _sha256_bytes(path.read_bytes())


def _schedule_content_hash(schedule: BankSchedule) -> str:
    """SHA-256 of a schedule's numerical content, independent of its source kind.

    The content is the (times, bank_angles) numbers themselves, serialised as IEEE-754
    big-endian doubles in a fixed, source-independent layout: the sample count, then every
    time, then every bank angle. Two schedules with the same numbers hash the same whether
    one came from a CSV and the other from explicit arrays — so tampering with the *source*
    (a CSV cell, a constant value) changes this hash and is caught on load.
    """
    times = schedule.times
    angles = schedule.bank_angles
    # Length prefix prevents (times || angles) ambiguity across differing splits.
    payload = struct.pack(">Q", len(times))
    payload += struct.pack(f">{len(times)}d", *times)
    payload += struct.pack(f">{len(angles)}d", *angles)
    return _sha256_bytes(payload)


# ---------------------------------------------------------------------------
# Schedule source description (capture) and reconstruction (replay)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _ScheduleSource:
    """A described bank-schedule source: how to write it down and how to rebuild it.

    Exactly one of the source kinds is populated. ``kind`` is one of
    :data:`SOURCE_CONSTANT`, :data:`SOURCE_ARRAYS`, :data:`SOURCE_CSV`.
    """

    kind: str
    # SOURCE_CONSTANT
    constant_rad: float | None = None
    constant_duration: float | None = None
    # SOURCE_ARRAYS
    times: tuple[float, ...] | None = None
    bank_angles_rad: tuple[float, ...] | None = None
    # SOURCE_CSV
    csv_path: str | None = None


def _describe_constant(
    bank_angle_rad: float, *, duration: float = 0.0
) -> _ScheduleSource:
    """Describe a constant-bank schedule source (value in radians)."""
    return _ScheduleSource(
        kind=SOURCE_CONSTANT,
        constant_rad=float(bank_angle_rad),
        constant_duration=float(duration),
    )


def _describe_arrays(times: Any, bank_angles_rad: Any) -> _ScheduleSource:
    """Describe an explicit (times, bank_angles) array source (angles in radians)."""
    return _ScheduleSource(
        kind=SOURCE_ARRAYS,
        times=tuple(float(t) for t in times),
        bank_angles_rad=tuple(float(a) for a in bank_angles_rad),
    )


def _describe_csv(path: str | Path) -> _ScheduleSource:
    """Describe a two-column (time_s, bank_angle_deg) CSV source."""
    return _ScheduleSource(kind=SOURCE_CSV, csv_path=str(path))


def _source_to_mapping(source: _ScheduleSource) -> dict[str, Any]:
    """Serialise a :class:`_ScheduleSource` to a stable, human-readable mapping."""
    if source.kind == SOURCE_CONSTANT:
        return {
            "kind": SOURCE_CONSTANT,
            "bank_angle_rad": source.constant_rad,
            "duration_s": source.constant_duration,
        }
    if source.kind == SOURCE_ARRAYS:
        return {
            "kind": SOURCE_ARRAYS,
            "times_s": list(source.times or ()),
            "bank_angles_rad": list(source.bank_angles_rad or ()),
        }
    if source.kind == SOURCE_CSV:
        return {"kind": SOURCE_CSV, "path": source.csv_path}
    raise SessionFormatError(f"Unknown bank-schedule source kind {source.kind!r}.")


def _source_from_mapping(raw: Any) -> _ScheduleSource:
    """Parse a schedule-source mapping back into a :class:`_ScheduleSource` (strict)."""
    if not isinstance(raw, dict):
        raise SessionFormatError("bank_schedule.source must be a mapping.")
    kind = raw.get("kind")
    if kind not in _VALID_SOURCES:
        raise SessionFormatError(
            f"bank_schedule.source.kind must be one of {_VALID_SOURCES}; got {kind!r}."
        )
    if kind == SOURCE_CONSTANT:
        return _ScheduleSource(
            kind=SOURCE_CONSTANT,
            constant_rad=float(_require(raw, "bank_angle_rad", "bank_schedule.source")),
            constant_duration=float(
                _require(raw, "duration_s", "bank_schedule.source")
            ),
        )
    if kind == SOURCE_ARRAYS:
        times = _require(raw, "times_s", "bank_schedule.source")
        angles = _require(raw, "bank_angles_rad", "bank_schedule.source")
        return _ScheduleSource(
            kind=SOURCE_ARRAYS,
            times=tuple(float(t) for t in times),
            bank_angles_rad=tuple(float(a) for a in angles),
        )
    # SOURCE_CSV
    return _ScheduleSource(
        kind=SOURCE_CSV, csv_path=str(_require(raw, "path", "bank_schedule.source"))
    )


def _reconstruct_schedule(
    source: _ScheduleSource,
    *,
    provenance: ProvenanceTag,
    base_dir: Path,
) -> BankSchedule:
    """Rebuild a :class:`BankSchedule` from a described source.

    Reconstruction is *pure replay of recorded inputs*: it rebuilds the same control
    history that was saved. It never consults a target endpoint.

    Args:
        source: The described source (constant / arrays / csv).
        provenance: The provenance to attach (recorded at save time, reattached on load).
        base_dir: Directory a relative CSV ``path`` is resolved against (the session-file
            directory).
    """
    if source.kind == SOURCE_CONSTANT:
        return BankSchedule.constant(
            float(source.constant_rad or 0.0),
            duration=float(source.constant_duration or 0.0),
            provenance=provenance,
        )
    if source.kind == SOURCE_ARRAYS:
        return BankSchedule(
            list(source.times or ()),
            list(source.bank_angles_rad or ()),
            provenance=provenance,
        )
    if source.kind == SOURCE_CSV:
        csv_path = Path(source.csv_path or "")
        if not csv_path.is_absolute():
            csv_path = (base_dir / csv_path).resolve()
        return BankSchedule.from_csv(csv_path, provenance=provenance)
    raise SessionFormatError(f"Unknown bank-schedule source kind {source.kind!r}.")


# ---------------------------------------------------------------------------
# Provenance (level only) serialisation — recorded as-is, never upgraded
# ---------------------------------------------------------------------------


def _provenance_levels(conditions: SimulationConditions) -> dict[str, str]:
    """Snapshot every component's provenance *level key*, exactly as it stands now.

    Pure read. The returned mapping records the level (e.g. ``"asserted"``) of each
    contributing component plus the run's overall weakest-link level. No value here is
    derived by upgrading or re-tagging anything; it is a transcription.
    """
    return {
        "vehicle": conditions.vehicle.provenance.level.key,
        "planet": conditions.planet.provenance.level.key,
        "aerodynamics": conditions.aerodynamic_calculator.provenance.level.key,
        "bank_schedule": conditions.bank_schedule.provenance.level.key,
        "overall": conditions.provenance.level.key,
    }


def _finite_aero_number(block: dict, key: str) -> float:
    try:
        value = float(_require(block, key, "aerodynamics"))
    except (TypeError, ValueError) as error:
        raise SessionFormatError(f"Invalid aerodynamics.{key}") from error
    if not math.isfinite(value):
        raise SessionFormatError(f"aerodynamics.{key} must be finite")
    return value


def _aerodynamics_block(calculator: Any) -> dict[str, Any]:
    from orp.core.aerodynamics.constant import ConstantCoefficientCalculator
    from orp.core.aerodynamics.newtonian import ModifiedNewtonianCalculator

    # Exact types: subclasses may add behavior this schema cannot reproduce.
    if type(calculator) is ConstantCoefficientCalculator:
        block = {
            "kind": "constant",
            "drag_coefficient": calculator.drag_coefficient,
            "lift_to_drag": calculator.lift_to_drag,
            "provenance": _provenance_block(calculator.provenance),
        }
    elif type(calculator) is ModifiedNewtonianCalculator:
        block = {
            "kind": "modified_newtonian",
            "stall_angle": calculator.get_stall_angle(),
        }
    else:
        raise SessionFormatError(
            "Unsupported aerodynamic calculator; cannot record an exact replay"
        )
    _load_aerodynamics(block)  # Validate before writing any file.
    return block


def _load_aerodynamics(block: Any) -> Any:
    from orp.core.aerodynamics.constant import ConstantCoefficientCalculator
    from orp.core.aerodynamics.newtonian import ModifiedNewtonianCalculator

    if not isinstance(block, dict):
        raise SessionFormatError(
            "Missing aerodynamics model; legacy sessions cannot identify it. Re-export from the original setup."
        )
    if block.get("kind") == "constant":
        cd = _finite_aero_number(block, "drag_coefficient")
        ld = _finite_aero_number(block, "lift_to_drag")
        if cd <= 0:
            raise SessionFormatError("aerodynamics.drag_coefficient must be positive")
        return ConstantCoefficientCalculator(
            cd, ld, provenance=_provenance_from_block(block.get("provenance"))
        )
    if block.get("kind") == "modified_newtonian":
        return ModifiedNewtonianCalculator(_finite_aero_number(block, "stall_angle"))
    raise SessionFormatError(
        "Unknown aerodynamics.kind; refusing to substitute a model"
    )


# ---------------------------------------------------------------------------
# Small strict-access helpers
# ---------------------------------------------------------------------------


def _require(mapping: Any, key: str, where: str) -> Any:
    """Return ``mapping[key]`` or raise :class:`SessionFormatError` naming the path."""
    if not isinstance(mapping, dict) or key not in mapping:
        raise SessionFormatError(f"Session is missing required key {where}.{key!r}.")
    return mapping[key]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def save_session(
    path: str | Path,
    *,
    conditions: SimulationConditions,
    vehicle_name: str,
    schedule_source: _ScheduleSource,
    vehicle_data_dir: str | Path | None = None,
) -> dict[str, Any]:
    """Write a reproducible-run session YAML to ``path`` and return the captured mapping.

    Saving is **pure recording**: it reads the objects it is given and writes them down. It
    does not mutate ``conditions``, re-tag any object, or upgrade any provenance level.

    Args:
        path: Destination ``.yaml`` file.
        conditions: The forward-run setup to capture (entry state, integrator settings,
            planet, vehicle, replayed bank schedule). Its frame is taken from
            ``conditions``; ORP's entry velocity is planet-relative, so the frame recorded
            is :data:`FRAME_PLANET_RELATIVE` unless overridden by a future field.
        vehicle_name: The library name (YAML stem, e.g. ``"apollo"``) of
            ``conditions.vehicle``. Used to locate the YAML file to hash and, on load, to
            reload the vehicle. Must match a file in ``vehicle_data_dir``.
        schedule_source: A description of how ``conditions.bank_schedule`` was sourced —
            build it with :func:`source_constant`, :func:`source_arrays`, or
            :func:`source_csv`. Its numbers must reproduce the schedule's content hash.
        vehicle_data_dir: Directory containing the vehicle YAML files. Defaults to the
            bundled library directory.

    Returns:
        The exact mapping that was serialised (useful for tests/inspection).

    Raises:
        FileNotFoundError: if the vehicle YAML does not exist (so it cannot be hashed).
    """
    import yaml  # lazy: importing this module must not require PyYAML

    path = Path(path)
    library = VehicleLibrary(
        Path(vehicle_data_dir) if vehicle_data_dir is not None else None
    )
    vehicle_yaml = library.data_dir / f"{vehicle_name}.yaml"
    if not vehicle_yaml.is_file():
        available = ", ".join(library.list_available()) or "(none)"
        raise FileNotFoundError(
            f"No vehicle named {vehicle_name!r} in {library.data_dir} (available: {available})."
        )

    document = _build_document(
        conditions=conditions,
        vehicle_name=vehicle_name,
        vehicle_yaml=vehicle_yaml,
        schedule_source=schedule_source,
    )

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        # sort_keys=False preserves our deliberate, stable key ordering.
        yaml.safe_dump(document, handle, sort_keys=False, default_flow_style=False)
    return document


def load_session(
    path: str | Path,
    *,
    vehicle_data_dir: str | Path | None = None,
) -> tuple[SimulationConditions, dict[str, Any]]:
    """Load a session YAML, verify every hash, and rebuild the exact run setup.

    Refusal over repair throughout: a vehicle- or schedule-content hash mismatch, a missing
    or invalid frame tag, or an unknown vehicle/planet name all raise; nothing is silently
    substituted or regenerated.

    Args:
        path: The session ``.yaml`` to load.
        vehicle_data_dir: Directory containing the vehicle YAML files (defaults to the
            bundled library). Overrides any directory recorded in the session, so a session
            is portable across checkouts.

    Returns:
        ``(conditions, document)`` — the rebuilt
        :class:`~orp.core.simulation.conditions.SimulationConditions` and the parsed session
        mapping.

    Raises:
        SessionFormatError: structurally invalid session (missing keys, bad frame tag, bad
            source kind).
        SessionIntegrityError: a recorded hash no longer matches the file/content on disk.
        FileNotFoundError: unknown vehicle name (no matching YAML).
        KeyError: unknown planet name.
    """
    import yaml  # lazy

    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, dict):
        raise SessionFormatError(f"{path}: top-level session YAML must be a mapping.")

    if document.get("format_version") != SESSION_FORMAT_VERSION:
        raise SessionFormatError(
            "Unsupported session version: aerodynamic replay requires version 2. Re-export from the original setup; do not guess the missing model."
        )
    aerodynamic_calculator = _load_aerodynamics(document.get("aerodynamics"))

    # --- vehicle: reload + verify YAML hash ---
    vehicle_block = _require(document, "vehicle", "session")
    vehicle_name = str(_require(vehicle_block, "name", "vehicle"))
    saved_vehicle_hash = str(_require(vehicle_block, "sha256", "vehicle"))

    library = VehicleLibrary(
        Path(vehicle_data_dir) if vehicle_data_dir is not None else None
    )
    vehicle_yaml = library.data_dir / f"{vehicle_name}.yaml"
    if not vehicle_yaml.is_file():
        available = ", ".join(library.list_available()) or "(none)"
        raise FileNotFoundError(
            f"No vehicle named {vehicle_name!r} in {library.data_dir} (available: {available})."
        )
    current_vehicle_hash = _sha256_file(vehicle_yaml)
    if current_vehicle_hash != saved_vehicle_hash:
        raise SessionIntegrityError(
            f"Vehicle YAML content changed for vehicle {vehicle_name!r} "
            f"({vehicle_yaml}): "
            f"saved sha256={saved_vehicle_hash}, current sha256={current_vehicle_hash}. "
            "Refusing to load: the run cannot be reproduced against a different vehicle."
        )
    vehicle: EntryVehicle = library.load(vehicle_name)

    # --- planet ---
    planet_block = _require(document, "planet", "session")
    planet_name = str(_require(planet_block, "name", "planet"))
    planet: Planet = planet_registry.by_name(planet_name)

    # --- entry state (frame tag mandatory + validated) ---
    entry = _require(document, "entry_state", "session")
    frame = entry.get("frame") if isinstance(entry, dict) else None
    if frame is None:
        raise SessionFormatError(
            "entry_state.frame is missing. A frame tag is mandatory and must be "
            f"one of {VALID_FRAMES}."
        )
    if frame not in VALID_FRAMES:
        raise SessionFormatError(
            f"entry_state.frame is {frame!r}; must be one of {VALID_FRAMES}."
        )
    if frame == FRAME_INERTIAL:
        raise SessionFormatError(
            "entry_state.frame is 'inertial', which session loading refuses. Session "
            "files record the planet-relative state the engine actually consumed; "
            "inertial entry states are converted to planet-relative at the CLI boundary "
            "via orp.core.frames. The path is: convert first, then save. No conversion "
            "is performed inside session loading."
        )

    # --- bank schedule: rebuild from source + verify content hash ---
    schedule_block = _require(document, "bank_schedule", "session")
    saved_schedule_hash = str(
        _require(schedule_block, "content_sha256", "bank_schedule")
    )
    schedule_prov = _provenance_from_block(schedule_block.get("provenance"))
    source = _source_from_mapping(_require(schedule_block, "source", "bank_schedule"))
    schedule = _reconstruct_schedule(
        source, provenance=schedule_prov, base_dir=path.parent
    )
    current_schedule_hash = _schedule_content_hash(schedule)
    if current_schedule_hash != saved_schedule_hash:
        raise SessionIntegrityError(
            "Bank schedule numerical content changed: "
            f"saved content_sha256={saved_schedule_hash}, "
            f"current content_sha256={current_schedule_hash}. "
            "Refusing to load: the replayed control history differs from what was saved."
        )

    # --- integrator + termination settings ---
    integ = _require(document, "integrator", "session")

    conditions = SimulationConditions(
        vehicle=vehicle,
        planet=planet,
        bank_schedule=schedule,
        aerodynamic_calculator=aerodynamic_calculator,
        entry_velocity=float(_require(entry, "entry_velocity", "entry_state")),
        entry_flight_path_angle=float(
            _require(entry, "entry_flight_path_angle", "entry_state")
        ),
        entry_altitude=float(_require(entry, "entry_altitude", "entry_state")),
        entry_heading=float(_require(entry, "entry_heading", "entry_state")),
        entry_latitude=float(_require(entry, "entry_latitude", "entry_state")),
        entry_longitude=float(_require(entry, "entry_longitude", "entry_state")),
        time_step=float(_require(integ, "time_step", "integrator")),
        max_simulation_time=float(_require(integ, "max_simulation_time", "integrator")),
        ground_altitude=float(_require(integ, "ground_altitude", "integrator")),
        minimum_velocity=float(_require(integ, "minimum_velocity", "integrator")),
    )
    return conditions, document


# ---------------------------------------------------------------------------
# Public source-description constructors (the supported schedule source kinds)
# ---------------------------------------------------------------------------


def source_constant(bank_angle_rad: float, *, duration: float = 0.0) -> _ScheduleSource:
    """A constant-bank schedule source (radians). See :class:`BankSchedule.constant`."""
    return _describe_constant(bank_angle_rad, duration=duration)


def source_arrays(times: Any, bank_angles_rad: Any) -> _ScheduleSource:
    """An explicit (times_s, bank_angles_rad) array source."""
    return _describe_arrays(times, bank_angles_rad)


def source_csv(path: str | Path) -> _ScheduleSource:
    """A two-column (time_s, bank_angle_deg) CSV source."""
    return _describe_csv(path)


# ---------------------------------------------------------------------------
# Document assembly / provenance block helpers
# ---------------------------------------------------------------------------


def _build_document(
    *,
    conditions: SimulationConditions,
    vehicle_name: str,
    vehicle_yaml: Path,
    schedule_source: _ScheduleSource,
) -> dict[str, Any]:
    """Assemble the ordered session mapping (pure recording; no mutation, no upgrades)."""
    schedule = conditions.bank_schedule
    return {
        "format_version": SESSION_FORMAT_VERSION,
        "vehicle": {
            "name": vehicle_name,
            "sha256": _sha256_file(vehicle_yaml),
            "provenance": _provenance_block(conditions.vehicle.provenance),
        },
        "planet": {
            "name": conditions.planet.name,
            "provenance": _provenance_block(conditions.planet.provenance),
        },
        "aerodynamics": _aerodynamics_block(conditions.aerodynamic_calculator),
        "entry_state": {
            # Mandatory frame tag. ORP's entry velocity is planet-relative.
            "frame": FRAME_PLANET_RELATIVE,
            "entry_altitude": float(conditions.entry_altitude),
            "entry_latitude": float(conditions.entry_latitude),
            "entry_longitude": float(conditions.entry_longitude),
            "entry_velocity": float(conditions.entry_velocity),
            "entry_flight_path_angle": float(conditions.entry_flight_path_angle),
            "entry_heading": float(conditions.entry_heading),
        },
        "bank_schedule": {
            "source": _source_to_mapping(schedule_source),
            "content_sha256": _schedule_content_hash(schedule),
            "provenance": _provenance_block(schedule.provenance),
        },
        "integrator": {
            "time_step": float(conditions.time_step),
            "max_simulation_time": float(conditions.max_simulation_time),
            "ground_altitude": float(conditions.ground_altitude),
            "minimum_velocity": float(conditions.minimum_velocity),
        },
        "provenance_levels": _provenance_levels(conditions),
    }


def _provenance_block(tag: ProvenanceTag) -> dict[str, str]:
    """Serialise a provenance tag (level key + source + notes) for the record."""
    return {"level": tag.level.key, "source": tag.source, "notes": tag.notes}


def _provenance_from_block(raw: Any) -> ProvenanceTag:
    """Rebuild a :class:`ProvenanceTag` from a recorded provenance block (best-effort).

    Used only to reattach the schedule's recorded provenance to the rebuilt schedule. A
    missing/invalid block falls back to ``NOT_VALIDATED`` rather than failing the load, but
    the schedule's *content* is independently hash-verified, so trust in the numbers does
    not rest on this block.
    """
    if not isinstance(raw, dict):
        return ProvenanceTag(ValidationLevel.NOT_VALIDATED)
    level = ValidationLevel.from_string(str(raw.get("level", "not_validated")))
    return ProvenanceTag(
        level=level, source=str(raw.get("source", "")), notes=str(raw.get("notes", ""))
    )


# Public aliases for the source constructors and the (private) dataclass, so callers and
# tests can name them without reaching through the underscore-prefixed implementation.
ScheduleSource = _ScheduleSource

__all__ += [
    "source_constant",
    "source_arrays",
    "source_csv",
    "ScheduleSource",
    "SESSION_FORMAT_VERSION",
    "FRAME_INERTIAL",
    "FRAME_PLANET_RELATIVE",
    "SOURCE_CONSTANT",
    "SOURCE_ARRAYS",
    "SOURCE_CSV",
]
