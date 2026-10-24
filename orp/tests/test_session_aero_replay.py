# ORP — Open Reentry Platform
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regression for the documented CLI/session aerodynamic-model mismatch."""

import pytest, yaml
from orp.core import session as S
from orp.core.aerodynamics.constant import ConstantCoefficientCalculator
from orp.core.aerodynamics.newtonian import ModifiedNewtonianCalculator
from orp.core.bank_schedule.schedule import BankSchedule
from orp.core.planet.registry import by_name
from orp.core.provenance.tags import ProvenanceTag, ValidationLevel
from orp.core.simulation import SimulationEngine
from orp.core.simulation.conditions import SimulationConditions
from orp.core.vehicles.library import load_vehicle
from orp.tests.test_session import _assert_trajectories_bit_identical


def setup(aero, planet="Earth", vehicle="apollo"):
    return SimulationConditions(
        vehicle=load_vehicle(vehicle),
        planet=by_name(planet),
        bank_schedule=BankSchedule.constant(0.4),
        aerodynamic_calculator=aero,
        max_simulation_time=30.0,
        entry_altitude=70000.0,
        time_step=0.2,
    )


def save(tmp_path, c):
    p = tmp_path / "run.yaml"
    S.save_session(
        p,
        conditions=c,
        vehicle_name=("msl" if c.planet.name == "Mars" else "apollo"),
        schedule_source=S.source_constant(0.4),
    )
    return p


@pytest.mark.parametrize("planet,vehicle", [("Earth", "apollo"), ("Mars", "msl")])
def test_constant_roundtrip_all_channels(tmp_path, planet, vehicle):
    aero = ConstantCoefficientCalculator(
        1.23,
        0.37,
        provenance=ProvenanceTag(
            ValidationLevel.ASSERTED, source="regression", notes="custom coefficients"
        ),
    )
    c = setup(aero, planet, vehicle)
    p = save(tmp_path, c)
    replayed, _ = S.load_session(p)
    _assert_trajectories_bit_identical(
        SimulationEngine().simulate(c), SimulationEngine().simulate(replayed)
    )
    assert type(replayed.aerodynamic_calculator) is ConstantCoefficientCalculator
    assert replayed.aerodynamic_calculator.provenance == aero.provenance


def test_newtonian_parameter_preserved(tmp_path):
    c = setup(ModifiedNewtonianCalculator(0.7))
    replayed, _ = S.load_session(save(tmp_path, c))
    assert replayed.aerodynamic_calculator.get_stall_angle() == 0.7


@pytest.mark.parametrize(
    "field,value",
    [
        ("kind", "unknown"),
        ("drag_coefficient", float("nan")),
        ("drag_coefficient", 0.0),
        ("lift_to_drag", float("inf")),
    ],
)
def test_bad_aerodynamics_refused(tmp_path, field, value):
    p = save(tmp_path, setup(ConstantCoefficientCalculator(1.2, 0.3)))
    d = yaml.safe_load(p.read_text())
    d.setdefault("aerodynamics", {})[field] = value
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(S.SessionFormatError):
        S.load_session(p)


def test_legacy_session_without_aero_refused(tmp_path):
    p = save(tmp_path, setup(ConstantCoefficientCalculator(1.2, 0.3)))
    d = yaml.safe_load(p.read_text())
    d.pop("aerodynamics", None)
    d["format_version"] = 1
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(S.SessionFormatError, match="aerodynamic"):
        S.load_session(p)


def test_unknown_model_refused_on_save(tmp_path):
    class Custom(ModifiedNewtonianCalculator):
        pass

    with pytest.raises(S.SessionFormatError, match="aerodynamic"):
        save(tmp_path, setup(Custom()))


@pytest.mark.parametrize("planet,vehicle", [("earth", "apollo"), ("mars", "msl")])
def test_real_cli_session_csv_replay(tmp_path, monkeypatch, planet, vehicle):
    from orp import cli
    from orp.gui import plots
    from orp.core.report import write_trajectory_csv

    monkeypatch.setattr(plots, "save_standard_plots", lambda *args, **kwargs: None)
    out = tmp_path / "cli"
    assert (
        cli.main(
            [
                "run",
                "--vehicle",
                vehicle,
                "--planet",
                planet,
                "--frame",
                "planet-relative",
                "--bank-deg",
                "60",
                "--max-time",
                "30",
                "--out",
                str(out),
            ]
        )
        == 0
    )
    replayed, _ = S.load_session(out / "session.yaml")
    write_trajectory_csv(
        SimulationEngine().simulate(replayed), tmp_path / "replayed.csv"
    )
    assert (out / "trajectory.csv").read_bytes() == (
        tmp_path / "replayed.csv"
    ).read_bytes()


def test_future_version_refused(tmp_path):
    p = save(tmp_path, setup(ConstantCoefficientCalculator(1.2, 0.3)))
    d = yaml.safe_load(p.read_text())
    d["format_version"] = 999
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(S.SessionFormatError, match="version"):
        S.load_session(p)
