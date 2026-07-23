import json
import os
import tempfile

import pytest
from fakes import FakeReactor
from klippy.extras.servo_calibration.dynamics import (
    PIN_LEAD_US_MAX,
    PIN_ZETA_MAX,
)
from test_servo_calibration_awd import (
    FakeGcmd,
    make_calibration,
    requires_tomllib,
    single_drive_rails,
)


def _pinned_profile(pin_zeta=0.05, pin_lead_us=100.0, pin_mass_x=0.3):
    return (
        "\n".join(
            [
                "version = 8",
                'axes = ["motor_a", "motor_b"]',
                'modes = ["x", "y"]',
                "frame = [[1.0, 0.0], [0.0, 1.0]]",
                "mass = [0.5, 0.5]",
                "viscous = [0.01, 0.01]",
                "coulomb = [9.0, 9.0]",
                "compliance = [1.5e-6, 1.5e-6]",
                "pin_mass = [%r, 0.0]" % (pin_mass_x,),
                "pin_zeta = [%r, 0.0]" % (pin_zeta,),
                "pin_lead_us = %r" % (pin_lead_us,),
            ]
        )
        + "\n"
    )


def _setup(pin_mass_x=0.3, residuals=None, **profile_kw):
    """Cartesian single-drive calibration with a baseline that pins mode x,
    and a fake `analyze` that stamps each recorded step with a residual from
    ``residuals`` (mm, aligned to step order) onto the mode's drive block."""
    sc, gcode = make_calibration(
        single_drive_rails(), coupled=False, reactor=FakeReactor(tick=0.0)
    )
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    path = os.path.join(tempfile.mkdtemp(), "baseline.toml")
    with open(path, "w") as f:
        f.write(_pinned_profile(pin_mass_x=pin_mass_x, **profile_kw))
    node.dynamics_profile = path

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            run_dir = argv[2]
            with open(os.path.join(run_dir, "manifest.json")) as mf:
                manifest = json.load(mf)
            steps = []
            for i, st in enumerate(manifest["steps"]):
                res = residuals[i] if residuals is not None else 0.0
                steps.append(
                    {
                        "name": st["name"],
                        "drives": {
                            "motor_a": {"metrics": {"pin_residual_mm": res}}
                        },
                    }
                )
            with open(os.path.join(run_dir, "results.json"), "w") as rf:
                json.dump({"steps": steps, "verdict": {}}, rf)
        return ""

    sc._run = fake_run
    return sc, gcode, node, path


@requires_tomllib
def test_pin_sweep_picks_the_minimum_residual():
    # residuals (um): 5, 2, 1, 3 -> minimum at the third value (ZETA=0.06).
    sc, _gcode, node, path = _setup(residuals=[5.0e-3, 2.0e-3, 1.0e-3, 3.0e-3])
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04,0.06,0.08",
        DWELL="1",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "minimum at ZETA=0.06" in report
    assert "ZETA=0.06" in report
    # the ready-to-run line carries the reconstructed FRF peak and preserves
    # the un-swept pin lead (baseline pin_lead_us=100)
    assert "SERVO_SET_COMPLIANCE PIN=X" in report
    assert "PIN_LEAD_US=100" in report

    # each value streamed live as pin_zeta, then the baseline restored last
    engine = sc.printer.lookup_object("motion_engine")
    streamed = [call[7][0] for call in engine.dynamics_calls]
    assert streamed[:4] == [0.02, 0.04, 0.06, 0.08]
    assert streamed[-1] == 0.05  # baseline pin_zeta restored
    assert node.live_dynamics_profile == path


@requires_tomllib
def test_pin_sweep_sweeps_lead_and_keeps_zeta():
    sc, _gcode, _node, _path = _setup(residuals=[3.0e-3, 1.0e-3, 4.0e-3])
    gcmd = FakeGcmd(
        MODE="X", FREQ="130", PARAM="LEAD", VALUES="0,150,300", DWELL="1"
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "minimum at LEAD=150" in report
    # LEAD is a whole-model scalar (arg index 8); zeta stays at baseline 0.05
    engine = sc.printer.lookup_object("motion_engine")
    assert [call[8] for call in engine.dynamics_calls][:3] == [
        0.0,
        150.0,
        300.0,
    ]
    assert engine.dynamics_calls[0][7][0] == 0.05
    assert "PIN_LEAD_US=150" in report
    assert "ZETA=0.05" in report


@requires_tomllib
def test_pin_sweep_refuses_unpinned_mode():
    sc, _gcode, _node, _path = _setup(pin_mass_x=0.0)
    gcmd = FakeGcmd(MODE="X", FREQ="130", VALUES="0.02,0.04", DWELL="1")
    with pytest.raises(Exception, match="not pinned"):
        sc.cmd_SERVO_SWEEP_PIN(gcmd)


@requires_tomllib
def test_pin_sweep_restores_baseline_on_failure_mid_sweep():
    sc, _gcode, node, path = _setup(residuals=[1.0e-3, 1.0e-3, 1.0e-3])
    engine = sc.printer.lookup_object("motion_engine")

    calls = {"n": 0}
    real = engine.set_dynamics_model

    def explode(*args):
        calls["n"] += 1
        if calls["n"] == 2:  # fail while streaming the second value
            raise RuntimeError("stream blew up")
        return real(*args)

    engine.set_dynamics_model = explode
    gcmd = FakeGcmd(
        MODE="X", FREQ="130", PARAM="ZETA", VALUES="0.02,0.04,0.06", DWELL="1"
    )
    with pytest.raises(Exception, match="stream blew up"):
        sc.cmd_SERVO_SWEEP_PIN(gcmd)
    # the finally block restored the pre-sweep model and cleared the run
    assert engine.dynamics_calls[-1][7] == [0.05, 0.0]
    assert engine.dynamics_calls[-1][8] == 100.0
    assert node.live_dynamics_profile == path
    assert sc._active_run is None


@requires_tomllib
def test_pin_sweep_errors_when_no_pin_residual():
    sc, _gcode, _node, _path = _setup(residuals=[0.0, 0.0])
    gcmd = FakeGcmd(MODE="X", FREQ="130", VALUES="0.02,0.04", DWELL="1")
    with pytest.raises(Exception, match="no pin residual"):
        sc.cmd_SERVO_SWEEP_PIN(gcmd)


@requires_tomllib
def test_pin_sweep_manifest_records_steps():
    sc, _gcode, _node, _path = _setup(residuals=[2.0e-3, 1.0e-3])
    gcmd = FakeGcmd(MODE="X", FREQ="130", VALUES="0.02,0.04", DWELL="1")
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    cap = sc.printer.lookup_object("servo_capture")
    run_dir = os.path.dirname(cap.starts[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        manifest = json.load(f)
    assert manifest["experiment"] == "pin_sweep"
    steps = manifest["steps"]
    assert [s["name"] for s in steps] == ["v0", "v1"]
    assert [s["swept"]["value"] for s in steps] == [0.02, 0.04]
    for s in steps:
        assert s["swept"]["t_end_s"] >= s["swept"]["t_start_s"]
    assert manifest["stroke_plan"]["param"] == "ZETA"
    assert manifest["stroke_plan"]["freq"] == 130.0


def _values(sc, param, text):
    return sc._parse_pin_sweep_values(FakeGcmd(VALUES=text), param)


@requires_tomllib
def test_pin_sweep_values_validation_reuses_set_rules():
    sc, _gcode, _node, _path = _setup()
    # count bounds: 2..12
    with pytest.raises(Exception, match="2..12"):
        _values(sc, "ZETA", "0.02")
    with pytest.raises(Exception, match="2..12"):
        _values(sc, "ZETA", ",".join("0.01" for _ in range(13)))
    # ZETA: above 0 and <= PIN_ZETA_MAX (the SERVO_SET_COMPLIANCE ZETA rule)
    with pytest.raises(Exception, match="ZETA"):
        _values(sc, "ZETA", "0.0,0.04")
    with pytest.raises(Exception, match="ZETA"):
        _values(sc, "ZETA", "0.04,%g" % (PIN_ZETA_MAX + 0.01,))
    assert _values(sc, "ZETA", "0.02,%g" % (PIN_ZETA_MAX,)) == [
        0.02,
        PIN_ZETA_MAX,
    ]
    # LEAD: >= 0 and <= PIN_LEAD_US_MAX (the PIN_LEAD_US rule)
    with pytest.raises(Exception, match="LEAD"):
        _values(sc, "LEAD", "-1,100")
    with pytest.raises(Exception, match="LEAD"):
        _values(sc, "LEAD", "100,%g" % (PIN_LEAD_US_MAX + 1.0,))
    assert _values(sc, "LEAD", "0,%g" % (PIN_LEAD_US_MAX,)) == [
        0.0,
        PIN_LEAD_US_MAX,
    ]
