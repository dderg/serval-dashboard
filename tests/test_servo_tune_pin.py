import json
import os
import tempfile

import pytest
from fakes import FakeReactor
from test_servo_calibration_awd import (
    FakeGcmd,
    make_calibration,
    requires_tomllib,
    single_drive_rails,
)
from test_servo_sweep_pin import FakeAccelChip


def _pinned_profile():
    # A baseline that pins mode x (pin_zeta 0.05, pin_lead_us 100). Both
    # modes are re-pinned from scratch by SERVO_TUNE_PIN.
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
                "pin_mass = [0.3, 0.0]",
                "pin_zeta = [0.05, 0.0]",
                "pin_lead_us = 100.0",
            ]
        )
        + "\n"
    )


# Per-(mode) zeta bowls and the lead bowl. Each staircase's residual is a
# convex function of the swept value with its minimum at the target, so the
# coarse stage lands on the nearest grid point and the fine stage refines.
ZETA_TARGET = {"x": 0.06, "y": 0.15}
LEAD_TARGET = 300.0


def _setup(accel_amps=None, accel_freq=131.5):
    extra_objs = None
    if accel_amps is not None:
        extra_objs = {"adxl345 tool": FakeAccelChip(accel_amps, accel_freq)}
    sc, gcode = make_calibration(
        single_drive_rails(),
        coupled=False,
        reactor=FakeReactor(tick=0.0),
        extra_objs=extra_objs,
    )
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    path = os.path.join(tempfile.mkdtemp(), "baseline.toml")
    with open(path, "w") as f:
        f.write(_pinned_profile())
    node.dynamics_profile = path

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            run_dir = argv[2]
            with open(os.path.join(run_dir, "manifest.json")) as mf:
                manifest = json.load(mf)
            plan = manifest["stroke_plan"]
            param = plan["param"]
            mode = plan["mode"]
            steps = []
            for st in manifest["steps"]:
                val = st["swept"]["value"]
                if param == "ZETA":
                    res = (1.0 + abs(val - ZETA_TARGET[mode]) * 20.0) * 1e-3
                else:
                    res = (1.0 + abs(val - LEAD_TARGET) / 100.0) * 1e-3
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


# ---- SERVO_TUNE_PIN full flow --------------------------------------------


@requires_tomllib
def test_tune_pin_full_flow_applies_per_mode_zeta_and_shared_lead():
    sc, _gcode, node, path = _setup()
    # Overrides skip the measurement stage; Y is the lower-frequency mode.
    gcmd = FakeGcmd(
        MODES="XY",
        DWELL="1",
        X_FREQ="216.8",
        X_PEAK="300",
        Y_FREQ="131.5",
        Y_PEAK="200",
    )
    sc.cmd_SERVO_TUNE_PIN(gcmd)
    report = " ".join(gcmd.responses)

    engine = sc.printer.lookup_object("motion_engine")
    final = engine.dynamics_calls[-1]
    # Per-mode winners. The synthetic residual bowl minimises at
    # ZETA_TARGET, but ZETA is picked by the knee rule, not argmin: the
    # lowest zeta scoring within ZETA_TOL of the best wins, because zeta is
    # inverse predictor gain and under-driving the pin leaves two spikes.
    # x: bowl is shallow across the grid, so the fine stage's 0.05 rung
    # scores within 15% of the 0.0632 minimum and the knee takes it.
    # y: bowl is steep enough that nothing below the 0.1518 minimum
    # qualifies, so the knee agrees with argmin.
    fine_y = 0.12 / 1.6 * (2.56 ** (3.0 / 4.0))
    assert final[7][0] == pytest.approx(0.05, rel=1e-6)
    assert final[7][1] == pytest.approx(fine_y, rel=1e-6)
    # lead is a whole-model scalar applied once, on the lowest-f mode (y)
    assert final[8] == 300.0
    assert "on mode y" in report

    # the tuned profile was written and left live (not the baseline)
    assert node.live_dynamics_profile != path
    assert node.live_dynamics_profile.endswith(".toml")
    assert os.path.exists(node.live_dynamics_profile)
    assert "written" in report
    # the ready-to-run line uses the per-mode X_ZETA/Y_ZETA spelling
    assert "X_ZETA=" in report
    assert "Y_ZETA=" in report
    assert "PIN_LEAD_US=300" in report
    # per-mode planner mode_inverse snippet: frequency_hz = measured f_b,
    # damping_ratio = the fine-ladder belt zeta winner.
    assert "type: mode_inverse" in report
    assert "frequency_hz: 216.8" in report
    assert "frequency_hz: 131.5" in report
    assert "damping_ratio: %.4g" % (0.05,) in report
    assert "damping_ratio: %.4g" % (fine_y,) in report
    assert sc._active_run is None


@requires_tomllib
def test_tune_pin_single_mode_runs_lead_on_that_mode():
    sc, _gcode, node, _path = _setup()
    gcmd = FakeGcmd(MODES="X", DWELL="1", X_FREQ="216.8", X_PEAK="300")
    sc.cmd_SERVO_TUNE_PIN(gcmd)
    report = " ".join(gcmd.responses)
    engine = sc.printer.lookup_object("motion_engine")
    final = engine.dynamics_calls[-1]
    assert final[7][0] == pytest.approx(0.05, rel=1e-6)
    # y stays unpinned (baseline pin_zeta 0.0)
    assert final[7][1] == 0.0
    assert final[8] == 300.0
    assert "on mode x" in report


@requires_tomllib
def test_tune_pin_restores_pre_tune_model_on_mid_failure():
    sc, _gcode, node, path = _setup()
    engine = sc.printer.lookup_object("motion_engine")

    calls = {"n": 0}
    real = engine.set_dynamics_model

    def explode(*args):
        calls["n"] += 1
        if calls["n"] == 2:  # blow up during the first coarse staircase
            raise RuntimeError("stream blew up")
        return real(*args)

    engine.set_dynamics_model = explode
    gcmd = FakeGcmd(
        MODES="XY",
        DWELL="1",
        X_FREQ="216.8",
        X_PEAK="300",
        Y_FREQ="131.5",
        Y_PEAK="200",
    )
    with pytest.raises(Exception, match="pre-tune model restored"):
        sc.cmd_SERVO_TUNE_PIN(gcmd)
    # the finally restored the pre-tune model (baseline pin state) last
    assert engine.dynamics_calls[-1][7] == [0.05, 0.0]
    assert engine.dynamics_calls[-1][8] == 100.0
    assert node.live_dynamics_profile == path
    assert sc._active_run is None


# ---- SERVO_SET_COMPLIANCE per-mode zeta ----------------------------------


@requires_tomllib
def test_set_compliance_x_zeta_only_touches_x():
    sc, _gcode, _path = _make_set_calibration()
    sc.cmd_SERVO_SET_COMPLIANCE(
        FakeGcmd(
            X_FREQ="216.8",
            Y_FREQ="131.5",
            PIN="XY",
            X_PEAK="300",
            Y_PEAK="200",
            X_ZETA="0.055",
        )
    )
    engine = sc.printer.lookup_object("motion_engine")
    pin_zeta = engine.dynamics_calls[-1][7]
    # x uses its own X_ZETA, y falls back to the shared ZETA default (0.02)
    assert pin_zeta[0] == pytest.approx(0.055)
    assert pin_zeta[1] == pytest.approx(0.02)


@requires_tomllib
def test_set_compliance_zeta_fallback_applies_to_both():
    sc, _gcode, _path = _make_set_calibration()
    sc.cmd_SERVO_SET_COMPLIANCE(
        FakeGcmd(
            X_FREQ="216.8",
            Y_FREQ="131.5",
            PIN="XY",
            X_PEAK="300",
            Y_PEAK="200",
            ZETA="0.08",
        )
    )
    engine = sc.printer.lookup_object("motion_engine")
    pin_zeta = engine.dynamics_calls[-1][7]
    assert pin_zeta[0] == pytest.approx(0.08)
    assert pin_zeta[1] == pytest.approx(0.08)


@requires_tomllib
def test_set_compliance_mixed_x_zeta_and_shared_zeta():
    sc, _gcode, _path = _make_set_calibration()
    sc.cmd_SERVO_SET_COMPLIANCE(
        FakeGcmd(
            X_FREQ="216.8",
            Y_FREQ="131.5",
            PIN="XY",
            X_PEAK="300",
            Y_PEAK="200",
            X_ZETA="0.055",
            ZETA="0.08",
        )
    )
    engine = sc.printer.lookup_object("motion_engine")
    pin_zeta = engine.dynamics_calls[-1][7]
    # X_ZETA wins for x; y takes the shared ZETA fallback
    assert pin_zeta[0] == pytest.approx(0.055)
    assert pin_zeta[1] == pytest.approx(0.08)


@requires_tomllib
def test_set_compliance_zeta_validation():
    sc, _gcode, _path = _make_set_calibration()
    base = dict(X_FREQ="216.8", PIN="X", X_PEAK="300")
    # zero rejected
    with pytest.raises(Exception, match="X_ZETA"):
        sc.cmd_SERVO_SET_COMPLIANCE(FakeGcmd(X_ZETA="0", **base))
    # non-finite rejected
    with pytest.raises(Exception, match="X_ZETA"):
        sc.cmd_SERVO_SET_COMPLIANCE(FakeGcmd(X_ZETA="inf", **base))
    # overdamped (> 1) accepted - no upper cap
    sc.cmd_SERVO_SET_COMPLIANCE(FakeGcmd(X_ZETA="1.4", **base))
    engine = sc.printer.lookup_object("motion_engine")
    assert engine.dynamics_calls[-1][7][0] == pytest.approx(1.4)


def _make_set_calibration():
    """A Cartesian single-drive calibration whose node profile pins nothing,
    so SERVO_SET_COMPLIANCE builds the pin from scratch."""
    sc, gcode = make_calibration(single_drive_rails(), coupled=False)
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    path = os.path.join(tempfile.mkdtemp(), "baseline.toml")
    with open(path, "w") as f:
        f.write(
            "\n".join(
                [
                    "version = 7",
                    'axes = ["motor_a", "motor_b"]',
                    'modes = ["x", "y"]',
                    "frame = [[1.0, 0.0], [0.0, 1.0]]",
                    "mass = [0.5, 0.5]",
                    "viscous = [0.01, 0.01]",
                    "coulomb = [9.0, 9.0]",
                    "compliance = [0.0, 0.0]",
                ]
            )
            + "\n"
        )
    node.dynamics_profile = path
    return sc, gcode, path


@requires_tomllib
def test_tune_pin_scores_accel_on_every_ladder_stage():
    # MODES=X keeps every staircase (coarse 7, fine 5, lead 5) at the same
    # tone f_b, so the chip's fixed-frequency tone lines up. The accel
    # minimum line surfaces once per stage and the pin verdict is unchanged.
    n_steps = 8 + 5 + 5
    amps = [1.0 + 0.1 * i for i in range(n_steps)]
    sc, _gcode, node, path = _setup(accel_amps=amps, accel_freq=131.5)
    gcmd = FakeGcmd(
        MODES="X",
        DWELL="1",
        X_FREQ="131.5",
        X_PEAK="200",
        ACCEL_CHIP="adxl345 tool",
    )
    sc.cmd_SERVO_TUNE_PIN(gcmd)
    report = " ".join(gcmd.responses)
    # every staircase stage (coarse zeta, fine zeta, lead) is decided on
    # the toolhead, not the drive-side residual
    assert report.count("picked on toolhead accel") == 3
    assert "picked ZETA=" in report
    assert "picked LEAD=" in report
    # a client was started for every scored step across all stages
    chip = sc.printer.lookup_object("adxl345 tool")
    assert len(chip.clients) == n_steps
    # residual verdict still drives the applied model + written profile
    assert node.live_dynamics_profile != path
    assert "X_ZETA=" in report


@requires_tomllib
def test_tune_pin_all_accel_captures_empty_fails_loud():
    # Every step's accel capture is empty - a broken accelerometer, not a
    # preference. Proceeding on the drive-side residual alone is how LEAD=0
    # beat the bench-correct 600, so the tune refuses instead, restoring
    # the pre-tune model.
    n_steps = 7 + 5 + 5
    sc, _gcode, node, path = _setup(
        accel_amps=[None] * n_steps, accel_freq=131.5
    )
    gcmd = FakeGcmd(
        MODES="X",
        DWELL="1",
        X_FREQ="131.5",
        X_PEAK="200",
        ACCEL_CHIP="adxl345 tool",
    )
    with pytest.raises(Exception, match="no staircase step yielded accel"):
        sc.cmd_SERVO_TUNE_PIN(gcmd)
    assert node.live_dynamics_profile == path
