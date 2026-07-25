import json
import math
import os
import tempfile

import pytest
from fakes import FakeReactor
from klippy.extras.servo_calibration.dynamics import (
    PIN_LEAD_US_MAX,
)
from test_servo_calibration_awd import (
    FakeGcmd,
    make_calibration,
    requires_tomllib,
    single_drive_rails,
)


class FakeAccelClient:
    def __init__(self, samples):
        self.samples = samples

    def finish_measurements(self):
        pass

    def has_valid_samples(self):
        return bool(self.samples)

    def get_samples(self):
        return self.samples


class FakeAccelChip:
    """Emits a pure tone at ``freq`` per started client; the per-step
    amplitude is popped from ``amps`` (None -> empty capture -> n/a). The
    three axes split the amplitude equally so the vector-magnitude bin the
    scorer computes recovers the requested amplitude."""

    def __init__(self, amps, freq, fs=1000.0, n=600):
        self.amps = list(amps)
        self.freq = freq
        self.fs = fs
        self.n = n
        self.clients = []
        self._i = 0

    def start_internal_client(self):
        amp = self.amps[self._i] if self._i < len(self.amps) else None
        self._i += 1
        samples = []
        if amp is not None:
            a = amp / math.sqrt(3.0)
            for k in range(self.n):
                x = a * math.sin(2.0 * math.pi * self.freq * (k / self.fs))
                samples.append((100.0 + k / self.fs, x, x, x))
        client = FakeAccelClient(samples)
        self.clients.append(client)
        return client


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


def _setup(
    pin_mass_x=0.3,
    residuals=None,
    accel_amps=None,
    accel_freq=130.0,
    **profile_kw,
):
    """Cartesian single-drive calibration with a baseline that pins mode x,
    and a fake `analyze` that stamps each recorded step with a residual from
    ``residuals`` (mm, aligned to step order) onto the mode's drive block.
    When ``accel_amps`` is given, an ``adxl345 tool`` accel chip emitting a
    per-step tone of that amplitude at ``accel_freq`` is registered."""
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
def test_pin_sweep_picks_the_minimum_residual_when_it_is_a_clear_win():
    # residuals (um): 5, 2, 1, 3. Nothing lower scores within ZETA_TOL of
    # the 1 um minimum, so the plain minimum is also the lowest admissible
    # zeta and the knee rule agrees with argmin.
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
    assert "picked ZETA=0.06" in report
    assert "X_ZETA=0.06" in report
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
def test_pin_sweep_prefers_the_lowest_zeta_on_a_flat_residual_shoulder():
    # residuals (um): 1.05, 1.02, 1.00, 3.0. Argmin is ZETA=0.02, but the
    # three low values are within noise of each other - and zeta is inverse
    # predictor gain, so the lowest one cancels the coupled mode hardest.
    # Taking the argmin here is what left the machine with two input-shaper
    # spikes instead of one.
    sc, _gcode, _node, _path = _setup(
        residuals=[1.05e-3, 1.02e-3, 1.0e-3, 3.0e-3]
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.005,0.01,0.02,0.04",
        DWELL="1",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "picked ZETA=0.005" in report
    assert "X_ZETA=0.005" in report
    # settling on the lowest rung means the useful value may be lower still
    assert "ladder floor" in report


@requires_tomllib
def test_pin_sweep_zeta_tol_zero_restores_plain_argmin():
    sc, _gcode, _node, _path = _setup(
        residuals=[1.05e-3, 1.02e-3, 1.0e-3, 3.0e-3]
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.005,0.01,0.02,0.04",
        DWELL="1",
        ZETA_TOL="0",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    assert "picked ZETA=0.02" in " ".join(gcmd.responses)


@requires_tomllib
def test_pin_sweep_lead_ignores_the_zeta_knee_rule():
    # LEAD is not an inverse gain - there is no "prefer lower" argument, so
    # a flat shoulder must still resolve to the actual minimum.
    sc, _gcode, _node, _path = _setup(residuals=[1.05e-3, 1.02e-3, 1.0e-3])
    gcmd = FakeGcmd(
        MODE="X", FREQ="130", PARAM="LEAD", VALUES="0,150,300", DWELL="1"
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "picked LEAD=300" in report
    assert "ladder floor" not in report


@requires_tomllib
def test_pin_sweep_sweeps_lead_and_keeps_zeta():
    sc, _gcode, _node, _path = _setup(residuals=[3.0e-3, 1.0e-3, 4.0e-3])
    gcmd = FakeGcmd(
        MODE="X", FREQ="130", PARAM="LEAD", VALUES="0,150,300", DWELL="1"
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "picked LEAD=150" in report
    # LEAD is a whole-model scalar (arg index 8); zeta stays at baseline 0.05
    engine = sc.printer.lookup_object("motion_engine")
    assert [call[8] for call in engine.dynamics_calls][:3] == [
        0.0,
        150.0,
        300.0,
    ]
    assert engine.dynamics_calls[0][7][0] == 0.05
    assert "PIN_LEAD_US=150" in report
    assert "X_ZETA=0.05" in report


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
    # Step names carry the swept value: the dashboard prints them as chart
    # legends, where "v0 v1" identified nothing.
    assert [s["name"] for s in steps] == ["zeta0p02", "zeta0p04"]
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
    # count: at least one value; single-value runs are legal, empty is not
    with pytest.raises(Exception, match="at least one"):
        _values(sc, "ZETA", " , ")
    assert _values(sc, "ZETA", "0.02") == [0.02]
    assert len(_values(sc, "ZETA", ",".join("0.01" for _ in range(13)))) == 13
    # ZETA: finite and > 0, no upper cap (overdamped predictors are legal -
    # the SERVO_SET_COMPLIANCE ZETA rule)
    with pytest.raises(Exception, match="ZETA"):
        _values(sc, "ZETA", "0.0,0.04")
    with pytest.raises(Exception, match="ZETA"):
        _values(sc, "ZETA", "0.04,inf")
    assert _values(sc, "ZETA", "0.02,1.4") == [0.02, 1.4]
    # LEAD: >= 0 and <= PIN_LEAD_US_MAX (the PIN_LEAD_US rule)
    with pytest.raises(Exception, match="LEAD"):
        _values(sc, "LEAD", "-1,100")
    with pytest.raises(Exception, match="LEAD"):
        _values(sc, "LEAD", "100,%g" % (PIN_LEAD_US_MAX + 1.0,))
    assert _values(sc, "LEAD", "0,%g" % (PIN_LEAD_US_MAX,)) == [
        0.0,
        PIN_LEAD_US_MAX,
    ]


@requires_tomllib
def test_pin_sweep_scores_gate_unexcited_steps():
    # A step whose capture shows (almost) no actual torque never carried the
    # tone (lapsed buzz): its near-zero residual is silence, not a win. It
    # must score None instead of beating honestly excited steps.
    sc, _gcode, _node, _path = _setup()
    gcmd = FakeGcmd({})

    def step(name, residual, torque_peak):
        return {
            "name": name,
            "drives": {
                "motor_a": {
                    "metrics": {
                        "pin_residual_mm": residual,
                        "torque": {"peak": torque_peak},
                    }
                }
            },
        }

    results = {
        "steps": [
            step("zeta0p02", 0.005, 400),
            step("zeta0p05", 0.002, 380),
            step("zeta0p1", 0.00001, 12),  # unexcited: 3% of run torque
        ]
    }
    rows = sc._pin_sweep_scores(gcmd, results, [0.02, 0.05, 0.1], "ZETA")
    assert rows[0][1] == 0.005
    assert rows[1][1] == 0.002
    assert rows[2][1] is None, "unexcited step must not score"


@requires_tomllib
def test_duplicate_values_refuse_before_any_motion():
    # Step names are the swept value alone (so the same zeta lines up
    # across runs); two values rendering to one name would silently share
    # a capture file, so the sweep must refuse up front.
    sc, _gcode, _node, _path = _setup()
    with pytest.raises(Exception, match="same step name 'zeta0p02'"):
        sc._pin_step_names(FakeGcmd({}), "ZETA", [0.02, 0.05, 0.02])


@requires_tomllib
def test_pin_sweep_amplitude_config_default_drives_the_tone():
    # [servo_calibration] pin_sweep_amplitude sets the dwell-tone amplitude
    # when AMPLITUDE= is omitted; the buzz call carries it in nanometers.
    sc, _gcode, _node, _path = _setup(residuals=[3.0e-3, 1.0e-3])
    assert sc.pin_sweep_amplitude_mm == 0.01  # config default
    assert sc.compliance_amplitude_mm == 0.02  # config default
    sc.pin_sweep_amplitude_mm = 0.025
    gcmd = FakeGcmd(MODE="X", FREQ="130", VALUES="0.02,0.04", DWELL="1")
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    engine = sc.printer.lookup_object("motion_engine")
    amps = {b[5] for b in engine.buzzes}
    assert amps == {25000}, amps


@requires_tomllib
def test_pin_sweep_reports_accel_column_and_minimum():
    # residual minimum at ZETA=0.06 (idx 2); accel minimum also there, so
    # the two agree and no disagreement note is printed.
    sc, _gcode, _node, _path = _setup(
        residuals=[5.0e-3, 2.0e-3, 1.0e-3, 3.0e-3],
        accel_amps=[3.0, 2.0, 1.0, 4.0],
        accel_freq=130.0,
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04,0.06,0.08",
        DWELL="1",
        ACCEL_CHIP="adxl345 tool",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    # per-step accel values flow into the table (mm/s^2 at the tone)
    assert "3.0 mm/s^2" in report
    assert "1.0 mm/s^2" in report
    # both metrics land on the same rung, so there is nothing to arbitrate
    assert "picked ZETA=0.06" in report
    assert "picked on toolhead accel" in report
    assert "would have picked" not in report


@requires_tomllib
def test_pin_sweep_applies_the_accel_winner_over_the_residual():
    # residual minimum at ZETA=0.06 (idx 2) but accel minimum at ZETA=0.02
    # (idx 0). The toolhead wins: the residual scores how well the rotor
    # holds its own path, which is the pin's mechanism, not its purpose.
    # On the bench the residual picked LEAD=0 while the accelerometer
    # picked LEAD=600, and 600 was plainly the better machine.
    sc, _gcode, node, path = _setup(
        residuals=[5.0e-3, 2.0e-3, 1.0e-3, 3.0e-3],
        accel_amps=[1.0, 3.0, 4.0, 5.0],
        accel_freq=130.0,
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04,0.06,0.08",
        DWELL="1",
        ACCEL_CHIP="adxl345 tool",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "picked ZETA=0.02" in report
    assert "picked on toolhead accel" in report
    # the residual's dissent is reported, not silently dropped
    assert "would have picked ZETA=0.06" in report
    assert "X_ZETA=0.02" in report


@requires_tomllib
def test_pin_sweep_accel_empty_capture_reports_na():
    # the middle step's accel capture yields no samples -> n/a, never a
    # fake zero; the other steps still report real accel values.
    sc, _gcode, _node, _path = _setup(
        residuals=[5.0e-3, 1.0e-3, 3.0e-3],
        accel_amps=[3.0, None, 2.0],
        accel_freq=130.0,
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04,0.06",
        DWELL="1",
        ACCEL_CHIP="adxl345 tool",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "/ n/a" in report
    assert "3.0 mm/s^2" in report
    assert "2.0 mm/s^2" in report


@requires_tomllib
def test_pin_sweep_all_accel_captures_empty_refuses_residual_fallback():
    # A fitted accelerometer that yields nothing on EVERY step is a broken
    # measurement, not a preference: silently re-picking on the drive-side
    # residual is how LEAD=0 beat the bench-correct 600.
    sc, _gcode, _node, _path = _setup(
        residuals=[5.0e-3, 1.0e-3, 3.0e-3],
        accel_amps=[None, None, None],
        accel_freq=130.0,
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04,0.06",
        DWELL="1",
        ACCEL_CHIP="adxl345 tool",
    )
    with pytest.raises(Exception, match="no staircase step yielded accel"):
        sc.cmd_SERVO_SWEEP_PIN(gcmd)


@requires_tomllib
def test_pin_sweep_writes_per_step_accel_csv_for_the_psd():
    # The scalar amplitude at the tone cannot show whether the pin
    # collapsed the coupled spike or merely added a second one - that needs
    # the spectrum. Keeping the samples is what makes analyze.rs emit
    # step.psd.accel, which the dashboard already renders beside the
    # following-error PSD.
    sc, _gcode, _node, _path = _setup(
        residuals=[2.0e-3, 1.0e-3],
        accel_amps=[3.0, 1.0],
        accel_freq=130.0,
    )
    gcmd = FakeGcmd(
        MODE="X",
        FREQ="130",
        PARAM="ZETA",
        VALUES="0.02,0.04",
        DWELL="1",
        ACCEL_CHIP="adxl345 tool",
    )
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    cap = sc.printer.lookup_object("servo_capture")
    run_dir = os.path.dirname(cap.starts[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        steps = json.load(f)["steps"]
    assert [s["accel"] for s in steps] == [
        "step_zeta0p02_accel.csv",
        "step_zeta0p04_accel.csv",
    ]
    for step in steps:
        path = os.path.join(run_dir, step["accel"])
        with open(path) as f:
            head, first = f.readline().strip(), f.readline().strip()
        # the exact shape analyze.rs:read_accel_csv parses
        assert head == "#time,accel_x,accel_y,accel_z"
        assert len(first.split(",")) == 4


@requires_tomllib
def test_pin_sweep_without_an_accel_chip_records_no_accel_file():
    # No accelerometer means no spectrum to keep - the step must say so
    # rather than naming a file that was never written.
    sc, _gcode, _node, _path = _setup(residuals=[2.0e-3, 1.0e-3])
    gcmd = FakeGcmd(MODE="X", FREQ="130", VALUES="0.02,0.04", DWELL="1")
    sc.cmd_SERVO_SWEEP_PIN(gcmd)
    cap = sc.printer.lookup_object("servo_capture")
    run_dir = os.path.dirname(cap.starts[0][0])
    with open(os.path.join(run_dir, "manifest.json")) as f:
        steps = json.load(f)["steps"]
    assert [s["accel"] for s in steps] == [None, None]
