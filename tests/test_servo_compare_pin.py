import json
import math
import os

import pytest
from fakes import FakeReactor
from test_servo_calibration_awd import (
    FakeGcmd,
    make_calibration,
    requires_tomllib,
    single_drive_rails,
)
from test_servo_sweep_pin import FakeAccelClient, _pinned_profile

GRAVITY_MM_S2 = 9810.0
# The header servo-ident's read_accel_csv skips and the column order it
# parses; a comparison is only charted if its CSVs match it.
ACCEL_CSV_HEADER = "#time,accel_x,accel_y,accel_z"


class FakeChirpChip:
    """Emits one client per started capture whose samples span the linear
    chirp FREQ_START->FREQ_END at HZ_PER_SEC. Each capture is a real
    oscillation: the two horizontal axes share a sine of total amplitude
    ``amp`` (per-value, popped from ``amps``; None -> empty capture) at the
    instantaneous chirp frequency, and the vertical axis carries gravity
    the way a real accelerometer reports it. The timestamps cover the whole
    sweep window, which is what the per-step accel PSD is computed over."""

    def __init__(self, freq_start, freq_end, hz_per_sec, amps=None, fs=1000.0):
        self.freq_start = freq_start
        self.freq_end = freq_end
        self.hz_per_sec = hz_per_sec
        self.amps = None if amps is None else list(amps)
        self.fs = fs
        self._i = 0
        self.clients = []

    def start_internal_client(self):
        if self.amps is None:
            amp = 1.0
        else:
            amp = self.amps[self._i] if self._i < len(self.amps) else None
        self._i += 1
        samples = []
        if amp is not None:
            duration = (self.freq_end - self.freq_start) / self.hz_per_sec
            n = int(duration * self.fs)
            a = amp / math.sqrt(2.0)
            for k in range(n + 1):
                dt = k / self.fs
                phase = (
                    2.0
                    * math.pi
                    * (self.freq_start * dt + self.hz_per_sec * dt * dt / 2.0)
                )
                swing = math.sin(phase)
                samples.append(
                    (100.0 + dt, a * swing, a * swing, GRAVITY_MM_S2)
                )
        client = FakeAccelClient(samples)
        self.clients.append(client)
        return client


def _setup(amps=None, freq_start=100.0, freq_end=104.0, hz_per_sec=5.0):
    chip = FakeChirpChip(freq_start, freq_end, hz_per_sec, amps=amps)
    sc, gcode = make_calibration(
        single_drive_rails(),
        coupled=False,
        reactor=FakeReactor(tick=0.0),
        extra_objs={"adxl345 tool": chip},
    )
    node = sc.printer.lookup_object("ethercat_node xy_drives")
    path = os.path.join(sc.dynamics_dir, "baseline.toml")
    with open(path, "w") as f:
        f.write(_pinned_profile())
    node.dynamics_profile = path
    return sc, gcode, node, path, chip


def _gcmd(**kw):
    base = dict(
        MODE="X",
        PARAM="ZETA",
        VALUES="0.02,0.05",
        FREQ_START="100",
        FREQ_END="104",
        HZ_PER_SEC="5",
        DWELL="0",
        NAME="cmp",
        ACCEL_CHIP="adxl345 tool",
    )
    base.update(kw)
    return FakeGcmd(**base)


def _run_dirs(sc):
    root = sc.captures_root
    return sorted(
        os.path.join(root, d)
        for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d))
    )


def _manifest_at(run_dir):
    with open(os.path.join(run_dir, "manifest.json")) as f:
        return json.load(f)


def _manifest(sc):
    dirs = _run_dirs(sc)
    assert len(dirs) == 1, dirs
    return _manifest_at(dirs[0])


def _steps(run_dir):
    return _manifest_at(run_dir)["steps"]


# ---- the run layout: one ordinary step per swept value -----------------


@requires_tomllib
def test_compare_writes_an_ordinary_run():
    sc, _gcode, _node, path, _chip = _setup(amps=[2.0, 6.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd())
    man = _manifest(sc)
    # Indistinguishable from any other calibration run at the top level:
    # that is what puts it in the dashboard's runs table as one more row.
    assert man["experiment"] == "pin_compare"
    assert man["tag"] == "cmp"
    assert man["axis"] == "X"
    assert man["command"].startswith("FAKE_CMD ")
    assert isinstance(man["created_utc"], str) and man["created_utc"]
    plan = man["stroke_plan"]
    assert plan["param"] == "ZETA"
    assert plan["freq_start"] == 100.0
    assert plan["freq_end"] == 104.0
    assert plan["baseline_profile"] == path
    # ... and indistinguishable at the step level too, which is what makes
    # the standard following-error and accel PSD charts render it: one
    # ordinary step per swept value, named for the value so the chart
    # legends say which zeta produced which trace.
    steps = man["steps"]
    assert [s["name"] for s in steps] == ["zeta0p02", "zeta0p05"]
    assert [s["swept"]["value"] for s in steps] == [0.02, 0.05]
    for step in steps:
        assert step["swept"]["t_end_s"] >= step["swept"]["t_start_s"]
        assert step["applied"] == []


@requires_tomllib
def test_every_sweep_starts_its_own_drive_capture():
    sc, _gcode, _node, _path, _chip = _setup(amps=[2.0, 6.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd())
    run_dir = _run_dirs(sc)[0]
    cap = sc.printer.lookup_object("servo_capture")
    # A .scap per sweep over the drives the mode's frame row excites - the
    # comparison had none at all while it only reduced the accelerometer,
    # which is why the following-error PSD came up empty.
    assert [p for p, _servos in cap.starts] == [
        os.path.join(run_dir, "step_zeta0p02.scap.zst"),
        os.path.join(run_dir, "step_zeta0p05.scap.zst"),
    ]
    assert [servos for _p, servos in cap.starts] == [["motor_a"]] * 2
    # started and stopped around each sweep, never left running across one
    assert cap.events == ["capture_start", "capture_stop"] * 2
    assert [s["capture"] for s in _steps(run_dir)] == [
        "step_zeta0p02.scap.zst",
        "step_zeta0p05.scap.zst",
    ]


@requires_tomllib
def test_every_sweep_writes_the_accel_csv_the_analyzer_parses():
    sc, _gcode, _node, _path, chip = _setup(amps=[2.0, 6.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd())
    run_dir = _run_dirs(sc)[0]
    steps = _steps(run_dir)
    assert [s["accel"] for s in steps] == [
        "step_zeta0p02_accel.csv",
        "step_zeta0p05_accel.csv",
    ]
    for step, client in zip(steps, chip.clients):
        with open(os.path.join(run_dir, step["accel"])) as f:
            lines = f.read().splitlines()
        assert lines[0] == ACCEL_CSV_HEADER
        rows = [[float(v) for v in line.split(",")] for line in lines[1:]]
        assert all(len(r) == 4 for r in rows)
        # Raw samples across the whole chirp, not a reduction and not a
        # settled tail: the PSD of a linear chirp IS the response curve,
        # and a truncated capture is a truncated band.
        assert len(rows) == len(client.samples)
        assert rows[0] == pytest.approx(list(client.samples[0]), abs=1e-6)
        assert rows[-1] == pytest.approx(list(client.samples[-1]), abs=1e-6)


@requires_tomllib
def test_a_sweep_that_captured_nothing_fails_loudly():
    """The accelerometer is the whole point of a comparison, so an empty
    capture stops the run instead of leaving a value-shaped hole in the
    overlay - and the baseline model still comes back."""
    sc, _gcode, node, path, _chip = _setup(amps=[1.0, None])
    with pytest.raises(Exception, match="measured no data"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.02,0.05"))
    assert node.live_dynamics_profile == path
    assert [s["name"] for s in _steps(_run_dirs(sc)[0])] == ["zeta0p02"]


@requires_tomllib
def test_compare_defaults_one_hz_per_sec_and_75_aph():
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0])
    gcmd = _gcmd(VALUES="0.02,0.05")
    del gcmd.params["HZ_PER_SEC"]
    sc.cmd_SERVO_COMPARE_PIN(gcmd)
    plan = _manifest(sc)["stroke_plan"]
    assert plan["hz_per_sec"] == 1.0
    assert plan["accel_per_hz"] == 75.0
    # wire amplitude is the displacement at freq_start
    assert plan["amplitude"] == pytest.approx(75.0 / (4.0 * math.pi**2 * 100.0))


@requires_tomllib
def test_compare_reports_each_sweep_and_the_run_dir():
    sc, _gcode, _node, _path, chip = _setup(amps=[2.0, 6.0])
    gcmd = _gcmd()
    sc.cmd_SERVO_COMPARE_PIN(gcmd)
    report = " ".join(gcmd.responses)
    # Progress names the step (the chart legend) and the sample count that
    # says the capture landed. No peak or ratio: nothing is reduced here
    # any more, and a single-bin amplitude means nothing on a chirp.
    n = len(chip.clients[0].samples)
    assert "step zeta0p02 captured %d accel samples" % (n,) in report
    assert "step zeta0p05 captured %d accel samples" % (n,) in report
    assert _run_dirs(sc)[0] in report


@requires_tomllib
def test_each_invocation_is_its_own_run():
    """The regression: two back-to-back comparisons under one NAME merged
    into a single NAME-keyed manifest that accumulated both sets of sweeps.
    They are two runs now, each holding only what it measured - including
    when both land inside the same run-directory second."""
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0, 1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.02,0.05"))
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.1,0.2"))
    dirs = _run_dirs(sc)
    assert len(dirs) == 2, "one command, one run - never a merge"
    assert sorted([s["swept"]["value"] for s in _steps(d)] for d in dirs) == [
        [0.02, 0.05],
        [0.1, 0.2],
    ]


@requires_tomllib
def test_a_second_run_may_sweep_a_different_param_or_mode():
    """Nothing to match against any more: separate runs, separate settings."""
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0, 1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="ZETA", VALUES="0.02,0.05"))
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="LEAD", VALUES="100,200"))
    runs = [_manifest_at(d) for d in _run_dirs(sc)]
    assert sorted(m["stroke_plan"]["param"] for m in runs) == ["LEAD", "ZETA"]
    assert sorted([s["name"] for s in m["steps"]] for m in runs) == [
        ["lead100", "lead200"],
        ["zeta0p02", "zeta0p05"],
    ]


# ---- streaming, baseline restore, required params ----------------------


@requires_tomllib
def test_compare_streams_each_value_then_restores_baseline():
    sc, _gcode, node, path, _chip = _setup(amps=[1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.02,0.05"))
    engine = sc.printer.lookup_object("motion_engine")
    streamed = [call[7][0] for call in engine.dynamics_calls]
    assert streamed[:2] == [0.02, 0.05]
    assert streamed[-1] == 0.05  # baseline pin_zeta restored last
    assert node.live_dynamics_profile == path


@requires_tomllib
def test_compare_restores_baseline_on_failure_mid_sweep():
    sc, _gcode, node, path, _chip = _setup(amps=[1.0, 1.0, 1.0])
    engine = sc.printer.lookup_object("motion_engine")
    real = engine.set_dynamics_model
    calls = {"n": 0}

    def explode(*args):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("stream blew up")
        return real(*args)

    engine.set_dynamics_model = explode
    with pytest.raises(Exception, match="stream blew up"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.02,0.05,0.1"))
    # the finally block restored the pre-sweep model (pin_zeta, pin_lead)
    assert engine.dynamics_calls[-1][7] == [0.05, 0.0]
    assert engine.dynamics_calls[-1][8] == 100.0
    assert node.live_dynamics_profile == path
    # the crashed run keeps the steps it did finish, and no more
    assert [s["name"] for s in _steps(_run_dirs(sc)[0])] == ["zeta0p02"]


@requires_tomllib
def test_compare_freq_streams_recomputed_compliance_and_pin_mass():
    # PARAM=FREQ sweeps the model f_b: compliance is 1/(2*pi*f)^2 per value
    # and pin_mass follows mass*(1-(f/f_peak)^2) with the coupled peak held
    # at the baseline's implied value (f_b=130 from c=1.5e-6, fraction 0.6
    # -> f_peak = 130/sqrt(0.4) ~= 205.5 Hz).
    sc, _gcode, node, path, _chip = _setup(amps=[1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="FREQ", VALUES="120,125"))
    engine = sc.printer.lookup_object("motion_engine")
    f_b = 1.0 / (2.0 * math.pi * math.sqrt(1.5e-6))
    f_peak = f_b / math.sqrt(1.0 - 0.3 / 0.5)
    for call, f in zip(engine.dynamics_calls, (120.0, 125.0)):
        assert call[5][0] == pytest.approx(1.0 / (2.0 * math.pi * f) ** 2)
        assert call[6][0] == pytest.approx(0.5 * (1.0 - (f / f_peak) ** 2))
        # the swept mode's other pin parameters ride along unchanged
        assert call[7] == [0.05, 0.0]
        assert call[8] == 100.0
    # baseline restored last: original compliance and pin_mass
    assert engine.dynamics_calls[-1][5][0] == pytest.approx(1.5e-6)
    assert engine.dynamics_calls[-1][6][0] == pytest.approx(0.3)
    assert node.live_dynamics_profile == path
    assert [s["name"] for s in _steps(_run_dirs(sc)[0])] == [
        "freq120",
        "freq125",
    ]


@requires_tomllib
def test_compare_freq_at_or_above_the_coupled_peak_rejects_before_motion():
    # f_b = f_peak implies pin_mass = 0: the pin stops existing. The check
    # needs the baseline, but still lands before the first excitation.
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="coupled peak"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="FREQ", VALUES="120,206"))
    assert sc.printer.lookup_object("motion_engine").buzzes == []
    assert _run_dirs(sc) == []


@requires_tomllib
def test_compare_freq_requires_an_actively_pinned_mode():
    # Mode y carries pin_mass 0 in the baseline: there is no pin whose
    # frequency could be swept.
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="actively pinned"):
        sc.cmd_SERVO_COMPARE_PIN(
            _gcmd(MODE="Y", PARAM="FREQ", VALUES="120,125")
        )
    assert sc.printer.lookup_object("motion_engine").buzzes == []
    assert _run_dirs(sc) == []


@requires_tomllib
def test_compare_requires_accel_chip():
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="ACCEL_CHIP"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(ACCEL_CHIP=None))
    # validation lands before the first excitation: nothing buzzed, and no
    # run directory was left behind to look like a measurement
    assert sc.printer.lookup_object("motion_engine").buzzes == []
    assert _run_dirs(sc) == []


@requires_tomllib
def test_compare_requires_freq_bounds_and_param():
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="PARAM"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="BOGUS"))
    with pytest.raises(Exception, match="FREQ_START"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(FREQ_START=None))
    with pytest.raises(Exception, match="FREQ_END"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(FREQ_END=None))
    assert sc.printer.lookup_object("motion_engine").buzzes == []


@requires_tomllib
def test_compare_analyzes_its_own_run_like_every_other_command():
    # Without this the dashboard gets a results-less run and the operator
    # has to press analyze by hand — every other calibration command
    # analyzes before it returns.
    sc, gcode, _node, _path, _chip = _setup()
    sc.cmd_SERVO_COMPARE_PIN(_gcmd())
    run_dir = _run_dirs(sc)[0]
    analyzed = [
        argv
        for kind, argv, _t in gcode.scripts
        if kind == "RUN" and len(argv) >= 3 and argv[1] == "analyze"
    ]
    assert [argv[2] for argv in analyzed] == [run_dir]
    assert os.path.exists(os.path.join(run_dir, "results.json"))
