"""Contracts of CalibrationHost._run_scope - the single lifecycle every
calibration command goes through. These tests exist because the stages used
to be three separate calls (_begin_run / record / analyze) that each command
had to remember; SERVO_COMPARE_PIN shipped forgetting two of them."""

import json
import os
import pathlib
import re
import sys
import tempfile

import pytest
from fakes import FakeConfig, FakeKin, FakeNode, FakeServoCapture, FakeToolhead
from fakes import FakeEngine as _FakeEngine
from fakes import FakeGcmd as _FakeGcmd
from fakes import FakeGcode as _FakeGcode
from fakes import FakePrinter as _FakePrinter
from klippy.extras import servo_axis, servo_calibration

SweepStep = servo_calibration.SweepStep


class FakeGcode(_FakeGcode):
    error = RuntimeError


class FakeGcmd(_FakeGcmd):
    error = RuntimeError

    def get_commandline(self):
        return "FAKE_CMD"


class FakeEngine(_FakeEngine):
    def sdo_read(self, handle, slot, index, subindex):
        return 2, 7


class FakePrinter(_FakePrinter):
    command_error = RuntimeError


def _rail(motor, axis):
    m = servo_axis.ServoMotor.__new__(servo_axis.ServoMotor)
    m.motor_name = motor
    m.node_name = "n"
    m.chain_index = 0
    m.invert_direction = False
    m.rotation_distance = 40.0
    m.encoder_counts_per_rev = 131072
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "axis " + axis
    rail.axis = axis
    rail.motors = [m]
    return rail


VERDICT = {
    "recommended_step": "s0",
    "reason": "clean",
    "flags": [],
}


def make_sc():
    gcode = FakeGcode()
    objs = {
        "gcode": gcode,
        "toolhead": FakeToolhead(
            FakeKin([_rail("motor_a", "x")], kind="corexy")
        ),
        "servo_capture": FakeServoCapture(),
        "motion_engine": FakeEngine(),
        "ethercat_node n": FakeNode(
            name="ethercat_node n", handle=1, slots={"motor_a": 0}
        ),
    }
    sc = servo_calibration.ServoCalibration(FakeConfig(FakePrinter(objs)))
    sc.captures_root = tempfile.mkdtemp()
    sc.servo_cal_binary = sys.executable

    def fake_run(gcmd, argv, timeout):
        gcode.scripts.append(("RUN", argv, timeout))
        if len(argv) >= 3 and argv[1] == "analyze":
            with open(os.path.join(argv[2], "results.json"), "w") as f:
                json.dump({"verdict": VERDICT}, f)

    sc._run = fake_run
    return sc, gcode


def _scope(sc, gcmd):
    return sc._run_scope(gcmd, "tracking", "t", "X", ["motor_a"], {})


def _analyze_runs(gcode):
    return [
        s
        for s in gcode.scripts
        if isinstance(s, tuple)
        and s[0] == "RUN"
        and len(s[1]) >= 2
        and s[1][1] == "analyze"
    ]


def test_scope_raises_when_no_step_recorded():
    sc, gcode = make_sc()
    gcmd = FakeGcmd()
    with pytest.raises(
        RuntimeError, match="without recording a single capture step"
    ):
        with _scope(sc, gcmd):
            pass
    assert sc._active_run is None
    assert _analyze_runs(gcode) == []


def test_scope_auto_analyzes_unanalyzed_run():
    sc, gcode = make_sc()
    gcmd = FakeGcmd()
    with _scope(sc, gcmd) as run:
        run.record_step(SweepStep("s0", {}, []))
        assert run.results is None
    runs = _analyze_runs(gcode)
    assert len(runs) == 1
    assert runs[0][1] == [sys.executable, "analyze", run.run_dir]
    assert run.results == {"verdict": VERDICT}
    assert sc._active_run is None


def test_scope_skips_reanalyze_when_covered():
    sc, gcode = make_sc()
    gcmd = FakeGcmd()
    with _scope(sc, gcmd) as run:
        run.record_step(SweepStep("s0", {}, []))
        results = sc._analyze_and_report(gcmd, run)
    assert results == {"verdict": VERDICT}
    assert len(_analyze_runs(gcode)) == 1


def test_scope_clears_active_run_and_skips_analyze_on_exception():
    sc, gcode = make_sc()
    gcmd = FakeGcmd()
    with pytest.raises(ValueError, match="boom"):
        with _scope(sc, gcmd) as run:
            run.record_step(SweepStep("s0", {}, []))
            raise ValueError("boom")
    assert _analyze_runs(gcode) == []
    assert sc._active_run is None


def test_incremental_analysis_covering_all_steps_counts():
    sc, gcode = make_sc()
    gcmd = FakeGcmd()
    with _scope(sc, gcmd) as run:
        run.record_step(SweepStep("s0", {}, []))
        sc._run_analyze(gcmd, run, incremental=True)
    runs = _analyze_runs(gcode)
    assert len(runs) == 1
    assert runs[0][1][-1] == "--incremental"
    assert run.results == {"verdict": VERDICT}


# Assignment only: reading self._active_run (sweep engine, _on_step_complete
# style None-checks) stays legal, so the lookahead keeps `==` comparisons out.
_ASSIGN_RE = re.compile(r"_active_run\s*=(?!=)")


def test_lifecycle_is_host_private():
    pkg = pathlib.Path(servo_calibration.__file__).parent
    offenders = {}
    for path in sorted(pkg.glob("*.py")):
        if path.name == "host.py":
            continue
        src = path.read_text()
        hits = []
        if "_begin_run(" in src:
            hits.append("_begin_run(")
        if _ASSIGN_RE.search(src):
            hits.append("_active_run assignment")
        if hits:
            offenders[path.name] = hits
    assert offenders == {}, (
        "the run lifecycle belongs to host.py's _run_scope: %s" % (offenders,)
    )
