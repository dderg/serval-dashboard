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


class FakeChirpChip:
    """Emits one client per started capture whose samples span the linear
    chirp FREQ_START->FREQ_END at HZ_PER_SEC. Each sample's three axes carry
    an equal share of a constant vector-magnitude accel (per-value amplitude
    popped from ``amps``; None -> empty capture). The timestamps map, via the
    reducer's f = freq_start + hz_per_sec*(t - t0), across the whole band."""

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
            a = amp / math.sqrt(3.0)
            for k in range(n + 1):
                t = 100.0 + k / self.fs
                samples.append((t, a, a, a))
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


def _manifest(sc, name="cmp"):
    with open(sc._compare_manifest_path(name)) as f:
        return json.load(f)


# ---- reduction / normalization math -----------------------------------


def test_chirp_reduction_bins_and_normalizes():
    sc, *_ = _setup()
    # freq_start=100, hz_per_sec=1 -> f = 100 + (t - t0). t0 is the first
    # sample time. bin idx = int(f - 100): samples at f in [100,101) share
    # bin 0 (center 100.5), [101,102) share bin 1 (center 101.5). A sample
    # past freq_end is dropped, never a fake tail bin.
    samples = [
        (0.0, 3.0, 4.0, 0.0),  # f=100.0 -> bin0, mag 5
        (0.5, 0.0, 6.0, 8.0),  # f=100.5 -> bin0, mag 10
        (1.2, 1.0, 2.0, 2.0),  # f=101.2 -> bin1, mag 3
        (5.0, 9.0, 9.0, 9.0),  # f=105.0 > freq_end=103 -> dropped
    ]
    curve, accel, ratio = sc._chirp_accel_curve(
        samples, 100.0, 103.0, 1.0, 75.0
    )
    assert curve == [100.5, 101.5]
    # per-bin mean of the 3-axis vector magnitude
    assert accel == pytest.approx([7.5, 3.0])
    # response ratio divides by the commanded accel ApH * f (the chirp is
    # constant velocity-amplitude, so commanded accel grows linearly in f)
    for f_c, a, r in zip(curve, accel, ratio):
        assert r == pytest.approx(a / (75.0 * f_c))


def test_chirp_reduction_empty_capture_is_empty_not_zero():
    sc, *_ = _setup()
    assert sc._chirp_accel_curve([], 100.0, 104.0, 5.0, 75.0) == ([], [], [])


# ---- manifest schema & append semantics --------------------------------


@requires_tomllib
def test_compare_writes_manifest_per_contract():
    sc, _gcode, _node, path, _chip = _setup(amps=[2.0, 6.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd())
    man = _manifest(sc)
    assert man["name"] == "cmp"
    assert man["mode"] == "x"
    assert man["param"] == "ZETA"
    assert man["freq_start"] == 100.0
    assert man["freq_end"] == 104.0
    assert man["baseline_profile"] == path
    assert isinstance(man["created_utc"], str) and man["created_utc"]
    assert [s["value"] for s in man["sweeps"]] == [0.02, 0.05]
    for s, amp in zip(man["sweeps"], (2.0, 6.0)):
        assert s["hz_per_sec"] == 5.0
        assert s["accel_per_hz"] == 75.0
        # wire amplitude is the displacement at freq_start
        assert s["amplitude_mm"] == pytest.approx(
            75.0 / (4.0 * math.pi**2 * 100.0)
        )
        n = len(s["curve_hz"])
        assert n > 0
        assert len(s["accel_mm_s2"]) == n
        assert len(s["response_ratio"]) == n
        # constant-magnitude synthetic capture -> every bin equals amp
        assert s["accel_mm_s2"] == pytest.approx([amp] * n)
        for f_c, a, r in zip(
            s["curve_hz"], s["accel_mm_s2"], s["response_ratio"]
        ):
            assert r == pytest.approx(a / (s["accel_per_hz"] * f_c))


@requires_tomllib
def test_compare_defaults_one_hz_per_sec_and_75_aph():
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0])
    gcmd = _gcmd(VALUES="0.02,0.05")
    del gcmd.params["HZ_PER_SEC"]
    sc.cmd_SERVO_COMPARE_PIN(gcmd)
    man = _manifest(sc)
    for s in man["sweeps"]:
        assert s["hz_per_sec"] == 1.0
        assert s["accel_per_hz"] == 75.0


@requires_tomllib
def test_compare_reports_peak_and_manifest_location():
    sc, _gcode, _node, _path, _chip = _setup(amps=[2.0, 6.0])
    gcmd = _gcmd()
    sc.cmd_SERVO_COMPARE_PIN(gcmd)
    report = " ".join(gcmd.responses)
    assert "peak response" in report
    assert sc._compare_manifest_path("cmp") in report


@requires_tomllib
def test_compare_appends_same_name():
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0, 1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.02,0.05"))
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(VALUES="0.1,0.2"))
    man = _manifest(sc)
    assert [s["value"] for s in man["sweeps"]] == [0.02, 0.05, 0.1, 0.2]


@requires_tomllib
def test_compare_append_param_mismatch_errors():
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0, 1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="ZETA", VALUES="0.02,0.05"))
    with pytest.raises(Exception, match="cannot append"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="LEAD", VALUES="100,200"))


@requires_tomllib
def test_compare_append_mode_mismatch_errors():
    sc, _gcode, _node, _path, _chip = _setup(amps=[1.0, 1.0, 1.0, 1.0])
    sc.cmd_SERVO_COMPARE_PIN(_gcmd(MODE="X"))
    with pytest.raises(Exception, match="cannot append"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(MODE="Y"))


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
    # nothing persisted on a failed run
    assert not os.path.exists(sc._compare_manifest_path("cmp"))


@requires_tomllib
def test_compare_requires_accel_chip():
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="ACCEL_CHIP"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(ACCEL_CHIP=None))


@requires_tomllib
def test_compare_requires_freq_bounds_and_param():
    sc, _gcode, _node, _path, _chip = _setup()
    with pytest.raises(Exception, match="PARAM"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(PARAM="BOGUS"))
    with pytest.raises(Exception, match="FREQ_START"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(FREQ_START=None))
    with pytest.raises(Exception, match="FREQ_END"):
        sc.cmd_SERVO_COMPARE_PIN(_gcmd(FREQ_END=None))
