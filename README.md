# serval-dashboard

Servo calibration and tuning toolchain for Serval (Kalico) — the `servo-cal`
analyzer + bench dashboard, the klippy calibration/tuning extras, and the
bench service. Installs like a Klipper plugin: symlinked python modules,
one binary, one systemd unit.

## Install / update

```sh
cd ~/serval-dashboard
./install.sh            # symlinks extras into ~/klipper, builds servo-cal, installs the service
```

Re-running `install.sh` updates everything.

## Layout

```
install.sh              install/update entry point (idempotent)
Cargo.toml              cargo workspace (member: servo-ident)
servo-ident/            rust crate: analyze/fit engine, servo-cal serve, web/ SPA
klippy_extras/          python modules symlinked into klippy/extras
  servo_calibration/    SERVO_* calibration commands
  servo_tuning.py       SERVO_TUNE / SERVO_DUMP_TUNING
  servo_strokes.py      stroke planning shared by calibration commands
  servo_strain_tune.py  strain map measurement/build/fit (runtime comp stays in kalico)
fixtures/servo_captures/  committed .scap.gz fixtures (demo + tests)
tests/                  pytest suite (needs a kalico checkout, see tests/conftest.py)
service/                servo-cal.service + launcher, servo-capture-prune.service/.timer, installed by install.sh
scripts/                servo-capture-prune retention tool (compress cold + budget prune; see docs/servo-calibration.md)
docs/                   dashboard / calibration / tuning docs
```

The runtime seam: kalico core keeps `servo_axis`, `servo_param`,
`servo_capture`, `servo_sync`, `servo_diff_trim`, and the slim
`servo_strain_comp` runtime (map apply/clear). This repo only imports
downward into kalico core; kalico never imports this repo.
