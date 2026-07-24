"""Unit tests for the servo-capture-prune core logic.

The prune tool is a shebang script without a ``.py`` extension (thin CLI over
an importable pure-python core); load it by path.
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import os
import time
from pathlib import Path

import pytest

_SCRIPT = (
    Path(__file__).resolve().parent.parent / "scripts" / "servo-capture-prune"
)
_loader = importlib.machinery.SourceFileLoader(
    "servo_capture_prune", str(_SCRIPT)
)
_spec = importlib.util.spec_from_loader("servo_capture_prune", _loader)
prune = importlib.util.module_from_spec(_spec)
import sys

sys.modules["servo_capture_prune"] = prune
_loader.exec_module(prune)

HOUR = 3600.0
GIB = 1024**3


def _mkrun(root: Path, name: str, *, age_hours: float, files: dict):
    """Create a run dir with files; set every file mtime to now-age_hours."""
    run = root / name
    run.mkdir(parents=True)
    mtime = time.time() - age_hours * HOUR
    for fname, size in files.items():
        p = run / fname
        p.write_bytes(b"\0" * size)
        os.utime(p, (mtime, mtime))
    os.utime(run, (mtime, mtime))
    return run


def _fake_compress(scap: Path, dry_run: bool) -> int:
    """Deterministic stand-in for zstd: 10x shrink, .zst sibling."""
    before = scap.stat().st_size
    after = max(1, before // 10)
    if not dry_run:
        dest = scap.with_name(scap.name + ".zst")
        dest.write_bytes(b"\0" * after)
        scap.unlink()
    return after


# --- compress-age selection -------------------------------------------------


def test_cold_run_selection_by_age(tmp_path):
    now = time.time()
    _mkrun(tmp_path, "hot", age_hours=1, files={"a.scap": 100})
    _mkrun(tmp_path, "cold", age_hours=72, files={"a.scap": 100})
    runs = prune.scan_runs(tmp_path)
    cold = prune.select_cold_runs(runs, 48 * HOUR, now)
    assert {r.path.name for r in cold} == {"cold"}


def test_compress_only_scap_and_skips_zst(tmp_path):
    _mkrun(
        tmp_path,
        "cold",
        age_hours=72,
        files={
            "a.scap": 1000,
            "b.scap.zst": 50,
            "manifest.json": 20,
            "results.json": 20,
        },
    )
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=100 * GIB,
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=24 * HOUR,
        now=time.time(),
        dry_run=False,
        compress_fn=_fake_compress,
    )
    assert len(plan.compress) == 1
    assert plan.compress[0].scap.name == "a.scap"
    # analysis artifacts stay readable
    assert (tmp_path / "cold" / "manifest.json").exists()
    assert (tmp_path / "cold" / "b.scap.zst").exists()
    assert (tmp_path / "cold" / "a.scap.zst").exists()
    assert not (tmp_path / "cold" / "a.scap").exists()


# --- LRU order by newest-file mtime -----------------------------------------


def test_deletion_order_oldest_first(tmp_path):
    _mkrun(tmp_path, "mid", age_hours=100, files={"a.scap": 10})
    _mkrun(tmp_path, "old", age_hours=200, files={"a.scap": 10})
    _mkrun(tmp_path, "new", age_hours=50, files={"a.scap": 10})
    runs = prune.scan_runs(tmp_path)
    order = [r.path.name for r in prune.deletion_order(runs)]
    assert order == ["old", "mid", "new"]


def test_lru_uses_newest_file_mtime(tmp_path):
    # 'a' dir created old, but touched recently via one fresh file.
    a = _mkrun(tmp_path, "a", age_hours=200, files={"a.scap": 10})
    fresh = a / "results.json"
    fresh.write_bytes(b"x")
    recent = time.time() - 1 * HOUR
    os.utime(fresh, (recent, recent))
    _mkrun(tmp_path, "b", age_hours=100, files={"a.scap": 10})
    runs = prune.scan_runs(tmp_path)
    order = [r.path.name for r in prune.deletion_order(runs)]
    assert order == ["b", "a"]  # a is now "newer" than b


# --- pin_compare last -------------------------------------------------------


def test_pin_compare_deleted_last(tmp_path):
    _mkrun(tmp_path, "run_new", age_hours=100, files={"a.scap": 10})
    _mkrun(
        tmp_path,
        "pin_compare/very_old",
        age_hours=500,
        files={"manifest.json": 10},
    )
    runs = prune.scan_runs(tmp_path)
    order = prune.deletion_order(runs)
    # pin_compare is oldest but must come last
    assert order[-1].is_pin_compare
    assert order[0].path.name == "run_new"


def test_budget_prunes_ordinary_before_pin_compare(tmp_path):
    _mkrun(tmp_path, "old_run", age_hours=10, files={"a.scap": 600})
    _mkrun(tmp_path, "new_run", age_hours=10, files={"a.scap": 600})
    _mkrun(
        tmp_path,
        "pin_compare/pc",
        age_hours=500,
        files={"manifest.json": 600},
    )
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=1000,  # bytes; total is 1800
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=1 * HOUR,
        now=time.time(),
        dry_run=True,
        compress_fn=_fake_compress,
    )
    # Need to drop 1 dir; ordinary run chosen, pin_compare untouched
    assert not plan.refused
    deleted = [r.path.name for r in plan.delete]
    assert "pc" not in deleted
    assert deleted[0] in {"old_run", "new_run"}


# --- min-keep refusal -------------------------------------------------------


def test_min_keep_refusal_deletes_nothing(tmp_path):
    # Everything is fresh (within min-keep) but over budget.
    _mkrun(tmp_path, "r1", age_hours=2, files={"a.scap": 900})
    _mkrun(tmp_path, "r2", age_hours=2, files={"a.scap": 900})
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=1000,
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=24 * HOUR,
        now=time.time(),
        dry_run=False,
        compress_fn=_fake_compress,
    )
    assert plan.refused
    assert plan.delete == []
    # nothing deleted
    assert (tmp_path / "r1").exists()
    assert (tmp_path / "r2").exists()


def test_main_returns_nonzero_on_refusal(tmp_path):
    _mkrun(tmp_path, "r1", age_hours=2, files={"a.scap": 900})
    _mkrun(tmp_path, "r2", age_hours=2, files={"a.scap": 900})
    rc = prune.main(
        [
            "--root",
            str(tmp_path),
            "--budget-gib",
            str(1000 / GIB),
            "--min-keep-hours",
            "24",
            "--cold-age-hours",
            "48",
            "--dry-run",
        ]
    )
    assert rc == 1
    assert (tmp_path / "r1").exists()


# --- dry-run does not modify ------------------------------------------------


def test_dry_run_modifies_nothing(tmp_path):
    _mkrun(tmp_path, "cold", age_hours=72, files={"a.scap": 5000})
    _mkrun(tmp_path, "old", age_hours=100, files={"a.scap": 5000})
    before = {p: p.stat().st_mtime for p in tmp_path.rglob("*")}
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=1000,
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=1 * HOUR,
        now=time.time(),
        dry_run=True,
        compress_fn=_fake_compress,
    )
    # plan proposes work but filesystem is untouched
    assert plan.compress
    after = {p: p.stat().st_mtime for p in tmp_path.rglob("*")}
    assert before == after
    assert (tmp_path / "cold" / "a.scap").exists()
    assert not (tmp_path / "cold" / "a.scap.zst").exists()


def test_budget_deletes_when_applied(tmp_path):
    _mkrun(tmp_path, "old_run", age_hours=10, files={"a.scap": 900})
    _mkrun(tmp_path, "new_run", age_hours=5, files={"a.scap": 900})
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=1000,
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=1 * HOUR,
        now=time.time(),
        dry_run=False,
        compress_fn=_fake_compress,
    )
    prune.apply_plan(plan)
    assert not (tmp_path / "old_run").exists()
    assert (tmp_path / "new_run").exists()


def test_real_zstd_roundtrip(tmp_path):
    if prune.shutil.which("zstd") is None:
        pytest.skip("zstd not installed")
    run = _mkrun(tmp_path, "cold", age_hours=72, files={})
    payload = run / "a.scap"
    payload.write_bytes(b"servo capture payload " * 1000)
    mtime = time.time() - 72 * HOUR
    os.utime(payload, (mtime, mtime))
    os.utime(run, (mtime, mtime))
    plan = prune.build_plan(
        root=tmp_path,
        budget_bytes=100 * GIB,
        cold_age_seconds=48 * HOUR,
        min_keep_seconds=24 * HOUR,
        now=time.time(),
        dry_run=False,
    )
    assert (run / "a.scap.zst").exists()
    assert not (run / "a.scap").exists()
    assert plan.compress[0].saved_bytes > 0
