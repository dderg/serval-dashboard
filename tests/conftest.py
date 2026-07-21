from __future__ import annotations

import os
import sys
from pathlib import Path

DASHBOARD_ROOT = Path(__file__).resolve().parent.parent


def _kalico_root() -> Path:
    candidates = [
        Path(os.environ["KALICO_ROOT"]).expanduser()
        if "KALICO_ROOT" in os.environ
        else None,
        Path("~/klipper").expanduser(),
        DASHBOARD_ROOT.parent / "kalico",
    ]
    for candidate in candidates:
        if candidate is not None and (candidate / "klippy").is_dir():
            return candidate.resolve()
    raise RuntimeError(
        "Kalico checkout not found: set KALICO_ROOT or provide ~/klipper or "
        "../kalico with a klippy directory"
    )


KALICO_ROOT = _kalico_root()
sys.path.insert(0, str(KALICO_ROOT))
sys.path.insert(0, str(KALICO_ROOT / "test"))

import klippy.extras  # noqa: E402

klippy.extras.__path__.insert(0, str(DASHBOARD_ROOT / "klippy_extras"))
