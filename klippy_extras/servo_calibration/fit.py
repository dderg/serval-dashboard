from __future__ import annotations

import json
import math
import os
import shutil
import time

try:
    import tomllib
except ImportError:
    tomllib = None

from collections.abc import Mapping
from typing import Any

from ... import structured_log
from .. import servo_axis, servo_strokes
from .dynamics import (
    DYNAMICS_TERM_KEYS,
    PIN_LEAD_US_MAX,
    PIN_ZETA_MAX,
    TUNE_MASS_FLOOR_FRACTION,
    TUNE_ZERO_FLOOR_STEPS,
    _copy_dynamics,
    _equal_or_opposite_columns,
    add_dynamics_direction_split,
    discover_dynamics_pairs,
    dynamics_torque_changes,
    parse_dynamics_profile,
    render_fit_dynamics_toml,
    send_dynamics_model,
    send_ff_lead,
)
from .measure import MeasureCommands
from .search import RmsLineSearch
from .search import Z as ACCEPT_Z
from .sweep import ExperimentRun, SweepStep


class DynamicsFitCommands(MeasureCommands):
    cmd_SERVO_FIT_DYNAMICS_help = (
        "Identify axis dynamics for torque feedforward. On coupled_xy this "
        "is an iterative closed-loop identification: it runs the "
        "TEST_SPEED-style XY pattern (always - there is no PATTERN option), "
        "fits mass/viscous/coulomb (all three always regressed - the "
        "friction columns keep the mass estimate unbiased), streams the "
        "APPLIED model into the running endpoint, and re-captures with the "
        "feedforward active - "
        "with FF in the loop the drives track the command, so regressing "
        "measured torque against commanded kinematics loses its bias - "
        "until the parameters move less than TOL (torque-weighted, at the "
        "excitation ceiling) between rounds. It then re-identifies once at "
        "MAX_ACCEL: a converged model that shifts more than DRIFT there is "
        "a fit artifact, not physics, and the command aborts with the "
        "numbers. No SPEEDS matrix: give the calibration envelope as "
        "MAX_ACCEL/MAX_SPEED limits (e.g. capped below ringing; defaults "
        "are the config grid maxima) - convergence rounds run at half "
        "MAX_ACCEL, speeds at half and full MAX_SPEED. ACCELS=<comma list> "
        "runs an identify-only sweep instead: one capture + fit per accel "
        "under whatever model is currently live (nothing is streamed or "
        "applied), reporting mass per accel and the torque-weighted change "
        "between neighbours - the m(accel) curve that says whether the "
        "model extrapolates. The live model is "
        "restored to the configured dynamics_profile afterwards (also on "
        "failure); without one the last fitted model stays live until "
        "RESTART. TERMS picks what the applied/written model keeps "
        "(default MASS: with velocity_ff on, the speed-loop integrator "
        "already supplies friction torque at all but reversal transients, "
        "and a wrong friction FF is worse than none; fitted-but-dropped "
        "values are reported and recorded as fitted_* keys so enabling "
        "TERMS=MASS,COULOMB later is a data-driven call). Writes a "
        "timestamped node-level profile from the "
        "MAX_ACCEL verification fit. On non-coupled kinematics the "
        "single-shot per-axis grid fit remains (a per-motor candidate "
        "cannot be streamed into a multi-drive node), with params as "
        "SERVO_MEASURE_INERTIA plus DRIVE. Optional TORQUE_NM + "
        "INERTIA_KGM2 add the C00.06 recommendation. Params TERMS (MASS) "
        "MAX_ACCEL "
        "MAX_SPEED TOL (0.05) DRIFT (0.15) MAX_ROUNDS (4) ITERATIONS "
        "DWELL_MS BOUND SMALL_SIZE NAME SERVOS TORQUE_NM INERTIA_KGM2"
    )

    def _dynamics_node(self, gcmd: Any, servos: list[str]) -> Any:
        nodes = {}
        for servo in servos:
            node, _slot = self._resolve_node_slot(servo)
            nodes[node.name] = node
        if len(nodes) != 1:
            raise gcmd.error(
                "servos %s span multiple ethercat nodes (%s) - the dynamics "
                "model is per-node" % (servos, sorted(nodes))
            )
        return nodes.popitem()[1]

    def _load_baseline_dynamics(
        self, gcmd: Any, node: Any
    ) -> tuple[str, dict[str, Any]]:
        explicit = gcmd.get("PROFILE", None)
        profile_path = explicit or node.get_live_dynamics_profile()
        if profile_path is None:
            raise gcmd.error(
                "no baseline dynamics profile - set dynamics_profile on "
                "[ethercat_node %s] or pass PROFILE= (per-motor profiles "
                "are not supported)" % (node.name,)
            )
        if explicit is None and profile_path != node.get_dynamics_profile():
            gcmd.respond_info(
                "baseline: %s (model left live by the previous tune, not "
                "the configured dynamics_profile)" % (profile_path,)
            )
        profile_path = os.path.expanduser(profile_path)
        try:
            with open(profile_path) as f:
                baseline = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "failed to load dynamics profile %s: %s" % (profile_path, e)
            )
        if len(baseline["axes"]) != node.get_drive_count():
            raise gcmd.error(
                "profile %s describes %d axes but node %s has %d drives"
                % (
                    profile_path,
                    len(baseline["axes"]),
                    node.name,
                    node.get_drive_count(),
                )
            )
        for profile_slot, motor in enumerate(baseline["axes"]):
            node_slot = node.get_slot_for_motor(motor)
            if node_slot is None:
                raise gcmd.error(
                    "profile %s axis %r is not a motor on node %s"
                    % (profile_path, motor, node.name)
                )
            if node_slot != profile_slot:
                raise gcmd.error(
                    "profile %s axis %r is at slot %d, but node %s maps it "
                    "to slot %d"
                    % (profile_path, motor, profile_slot, node.name, node_slot)
                )
        return profile_path, baseline

    def _direction_split_baseline(
        self, gcmd: Any, kin: Any, baseline: dict[str, Any]
    ) -> dict[str, Any]:
        if baseline.get("pairs"):
            return baseline
        pair_slots = None
        if kin.coupled_xy():
            layout = servo_axis.corexy_fit_layout(gcmd, kin)
            pair_slots = layout["pairs"]
        derived = _copy_dynamics(baseline)
        if pair_slots is not None:
            pairs = [part.split(",") for part in pair_slots.split(";") if part]
            axis_index = {name: i for i, name in enumerate(baseline["axes"])}
            columns = [list(col) for col in zip(*baseline["frame"])]
            claimed: set[str] = set()
            for slots in pairs:
                if len(slots) != 2:
                    raise gcmd.error(
                        "kinematic AWD pair must contain exactly two slots "
                        "(got %s)" % (slots,)
                    )
                if slots[0] == slots[1]:
                    raise gcmd.error(
                        "kinematic AWD pair slots must be distinct (got %s)"
                        % (slots,)
                    )
                overlap = claimed.intersection(slots)
                if overlap:
                    raise gcmd.error(
                        "kinematic AWD pairs overlap at slots %s"
                        % (sorted(overlap),)
                    )
                if any(s not in axis_index for s in slots):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not match profile axes %s"
                        % (slots, baseline["axes"])
                    )
                claimed.update(slots)
                first, second = (axis_index[s] for s in slots)
                if not _equal_or_opposite_columns(
                    columns[first], columns[second]
                ):
                    raise gcmd.error(
                        "kinematic AWD pair %s does not have equal parallel "
                        "or antiparallel frame columns" % (slots,)
                    )
            derived["pairs"] = [
                {"slots": slots, "direction_split": 0.0} for slots in pairs
            ]
        else:
            try:
                derived["pairs"] = discover_dynamics_pairs(baseline)
            except ValueError as e:
                raise gcmd.error("cannot derive dynamics pairs: %s" % (e,))
        if not derived["pairs"]:
            raise gcmd.error(
                "TERM=DIRECTION_SPLIT found no explicit [[pair]] tables, "
                "kinematic AWD pairs, or groups of exactly two equal "
                "parallel/antiparallel frame columns"
            )
        return derived

    def _corexy_frame(
        self, gcmd: Any, kin: Any
    ) -> tuple[list[str], list[str], list[list[float]]]:
        rails = servo_strokes.axis_rails(gcmd, kin, "X")
        slots: list[tuple[str, int, float, int]] = []
        for belt_index, rail in enumerate(rails):
            motors = servo_axis.rail_motors_in_slot_order(rail)
            drives = len(motors)
            for m in motors:
                sign = -1.0 if m.get_invert_direction() else 1.0
                slots.append((m.get_motor_name(), belt_index, sign, drives))
        axes = [name for name, _b, _s, _d in slots]
        frame_x = [sign / (2.0 * drives) for _n, _b, sign, drives in slots]
        frame_y = [
            (sign if belt == 0 else -sign) / (2.0 * drives)
            for _n, belt, sign, drives in slots
        ]
        return axes, ["x", "y"], [frame_x, frame_y]

    def _fit_plan(self, gcmd: Any) -> dict[str, Any]:
        kin = self._kin()
        if kin.coupled_xy():
            layout = servo_axis.corexy_fit_layout(gcmd, kin)
            servo_strokes.check_servos_override(gcmd, layout)
            axes, modes, frame = self._corexy_frame(gcmd, kin)
            return {
                "corexy": True,
                "servos": layout["servos"],
                "axes": axes,
                "modes": modes,
                "frame": frame,
                "axis": "X",
                "rails": servo_strokes.axis_rails(gcmd, kin, "X"),
            }
        self._reject_corexy_only_params(gcmd)
        axis = gcmd.get("AXIS", "X").upper()
        drive = servo_strokes.scalar_fit_drive(gcmd, kin)
        servos = servo_strokes.axis_servos(gcmd, kin, axis)
        axes = [drive if drive is not None else servos[0]]
        return {
            "corexy": False,
            "servos": servos,
            "axes": axes,
            "modes": list(axes),
            "frame": [[1.0]],
            "axis": axis,
            "rails": None,
        }

    def _rotation_distance(self, gcmd: Any, servos: list[str]) -> float:
        distances = {
            self._resolve_motor(s).get_rotation_distance() for s in servos
        }
        if len(distances) != 1:
            raise gcmd.error(
                "drives disagree on rotation_distance (%s); cannot fit"
                % (sorted(distances),)
            )
        return distances.pop()

    def _fit_argv_for(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        scap: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
        response: str | None = None,
    ) -> list[str]:
        argv = [
            self._servo_cal(gcmd),
            "fit",
            "--capture",
            scap,
            "--frame",
            ";".join(
                ",".join("%g" % (f,) for f in row) for row in plan["frame"]
            ),
            "--modes",
            ",".join(plan["modes"]),
            "--axes",
            ",".join(plan["axes"]),
            "--out",
            out_path,
            "--rotation-distance-mm",
            "%g" % (self._rotation_distance(gcmd, plan["servos"]),),
        ]
        if torque is not None:
            argv += [
                "--rated-torque-nm",
                "%g" % (torque,),
                "--rotor-inertia-kgm2",
                "%g" % (inertia,),
            ]
        if response is not None:
            argv += ["--response", response]
        return argv

    def _run_fit(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if self._kin().coupled_xy():
            if gcmd.get("ACCELS", None) is not None:
                return self._run_fit_sweep(gcmd, name, torque, inertia)
            return self._run_fit_iterative(gcmd, name, torque, inertia)
        return self._run_fit_grid(gcmd, name, torque, inertia)

    def _run_fit_grid(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if gcmd.get_int("PATTERN", 0):
            self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        run = self._begin_run(
            gcmd,
            "inertia_grid",
            name,
            plan["axis"],
            plan["servos"],
            self._grid_stroke_plan(gcmd),
            plan["rails"],
        )
        try:
            self._measure_inertia(gcmd, name)
            run.record_step(SweepStep(name, {}, []))
            out_path = self._dynamics_out_path(gcmd, run, name)
            argv = self._fit_argv_for(
                gcmd, plan, run.step_scap(name), out_path, torque, inertia
            )
            text = self._run(gcmd, argv, 120.0)
            gcmd.respond_info(
                "dynamics profile: %s | run %s" % (out_path, run.run_dir)
            )
        finally:
            self._active_run = None
        return run, text, out_path

    def _reject_fit_grid_params(self, gcmd: Any) -> None:
        stale = [
            p for p in ("SPEEDS", "PATTERN") if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: the iterative fit has no excitation matrix and always "
                "runs the XY pattern - give the calibration envelope as "
                "MAX_ACCEL/MAX_SPEED limits, or ACCELS=<comma list> for an "
                "identify-only sweep" % (", ".join(stale),)
            )

    def _validate_fit_slots(
        self, gcmd: Any, node: Any, profile: dict[str, Any]
    ) -> None:
        for slot, motor in enumerate(profile["axes"]):
            if node.get_slot_for_motor(motor) != slot:
                raise gcmd.error(
                    "fitted profile axis %r is at slot %d but node %s maps "
                    "it to %s - cannot stream the candidate model"
                    % (motor, slot, node.name, node.get_slot_for_motor(motor))
                )

    def _fit_round(
        self,
        gcmd: Any,
        plan: dict[str, Any],
        run: ExperimentRun,
        step: str,
        out_path: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[dict[str, Any], str]:
        argv = self._fit_argv_for(
            gcmd, plan, run.step_scap(step), out_path, torque, inertia
        )
        text = self._run(gcmd, argv, 120.0)
        try:
            with open(out_path) as f:
                fitted = parse_dynamics_profile(f.read())
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit for step %s produced an unusable profile "
                "%s: %s" % (step, out_path, e)
            )
        return fitted, text

    def _dynamics_params_line(self, profile: dict[str, Any]) -> str:
        return " | ".join(
            "%s mass %.5g viscous %.5g coulomb %.5g"
            % (
                mode,
                profile["mass"][k],
                profile["viscous"][k],
                profile["coulomb"][k],
            )
            for k, mode in enumerate(profile["modes"])
        )

    def _run_fit_iterative(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_fit_grid_params(gcmd)
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        node = self._dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        restore = None
        restore_path = None
        fit_written_path = None
        if node.get_live_dynamics_profile() is not None:
            restore_path, restore = self._load_baseline_dynamics(gcmd, node)
        baseline_lead_us = (
            restore.get("ff_lead_us", 0.0) if restore is not None else 0.0
        )
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        tol = gcmd.get_float("TOL", 0.05, above=0.0)
        drift = gcmd.get_float("DRIFT", 0.15, above=0.0)
        max_rounds = gcmd.get_int("MAX_ROUNDS", 4, minval=2)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS").split(",")
            if t.strip()
        ]
        if (
            not terms
            or any(t not in DYNAMICS_TERM_KEYS for t in terms)
            or "MASS" not in terms
        ):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB and include MASS (got %r)" % (gcmd.get("TERMS", ""),)
            )
        dropped = [
            key for term, key in DYNAMICS_TERM_KEYS.items() if term not in terms
        ]

        def applied_model(full: dict[str, Any]) -> dict[str, Any]:
            trimmed = _copy_dynamics(full)
            for key in dropped:
                trimmed[key] = [0.0] * len(full[key])
            return trimmed

        def round_line(full: dict[str, Any], trimmed: dict[str, Any]) -> str:
            line = self._dynamics_params_line(trimmed)
            if dropped:
                line += " | fitted but not applied: " + ", ".join(
                    "%s [%s]"
                    % (key, ", ".join("%.5g" % (v,) for v in full[key]))
                    for key in dropped
                )
            return line

        converge_accel = max_accel / 2.0
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "converge_accel": converge_accel,
            "speeds": speeds,
            "tol": tol,
            "drift": drift,
            "max_rounds": max_rounds,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_fit",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str, accel: float) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    accel,
                    iterations,
                    dwell,
                )
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": accel}, []))

        def torque_changes(
            prev: dict[str, Any],
            new: dict[str, Any],
            accel: float,
        ) -> list[float]:
            try:
                return dynamics_torque_changes(prev, new, accel, max_speed)
            except ValueError as e:
                raise gcmd.error(str(e))

        applied = False
        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd,
                points,
                start_x,
                start_y,
                [converge_accel, max_accel],
                speeds,
            )
            prev = None
            fitted = None
            converged = False
            last_change = None
            rounds_run = 0
            for round_i in range(max_rounds):
                step = "fit_r%d" % (round_i,)
                capture_round(step, converge_accel)
                fitted_full, _text = self._fit_round(
                    gcmd,
                    plan,
                    run,
                    step,
                    os.path.join(run.run_dir, "dynamics_%s.toml" % (step,)),
                    None,
                    None,
                )
                fitted = applied_model(fitted_full)
                rounds_run = round_i + 1
                if round_i == 0:
                    self._validate_fit_slots(gcmd, node, fitted)
                send_dynamics_model(engine, handle, fitted)
                applied = True
                if prev is None:
                    gcmd.respond_info(
                        "round %d: %s (feedforward now live for the next "
                        "round)" % (round_i, round_line(fitted_full, fitted))
                    )
                else:
                    last_change = max(
                        torque_changes(prev, fitted, converge_accel)
                    )
                    gcmd.respond_info(
                        "round %d: %s | torque-weighted change %.1f%% "
                        "(TOL %.1f%%)"
                        % (
                            round_i,
                            round_line(fitted_full, fitted),
                            100.0 * last_change,
                            100.0 * tol,
                        )
                    )
                    if last_change <= tol:
                        converged = True
                        break
                prev = fitted
            if not converged:
                raise gcmd.error(
                    "dynamics fit did not converge in %d rounds at accel "
                    "%.0f (last torque-weighted change %.1f%% > TOL %.1f%%) "
                    "- the identification is not settling; inspect run %s"
                    % (
                        max_rounds,
                        converge_accel,
                        100.0
                        * (last_change if last_change is not None else 1.0),
                        100.0 * tol,
                        run.run_dir,
                    )
                )
            capture_round("fit_verify", max_accel)
            verified_full, text = self._fit_round(
                gcmd,
                plan,
                run,
                "fit_verify",
                os.path.join(run.run_dir, "dynamics_fit_verify.toml"),
                torque,
                inertia,
            )
            verified = applied_model(verified_full)
            shift = max(torque_changes(fitted, verified, max_accel))
            if shift > drift:
                raise gcmd.error(
                    "converged model does not hold at MAX_ACCEL %.0f: "
                    "re-identification shifted the parameters %.1f%% "
                    "(DRIFT %.1f%%) - converged %s vs verify %s | the fit "
                    "at accel %.0f was an artifact of that operating "
                    "point, not physics; lower MAX_ACCEL below the "
                    "regime change or investigate | run %s"
                    % (
                        max_accel,
                        100.0 * shift,
                        100.0 * drift,
                        self._dynamics_params_line(fitted),
                        self._dynamics_params_line(verified),
                        converge_accel,
                        run.run_dir,
                    )
                )
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        verified,
                        verified_full,
                        terms,
                        run.run_dir,
                        baseline_lead_us,
                    )
                )
            fit_written_path = out_path
            run.manifest["dynamics_fit"] = {
                "rounds": rounds_run,
                "converged_change": last_change,
                "verify_shift": shift,
                "terms": [t.lower() for t in terms],
                "fitted_not_applied": {
                    key: verified_full[key] for key in dropped
                },
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_fit",
                run_dir=run.run_dir,
                rounds=rounds_run,
                converged_change=last_change,
                verify_shift=shift,
                profile=out_path,
            )
            gcmd.respond_info(
                "converged in %d rounds (change %.1f%%), holds at MAX_ACCEL "
                "%.0f (shift %.1f%% <= DRIFT %.1f%%) | dynamics profile: %s "
                "| run %s"
                % (
                    rounds_run,
                    100.0 * (last_change or 0.0),
                    max_accel,
                    100.0 * shift,
                    100.0 * drift,
                    out_path,
                    run.run_dir,
                )
            )
        finally:
            try:
                if applied:
                    if restore is not None:
                        send_dynamics_model(engine, handle, restore)
                        node.set_live_dynamics_profile(restore_path)
                        gcmd.respond_info(
                            "live dynamics model restored to baseline %s"
                            % (restore_path,)
                        )
                    else:
                        node.set_live_dynamics_profile(fit_written_path)
                        gcmd.respond_info(
                            "WARNING: no dynamics_profile configured - the "
                            "last fitted model stays live until RESTART"
                        )
            finally:
                self._restore()
                self._active_run = None
        return run, text, out_path

    def _run_fit_sweep(
        self,
        gcmd: Any,
        name: str,
        torque: float | None,
        inertia: float | None,
    ) -> tuple[ExperimentRun, str, str]:
        """Identify-only m(accel) curve: one pattern capture + fit per
        ACCELS entry, run under whatever dynamics model is currently live
        (nothing is streamed), so the points differ only in accel."""
        if tomllib is None:
            raise gcmd.error(
                "SERVO_FIT_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        stale = [
            p
            for p in (
                "SPEEDS",
                "PATTERN",
                "TOL",
                "DRIFT",
                "MAX_ROUNDS",
                "MAX_ACCEL",
                "TERMS",
            )
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: ACCELS runs an identify-only sweep - it takes only "
                "MAX_SPEED, ITERATIONS, DWELL_MS, NAME, SERVOS and the "
                "pattern geometry" % (", ".join(stale),)
            )
        self._reject_pattern_stroke_bounds(gcmd)
        plan = self._fit_plan(gcmd)
        raw = gcmd.get("ACCELS")
        try:
            accels = [float(v) for v in raw.split(",") if v.strip()]
        except ValueError:
            raise gcmd.error(
                "ACCELS must be a comma list of accelerations (got %r)" % (raw,)
            )
        if (
            len(accels) < 2
            or any(a <= 0.0 for a in accels)
            or sorted(accels) != accels
            or len(set(accels)) != len(accels)
        ):
            raise gcmd.error(
                "ACCELS wants at least two distinct ascending positive "
                "accelerations (got %r)" % (raw,)
            )
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        iterations = gcmd.get_int("ITERATIONS", self.iterations, minval=1)
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "accels": accels,
            "max_speed": max_speed,
            "speeds": speeds,
            "iterations": iterations,
            "dwell_ms": dwell,
        }
        stroke_plan.update(pattern_plan)
        run = self._begin_run(
            gcmd,
            "dynamics_sweep",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )
        text = ""
        out_path = ""
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, accels, speeds
            )
            fits: list[tuple[float, dict[str, Any]]] = []
            for accel in accels:
                step = "fit_a%d" % (round(accel),)
                self._start_capture(step, plan["servos"])
                self._goto_xy(start_x, start_y, dwell)
                for speed in speeds:
                    servo_strokes.emit_pattern(
                        self.gcode,
                        points,
                        start_x,
                        start_y,
                        speed,
                        accel,
                        iterations,
                        dwell,
                    )
                self._stop_capture()
                run.record_step(SweepStep(step, {"accel": accel}, []))
                out_path = os.path.join(
                    run.run_dir, "dynamics_%s.toml" % (step,)
                )
                fitted, text = self._fit_round(
                    gcmd, plan, run, step, out_path, torque, inertia
                )
                fits.append((accel, fitted))
                gcmd.respond_info(
                    "accel %.0f: %s"
                    % (accel, self._dynamics_params_line(fitted))
                )
            for (a0, f0), (a1, f1) in zip(fits, fits[1:]):
                try:
                    change = max(dynamics_torque_changes(f0, f1, a1, max_speed))
                except ValueError as e:
                    raise gcmd.error(str(e))
                gcmd.respond_info(
                    "accel %.0f -> %.0f: torque-weighted change %.1f%%"
                    % (a0, a1, 100.0 * change)
                )
            modes = fits[0][1]["modes"]
            curve = {
                mode: [f[1]["mass"][k] for f in fits]
                for k, mode in enumerate(modes)
            }
            for mode in modes:
                masses = curve[mode]
                lo, hi = min(masses), max(masses)
                gcmd.respond_info(
                    "mode %s mass(accel): %s | spread %.1f%%"
                    % (
                        mode,
                        ", ".join(
                            "%.0f: %.5g" % (a, m)
                            for (a, _f), m in zip(fits, masses)
                        ),
                        200.0 * (hi - lo) / (hi + lo),
                    )
                )
            run.manifest["dynamics_sweep"] = {
                "accels": accels,
                "mass": curve,
                "max_speed": max_speed,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_sweep",
                run_dir=run.run_dir,
                accels=accels,
            )
            gcmd.respond_info(
                "identify-only sweep done - nothing was applied | run %s"
                % (run.run_dir,)
            )
        finally:
            self._restore()
            self._active_run = None
        return run, text, out_path

    def cmd_SERVO_FIT_DYNAMICS(self, gcmd: Any) -> None:
        torque, inertia = self._motor(gcmd, required=False)
        self._run_fit(gcmd, gcmd.get("NAME", "ident"), torque, inertia)

    def _reject_tune_dynamics_params(self, gcmd: Any) -> None:
        stale = [
            p
            for p in ("ACCELS", "SPEEDS", "PATTERN")
            if gcmd.get(p, None) is not None
        ]
        if stale:
            raise gcmd.error(
                "%s: SERVO_TUNE_DYNAMICS always drives the XY pattern "
                "excitation at MAX_ACCEL/MAX_SPEED - it has no excitation "
                "matrix and no PATTERN toggle to override" % (", ".join(stale),)
            )

    def _load_ferr_fit(self, gcmd: Any, path: str) -> dict[str, Any]:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            raise gcmd.error(
                "servo-cal fit --response ferr produced an unusable result "
                "%s: %s" % (path, e)
            )
        if data.get("version") != 3:
            raise gcmd.error(
                "ferr fit %s: unsupported version %r (expected 3) - rebuild "
                "servo-cal (./install.sh), the binary predates the "
                "per-term transient-window rms objective"
                % (path, data.get("version"))
            )
        n_modes = len(data.get("modes", []))
        for key in ("ferr_rms_raw", "onset_bias"):
            vec = data.get(key)
            if not isinstance(vec, list) or len(vec) != n_modes:
                raise gcmd.error(
                    "ferr fit %s has no per-mode %s - rebuild servo-cal "
                    "(./install.sh), the binary predates the "
                    "rms-objective tuner" % (path, key)
                )
        ff = data.get("ferr_rms_ff")
        if not isinstance(ff, dict):
            raise gcmd.error(
                "ferr fit %s has no ferr_rms_ff dict - rebuild servo-cal "
                "(./install.sh), the binary predates the transient-window "
                "rms objective" % (path,)
            )
        for term in ("mass", "viscous", "coulomb", "lead"):
            entry = ff.get(term)
            if not isinstance(entry, dict):
                raise gcmd.error(
                    "ferr fit %s: ferr_rms_ff[%r] is missing or not a dict"
                    % (path, term)
                )
            for field in ("rms", "sigma", "windows"):
                vec = entry.get(field)
                if not isinstance(vec, list) or len(vec) != n_modes:
                    raise gcmd.error(
                        "ferr fit %s: ferr_rms_ff[%r][%r] must be a list of "
                        "%d per-mode values" % (path, term, field, n_modes)
                    )
        split = ff.get("direction_split")
        if not isinstance(split, dict):
            raise gcmd.error(
                "ferr fit %s: ferr_rms_ff['direction_split'] is missing or "
                "not a dict - rebuild servo-cal (./install.sh), the binary "
                "predates the direction-split objective" % (path,)
            )
        split_fields = [
            split.get(f)
            for f in ("pairs", "lambda", "q", "rms", "sigma", "windows")
        ]
        if any(not isinstance(v, list) for v in split_fields) or (
            len({len(v) for v in split_fields}) != 1
        ):
            raise gcmd.error(
                "ferr fit %s: ferr_rms_ff['direction_split'] fields "
                "pairs/lambda/q/rms/sigma/windows must be equal-length lists "
                "- rebuild servo-cal (./install.sh)" % (path,)
            )
        return data

    cmd_SERVO_TUNE_DYNAMICS_help = (
        "Empirical closed-loop dynamics tuner on coupled_xy: coordinate "
        "descent that MEASURES tracking error instead of trusting a "
        "fitted correlation. Each round streams the trial model to the "
        "running endpoint (no restart), captures one XY pattern run at "
        "MAX_ACCEL/MAX_SPEED, and scores each mode by the TRANSIENT-WINDOW "
        "rms of its following error - the excursion in the short window "
        "right after each commanded transition, where feedforward has "
        "authority before the inner servo loop corrects it (whole-capture "
        "rms diluted these transients ~10x). (The ferr/accel regression "
        "is still fitted and reported per round, but only as a direction "
        "hint and diagnostic: on the bench its zero landed at 2.2x the "
        "rms-optimal mass while tracking got worse every round.) Terms "
        "tune one at a time in TERMS order, both modes per capture, each "
        "as a 1-D line search: the mass probe's first direction follows "
        "the ONSET BIAS (mean sign(accel)*ferr right after each accel "
        "step - the manual heuristic: only the first excursion when "
        "torque lands carries clean command-path sign, before the "
        "drive's own compensation reacts; positive = under-fed), other "
        "terms follow their regression coefficient's sign; a failed "
        "first probe flips once, the step grows while the rms clears a "
        "2-sigma deadband measured from per-window scatter (relative "
        "change capped at 40%% per probe), and the first non-improving "
        "probe triggers one "
        "parabolic refine through the bracket; ties go to the best "
        "measured value. Viscous/coulomb are floored at zero (a "
        "zero-valued term probes up by a fixed floor step), mass at 10%% "
        "of its baseline. TERMS=LEAD tunes the feedforward LEAD TIME as "
        "one shared node-global value (seconds, continuous - the "
        "endpoint peeks the command ring at an arbitrary future "
        "nanosecond, so it is not quantized to whole cycles): scored on "
        "the mean of both modes' decel-to-stop window rms (corner exits "
        "- where timing error integrates into a direction-locked "
        "overshoot lobe), first direction from the summed "
        "onset bias (positive = FF lands late = probe up), floored at "
        "zero with a half-cycle floor step. The tuned lead stays live "
        "until RESTART; the written dynamics TOML always carries "
        "ff_lead_us (tuned when LEAD is in TERMS, else the baseline "
        "value passes through). Passes over the terms repeat until a "
        "full pass improves nothing, then the best model is written as "
        "a dynamics TOML and left LIVE (point [ethercat_node] "
        "dynamics_profile at it and RESTART to keep it). There is no "
        "round budget: the search runs until it converges (kill it if "
        "it overstays). torque_saturated aborts, restores the baseline "
        "and configured lead and writes nothing; resonance_detected "
        "only warns. The baseline is PROFILE=, else the model left LIVE "
        "by the previous tune this session, else the node-level "
        "[ethercat_node] dynamics_profile (per-motor profiles are not "
        "supported) - chained tunes refine each other's output, not the "
        "configured profile. RESUME=<run_dir> replays a crashed tune's "
        "rounds from its ferr_r*.json fits instead of recapturing (the "
        "search is deterministic, so round i reproduces the same trial); "
        "it requires the identical command line and the same live "
        "baseline model, and picks up with real captures at the first "
        "round the old run is missing. Params MAX_ACCEL MAX_SPEED STEP "
        "(0.15) TERMS (mass,viscous,coulomb,lead) NAME (tune) PROFILE "
        "RESUME SERVOS BOUND SMALL_SIZE"
    )

    def cmd_SERVO_TUNE_DYNAMICS(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires Python 3.11+ (tomllib)"
            )
        self._reject_tune_dynamics_params(gcmd)
        kin = self._kin()
        if not kin.coupled_xy():
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS requires coupled_xy kinematics - the "
                "ferr regression needs the mode-space frame"
            )
        plan = self._fit_plan(gcmd)
        node = self._dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        baseline_modes = baseline["modes"]
        if len(baseline_modes) != 2 or not {"x", "y"} <= set(baseline_modes):
            raise gcmd.error(
                "SERVO_TUNE_DYNAMICS needs a 2-mode profile with x and y "
                "modes; profile %s has modes %s"
                % (profile_path, baseline_modes)
            )
        terms = [
            t.strip().upper()
            for t in gcmd.get("TERMS", "MASS,VISCOUS,COULOMB,LEAD").split(",")
            if t.strip()
        ]
        allowed_terms = set(DYNAMICS_TERM_KEYS) | {"LEAD", "DIRECTION_SPLIT"}
        if not terms or any(t not in allowed_terms for t in terms):
            raise gcmd.error(
                "TERMS must be a comma list drawn from MASS, VISCOUS, "
                "COULOMB, LEAD, DIRECTION_SPLIT (got %r)"
                % (gcmd.get("TERMS", ""),)
            )
        lead_enabled = "LEAD" in terms
        cycle_us = node.get_cycle_us()
        configured_lead_s = baseline.get("ff_lead_us", 0.0) * 1e-6
        split_enabled = "DIRECTION_SPLIT" in terms
        if split_enabled:
            baseline = self._direction_split_baseline(gcmd, kin, baseline)
        max_accel = gcmd.get_float("MAX_ACCEL", max(self.accels), above=0.0)
        max_speed = gcmd.get_float("MAX_SPEED", max(self.speeds), above=0.0)
        step_frac = gcmd.get_float("STEP", 0.15, minval=0.02, maxval=0.5)
        name = gcmd.get("NAME", "tune")
        resume_dir = gcmd.get("RESUME", None)
        dwell = self.dwell_ms
        iterations = self.iterations
        speeds = [max_speed / 2.0, max_speed]
        points, start_x, start_y, pattern_plan = self._pattern_geometry_params(
            gcmd
        )
        stroke_plan = {
            "max_accel": max_accel,
            "max_speed": max_speed,
            "speeds": speeds,
            "step": step_frac,
            "objective": "transient_rms",
            "accept_z": ACCEPT_Z,
            "terms": [t.lower() for t in terms],
            "iterations": iterations,
            "dwell_ms": dwell,
            "lead_us": configured_lead_s * 1e6 if lead_enabled else None,
        }
        stroke_plan.update(pattern_plan)
        if resume_dir is not None:
            resume_dir = os.path.expanduser(resume_dir)
            manifest_path = os.path.join(resume_dir, "manifest.json")
            if not os.path.isfile(manifest_path):
                raise gcmd.error(
                    "RESUME dir %s has no manifest.json" % (resume_dir,)
                )
            with open(manifest_path) as f:
                prev_manifest = json.load(f)
            if prev_manifest.get("experiment") != "dynamics_tune":
                raise gcmd.error(
                    "RESUME dir %s is a %r run, not dynamics_tune"
                    % (resume_dir, prev_manifest.get("experiment"))
                )
            prev_plan = prev_manifest.get("stroke_plan") or {}
            diff = sorted(
                k for k in stroke_plan if prev_plan.get(k) != stroke_plan[k]
            )
            if diff:
                raise gcmd.error(
                    "RESUME run %s was made with different settings (%s "
                    "differ) - the deterministic replay would misattribute "
                    "its ferr fits; re-issue the identical command"
                    % (resume_dir, ", ".join(diff))
                )
        run = self._begin_run(
            gcmd,
            "dynamics_tune",
            name,
            plan["axis"],
            plan["servos"],
            stroke_plan,
            plan["rails"],
        )

        def capture_round(step: str) -> None:
            self._start_capture(step, plan["servos"])
            self._goto_xy(start_x, start_y, dwell)
            for speed in speeds:
                servo_strokes.emit_pattern(
                    self.gcode,
                    points,
                    start_x,
                    start_y,
                    speed,
                    max_accel,
                    iterations,
                    dwell,
                )
            self._stop_capture()
            run.record_step(SweepStep(step, {"accel": max_accel}, []))

        current = _copy_dynamics(baseline)
        current_lead = configured_lead_s
        rounds_history: list[dict[str, Any]] = []
        search_summaries: list[dict[str, Any]] = []
        measured: dict[tuple[float, ...], dict[str, Any]] = {}
        applied = False
        success = False

        def model_key(
            values: Mapping[str, Any], lead_s: float
        ) -> tuple[float, ...]:
            coeffs = tuple(
                float(v)
                for term in ("MASS", "VISCOUS", "COULOMB")
                for v in values[DYNAMICS_TERM_KEYS[term]]
            )
            splits = tuple(
                round(float(pair["direction_split"]), 9)
                for pair in values.get("pairs", [])
            )
            return coeffs + splits + (round(lead_s * 1e9),)

        def ferr_result(round_i: int, ferr: dict[str, Any]) -> dict[str, Any]:
            if ferr.get("modes") != plan["modes"]:
                raise gcmd.error(
                    "servo-cal fit --response ferr modes %s do not "
                    "match the requested modes %s"
                    % (ferr.get("modes"), plan["modes"])
                )
            return {
                "round": round_i,
                "rms": [float(v) for v in ferr["ferr_rms_raw"]],
                "ff": ferr["ferr_rms_ff"],
                "coef": ferr["coef"],
                "stderr": ferr["stderr"],
                "onset": [float(v) for v in ferr["onset_bias"]],
                "samples": ferr.get("samples"),
            }

        def measure(
            round_i: int, trial: dict[str, Any], lead_s: float
        ) -> dict[str, Any]:
            if resume_dir is not None:
                src = os.path.join(resume_dir, "ferr_r%d.json" % (round_i,))
                if os.path.isfile(src):
                    ferr = self._load_ferr_fit(gcmd, src)
                    result = ferr_result(round_i, ferr)
                    shutil.copyfile(
                        src,
                        os.path.join(run.run_dir, "ferr_r%d.json" % (round_i,)),
                    )
                    gcmd.respond_info(
                        "r%d replayed from %s (no capture)" % (round_i, src)
                    )
                    return result
            send_dynamics_model(engine, handle, trial)
            if lead_enabled:
                send_ff_lead(engine, handle, node, plan["servos"], lead_s)
            step = "tune_r%d" % (round_i,)
            capture_round(step)
            results = self._run_analyze(gcmd, run, incremental=True)
            flags = set(self._step_flags(results, step))
            if "torque_saturated" in flags:
                raise gcmd.error(
                    "step %s hit the torque rail - clipped strokes "
                    "cannot score tracking error, aborting "
                    "SERVO_TUNE_DYNAMICS" % (step,)
                )
            if "resonance_detected" in flags:
                gcmd.respond_info(
                    "WARNING step %s flagged resonance_detected - "
                    "continuing (feedforward tuning does not move the "
                    "loop's resonances)" % (step,)
                )
            ferr_out = os.path.join(run.run_dir, "ferr_r%d.json" % (round_i,))
            argv = self._fit_argv_for(
                gcmd,
                plan,
                run.step_scap(step),
                ferr_out,
                None,
                None,
                response="ferr",
            )
            self._run(gcmd, argv, 120.0)
            ferr = self._load_ferr_fit(gcmd, ferr_out)
            return ferr_result(round_i, ferr)

        def term_objective(
            cached: dict[str, Any], ff_key: str
        ) -> tuple[list[float], list[float]]:
            entry = cached["ff"].get(ff_key)
            if entry is None:
                raise gcmd.error(
                    "ferr fit has no ferr_rms_ff[%r] to score" % (ff_key,)
                )
            if ff_key == "direction_split":
                n_pairs = len(current["pairs"])
                for field in (
                    "pairs",
                    "lambda",
                    "q",
                    "rms",
                    "sigma",
                    "windows",
                ):
                    vec = entry.get(field)
                    if not isinstance(vec, list) or len(vec) != n_pairs:
                        raise gcmd.error(
                            "ferr fit direction_split[%r] must be a list of "
                            "%d per-pair values matching the profile pairs - "
                            "rebuild servo-cal (./install.sh)"
                            % (field, n_pairs)
                        )
                for pair_idx, pair in enumerate(current["pairs"]):
                    label = pair["slots"][0]
                    if not entry["windows"][pair_idx] or (
                        entry["rms"][pair_idx] is None
                    ):
                        raise gcmd.error(
                            "direction_split pair %s has no direction-run "
                            "windows - the excitation never reversed it, "
                            "feedforward cannot be scored" % (label,)
                        )
                    if entry["sigma"][pair_idx] is None:
                        raise gcmd.error(
                            "direction_split pair %s has fewer than 2 windows "
                            "per direction so its scatter (sigma) is "
                            "unmeasurable - cannot apply the 2-sigma "
                            "acceptance test" % (label,)
                        )
                return (
                    [float(r) for r in entry["rms"]],
                    [float(s) for s in entry["sigma"]],
                )
            rms_v = entry["rms"]
            sigma_v = entry["sigma"]
            windows_v = entry["windows"]
            for fit_idx, mode in enumerate(plan["modes"]):
                if not windows_v[fit_idx] or rms_v[fit_idx] is None:
                    raise gcmd.error(
                        "term %s mode %s has no transient windows - the "
                        "excitation never triggered it, feedforward cannot "
                        "be scored" % (ff_key, mode)
                    )
                if sigma_v[fit_idx] is None:
                    raise gcmd.error(
                        "term %s mode %s has fewer than 2 transient windows "
                        "so its per-window scatter (sigma) is unmeasurable - "
                        "cannot apply the 2-sigma acceptance test"
                        % (ff_key, mode)
                    )
            return (
                [float(r) for r in rms_v],
                [float(s) for s in sigma_v],
            )

        try:
            out_path = self._dynamics_out_path(gcmd, run, name)
            self._prep("X", dwell)
            self._prep("Y", dwell)
            self._pattern_reach_report(
                gcmd, points, start_x, start_y, [max_accel], speeds
            )
            phase_idx = 0
            searches: dict[str, RmsLineSearch] | None = None
            pass_improved = False
            round_i = 0
            while True:
                term = terms[phase_idx]
                is_lead = term == "LEAD"
                is_split = term == "DIRECTION_SPLIT"
                key = (
                    None if (is_lead or is_split) else DYNAMICS_TERM_KEYS[term]
                )
                trial = _copy_dynamics(current)
                trial_lead = current_lead
                if searches is not None:
                    if is_lead:
                        search = searches["xy"]
                        trial_lead = (
                            search.best if search.done else search.trial
                        )
                    elif is_split:
                        for pair_idx, pair in enumerate(current["pairs"]):
                            search = searches[pair["slots"][0]]
                            value = search.best if search.done else search.trial
                            delta = (
                                value
                                - trial["pairs"][pair_idx]["direction_split"]
                            )
                            trial = add_dynamics_direction_split(
                                trial, pair_idx, delta
                            )
                    else:
                        for mode, search in searches.items():
                            idx = baseline_modes.index(mode)
                            trial[key][idx] = (
                                search.best if search.done else search.trial
                            )
                cache_key = model_key(trial, trial_lead)
                cached = measured.get(cache_key)
                ff_key = term.lower()
                if cached is None:
                    applied = True
                    cached = measure(round_i, trial, trial_lead)
                    measured[cache_key] = cached
                    obj_rms, obj_sigma = term_objective(cached, ff_key)
                    rms = cached["rms"]
                    label = "baseline" if searches is None else term.lower()
                    if is_lead:
                        n_modes = len(obj_rms)
                        line = "xy lead=%.1fus rms=%.2fum (onset %+.2fum)" % (
                            trial_lead * 1e6,
                            sum(obj_rms) / n_modes * 1e3,
                            sum(cached["onset"]) * 1e3,
                        )
                    elif is_split:
                        split_q = cached["ff"]["direction_split"]["q"]
                        line = " | ".join(
                            "%s split=%.4f q=%+.2fum rms=%.2fum"
                            % (
                                pair["slots"][0],
                                trial["pairs"][pair_idx]["direction_split"],
                                float(split_q[pair_idx]) * 1e3,
                                obj_rms[pair_idx] * 1e3,
                            )
                            for pair_idx, pair in enumerate(current["pairs"])
                        )
                    else:
                        line = " | ".join(
                            "%s %s=%.6g rms=%.2fum (onset %+.2fum, g=%+.3g)"
                            % (
                                mode,
                                key,
                                trial[key][baseline_modes.index(mode)],
                                obj_rms[fit_idx] * 1e3,
                                cached["onset"][fit_idx] * 1e3,
                                cached["coef"][key][fit_idx],
                            )
                            for fit_idx, mode in enumerate(plan["modes"])
                        )
                    gcmd.respond_info("r%d [%s] %s" % (round_i, label, line))
                    rounds_history.append(
                        {
                            "round": round_i,
                            "term": label,
                            "values": {
                                DYNAMICS_TERM_KEYS[t]: list(
                                    trial[DYNAMICS_TERM_KEYS[t]]
                                )
                                for t in terms
                                if t not in ("LEAD", "DIRECTION_SPLIT")
                            },
                            "direction_split": (
                                [
                                    {
                                        "slots": list(pair["slots"]),
                                        "direction_split": pair[
                                            "direction_split"
                                        ],
                                    }
                                    for pair in trial["pairs"]
                                ]
                                if split_enabled
                                else None
                            ),
                            "lead_us": (
                                trial_lead * 1e6 if lead_enabled else None
                            ),
                            "ferr_rms_raw": list(rms),
                            "ferr_rms_ff": cached["ff"],
                            "coef": dict(cached["coef"]),
                            "stderr": dict(cached["stderr"]),
                            "onset_bias": list(cached["onset"]),
                            "samples": cached["samples"],
                        }
                    )
                    round_i += 1
                else:
                    obj_rms, obj_sigma = term_objective(cached, ff_key)
                if searches is None:
                    searches = {}
                    if is_lead:
                        hint = sum(cached["onset"])
                        step_size = (
                            step_frac * current_lead
                            if current_lead > 0.0
                            else 0.5 * cycle_us * 1e-6
                        )
                        n_modes = len(obj_rms)
                        searches["xy"] = RmsLineSearch(
                            current_lead,
                            sum(obj_rms) / n_modes,
                            math.hypot(*obj_sigma) / n_modes,
                            step_size,
                            lo=0.0,
                            hint=hint if hint != 0.0 else 1.0,
                        )
                    elif is_split:
                        split_q = cached["ff"]["direction_split"]["q"]
                        for pair_idx, pair in enumerate(current["pairs"]):
                            value = pair["direction_split"]
                            step_size = max(step_frac * abs(value), 0.02)
                            q = float(split_q[pair_idx])
                            searches[pair["slots"][0]] = RmsLineSearch(
                                value,
                                obj_rms[pair_idx],
                                obj_sigma[pair_idx],
                                step_size,
                                lo=-0.45,
                                hi=0.45,
                                hint=-q if q != 0.0 else 1.0,
                            )
                    else:
                        for fit_idx, mode in enumerate(plan["modes"]):
                            idx = baseline_modes.index(mode)
                            value = current[key][idx]
                            if term == "MASS":
                                lo = (
                                    TUNE_MASS_FLOOR_FRACTION
                                    * baseline[key][idx]
                                )
                                step_size = step_frac * abs(value)
                            else:
                                lo = 0.0
                                step_size = (
                                    step_frac * abs(value)
                                    if value != 0.0
                                    else TUNE_ZERO_FLOOR_STEPS[term]
                                )
                            hint = float(cached["coef"][key][fit_idx])
                            if (
                                term == "MASS"
                                and cached["onset"][fit_idx] != 0.0
                            ):
                                hint = cached["onset"][fit_idx]
                            searches[mode] = RmsLineSearch(
                                value,
                                obj_rms[fit_idx],
                                obj_sigma[fit_idx],
                                step_size,
                                lo=lo,
                                hint=hint if hint != 0.0 else 1.0,
                            )
                elif is_lead:
                    search = searches["xy"]
                    if not search.done:
                        n_modes = len(obj_rms)
                        search.feed(
                            sum(obj_rms) / n_modes,
                            math.hypot(*obj_sigma) / n_modes,
                        )
                elif is_split:
                    for pair_idx, pair in enumerate(current["pairs"]):
                        search = searches[pair["slots"][0]]
                        if not search.done:
                            search.feed(obj_rms[pair_idx], obj_sigma[pair_idx])
                else:
                    for fit_idx, mode in enumerate(plan["modes"]):
                        search = searches[mode]
                        if not search.done:
                            search.feed(obj_rms[fit_idx], obj_sigma[fit_idx])
                if all(search.done for search in searches.values()):
                    lines = []
                    if is_lead:
                        search = searches["xy"]
                        pass_improved = pass_improved or search.improved
                        current_lead = search.best
                        lines.append(
                            "xy %.1fus @ %.2fum (%s)"
                            % (
                                search.best * 1e6,
                                search.best_rms * 1e3,
                                search.note,
                            )
                        )
                        search_summaries.append(
                            {
                                "term": "lead",
                                "mode": "xy",
                                "best": search.best,
                                "best_rms": search.best_rms,
                                "best_sigma": search.best_sigma,
                                "improved": search.improved,
                                "note": search.note,
                                "probes": len(search.history) - 1,
                            }
                        )
                    elif is_split:
                        for pair_idx, pair in enumerate(current["pairs"]):
                            search = searches[pair["slots"][0]]
                            pass_improved = pass_improved or search.improved
                            current["pairs"][pair_idx]["direction_split"] = (
                                search.best
                            )
                            lines.append(
                                "%s %.4f @ %.2fum (%s)"
                                % (
                                    pair["slots"][0],
                                    search.best,
                                    search.best_rms * 1e3,
                                    search.note,
                                )
                            )
                            search_summaries.append(
                                {
                                    "term": "direction_split",
                                    "mode": pair["slots"][0],
                                    "best": search.best,
                                    "best_rms": search.best_rms,
                                    "best_sigma": search.best_sigma,
                                    "improved": search.improved,
                                    "note": search.note,
                                    "probes": len(search.history) - 1,
                                }
                            )
                    else:
                        for fit_idx, mode in enumerate(plan["modes"]):
                            search = searches[mode]
                            pass_improved = pass_improved or search.improved
                            idx = baseline_modes.index(mode)
                            current[key][idx] = search.best
                            lines.append(
                                "%s %.6g @ %.2fum (%s)"
                                % (
                                    mode,
                                    search.best,
                                    search.best_rms * 1e3,
                                    search.note,
                                )
                            )
                            search_summaries.append(
                                {
                                    "term": term.lower(),
                                    "mode": mode,
                                    "best": search.best,
                                    "best_rms": search.best_rms,
                                    "best_sigma": search.best_sigma,
                                    "improved": search.improved,
                                    "note": search.note,
                                    "probes": len(search.history) - 1,
                                }
                            )
                    gcmd.respond_info(
                        "%s settled: %s" % (term.lower(), " | ".join(lines))
                    )
                    searches = None
                    phase_idx += 1
                    if phase_idx == len(terms):
                        if not pass_improved:
                            success = True
                            break
                        phase_idx = 0
                        pass_improved = False
            send_dynamics_model(engine, handle, current)
            if lead_enabled:
                send_ff_lead(engine, handle, node, plan["servos"], current_lead)
            with open(out_path, "w") as f:
                f.write(
                    render_fit_dynamics_toml(
                        current,
                        current,
                        [t.lower() for t in terms if t != "LEAD"],
                        run.run_dir,
                        current_lead * 1e6,
                    )
                )
            node.set_live_dynamics_profile(out_path)
            run.manifest["dynamics_tune"] = {
                "terms": [t.lower() for t in terms],
                "max_accel": max_accel,
                "max_speed": max_speed,
                "step": step_frac,
                "objective": "transient_rms",
                "accept_z": ACCEPT_Z,
                "rounds": rounds_history,
                "search": search_summaries,
                "lead_us": current_lead * 1e6 if lead_enabled else None,
                "converged": True,
                "profile": out_path,
            }
            run.write()
            structured_log.event(
                "calibration",
                "dynamics_tune",
                run_dir=run.run_dir,
                rounds=len(rounds_history),
                profile=out_path,
            )
            lead_note = ""
            if lead_enabled:
                lead_note = (
                    " | tuned ff lead %.1fus - carried in the tuned profile"
                    % (current_lead * 1e6,)
                )
            gcmd.respond_info(
                "SERVO_TUNE_DYNAMICS converged in %d captures | tuned "
                "dynamics profile: %s | tuned model stays live until "
                "RESTART - point [ethercat_node %s] dynamics_profile at "
                "it to keep it%s | run %s"
                % (
                    len(rounds_history),
                    out_path,
                    node.name,
                    lead_note,
                    run.run_dir,
                )
            )
        finally:
            try:
                if applied and not success:
                    send_dynamics_model(engine, handle, baseline)
                    if lead_enabled:
                        send_ff_lead(
                            engine,
                            handle,
                            node,
                            plan["servos"],
                            configured_lead_s,
                        )
                    node.set_live_dynamics_profile(profile_path)
                    gcmd.respond_info(
                        "live dynamics model restored to baseline %s"
                        % (profile_path,)
                    )
            finally:
                self._restore()
                self._active_run = None

    cmd_SERVO_SET_COMPLIANCE_help = (
        "Write the per-mode belt-compliance feedforward term 1/omega_b^2 "
        "into the dynamics profile and stream it live (no restart). With "
        "a nonzero compliance the endpoint inverts the two-mass plant: "
        "the rotor leads the commanded trajectory by accel/omega_b^2 - "
        "exactly the belt stretch the accel consumes - so the carriage "
        "follows the planner curve without ringing from commanded "
        "motion (jerk and snap terms land on the 60B1h/60B2h streams "
        "automatically). X_FREQ/Y_FREQ are the LOCKED-ROTOR belt "
        "frequencies in Hz per Cartesian mode - the frequency the "
        "carriage rings at when the rotor holds still. This sits ABOVE "
        "the coupled frequency a plain ringdown measures (there the "
        "rotor recoils on the position-loop spring in series with the "
        "belt), so feeding the raw ringdown frequency OVER-corrects: "
        "start above the measured value and tune down, or 0 to disable "
        "a mode. An omitted mode keeps its current profile value. The "
        "profile is written as a new timestamped v7 TOML (never "
        "overwritten) and left LIVE until RESTART - point "
        "[ethercat_node] dynamics_profile at it to keep it. Residual "
        "excitation the command didn't cause (cogging, reversals, model "
        "error) still rings at the coupled frequency - keep a light "
        "input shaper or the belt damper for that. Baseline profile "
        "resolution matches SERVO_TUNE_DYNAMICS (PROFILE=, else the "
        "live-tuned model, else the configured node profile). PIN "
        "(XY|X|Y) pins the rotor for those modes: instead of the "
        "position/velocity lead, the endpoint holds a predictive "
        "torque against the modelled deflection, replacing the "
        "compliance lead for that mode with an active hold. Pinning "
        "needs the mode's FRF peak X_PEAK/Y_PEAK (Hz) alongside its "
        "notch f_b (X_FREQ/Y_FREQ now or nonzero baseline compliance): "
        "the per-mode load fraction is 1-(f_b/f_peak)^2 and the pinned "
        "mass is mass*fraction (run SERVO_MEASURE_COMPLIANCE, which "
        "reports f_peak per mode). ZETA (default 0.02) is the "
        "hold damping and PIN_LEAD_US (default 0) advances the hold. "
        "A pinned mode must have nonzero compliance (given now or in "
        "the baseline). PIN=0 clears the pins but keeps compliance. "
        "The profile is written v8 while any mode is pinned. Params "
        "X_FREQ Y_FREQ (Hz, >= 20; 0 disables) PIN (XY|X|Y|0) "
        "X_PEAK Y_PEAK (Hz, FRF peak) ZETA (0.02) PIN_LEAD_US (us) "
        "NAME (compliance) PROFILE SERVOS"
    )

    def cmd_SERVO_SET_COMPLIANCE(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error(
                "SERVO_SET_COMPLIANCE requires Python 3.11+ (tomllib)"
            )
        freq_params = {"x": "X_FREQ", "y": "Y_FREQ"}
        freq_by_mode = {}
        for mode, param in freq_params.items():
            freq = gcmd.get_float(param, None)
            if freq is None:
                continue
            if freq != 0.0 and freq < 20.0:
                raise gcmd.error(
                    "%s must be >= 20 Hz (got %g) - the endpoint rejects "
                    "softer modes as typos" % (param, freq)
                )
            freq_by_mode[mode] = freq
        pin = self._parse_pin_request(gcmd)
        if not freq_by_mode and pin is None:
            raise gcmd.error(
                "give X_FREQ= and/or Y_FREQ= (Hz, locked-rotor belt "
                "frequency; 0 disables), or PIN= to pin the rotor"
            )
        self._apply_compliance(
            gcmd, freq_by_mode, gcmd.get("NAME", "compliance"), pin
        )

    def _parse_pin_request(self, gcmd: Any) -> dict[str, Any] | None:
        """Parse PIN/X_PEAK/Y_PEAK/ZETA/PIN_LEAD_US. Returns None when
        PIN is absent (baseline pin state is preserved); otherwise a
        request dict whose empty ``modes`` set means an explicit PIN=0
        clear."""
        raw = gcmd.get("PIN", None)
        if raw is None:
            return None
        text = raw.strip().upper()
        modes: set[str] = set()
        if text not in ("0", ""):
            mode_map = {"X": "x", "Y": "y"}
            for ch in text:
                if ch not in mode_map:
                    raise gcmd.error(
                        "PIN must be XY, X, Y, or 0 (got %r)" % (raw,)
                    )
                modes.add(mode_map[ch])
        zeta = gcmd.get_float("ZETA", 0.02, above=0.0, maxval=PIN_ZETA_MAX)
        pin_lead_us = gcmd.get_float(
            "PIN_LEAD_US", 0.0, minval=0.0, maxval=PIN_LEAD_US_MAX
        )
        peak_params = {"x": "X_PEAK", "y": "Y_PEAK"}
        peaks: dict[str, float] = {}
        for mode, param in peak_params.items():
            val = gcmd.get_float(param, None, above=0.0)
            if val is not None:
                peaks[mode] = val
        missing_peak = sorted(m for m in modes if m not in peaks)
        if missing_peak:
            raise gcmd.error(
                "PIN=%s requires %s (the FRF peak in Hz per mode); run "
                "SERVO_MEASURE_COMPLIANCE, which reports f_peak per mode"
                % (
                    "".join(m.upper() for m in sorted(modes)),
                    ", ".join(peak_params[m] for m in missing_peak),
                )
            )
        return {
            "modes": modes,
            "peaks": peaks,
            "zeta": zeta,
            "pin_lead_us": pin_lead_us,
        }

    def _apply_pin(
        self,
        gcmd: Any,
        updated: dict[str, Any],
        pin: dict[str, Any],
        changed: list[str],
    ) -> None:
        """Apply the pin request onto ``updated`` (already carrying the
        baseline pin state), mutating pin_mass/pin_zeta/pin_lead_us."""
        modes = pin["modes"]
        if not modes:
            # explicit PIN=0: clear every pin, keep compliance untouched
            for i in range(len(updated["modes"])):
                updated["pin_mass"][i] = 0.0
                updated["pin_zeta"][i] = 0.0
            updated["pin_lead_us"] = 0.0
            changed.append("pin: cleared")
            return
        missing = sorted(modes - set(updated["modes"]))
        if missing:
            raise gcmd.error(
                "PIN mode(s) %s not in profile (modes %s)"
                % (", ".join(m.upper() for m in missing), updated["modes"])
            )
        updated["pin_lead_us"] = pin["pin_lead_us"]
        for mode_i, mode in enumerate(updated["modes"]):
            if mode not in modes:
                continue
            if not updated["compliance"][mode_i] > 0.0:
                raise gcmd.error(
                    "PIN=%s needs mode %s to have nonzero compliance - "
                    "give %s_FREQ= now or set it in the baseline"
                    % (mode.upper(), mode, mode.upper())
                )
            f_b = 1.0 / (
                2.0 * math.pi * math.sqrt(updated["compliance"][mode_i])
            )
            f_peak = pin["peaks"][mode]
            if not f_peak > f_b:
                raise gcmd.error(
                    "PIN=%s peak %.1f Hz must sit above the notch f_b "
                    "%.1f Hz" % (mode.upper(), f_peak, f_b)
                )
            fraction = 1.0 - (f_b / f_peak) ** 2
            pin_mass = updated["mass"][mode_i] * fraction
            updated["pin_mass"][mode_i] = pin_mass
            updated["pin_zeta"][mode_i] = pin["zeta"]
            changed.append(
                "%s: pinned m_L=%.2f*mass=%.3gkg (f_b %.1f / peak %.1f) "
                "zeta=%.3g lead=%.0fus"
                % (
                    mode,
                    fraction,
                    pin_mass,
                    f_b,
                    f_peak,
                    pin["zeta"],
                    pin["pin_lead_us"],
                )
            )

    def _apply_compliance(
        self,
        gcmd: Any,
        freq_by_mode: dict[str, float],
        name: str,
        pin: dict[str, Any] | None = None,
    ) -> None:
        """Write freq_by_mode (Hz; 0 disables) into a new dynamics
        profile and stream it live. Modes absent from the dict keep
        their current compliance; ``pin`` (when given) sets the pin-rotor
        hold, otherwise the baseline pin state is preserved."""
        plan = self._fit_plan(gcmd)
        node = self._dynamics_node(gcmd, plan["servos"])
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        unmatched = sorted(set(freq_by_mode) - set(baseline["modes"]))
        if unmatched:
            raise gcmd.error(
                "mode(s) %s not in profile %s (modes %s)"
                % (", ".join(unmatched), profile_path, baseline["modes"])
            )
        updated = _copy_dynamics(baseline)
        updated["ff_lead_us"] = baseline.get("ff_lead_us", 0.0)
        changed = []
        for mode_i, mode in enumerate(updated["modes"]):
            if mode not in freq_by_mode:
                continue
            freq = freq_by_mode[mode]
            if freq == 0.0:
                updated["compliance"][mode_i] = 0.0
                changed.append("%s: off" % (mode,))
                continue
            c = 1.0 / (2.0 * math.pi * freq) ** 2
            updated["compliance"][mode_i] = c
            # lead per commanded accel: c mm per mm/s^2 = c*1e9 um per m/s^2
            changed.append(
                "%s: %.1f Hz -> %.3g s^2 (lead %.0f um at 50 m/s^2)"
                % (mode, freq, c, c * 5.0e4 * 1e3)
            )
        if pin is not None:
            self._apply_pin(gcmd, updated, pin, changed)
        # guard the model invariant: a pinned mode needs live compliance
        for mode_i, mode in enumerate(updated["modes"]):
            if updated["pin_mass"][mode_i] > 0.0 and not (
                updated["compliance"][mode_i] > 0.0
            ):
                raise gcmd.error(
                    "mode %s is pinned but its compliance is 0 - keep "
                    "%s_FREQ nonzero or clear the pin with PIN=0"
                    % (mode, mode.upper())
                )
        os.makedirs(self.dynamics_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(
            self.dynamics_dir, "dynamics_%s_%s.toml" % (name, stamp)
        )
        suffix = 1
        while os.path.exists(out_path):
            # never overwrite; same-second re-issues get a counter suffix
            out_path = os.path.join(
                self.dynamics_dir,
                "dynamics_%s_%s-%d.toml" % (name, stamp, suffix),
            )
            suffix += 1
        with open(out_path, "w") as f:
            f.write(
                render_fit_dynamics_toml(
                    updated,
                    updated,
                    [],
                    "servo_set_compliance",
                    baseline.get("ff_lead_us", 0.0),
                )
            )
        send_dynamics_model(engine, handle, updated)
        node.set_live_dynamics_profile(out_path)
        structured_log.event(
            "calibration",
            "set_compliance",
            profile=out_path,
            compliance=updated["compliance"],
            pin_mass=updated["pin_mass"],
        )
        gcmd.respond_info(
            "compliance %s | written %s | model live until RESTART - "
            "point [ethercat_node %s] dynamics_profile at it to keep it"
            % (" | ".join(changed), out_path, node.name)
        )

    cmd_SERVO_MEASURE_COMPLIANCE_help = (
        "Measure the LOCKED-ROTOR belt frequency f_b per Cartesian mode "
        "- the number SERVO_SET_COMPLIANCE wants - from a mode-patterned "
        "swept position buzz at standstill. The analysis is the "
        "instrumental-variable FRF from measured torque (6077h) to rotor "
        "position with the commanded buzz as instrument: its "
        "anti-resonance notch is exactly sqrt(k_belt/m_load)/2pi and is "
        "invariant under the position loop (plant zeros cannot be moved "
        "by feedback). f_b sits above the familiar coupled ringdown "
        "frequency and below the plant's two-mass peak, which is "
        "reported alongside as a sanity anchor. Excitation is gentle by "
        "construction: at the notch the rotor barely moves. Flags: "
        "compliance_notch_shallow (< 6 dB - raise AMPLITUDE or narrow "
        "the band), compliance_flanks_incoherent, "
        "compliance_peak_below_notch (model violation - do not apply). "
        "Measurement only: it changes nothing on the drives - it prints "
        "the ready-to-run SERVO_SET_COMPLIANCE line (with X_PEAK/Y_PEAK "
        "so it is pin-complete), which writes the v7 "
        "profile and streams it live; point [ethercat_node] "
        "dynamics_profile at the written TOML to survive RESTART. Params "
        "MODE=XY|X|Y FREQ_START (60) FREQ_END (320) HZ_PER_SEC (1) "
        "DURATION AMPLITUDE (0.02) RAMP DWELL_MS NAME (compliance)"
    )

    def cmd_SERVO_MEASURE_COMPLIANCE(self, gcmd: Any) -> None:
        kin = self._kin()
        spatial = servo_strokes.spatial_frame(kin)
        if spatial is None:
            raise gcmd.error(
                "SERVO_MEASURE_COMPLIANCE needs servo rails on X/Y - no "
                "spatial frame available"
            )
        mode_req = gcmd.get("MODE", "XY").upper()
        wanted = [m for m in ("x", "y") if m.upper() in mode_req]
        modes = [m for m in spatial["modes"] if m in wanted]
        if not modes:
            raise gcmd.error(
                "MODE=%s selects none of the spatial modes %s"
                % (mode_req, spatial["modes"])
            )
        freq_start = gcmd.get_float("FREQ_START", 60.0, above=0.0)
        freq_end = gcmd.get_float("FREQ_END", 320.0, above=freq_start)
        if freq_end > self.MAX_BUZZ_FREQ_HZ:
            raise gcmd.error(
                "buzz frequencies must stay at or below %.0f Hz"
                % (self.MAX_BUZZ_FREQ_HZ,)
            )
        amplitude = gcmd.get_float("AMPLITUDE", 0.02, above=0.0)
        if amplitude > self.MAX_DIFFERENTIAL_AMPLITUDE_MM:
            raise gcmd.error(
                "AMPLITUDE %.3f mm exceeds the %.1f mm buzz ceiling"
                % (amplitude, self.MAX_DIFFERENTIAL_AMPLITUDE_MM)
            )
        # Slow default: more dwell per bin right where the response is
        # smallest (the notch) and more Welch segments per band.
        hz_per_sec = gcmd.get_float("HZ_PER_SEC", 1.0, above=0.0)
        duration = gcmd.get_float("DURATION", 0.0, minval=0.0)
        if duration <= 0.0:
            duration = max((freq_end - freq_start) / hz_per_sec, 0.5)
        ramp = gcmd.get_float(
            "RAMP", min(0.1 * duration, 3.0 / freq_start), above=0.0
        )
        dwell = gcmd.get_int("DWELL_MS", self.dwell_ms, minval=0)
        name = gcmd.get("NAME", "compliance")
        servos = list(spatial["axes"])
        node = self._dynamics_node(gcmd, servos)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        slot_for = {}
        invert_for = {}
        for servo in servos:
            slot = node.get_slot_for_motor(servo)
            if slot is None:
                raise gcmd.error(
                    "motor %r is not on node %s" % (servo, node.name)
                )
            slot_for[servo] = slot
            invert_for[servo] = bool(
                getattr(self._resolve_motor(servo), "invert_direction", False)
            )
        stroke_plan = {
            "freq_start": freq_start,
            "freq_end": freq_end,
            "hz_per_sec": hz_per_sec,
            "duration": duration,
            "ramp": ramp,
            "amplitude": amplitude,
            "dwell_ms": dwell,
            "modes": modes,
        }
        run = self._begin_run(
            gcmd, "compliance", name, "XY", servos, stroke_plan
        )
        try:
            self._prep("X", dwell)
            self._prep("Y", dwell)
            reactor = self.printer.get_reactor()
            for mode in modes:
                row = spatial["frame"][spatial["modes"].index(mode)]
                slot_mask = 0
                sign_mask = 0
                step_servos = []
                for servo, weight in zip(servos, row):
                    if weight == 0.0:
                        continue
                    slot = slot_for[servo]
                    slot_mask |= 1 << slot
                    step_servos.append(servo)
                    # The buzz sign acts in command mm (before the signed
                    # counts-per-mm); the spatial frame is in RAW drive mm
                    # with invert folded in - unfold it for the mask.
                    sign_cmd = (1.0 if weight > 0.0 else -1.0) * (
                        -1.0 if invert_for[servo] else 1.0
                    )
                    if sign_cmd < 0.0:
                        sign_mask |= 1 << slot
                gcmd.respond_info(
                    "compliance sweep, mode %s: %.0f->%.0f Hz over %.1f s, "
                    "amplitude %.3f mm on %s"
                    % (
                        mode,
                        freq_start,
                        freq_end,
                        duration,
                        amplitude,
                        "+".join(step_servos),
                    )
                )
                self._start_capture(mode, step_servos)
                try:
                    engine.resonance_buzz(
                        handle,
                        slot_mask,
                        sign_mask,
                        int(round(freq_start * 1000.0)),
                        int(round(freq_end * 1000.0)),
                        int(round(amplitude * 1e6)),
                        int(round(duration * 1000.0)),
                        int(round(ramp * 1000.0)),
                    )
                    reactor.pause(reactor.monotonic() + duration + 0.2)
                finally:
                    self._stop_capture()
                run.record_step(SweepStep(mode, {}, []))
                if dwell:
                    reactor.pause(reactor.monotonic() + dwell / 1000.0)
            results = self._analyze_and_report(gcmd, run)
        finally:
            self._active_run = None
        freq_by_mode = {}
        peak_by_mode = {}
        flagged = []
        for step in results.get("steps", []):
            comp = step.get("compliance")
            if comp is None:
                continue
            if step.get("flags"):
                flagged.append(
                    "%s: %s" % (step["name"], ",".join(step["flags"]))
                )
            freq_by_mode[comp["mode"]] = comp["f_notch_hz"]
            peak = comp.get("f_peak_hz")
            if peak is not None:
                peak_by_mode[comp["mode"]] = peak
        if not freq_by_mode:
            raise gcmd.error(
                "the analysis produced no compliance results - is servo-cal "
                "up to date? (rebuild with ./install.sh)"
            )
        parts = []
        for m, f in sorted(freq_by_mode.items()):
            parts.append("%s_FREQ=%.1f" % (m.upper(), f))
            if m in peak_by_mode:
                parts.append("%s_PEAK=%.1f" % (m.upper(), peak_by_mode[m]))
        apply_line = "SERVO_SET_COMPLIANCE " + " ".join(parts)
        if flagged:
            gcmd.respond_info(
                "flagged steps: %s - re-measure before applying; to "
                "override anyway: %s" % ("; ".join(flagged), apply_line)
            )
            return
        gcmd.respond_info(
            "to apply (streams live, writes a v7 profile): %s | then "
            "point [ethercat_node %s] dynamics_profile at the written "
            "TOML to keep it across RESTART" % (apply_line, node.name)
        )

    cmd_SERVO_SWEEP_PIN_help = (
        "Staircase-tune one pin-rotor parameter by dwelling a constant "
        "engine buzz as a fixed tone (freq_start==freq_end=FREQ, typically "
        "the mode's notch f_b) in a single mode's frame pattern and "
        "re-streaming the dynamics model live at each step. PARAM (ZETA or "
        "LEAD) steps through VALUES= (2..12, each validated by the same "
        "rules as SERVO_SET_COMPLIANCE ZETA / PIN_LEAD_US); the OTHER pin "
        "parameter stays at its current baseline value. The pin runs "
        "THROUGH the tone - a model swap rebuilds the endpoint's pin state, "
        "so each step's residual demodulator restarts and settles within "
        "the dwell. Scoring is measurement only: after the capture stops "
        "the settled pin-residual magnitude |pin_res| at the tone is read "
        "per step (score is settled MAGNITUDE, not phase - the residual "
        "phase walks 0->180 across the notch naturally and is not a "
        "tuning target), a value->residual table is printed, and the "
        "minimum wins. Nothing is left applied: it prints the ready-to-run "
        "SERVO_SET_COMPLIANCE line with the winning value substituted (the "
        "house pattern - measure prints, set applies). The mode must "
        "already be pinned in the baseline (pin_mass>0) - pin it first "
        "with SERVO_SET_COMPLIANCE PIN=. Baseline profile resolution "
        "matches SERVO_SET_COMPLIANCE (PROFILE=, else the live-tuned "
        "model, else the configured node profile); the pre-sweep model is "
        "restored at the end (also on failure). Params MODE=X|Y FREQ (Hz) "
        "PARAM (ZETA|LEAD, ZETA) VALUES (comma list) DWELL (s, 3) "
        "AMPLITUDE (mm, 0.01) NAME (pin_sweep) PROFILE"
    )

    def _parse_pin_sweep_values(self, gcmd: Any, param: str) -> list[float]:
        raw = gcmd.get("VALUES", None)
        if raw is None:
            raise gcmd.error("VALUES= is required (comma list of 2..12 values)")
        parts = [p.strip() for p in raw.split(",") if p.strip()]
        if not 2 <= len(parts) <= 12:
            raise gcmd.error(
                "VALUES must list 2..12 comma-separated values (got %d)"
                % (len(parts),)
            )
        values: list[float] = []
        for p in parts:
            try:
                v = float(p)
            except ValueError:
                raise gcmd.error("VALUES entry %r is not a number" % (p,))
            # Same rules as SERVO_SET_COMPLIANCE: ZETA above 0 and <= max
            # (get_float above=0.0, maxval=PIN_ZETA_MAX); PIN_LEAD_US in
            # [0, PIN_LEAD_US_MAX] (minval=0.0, maxval=PIN_LEAD_US_MAX).
            if param == "ZETA":
                if not 0.0 < v <= PIN_ZETA_MAX:
                    raise gcmd.error(
                        "VALUES ZETA entry %g must be > 0 and <= %g (same "
                        "rule as SERVO_SET_COMPLIANCE ZETA)" % (v, PIN_ZETA_MAX)
                    )
            elif not 0.0 <= v <= PIN_LEAD_US_MAX:
                raise gcmd.error(
                    "VALUES LEAD entry %g must be >= 0 and <= %g (same rule "
                    "as SERVO_SET_COMPLIANCE PIN_LEAD_US)"
                    % (v, PIN_LEAD_US_MAX)
                )
            values.append(v)
        return values

    def _pin_sweep_model(
        self,
        baseline: dict[str, Any],
        mode_i: int,
        param: str,
        value: float,
    ) -> dict[str, Any]:
        """Copy the baseline and set the one swept pin parameter on the
        pinned mode; the other pin parameter is preserved by the copy."""
        updated = _copy_dynamics(baseline)
        updated["ff_lead_us"] = baseline.get("ff_lead_us", 0.0)
        if param == "ZETA":
            updated["pin_zeta"][mode_i] = value
        else:  # LEAD -> pin_lead_us is a whole-model scalar
            updated["pin_lead_us"] = value
        return updated

    def _pin_sweep_scores(
        self, gcmd: Any, results: dict[str, Any], values: list[float]
    ) -> list[tuple[float, float | None]]:
        """Read the settled pin-residual magnitude the analyzer already
        produces per step (drives[*].metrics.pin_residual_mm, the low-passed
        |phasor| at the tone). The mode's residual rides the drive block of
        the same index, so the max over the step's captured drives isolates
        it. Errors when no pin channels are present or every step reads ~0."""
        by_name = {s.get("name"): s for s in results.get("steps") or []}
        rows: list[tuple[float, float | None]] = []
        for i, value in enumerate(values):
            step = by_name.get("v%d" % (i,))
            residual_mm: float | None = None
            if step is not None:
                for drive in (step.get("drives") or {}).values():
                    mag = (drive.get("metrics") or {}).get("pin_residual_mm")
                    if mag is not None:
                        residual_mm = (
                            mag
                            if residual_mm is None
                            else max(residual_mm, mag)
                        )
            rows.append((value, residual_mm))
        scored = [(v, r) for v, r in rows if r is not None]
        if not scored or all(r <= 1e-9 for _v, r in scored):
            raise gcmd.error(
                "no pin residual in the capture - the swept mode must be "
                "actively pinned (pin_mass>0) and the kalico build must "
                "stream pin_res_re/pin_res_im (rebuild the endpoint with "
                "./install.sh)"
            )
        return rows

    def cmd_SERVO_SWEEP_PIN(self, gcmd: Any) -> None:
        if tomllib is None:
            raise gcmd.error("SERVO_SWEEP_PIN requires Python 3.11+ (tomllib)")
        kin = self._kin()
        spatial = servo_strokes.spatial_frame(kin)
        if spatial is None:
            raise gcmd.error(
                "SERVO_SWEEP_PIN needs servo rails on X/Y - no spatial frame "
                "available"
            )
        mode_req = gcmd.get("MODE", "").upper()
        wanted = [m for m in ("x", "y") if m.upper() in mode_req]
        modes = [m for m in spatial["modes"] if m in wanted]
        if len(modes) != 1:
            raise gcmd.error(
                "MODE= must select exactly one spatial mode (X or Y); got "
                "%r from %s" % (mode_req, spatial["modes"])
            )
        mode = modes[0]
        freq = gcmd.get_float("FREQ", None, above=0.0)
        if freq is None:
            raise gcmd.error("FREQ= is required (the dwell tone in Hz)")
        if freq < 20.0 or freq > self.MAX_BUZZ_FREQ_HZ:
            raise gcmd.error(
                "FREQ must be between 20 and %.0f Hz (the dwell tone, "
                "typically f_b)" % (self.MAX_BUZZ_FREQ_HZ,)
            )
        param = gcmd.get("PARAM", "ZETA").upper()
        if param not in ("ZETA", "LEAD"):
            raise gcmd.error("PARAM must be ZETA or LEAD (got %r)" % (param,))
        values = self._parse_pin_sweep_values(gcmd, param)
        dwell_s = gcmd.get_float("DWELL", 3.0, minval=1.0)
        amplitude = gcmd.get_float("AMPLITUDE", 0.01, above=0.0)
        if amplitude > self.MAX_DIFFERENTIAL_AMPLITUDE_MM:
            raise gcmd.error(
                "AMPLITUDE %.3f mm exceeds the %.1f mm buzz ceiling"
                % (amplitude, self.MAX_DIFFERENTIAL_AMPLITUDE_MM)
            )
        name = gcmd.get("NAME", "pin_sweep")
        servos = list(spatial["axes"])
        node = self._dynamics_node(gcmd, servos)
        handle = node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % (node.name,)
            )
        engine = self.printer.lookup_object("motion_engine")
        profile_path, baseline = self._load_baseline_dynamics(gcmd, node)
        if mode not in baseline["modes"]:
            raise gcmd.error(
                "mode %s not in profile %s (modes %s)"
                % (mode, profile_path, baseline["modes"])
            )
        mode_i = baseline["modes"].index(mode)
        if not baseline["pin_mass"][mode_i] > 0.0:
            raise gcmd.error(
                "mode %s is not pinned in the baseline (pin_mass=0) - pin it "
                "first with SERVO_SET_COMPLIANCE PIN=%s %s_PEAK=... then "
                "sweep" % (mode, mode.upper(), mode.upper())
            )
        slot_for: dict[str, int] = {}
        invert_for: dict[str, bool] = {}
        for servo in servos:
            slot = node.get_slot_for_motor(servo)
            if slot is None:
                raise gcmd.error(
                    "motor %r is not on node %s" % (servo, node.name)
                )
            slot_for[servo] = slot
            invert_for[servo] = bool(
                getattr(self._resolve_motor(servo), "invert_direction", False)
            )
        row = spatial["frame"][spatial["modes"].index(mode)]
        slot_mask = 0
        sign_mask = 0
        step_servos: list[str] = []
        for servo, weight in zip(servos, row):
            if weight == 0.0:
                continue
            slot = slot_for[servo]
            slot_mask |= 1 << slot
            step_servos.append(servo)
            sign_cmd = (1.0 if weight > 0.0 else -1.0) * (
                -1.0 if invert_for[servo] else 1.0
            )
            if sign_cmd < 0.0:
                sign_mask |= 1 << slot
        # One tone spans every dwell; size it to cover the whole staircase
        # plus the per-step re-stream overhead, and let it lapse (the buzz
        # is duration-bounded - there is no early-stop command).
        ramp = min(0.3, 3.0 / freq)
        total_s = len(values) * (dwell_s + 0.5) + ramp + 1.0
        stroke_plan = {
            "freq": freq,
            "amplitude": amplitude,
            "dwell_s": dwell_s,
            "ramp": ramp,
            "mode": mode,
            "param": param,
            "values": values,
        }
        run = self._begin_run(
            gcmd, "pin_sweep", name, "XY", servos, stroke_plan
        )
        reactor = self.printer.get_reactor()
        gcmd.respond_info(
            "pin sweep, mode %s: %s over %d values x %.1f s at %.1f Hz, "
            "amplitude %.3f mm on %s"
            % (
                mode,
                param,
                len(values),
                dwell_s,
                freq,
                amplitude,
                "+".join(step_servos),
            )
        )
        self._prep("X", 0)
        self._prep("Y", 0)
        try:
            engine.resonance_buzz(
                handle,
                slot_mask,
                sign_mask,
                int(round(freq * 1000.0)),
                int(round(freq * 1000.0)),
                int(round(amplitude * 1e6)),
                int(round(total_s * 1000.0)),
                int(round(ramp * 1000.0)),
            )
            for i, value in enumerate(values):
                updated = self._pin_sweep_model(baseline, mode_i, param, value)
                send_dynamics_model(engine, handle, updated)
                step_name = "v%d" % (i,)
                t_start = round(reactor.monotonic(), 3)
                self._start_capture(step_name, step_servos)
                try:
                    reactor.pause(reactor.monotonic() + dwell_s)
                finally:
                    self._stop_capture()
                run.record_step(
                    SweepStep(
                        step_name,
                        {
                            "value": value,
                            "t_start_s": t_start,
                            "t_end_s": round(reactor.monotonic(), 3),
                        },
                        [],
                    )
                )
            results = self._run_analyze(gcmd, run)
        finally:
            # Restore the pre-sweep model (also on failure), matching the
            # gain-sweep restore discipline.
            try:
                send_dynamics_model(engine, handle, baseline)
                node.set_live_dynamics_profile(profile_path)
            finally:
                self._active_run = None
        rows = self._pin_sweep_scores(gcmd, results, values)
        table = ", ".join(
            "%g -> %s" % (v, "%.2f um" % (r * 1e3,) if r is not None else "n/a")
            for v, r in rows
        )
        best_value, best_res = min(
            ((v, r) for v, r in rows if r is not None), key=lambda t: t[1]
        )
        # Reconstruct the FRF peak the baseline pin encodes so the printed
        # line is fully specified: pin_mass = mass*(1-(f_b/f_peak)^2).
        comp = baseline["compliance"][mode_i]
        f_b = 1.0 / (2.0 * math.pi * math.sqrt(comp))
        fraction = baseline["pin_mass"][mode_i] / baseline["mass"][mode_i]
        f_peak = f_b / math.sqrt(max(1.0 - fraction, 1e-9))
        zeta_val = (
            best_value if param == "ZETA" else baseline["pin_zeta"][mode_i]
        )
        lead_val = (
            best_value if param == "LEAD" else baseline.get("pin_lead_us", 0.0)
        )
        apply_line = (
            "SERVO_SET_COMPLIANCE PIN=%s %s_PEAK=%.1f ZETA=%g PIN_LEAD_US=%g"
            % (mode.upper(), mode.upper(), f_peak, zeta_val, lead_val)
        )
        structured_log.event(
            "calibration",
            "pin_sweep",
            mode=mode,
            param=param,
            values=values,
            residuals_um=[
                None if r is None else round(r * 1e3, 4) for _v, r in rows
            ],
            best_value=best_value,
            run_dir=run.run_dir,
        )
        gcmd.respond_info(
            "pin sweep %s (mode %s) residual: %s | minimum at %s=%g "
            "(%.2f um) | to apply: %s"
            % (
                param,
                mode,
                table,
                param,
                best_value,
                best_res * 1e3,
                apply_line,
            )
        )
