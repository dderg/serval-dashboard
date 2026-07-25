import { html } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { useQuery } from "@tanstack/preact-query";
import { psdPlot } from "./uplot-chart";
import { PALETTE, state } from "./state";
import { useStore } from "./store";
import { runData, runPinCompareQuery } from "./queries/runs";
import type { PinCompare, PinCompareSweep } from "./api/runs";

// --- pin compare: overlay one resonance sweep per pin-parameter value -------

const PIN_COMPARE_EXPERIMENT = "pin_compare";

/// Color a sweep by its position in the manifest so a series keeps its color
/// even as others are hidden.
function sweepColor(index: number): string {
  return PALETTE[index % PALETTE.length];
}

function formatNumber(v: number): string {
  if (!Number.isFinite(v)) return String(v);
  const rounded = Math.round(v * 1000) / 1000;
  return String(rounded);
}

/// Stable per-sweep label: '<PARAM>=<value> @<hz_per_sec> Hz/s <ApH>', the
/// excitation strength included so sweeps taken at different levels stay
/// distinguishable in the overlay.
function sweepLabel(param: string, sweep: PinCompareSweep): string {
  return (
    `${param}=${formatNumber(sweep.value)} @${formatNumber(sweep.hz_per_sec)} Hz/s ` +
    `${formatNumber(sweep.accel_per_hz)} ApH`
  );
}

/// The comparison run the overlay charts: the most recently selected run of
/// the pin_compare experiment. Selecting an ordinary run leaves the last
/// comparison charted; deselecting it hides the section.
function activeCompareRun(): string | null {
  let active: string | null = null;
  for (const name of state.selected) {
    if (runData(name)?.experiment === PIN_COMPARE_EXPERIMENT) active = name;
  }
  return active;
}

/// Raw accel amplitude on a zero-based linear axis, always. The whole point
/// of the overlay is comparing spike heights between swept values: a log
/// axis flattens exactly that comparison, and the normalized response ratio
/// divides out the commanded accel, hiding how big the spike actually is.
function PinCompareChart({ compare, hidden }: { compare: PinCompare; hidden: Set<number> }) {
  const hostRef = useRef<HTMLDivElement>(null);
  const hiddenSig = [...hidden].sort((a, b) => a - b).join(",");
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const traces = compare.sweeps
      .map((sweep, index) => ({ sweep, index }))
      .filter(({ index }) => !hidden.has(index))
      .map(({ sweep, index }) => ({
        freq: sweep.curve_hz,
        y: sweep.accel_mm_s2,
        color: sweepColor(index),
        dashed: false,
        label: sweepLabel(compare.param, sweep),
      }));
    if (!traces.length) {
      host.replaceChildren();
      return;
    }
    const plot = psdPlot(host, {
      width: 860,
      height: 320,
      traces,
      band: [compare.freq_start, compare.freq_end],
      yTitle: "accel (mm/s²)",
      linear: true,
      zeroFloor: true,
      formatValue: (v) => (Math.abs(v) >= 1000 ? v.toExponential(2) : v.toFixed(0)),
    });
    return () => plot.destroy();
  }, [compare, hiddenSig]);
  return html`<div class="chart-box"><div ref=${hostRef}></div></div>`;
}

function PinCompareLegend({
  compare,
  hidden,
  onToggle,
}: {
  compare: PinCompare;
  hidden: Set<number>;
  onToggle: (index: number) => void;
}) {
  return html`<div class="legend">
    ${compare.sweeps.map(
      (sweep, index) => html`<span
        key=${index}
        role="button"
        title="click to show/hide"
        style=${`cursor:pointer;opacity:${hidden.has(index) ? 0.35 : 1}`}
        onClick=${() => onToggle(index)}
        ><span class="swatch" style=${`background:${sweepColor(index)}`}></span>${sweepLabel(
          compare.param,
          sweep
        )}</span
      >`
    )}
  </div>`;
}

/// Keyed on the run, so switching comparisons remounts and every sweep
/// starts visible again.
function PinCompareBody({ run }: { run: string }) {
  const [hidden, setHidden] = useState<Set<number>>(new Set());
  const detail = useQuery(runPinCompareQuery(run));

  const onToggle = (index: number) => {
    setHidden((prev) => {
      const next = new Set(prev);
      if (next.has(index)) next.delete(index);
      else next.add(index);
      return next;
    });
  };

  if (detail.error) {
    return html`<p class="note">failed to load ${run}: ${String(detail.error)}</p>`;
  }
  const compare = detail.data;
  if (!compare) return html`<p class="note">loading ${run}…</p>`;
  return html`<${PinCompareChart} compare=${compare} hidden=${hidden} />
    <${PinCompareLegend} compare=${compare} hidden=${hidden} onToggle=${onToggle} />`;
}

/// Pin-compare overlay as a tune-tab section, driven by the runs-table
/// selection: it appears only while a pin_compare run is selected there and
/// charts exactly that run's curves — no dropdown of its own.
function PinCompareSection() {
  useStore();
  const active = activeCompareRun();
  if (active == null) return null;
  return html`<section class="pin-compare-section">
    <div class="section-head"><h2>pin compare — ${active}</h2></div>
    <${PinCompareBody} key=${active} run=${active} />
  </section>`;
}

export { PinCompareSection, PIN_COMPARE_EXPERIMENT, sweepLabel };
