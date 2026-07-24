import { html } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { useQuery } from "@tanstack/preact-query";
import { psdPlot } from "./uplot-chart";
import { PALETTE, state } from "./state";
import { useStore } from "./store";
import { getPinCompare, sweepLabel } from "./api/pin-compare";
import type { PinCompareManifest } from "./api/pin-compare";

// --- pin compare: overlay one resonance sweep per pin-parameter value -------

type YMode = "ratio" | "accel";
type YScale = "log" | "linear";

/// Color a sweep by its position in the manifest so a series keeps its color
/// even as others are hidden.
function sweepColor(index: number): string {
  return PALETTE[index % PALETTE.length];
}

function PinCompareChart({
  manifest,
  yMode,
  yScale,
  hidden,
}: {
  manifest: PinCompareManifest;
  yMode: YMode;
  yScale: YScale;
  hidden: Set<number>;
}) {
  const hostRef = useRef<HTMLDivElement>(null);
  const hiddenSig = [...hidden].sort((a, b) => a - b).join(",");
  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const traces = manifest.sweeps
      .map((sweep, index) => ({ sweep, index }))
      .filter(({ index }) => !hidden.has(index))
      .map(({ sweep, index }) => ({
        freq: sweep.curve_hz,
        y: yMode === "ratio" ? sweep.response_ratio : sweep.accel_mm_s2,
        color: sweepColor(index),
        dashed: false,
        label: sweepLabel(manifest.param, sweep),
      }));
    if (!traces.length) {
      host.replaceChildren();
      return;
    }
    const linear = yScale === "linear";
    const plot = psdPlot(host, {
      width: 860,
      height: 320,
      traces,
      band: [manifest.freq_start, manifest.freq_end],
      yTitle:
        yMode === "ratio" ? "response ratio (accel / commanded)" : "accel (mm/s²)",
      linear,
      zeroFloor: yMode === "accel",
      formatValue: (v) =>
        yMode === "ratio"
          ? v.toPrecision(3)
          : Math.abs(v) >= 1000
            ? v.toExponential(2)
            : v.toFixed(0),
    });
    return () => plot.destroy();
  }, [manifest, yMode, yScale, hiddenSig]);
  return html`<div class="chart-box"><div ref=${hostRef}></div></div>`;
}

function PinCompareLegend({
  manifest,
  hidden,
  onToggle,
}: {
  manifest: PinCompareManifest;
  hidden: Set<number>;
  onToggle: (index: number) => void;
}) {
  return html`<div class="legend">
    ${manifest.sweeps.map(
      (sweep, index) => html`<span
        key=${index}
        role="button"
        title="click to show/hide"
        style=${`cursor:pointer;opacity:${hidden.has(index) ? 0.35 : 1}`}
        onClick=${() => onToggle(index)}
        ><span class="swatch" style=${`background:${sweepColor(index)}`}></span>${sweepLabel(
          manifest.param,
          sweep
        )}</span
      >`
    )}
  </div>`;
}

/// Pin-compare overlay as a tune-tab section, driven by the runs-table
/// selection: it appears only while a pin-compare row is selected there
/// (state.pinCompareSelected) and shows exactly that comparison — no
/// dropdown of its own.
function PinCompareSection() {
  useStore();
  const active = state.pinCompareSelected;
  const [yMode, setYMode] = useState<YMode>("ratio");
  const [yScale, setYScale] = useState<YScale>("log");
  const [hidden, setHidden] = useState<Set<number>>(new Set());

  const detail = useQuery({
    queryKey: ["pin-compare", active],
    queryFn: () => getPinCompare(active as string),
    enabled: active != null,
  });
  const manifest = active != null ? (detail.data ?? null) : null;

  // A fresh comparison starts with every sweep visible.
  useEffect(() => {
    setHidden(new Set());
  }, [active]);

  const onToggle = (index: number) => {
    setHidden((prev) => {
      const next = new Set(prev);
      if (next.has(index)) next.delete(index);
      else next.add(index);
      return next;
    });
  };

  if (active == null) return null;

  return html`<section class="pin-compare-section">
    <div class="section-head"><h2>pin compare — ${active}</h2></div>
      <div class="section-tools">
        <label>y
          <select
            value=${yMode}
            onChange=${(e: Event) => setYMode((e.target as HTMLSelectElement).value as YMode)}
          >
            <option value="ratio">normalized ratio</option>
            <option value="accel">raw accel</option>
          </select>
        </label>
        <label>scale
          <select
            value=${yScale}
            onChange=${(e: Event) => setYScale((e.target as HTMLSelectElement).value as YScale)}
          >
            <option value="log">log</option>
            <option value="linear">linear</option>
          </select>
        </label>
      </div>
      ${detail.error
        ? html`<p class="note">failed to load ${active}: ${String(detail.error)}</p>`
        : null}
      ${manifest
        ? html`<${PinCompareChart}
              manifest=${manifest}
              yMode=${yMode}
              yScale=${yScale}
              hidden=${hidden}
            />
            <${PinCompareLegend} manifest=${manifest} hidden=${hidden} onToggle=${onToggle} />`
        : html`<p class="note">loading ${active}…</p>`}
  </section>`;
}

export { PinCompareSection, sweepColor };
