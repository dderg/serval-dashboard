import { html } from "htm/preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { useQuery } from "@tanstack/preact-query";
import { psdPlot } from "./uplot-chart";
import { PALETTE } from "./state";
import { listPinCompares, getPinCompare, sweepLabel } from "./api/pin-compare";
import type { PinCompareManifest } from "./api/pin-compare";

// --- pin compare: overlay one resonance sweep per pin-parameter value -------

const SELECTED_KEY = "servoCalPinCompareSelected";

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

function PinCompareView() {
  const list = useQuery({ queryKey: ["pin-compare"], queryFn: listPinCompares });
  const names = list.data ?? [];
  const [selected, setSelected] = useState<string | null>(
    () => localStorage.getItem(SELECTED_KEY)
  );
  const [yMode, setYMode] = useState<YMode>("ratio");
  const [yScale, setYScale] = useState<YScale>("log");
  const [hidden, setHidden] = useState<Set<number>>(new Set());

  // Default to the newest comparison once the list loads, and drop a stale
  // localStorage selection that no longer exists.
  const validNames = names.map((n) => n.name).join("|");
  useEffect(() => {
    if (!names.length) return;
    if (!selected || !names.some((n) => n.name === selected)) {
      setSelected(names[0].name);
    }
  }, [validNames]);

  const active = selected && names.some((n) => n.name === selected) ? selected : null;
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

  const onSelect = (name: string) => {
    setSelected(name);
    localStorage.setItem(SELECTED_KEY, name);
  };
  const onToggle = (index: number) => {
    setHidden((prev) => {
      const next = new Set(prev);
      if (next.has(index)) next.delete(index);
      else next.add(index);
      return next;
    });
  };

  return html`<main class="analysis">
    <section class="section">
      <div class="section-head"><h2>pin compare</h2></div>
      <div class="section-tools">
        <label>comparison
          <select
            value=${active ?? ""}
            onChange=${(e: Event) => onSelect((e.target as HTMLSelectElement).value)}
          >
            ${names.length === 0
              ? html`<option value="">no comparisons</option>`
              : names.map(
                  (n) => html`<option key=${n.name} value=${n.name}
                    >${n.name} — ${n.param} (${n.n_sweeps} sweeps)</option
                  >`
                )}
          </select>
        </label>
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
      ${list.error
        ? html`<p class="note">failed to load comparisons: ${String(list.error)}</p>`
        : null}
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
        : names.length
          ? html`<p class="note">select a comparison to overlay its sweeps</p>`
          : html`<p class="note">
              no pin comparisons yet — run SERVO_PIN_COMPARE_SWEEP to create one
            </p>`}
    </section>
  </main>`;
}

function PinComparePage() {
  return html`<div class="workspace single"><${PinCompareView} /></div>`;
}

export { PinComparePage, sweepColor };
