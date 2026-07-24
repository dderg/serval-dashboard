import { client, unwrap } from "./client";

import type { components } from "./openapi.generated";

type Schema = components["schemas"];
export type PinCompareSummary = Schema["PinCompareSummary"];
export type PinCompareSweep = Schema["PinCompareSweep"];
export type PinCompareManifest = Schema["PinCompareManifest"];

export async function listPinCompares() {
  return unwrap(await client.GET("/api/pin-compare"));
}

export async function getPinCompare(name: string) {
  return unwrap(await client.GET("/api/pin-compare/{name}", { params: { path: { name } } }));
}

/// Stable per-sweep label: '<PARAM>=<value> @<hz_per_sec> Hz/s', plus the
/// excitation strength when recorded (ApH in mm/s^2 per Hz) so sweeps taken
/// at different excitation levels stay distinguishable in the overlay.
export function sweepLabel(param: string, sweep: PinCompareSweep): string {
  const base = `${param}=${formatNumber(sweep.value)} @${formatNumber(sweep.hz_per_sec)} Hz/s`;
  return sweep.accel_per_hz != null ? `${base} ${formatNumber(sweep.accel_per_hz)} ApH` : base;
}

function formatNumber(v: number): string {
  if (!Number.isFinite(v)) return String(v);
  const rounded = Math.round(v * 1000) / 1000;
  return String(rounded);
}
