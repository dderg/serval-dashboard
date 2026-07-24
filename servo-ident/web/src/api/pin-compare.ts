// The pin-compare routes (GET /api/pin-compare, /api/pin-compare/{name}) are
// served by the same host process as the typed run routes, but their schemas
// are not yet in the committed openapi.generated.ts (regenerating it is a
// gated step), so this loader talks to them with a plain typed fetch rather
// than the openapi-fetch client. The shapes below mirror the Rust
// PinCompareSummary / PinCompareManifest contract in serve.rs exactly.

export interface PinCompareSummary {
  name: string;
  mode: string;
  param: string;
  n_sweeps: number;
  created_utc: string;
}

export interface PinCompareSweep {
  value: number;
  hz_per_sec: number;
  /** mm/s^2 per Hz (commanded accel = ApH * f); absent in pre-ApH manifests. */
  accel_per_hz?: number;
  amplitude_mm: number;
  curve_hz: number[];
  accel_mm_s2: number[];
  response_ratio: number[];
}

export interface PinCompareManifest {
  name: string;
  created_utc: string;
  mode: string;
  param: string;
  freq_start: number;
  freq_end: number;
  baseline_profile: string | null;
  sweeps: PinCompareSweep[];
}

async function getJson<T>(url: string): Promise<T> {
  const response = await globalThis.fetch(url);
  if (!response.ok) {
    const detail = await response.text().catch(() => "");
    throw new Error(`${response.status} ${url}: ${detail || response.statusText}`);
  }
  return (await response.json()) as T;
}

export async function listPinCompares(): Promise<PinCompareSummary[]> {
  return getJson<PinCompareSummary[]>("/api/pin-compare");
}

export async function getPinCompare(name: string): Promise<PinCompareManifest> {
  return getJson<PinCompareManifest>(`/api/pin-compare/${encodeURIComponent(name)}`);
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
