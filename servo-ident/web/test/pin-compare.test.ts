import { afterEach, beforeAll, expect, test } from "bun:test";
import { registerDom } from "./dom";
import type * as PinCompareMod from "../src/api/pin-compare";

// The loaders go through openapi-fetch, which builds a Request from a
// relative route — that only resolves once happy-dom has installed a
// location-aware Request, so the module must import after registration.
registerDom();

let api: typeof PinCompareMod;

beforeAll(async () => {
  api = await import("../src/api/pin-compare");
});

const realFetch = globalThis.fetch;

function stubFetch(handler: (path: string) => { status: number; body: string }) {
  globalThis.fetch = (async (input: RequestInfo | URL) => {
    const url = input instanceof Request ? input.url : String(input);
    const { status, body } = handler(new URL(url).pathname);
    return new Response(body, { status });
  }) as typeof fetch;
}

afterEach(() => {
  globalThis.fetch = realFetch;
});

test("listPinCompares parses the summary array from /api/pin-compare", async () => {
  let hit = "";
  stubFetch((path) => {
    hit = path;
    return {
      status: 200,
      body: JSON.stringify([
        { name: "zeta-x", mode: "x", param: "ZETA", n_sweeps: 3, created_utc: "2026-07-24T00:00:00Z" },
      ]),
    };
  });
  const rows = await api.listPinCompares();
  expect(hit).toBe("/api/pin-compare");
  expect(rows).toHaveLength(1);
  expect(rows[0].name).toBe("zeta-x");
  expect(rows[0].n_sweeps).toBe(3);
});

test("getPinCompare fetches the manifest by name, url-encoded", async () => {
  let hit = "";
  stubFetch((path) => {
    hit = path;
    return {
      status: 200,
      body: JSON.stringify({
        name: "a b",
        created_utc: "2026-07-24T00:00:00Z",
        mode: "y",
        param: "LEAD",
        freq_start: 10,
        freq_end: 120,
        baseline_profile: null,
        sweeps: [],
      }),
    };
  });
  const manifest = await api.getPinCompare("a b");
  expect(hit).toBe("/api/pin-compare/a%20b");
  expect(manifest.param).toBe("LEAD");
});

test("loaders throw carrying the status and the server's reason", async () => {
  stubFetch(() => ({ status: 404, body: "no such comparison" }));
  await expect(api.getPinCompare("missing")).rejects.toThrow(/404 .*no such comparison/);
});

test("sweepLabel encodes param, value, and sweep rate", () => {
  const sweep: PinCompareMod.PinCompareSweep = {
    value: 0.125,
    hz_per_sec: 5,
    amplitude_mm: 0.4,
    curve_hz: [],
    accel_mm_s2: [],
    response_ratio: [],
  };
  expect(api.sweepLabel("ZETA", sweep)).toBe("ZETA=0.125 @5 Hz/s");
});
