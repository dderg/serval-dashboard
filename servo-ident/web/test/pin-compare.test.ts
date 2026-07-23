import { afterEach, expect, test } from "bun:test";
import { listPinCompares, getPinCompare, sweepLabel } from "../src/api/pin-compare";
import type { PinCompareSweep } from "../src/api/pin-compare";

const realFetch = globalThis.fetch;

function stubFetch(handler: (url: string) => { status: number; body: string }) {
  globalThis.fetch = (async (input: RequestInfo | URL) => {
    const url = typeof input === "string" ? input : input.toString();
    const { status, body } = handler(url);
    return new Response(body, { status });
  }) as typeof fetch;
}

afterEach(() => {
  globalThis.fetch = realFetch;
});

test("listPinCompares parses the summary array from /api/pin-compare", async () => {
  let hit = "";
  stubFetch((url) => {
    hit = url;
    return {
      status: 200,
      body: JSON.stringify([
        { name: "zeta-x", mode: "x", param: "ZETA", n_sweeps: 3, created_utc: "2026-07-24T00:00:00Z" },
      ]),
    };
  });
  const rows = await listPinCompares();
  expect(hit).toBe("/api/pin-compare");
  expect(rows).toHaveLength(1);
  expect(rows[0].name).toBe("zeta-x");
  expect(rows[0].n_sweeps).toBe(3);
});

test("getPinCompare fetches the manifest by name, url-encoded", async () => {
  let hit = "";
  stubFetch((url) => {
    hit = url;
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
  const manifest = await getPinCompare("a b");
  expect(hit).toBe("/api/pin-compare/a%20b");
  expect(manifest.param).toBe("LEAD");
});

test("loaders throw with status and url when the response is not ok", async () => {
  stubFetch(() => ({ status: 404, body: "no such comparison" }));
  await expect(getPinCompare("missing")).rejects.toThrow(
    "404 /api/pin-compare/missing: no such comparison"
  );
});

test("sweepLabel encodes param, value, and sweep rate", () => {
  const sweep: PinCompareSweep = {
    value: 0.125,
    hz_per_sec: 5,
    amplitude_mm: 0.4,
    curve_hz: [],
    accel_mm_s2: [],
    response_ratio: [],
  };
  expect(sweepLabel("ZETA", sweep)).toBe("ZETA=0.125 @5 Hz/s");
});
