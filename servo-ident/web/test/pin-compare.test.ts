import { afterEach, beforeAll, expect, test } from "bun:test";
import { registerDom } from "./dom";
import type * as RunsApiMod from "../src/api/runs";
import type * as PinCompareMod from "../src/pin-compare";

// The loaders go through openapi-fetch, which builds a Request from a
// relative route — that only resolves once happy-dom has installed a
// location-aware Request, so the module must import after registration.
registerDom();

let api: typeof RunsApiMod;
let pinCompare: typeof PinCompareMod;

beforeAll(async () => {
  api = await import("../src/api/runs");
  pinCompare = await import("../src/pin-compare");
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

test("getRunPinCompare fetches the run's comparison block, url-encoded", async () => {
  let hit = "";
  stubFetch((path) => {
    hit = path;
    return {
      status: 200,
      body: JSON.stringify({
        mode: "y",
        param: "LEAD",
        freq_start: 10,
        freq_end: 120,
        baseline_profile: null,
        sweeps: [],
      }),
    };
  });
  const compare = await api.getRunPinCompare("a b");
  expect(hit).toBe("/api/runs/a%20b/pin_compare");
  expect(compare.param).toBe("LEAD");
});

test("a run that is not a comparison throws with the status and reason", async () => {
  stubFetch(() => ({ status: 404, body: "run is not a pin comparison" }));
  await expect(api.getRunPinCompare("plain_run")).rejects.toThrow(
    /404 .*not a pin comparison/
  );
});

test("sweepLabel encodes param, value, sweep rate, and excitation strength", () => {
  const sweep: RunsApiMod.PinCompareSweep = {
    value: 0.125,
    hz_per_sec: 5,
    accel_per_hz: 75,
    amplitude_mm: 0.4,
    curve_hz: [],
    accel_mm_s2: [],
    response_ratio: [],
  };
  expect(pinCompare.sweepLabel("ZETA", sweep)).toBe("ZETA=0.125 @5 Hz/s 75 ApH");
});
