import { afterAll, beforeAll, expect, test } from "bun:test";
import { act } from "preact/test-utils";
import {
  registerDom,
  installFetchStub,
  installDomHarness,
  indexHtmlBody,
  fixtureJson,
  nextFrame,
  settleDom,
  resetRunState,
} from "./dom";
import type * as ClientMod from "../src/queries/client";
import type * as RunsQueryMod from "../src/queries/runs";
import type * as RunsMod from "../src/runs";
import type * as StateMod from "../src/state";
import type { RunSummary } from "../src/api/runs";
import type { html as htmlTag, render as renderFn } from "htm/preact";
import type { VNode } from "preact";

registerDom();
const { unmatched } = installFetchStub();

// TanStack batches observer notifications on a real timer; a synchronous
// scheduler makes query-driven rerenders flush inside the settle loop.
// The notify manager is process-global, so afterAll puts the default back —
// query-core's own default is `systemSetTimeoutZero`, which it does not export.
const { notifyManager } = await import("@tanstack/preact-query");
const defaultScheduler = (cb: () => void) => setTimeout(cb, 0);
notifyManager.setScheduler((cb: () => void) => cb());

const PIN_RUN = "pin_20260725_120000";
const PIN_COMMAND = "SERVO_SWEEP_PIN MODE=X PARAM=ZETA FREQ=137 VALUES=0.005,0.02 NAME=pin";
// The staircase names each step after the value it swept — those names are
// what the chart legends print, so they are the thing under test.
const STEP_NAMES = ["v0_zeta0p005", "v1_zeta0p02"];
const STEP_ZETA = [0.005, 0.02];

const pinSummary: RunSummary = {
  name: PIN_RUN,
  mtime_utc: "2026-07-25T12:00:00Z",
  experiment: "pin_sweep",
  tag: "pin",
  axis: "X",
  command: PIN_COMMAND,
  has_results: true,
  verdict: null,
  note: null,
};

/// A pin staircase carries the same per-step payload shape as the captured
/// gain sweep — following-error PSD plus the toolhead accel PSD — so the
/// fixtures are reused with the staircase's experiment and step names.
function stepDoc<T extends { steps: { name: string }[] }>(name: string): T {
  const doc = fixtureJson<T>(name);
  if (doc.steps.length !== STEP_NAMES.length) {
    throw new Error(
      `${name} fixture has ${doc.steps.length} steps, this test needs ${STEP_NAMES.length}`
    );
  }
  doc.steps.forEach((s, i) => {
    s.name = STEP_NAMES[i];
  });
  return doc;
}

interface PlotFixtureStep {
  name: string;
  psd: { accel: unknown } | null;
  path: { cmd_x_mm: number[]; cmd_y_mm: number[]; act_x_mm: number[]; act_y_mm: number[] } | null;
}

const pinPlot = stepDoc<{ version: number; steps: PlotFixtureStep[] }>("plot_series");
if (!pinPlot.steps.every((s) => s.psd && s.psd.accel)) {
  throw new Error("plot_series fixture lost its per-step accel PSD — nothing left to assert");
}

const pinManifest = stepDoc<{
  experiment: string;
  command: string;
  tag: string;
  axis: string;
  steps: { name: string; swept: Record<string, number> }[];
}>("manifest");
pinManifest.experiment = "pin_sweep";
pinManifest.command = PIN_COMMAND;
pinManifest.tag = "pin";
pinManifest.steps.forEach((s, i) => {
  s.swept = { zeta: STEP_ZETA[i] };
});

const pinResults = stepDoc<{ version: number; steps: { name: string }[] }>("results");

const pinPath = {
  version: 1,
  steps: pinPlot.steps
    .filter((s) => s.path)
    .map((s) => ({
      name: s.name,
      n_records: s.path!.cmd_x_mm.length,
      truncated: false,
      path: s.path,
    })),
};

const baseFetch = globalThis.fetch;
const json = (body: unknown) =>
  new Response(JSON.stringify(body), {
    status: 200,
    headers: { "Content-Type": "application/json" },
  });

// The shared stub serves one gain_sweep run; this layer appends the pin
// staircase after it, so the staircase only reaches the tune table if the
// page's experiment allow-list admits it.
globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
  const req = input instanceof Request ? input : new Request(input, init);
  const path = new URL(req.url, "http://127.0.0.1/").pathname;
  if (path === "/api/runs") {
    return json([...fixtureJson<RunSummary[]>("runs"), pinSummary]);
  }
  if (path === `/api/runs/${PIN_RUN}/manifest`) return json(pinManifest);
  if (path === `/api/runs/${PIN_RUN}/results`) return json(pinResults);
  if (path === `/api/runs/${PIN_RUN}/plot_series`) return json(pinPlot);
  if (path === `/api/runs/${PIN_RUN}/path`) return json(pinPath);
  return baseFetch(req);
}) as typeof fetch;

const { consoleErrors, cleanup } = installDomHarness();

async function settle() {
  await settleDom();
  await nextFrame();
}

let html: typeof htmlTag;
let render: typeof renderFn;
let client: typeof ClientMod;
let runsQ: typeof RunsQueryMod;
let runs: typeof RunsMod;
let state: typeof StateMod.state;

/// bun test shares one process and this file drives the runs table, so it
/// hands back the table state it found. Restoring is not optional for the
/// colors: a neighbouring file's charts call `runColor` for every selected
/// run, which throws when the map is empty. See resetRunState in dom.ts for
/// why this file restores while pin-compare-row.test.ts clears outright.
interface TableSelection {
  selected: string[];
  pinned: string[];
  colors: [string, string][];
  autoSelected: boolean;
}
let priorSelection: TableSelection;

function mountHost(): HTMLElement {
  const el = document.getElementById("pin-sweep-root");
  if (!el) throw new Error("no #pin-sweep-root");
  return el;
}

function rowFor(name: string): HTMLTableRowElement {
  const rows = [...document.querySelectorAll<HTMLTableRowElement>("#journal-body tr")];
  // the run name lives in the time cell's tooltip, not in any cell's text
  const row = rows.find((r) => r.querySelector(`td[title^="${name} "]`));
  if (!row) throw new Error(`no row for ${name} in ${rows.map((r) => r.textContent)}`);
  return row;
}

function chartBox(containerId: string): HTMLElement {
  const boxes = document.querySelectorAll<HTMLElement>(`#${containerId} .chart-box`);
  if (boxes.length !== 1) throw new Error(`#${containerId} holds ${boxes.length} chart boxes`);
  return boxes[0];
}

beforeAll(async () => {
  document.body.innerHTML = indexHtmlBody();
  document.body.insertAdjacentHTML("afterbegin", `<input type="text" id="moonraker-url">`);
  const host = document.createElement("div");
  host.id = "pin-sweep-root";
  document.body.appendChild(host);
  // App modules read localStorage/the DOM at import time, so they must load
  // after happy-dom is registered — the module-loading-boundary exception.
  ({ html, render } = await import("htm/preact"));
  client = await import("../src/queries/client");
  runsQ = await import("../src/queries/runs");
  runs = await import("../src/runs");
  ({ state } = await import("../src/state"));

  priorSelection = {
    selected: [...state.selected],
    pinned: [...state.pinned],
    colors: [...state.runColors],
    autoSelected: state.autoSelected,
  };
  state.selected.clear();
  state.pinned.clear();
  state.runColors.clear();
  // Chart filters are module singletons too; a stale one from another file
  // would silently thin these traces.
  state.stepFilter = null;
  state.accelAxisFilter = null;

  // Deliberately not `startRunsPolling`: that installs a module-level observer
  // singleton with no teardown, and a later file's own startRunsPolling would
  // early-return onto this file's dead observer. The table renders off the
  // query cache plus `notify()`, so fetching the list and reconciling once is
  // the same input without the process-global subscription.
  await runs.reconcileRuns(await client.queryClient.fetchQuery(runsQ.runsQuery()));
  await settle();
  render(html`<${client.QueryRoot}><${runs.TunePage} /><//>`, mountHost());
  await settle();
  await settle();

  await act(async () => {
    rowFor(PIN_RUN).click();
    await settle();
  });
  for (let i = 0; i < 3; i++) await settle();
});

afterAll(async () => {
  render(null as unknown as VNode, mountHost());
  cleanup();
  globalThis.fetch = baseFetch;
  notifyManager.setScheduler(defaultScheduler);
  state.stepFilter = null;
  state.accelAxisFilter = null;
  await resetRunState([PIN_RUN]);
  for (const name of priorSelection.selected) state.selected.add(name);
  for (const name of priorSelection.pinned) state.pinned.add(name);
  for (const [name, color] of priorSelection.colors) state.runColors.set(name, color);
  state.autoSelected = priorSelection.autoSelected;
});

test("a pin staircase is chartable on the tune page", () => {
  expect(consoleErrors).toEqual([]);
  expect(unmatched).toEqual([]);
  const row = rowFor(PIN_RUN);
  expect(row.className).toContain("selectable");
  expect(row.className).toContain("selected");
});

test("a selected pin staircase mounts the following-error PSD and the accel PSD", () => {
  const ferr = chartBox("psd-charts");
  expect(ferr.querySelector("h3")?.textContent).toBe("following error");
  expect(ferr.querySelector(".uplot")).not.toBeNull();

  const section = document.getElementById("accel-psd-section");
  expect(section).not.toBeNull();
  expect((section as HTMLElement).hidden).toBe(false);
  const accel = chartBox("accel-psd-charts");
  expect(accel.querySelector("h3")?.textContent).toBe("accelerometer");
  expect(accel.querySelector(".uplot")).not.toBeNull();
});

test("both PSD legends name the swept value of every step", () => {
  for (const id of ["psd-charts", "accel-psd-charts"]) {
    const legend = chartBox(id).querySelector(".legend")?.textContent ?? "";
    for (const step of STEP_NAMES) expect(legend).toContain(step);
  }
});

test("the accel PSD carries its own step chips, so its traces can be thinned in place", () => {
  const chips = [...document.querySelectorAll("#accel-psd-step-chips button.chip")];
  expect(chips.map((c) => c.textContent)).toEqual(["all", ...STEP_NAMES]);
});
