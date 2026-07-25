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
  RUN_NAME,
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

const COMPARE_RUN = "cmp_20260725_101500";
const COMPARE_COMMAND = "SERVO_COMPARE_PIN MODE=Y PARAM=LEAD VALUES=0,600 NAME=cmp";
// A comparison names each step after the value it swept, exactly as the pin
// staircase does — those names are what the chart legends print, so they are
// the thing under test.
const STEP_NAMES = ["v0_lead0", "v1_lead600"];
const STEP_LEAD = [0, 600];

const compareSummary: RunSummary = {
  name: COMPARE_RUN,
  mtime_utc: "2026-07-25T10:15:00Z",
  experiment: "pin_compare",
  tag: "cmp",
  axis: "Y",
  command: COMPARE_COMMAND,
  has_results: true,
  verdict: null,
  note: null,
};

/// A comparison carries the same per-step payload as any other captured
/// sweep — following-error PSD plus the toolhead accel PSD — so the gain
/// sweep fixtures are reused under the comparison's experiment and step names.
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

const comparePlot = stepDoc<{ version: number; steps: PlotFixtureStep[] }>("plot_series");
if (!comparePlot.steps.every((s) => s.psd && s.psd.accel)) {
  throw new Error("plot_series fixture lost its per-step accel PSD — nothing left to assert");
}

// The ambient block stays as the fixture wrote it, so the diff against the
// neighbouring gain sweep is empty and the row's diff column reads like any
// other run's.
const compareManifest = stepDoc<{
  experiment: string;
  command: string;
  tag: string;
  axis: string;
  steps: { name: string; swept: Record<string, number> }[];
}>("manifest");
compareManifest.experiment = "pin_compare";
compareManifest.command = COMPARE_COMMAND;
compareManifest.tag = "cmp";
compareManifest.axis = "Y";
compareManifest.steps.forEach((s, i) => {
  s.swept = { value: STEP_LEAD[i] };
});

const compareResults = stepDoc<{ version: number; steps: { name: string }[] }>("results");

const comparePath = {
  version: 1,
  steps: comparePlot.steps
    .filter((s) => s.path)
    .map((s) => ({
      name: s.name,
      n_records: s.path!.cmd_x_mm.length,
      truncated: false,
      path: s.path,
    })),
};

// The shared stub serves one gain_sweep run; this layer prepends a
// pin comparison so both row kinds render side by side. Every endpoint it
// answers is one the gain sweep answers too — a comparison has no route of
// its own any more.
const posted: string[] = [];
const baseFetch = globalThis.fetch;
const json = (body: unknown) =>
  new Response(JSON.stringify(body), {
    status: 200,
    headers: { "Content-Type": "application/json" },
  });

globalThis.fetch = (async (input: RequestInfo | URL, init?: RequestInit) => {
  const req = input instanceof Request ? input : new Request(input, init);
  const path = new URL(req.url, "http://127.0.0.1/").pathname;
  if (req.method.toUpperCase() === "POST") {
    posted.push(`POST ${path} ${await req.clone().text()}`);
    return json({ note: "" });
  }
  if (path === "/api/runs") {
    return json([compareSummary, ...fixtureJson<RunSummary[]>("runs")]);
  }
  if (path === `/api/runs/${COMPARE_RUN}/manifest`) return json(compareManifest);
  if (path === `/api/runs/${COMPARE_RUN}/results`) return json(compareResults);
  if (path === `/api/runs/${COMPARE_RUN}/plot_series`) return json(comparePlot);
  if (path === `/api/runs/${COMPARE_RUN}/path`) return json(comparePath);
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

function pageRoot(): HTMLElement {
  const el = document.getElementById("page-root");
  if (!el) throw new Error("no #page-root");
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
  host.id = "page-root";
  document.body.appendChild(host);
  // App modules read localStorage/the DOM at import time, so they must load
  // after happy-dom is registered — the module-loading-boundary exception.
  ({ html, render } = await import("htm/preact"));
  client = await import("../src/queries/client");
  runsQ = await import("../src/queries/runs");
  runs = await import("../src/runs");
  ({ state } = await import("../src/state"));

  state.selected.clear();
  state.pinned.clear();
  state.runColors.clear();
  // The comparison heads the list and carries results, so the initial
  // auto-select would chart it before the selection test ever clicks. Latching
  // the flag makes the click the only thing that selects; afterAll's
  // resetRunState unlatches it again.
  state.autoSelected = true;
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
  render(html`<${client.QueryRoot}><${runs.TunePage} /><//>`, pageRoot());
  await settle();
  await settle();
});

afterAll(async () => {
  render(null as unknown as VNode, pageRoot());
  cleanup();
  globalThis.fetch = baseFetch;
  notifyManager.setScheduler(defaultScheduler);
  state.stepFilter = null;
  state.accelAxisFilter = null;
  await resetRunState([COMPARE_RUN]);
});

test("a comparison is one more row, structurally identical to a gain sweep", () => {
  expect(consoleErrors).toEqual([]);
  expect(unmatched).toEqual([]);
  const compare = rowFor(COMPARE_RUN);
  const ordinary = rowFor(RUN_NAME);
  // same columns in the same order; the `empty` modifier is per-row content
  // state (the diff column has text for one run and not the other)
  const columns = (row: HTMLTableRowElement) =>
    [...row.querySelectorAll("td")].map((td) => td.className.replace(" empty", ""));
  expect(columns(compare)).toEqual(columns(ordinary));
  expect(columns(compare)).toEqual(["", "", "", "diff", "run-note", "actions"]);
  // the tag column reads like every other run — no "pin_compare/<name>" prefix
  expect(compare.children[2].textContent?.trim()).toBe("cmp Y");
});

test("a comparison offers the same → console prefill as any other run", () => {
  // The run's command line lives in its manifest and reaches the console
  // through this button; the runs table deliberately does not spend a
  // column repeating it.
  const button = rowFor(COMPARE_RUN).querySelector<HTMLButtonElement>("td.actions button");
  expect(button?.textContent).toContain("console");
  expect(button?.disabled).toBe(false);
});

test("a comparison takes a note like any other run", async () => {
  const cell = rowFor(COMPARE_RUN).querySelector<HTMLElement>("td.run-note");
  expect(cell!.className).toContain("empty");
  expect(cell!.textContent?.trim()).toBe("add note…");
  cell!.click();
  await settle();
  const input = rowFor(COMPARE_RUN).querySelector<HTMLInputElement>("input.run-note-input");
  expect(input).not.toBeNull();
  input!.value = "lead ladder, cold frame";
  input!.dispatchEvent(new KeyboardEvent("keydown", { key: "Enter", bubbles: true }));
  await settle();
  expect(posted).toContain(
    `POST /api/runs/${COMPARE_RUN}/note {"note":"lead ladder, cold frame"}`
  );
});

test("selecting a comparison mounts the standard PSD sections, deselecting drops them", async () => {
  expect(document.querySelector("#psd-charts .chart-box")).toBeNull();

  // act() flushes the effects that subscribe the chart query observers;
  // outside a browser paint they would otherwise stay pending
  await act(async () => {
    rowFor(COMPARE_RUN).click();
    await settle();
  });
  for (let i = 0; i < 3; i++) await settle();

  const ferr = chartBox("psd-charts");
  expect(ferr.querySelector("h3")?.textContent).toBe("following error");
  expect(ferr.querySelector(".uplot")).not.toBeNull();

  const accelSection = document.getElementById("accel-psd-section") as HTMLElement;
  expect(accelSection.hidden).toBe(false);
  const accel = chartBox("accel-psd-charts");
  expect(accel.querySelector("h3")?.textContent).toBe("accelerometer");
  expect(accel.querySelector(".uplot")).not.toBeNull();

  // one trace per swept value, named by the step the comparison recorded
  for (const id of ["psd-charts", "accel-psd-charts"]) {
    const legend = chartBox(id).querySelector(".legend")?.textContent ?? "";
    for (const step of STEP_NAMES) expect(legend).toContain(step);
  }

  rowFor(COMPARE_RUN).click();
  for (let i = 0; i < 3; i++) await settle();
  expect(document.querySelector("#psd-charts .chart-box")).toBeNull();
  expect(accelSection.hidden).toBe(true);
});
