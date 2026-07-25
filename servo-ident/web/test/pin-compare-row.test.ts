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
const COMPARE_COMMAND = "SERVO_COMPARE_PIN MODE=Y PARAM=LEAD VALUES=0,300,600 NAME=cmp";

const compareSummary: RunSummary = {
  name: COMPARE_RUN,
  mtime_utc: "2026-07-25T10:15:00Z",
  experiment: "pin_compare",
  tag: "cmp",
  axis: "Y",
  command: COMPARE_COMMAND,
  has_results: false,
  verdict: null,
  note: null,
};

const compareBlock = {
  mode: "y",
  param: "LEAD",
  freq_start: 100,
  freq_end: 150,
  baseline_profile: "/cfg/baseline.toml",
  sweeps: [0, 300, 600].map((value, i) => ({
    value,
    hz_per_sec: 3,
    accel_per_hz: 75,
    amplitude_mm: 0.019,
    curve_hz: [117.5, 122.5],
    accel_mm_s2: [9000 - i * 500, 7000 - i * 500],
    response_ratio: [1.02, 0.79],
  })),
};

// The shared stub serves one gain_sweep run; this layer prepends a
// pin comparison so both row kinds render side by side, and answers the
// endpoints only that run has.
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
  if (path === `/api/runs/${COMPARE_RUN}/pin_compare`) return json(compareBlock);
  if (path === `/api/runs/${COMPARE_RUN}/manifest`) {
    return json({
      version: 1,
      experiment: "pin_compare",
      command: COMPARE_COMMAND,
      tag: "cmp",
      axis: "Y",
      steps: [],
      // same ambient as the neighbouring run so the diff column stays empty
      // — a real comparison manifest carries one, `_begin_run` writes it
      ambient: fixtureJson<{ ambient: unknown }>("manifest").ambient,
      pin_compare: compareBlock,
    });
  }
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
  expect(columns(compare)).toEqual(["", "", "", "run-command", "diff", "run-note", "actions"]);
  // the tag column reads like every other run — no "pin_compare/<name>" prefix
  expect(compare.children[2].textContent?.trim()).toBe("cmp Y");
});

test("the command that made the run is shown, truncated with the full text as a tooltip", () => {
  const cell = rowFor(COMPARE_RUN).querySelector<HTMLElement>("td.run-command");
  expect(cell).not.toBeNull();
  expect(cell!.textContent?.trim()).toBe(COMPARE_COMMAND);
  expect(cell!.getAttribute("title")).toBe(COMPARE_COMMAND);
  // the CSS ellipsis is what truncates; the cell must not wrap it away
  expect(rowFor(RUN_NAME).querySelector("td.run-command")).not.toBeNull();
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

test("selecting the comparison row charts it, and deselecting hides the section", async () => {
  expect(document.querySelector(".pin-compare-section")).toBeNull();

  // act() flushes the effects that subscribe the section's query observer;
  // outside a browser paint they would otherwise stay pending
  await act(async () => {
    rowFor(COMPARE_RUN).click();
    await settle();
  });
  for (let i = 0; i < 3; i++) await settle();
  const section = document.querySelector(".pin-compare-section");
  expect(section).not.toBeNull();
  expect(section!.querySelector("h2")?.textContent).toContain(COMPARE_RUN);
  // raw accel on a linear axis only — no y-mode or scale selectors
  expect(section!.querySelectorAll("select").length).toBe(0);
  expect(section!.querySelectorAll('.legend span[role="button"]').length).toBe(3);
  expect(section!.querySelector(".legend")?.textContent).toContain("LEAD=0 @3 Hz/s 75 ApH");

  rowFor(COMPARE_RUN).click();
  await settle();
  expect(document.querySelector(".pin-compare-section")).toBeNull();
});
