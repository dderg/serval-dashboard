import { GlobalRegistrator } from "@happy-dom/global-registrator";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import type { RunSummary } from "../src/api/runs";

// happy-dom's canvas has no 2d context, but uPlot draws unconditionally.
// This mock answers every ctx method with a no-op (measureText and
// gradients need real-shaped return values) so chart code runs to
// completion; pixel output is out of scope for these tests.
function mock2dContext(): CanvasRenderingContext2D {
  const noop = () => undefined;
  const target: Record<string | symbol, unknown> = {
    measureText: () => ({ width: 0 }),
    createLinearGradient: () => ({ addColorStop: noop }),
    getImageData: () => ({ data: new Uint8ClampedArray(4) }),
  };
  return new Proxy(target, {
    get(t, prop) {
      if (!(prop in t)) t[prop] = noop;
      return t[prop];
    },
    set(t, prop, value) {
      t[prop] = value;
      return true;
    },
  }) as unknown as CanvasRenderingContext2D;
}

function registerDom() {
  if (GlobalRegistrator.isRegistered) return;
  GlobalRegistrator.register({ url: "http://127.0.0.1/" });
  (globalThis as Record<string, any>).Path2D ??= class Path2D {
    addPath() {}
    moveTo() {}
    lineTo() {}
    rect() {}
    closePath() {}
    arc() {}
  };
  const proto = (globalThis as Record<string, any>).HTMLCanvasElement.prototype;
  const contexts = new WeakMap<object, CanvasRenderingContext2D>();
  proto.getContext = function () {
    if (!contexts.has(this)) contexts.set(this, mock2dContext());
    return contexts.get(this);
  };
}

function fixture(name: string): string {
  return readFileSync(join(import.meta.dir, "fixtures", `${name}.json`), "utf-8");
}

function fixtureJson<T>(name: string): T {
  return JSON.parse(fixture(name)) as T;
}

const RUN_NAME = (fixtureJson<{ name: string }[]>("runs"))[0].name;

interface FixturePathStep {
  name: string;
  path: { cmd_x_mm: number[]; cmd_y_mm: number[]; act_x_mm: number[]; act_y_mm: number[] } | null;
}

function runPathFromPlotSeries(): string {
  const plot = fixtureJson<{ steps: FixturePathStep[] }>("plot_series");
  const steps = plot.steps
    .filter((s) => s.path)
    .map((s) => ({
      name: s.name,
      n_records: s.path!.cmd_x_mm.length,
      truncated: false,
      path: s.path,
    }));
  return JSON.stringify({ version: 1, steps });
}

/// fetch stub covering everything boot touches: the servo-cal API answers
/// from the captured demo fixtures, moonraker answers minimally healthy.
/// Anything else is a test bug and fails loudly.
function installFetchStub(): { unmatched: string[] } {
  const unmatched: string[] = [];
  const json = (body: string) =>
    new Response(body, { status: 200, headers: { "Content-Type": "application/json" } });
  const routes: [RegExp, () => Response][] = [
    [/^\/api\/runs$/, () => json(fixture("runs"))],
    [/^\/api\/drive_state$/, () => json(fixture("drive_state"))],
    [/^\/api\/live$/, () => json(fixture("live"))],
    [/^\/api\/live_tap$/, () => json(JSON.stringify({ status: "connecting" }))],
    [new RegExp(`^/api/runs/${RUN_NAME}/manifest$`), () => json(fixture("manifest"))],
    [new RegExp(`^/api/runs/${RUN_NAME}/results$`), () => json(fixture("results"))],
    [new RegExp(`^/api/runs/${RUN_NAME}/plot_series$`), () => json(fixture("plot_series"))],
    [new RegExp(`^/api/runs/${RUN_NAME}/path$`), () => json(runPathFromPlotSeries())],
    [/\/server\/info$/, () => json(JSON.stringify({ result: { klippy_state: "ready" } }))],
    [/\/printer\/gcode\/help$/, () => json(JSON.stringify({ result: {} }))],
  ];
  globalThis.fetch = (async (input: RequestInfo | URL) => {
    const url = input instanceof Request ? input.url : String(input);
    const path = url.startsWith("http") ? new URL(url).pathname : url.split("?")[0];
    for (const [re, respond] of routes) {
      if (re.test(path)) return respond();
    }
    unmatched.push(url);
    return new Response(`no fixture for ${url}`, { status: 404 });
  }) as typeof fetch;
  return { unmatched };
}

function indexHtmlBody(): string {
  const html = readFileSync(join(import.meta.dir, "..", "index.html"), "utf-8");
  const body = /<body>([\s\S]*)<\/body>/.exec(html);
  if (!body) throw new Error("index.html has no <body>");
  return body[1].replace(/<script[\s\S]*?<\/script>/g, "");
}

function nextFrame(): Promise<void> {
  const { promise, resolve } = Promise.withResolvers<void>();
  requestAnimationFrame(() => resolve());
  return promise;
}

function nextTask(): Promise<void> {
  const { promise, resolve } = Promise.withResolvers<void>();
  const channel = new MessageChannel();
  channel.port1.onmessage = () => resolve();
  channel.port2.postMessage(0);
  return promise;
}

async function settleDom(rounds = 1): Promise<void> {
  for (let i = 0; i < rounds; i++) {
    await nextFrame();
    await nextTask();
  }
}

/// Settle until `measure()` is nonzero and has stopped changing, then return
/// it. Charts mount across several async hops (query resolve -> render ->
/// effect -> uPlot construction) AND they mount incrementally, several
/// sections at a time. A fixed number of `settleDom()` rounds is a bet on how
/// many hops today's code takes — add a chart section and the bet silently
/// starts losing on some runs. Waiting for the count to appear is not enough
/// either: a caller that snapshots the DOM mid-mount sees it grow underneath
/// itself and blames whatever it did next. Quiescence is the real
/// precondition. Throws rather than returning early, because a timeout means
/// the thing under test never settled.
async function settleUntilStable(
  measure: () => number,
  what: string,
  maxRounds = 80,
): Promise<number> {
  // One repeat is not quiescence: sections mount in batches, and the gap
  // between two batches is easily a whole settle round wide.
  const stableRounds = 3;
  let previous = -1;
  let repeats = 0;
  for (let i = 0; i < maxRounds; i++) {
    const current = measure();
    repeats = current > 0 && current === previous ? repeats + 1 : 0;
    if (repeats >= stableRounds) return current;
    previous = current;
    await settleDom();
    await nextFrame();
  }
  throw new Error(`settleUntilStable: ${what} never settled in ${maxRounds} rounds`);
}

/// `state` and the query cache are module singletons, and bun test runs every
/// file in one process: a file that renders the runs table and clicks rows
/// hands the next file its selection, its `autoSelected` latch and its cached
/// per-run queries. Reset what the table touched, and drop the cached queries
/// of runs a file invented — `refetchQueries({ queryKey: ["runs"] })` matches
/// by prefix, so a stale `["runs", <name>, …]` entry is refetched by whoever
/// runs next and hits their fetch stub as an unknown URL.
///
/// Callers differ in what they do with the SELECTION on the way out, and that
/// difference is deliberate: file order here is filesystem order, and
/// tune-journal.test.ts reads the selection boot.test.ts leaves behind. A file
/// that runs between them must hand that selection back (snapshot/restore); a
/// file that clears it outright only works when it happens to sort earlier.
/// Converging both onto restore-always was tried and made the suite
/// intermittently red — 2 failures in tune-journal every third run or so.
/// Leave the shapes alone unless you also fix that ordering dependency.
///
/// Imported dynamically for the same reason every test file does it: these
/// modules read localStorage and the DOM at load, so they may not be pulled in
/// before `registerDom()` has run in the file that imports this helper.
async function resetRunState(syntheticRuns: string[] = []): Promise<void> {
  const { state } = await import("../src/state");
  const { queryClient } = await import("../src/queries/client");
  const { runKeys } = await import("../src/queries/runs");
  const invented = new Set(syntheticRuns);
  for (const name of invented) queryClient.removeQueries({ queryKey: runKeys.run(name) });
  // The list query itself is kept — removing it would orphan any polling
  // observer still subscribed to it — so only the invented rows are dropped.
  queryClient.setQueryData<RunSummary[]>(runKeys.all, (cached) =>
    cached?.filter((r) => !invented.has(r.name)),
  );
  state.selected.clear();
  state.pinned.clear();
  state.runColors.clear();
  state.autoSelected = false;
  state.console.text = "";
}

function installDomHarness() {
  const intervals: Timer[] = [];
  const realSetInterval = globalThis.setInterval;
  (globalThis as Record<string, unknown>).setInterval = ((fn: TimerHandler, ms?: number) => {
    const id = realSetInterval(fn as () => void, ms ?? 0);
    intervals.push(id);
    return id;
  }) as typeof setInterval;
  const consoleErrors: unknown[][] = [];
  const realConsoleError = console.error;
  console.error = (...args: unknown[]) => {
    consoleErrors.push(args);
    realConsoleError(...args);
  };
  return {
    consoleErrors,
    cleanup() {
      for (const id of intervals) clearInterval(id);
      globalThis.setInterval = realSetInterval;
      console.error = realConsoleError;
    },
  };
}

export { registerDom, installFetchStub, installDomHarness, indexHtmlBody, fixtureJson, nextFrame, nextTask, settleDom, settleUntilStable, resetRunState, RUN_NAME };
