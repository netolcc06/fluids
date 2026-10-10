import type { FluidsModule, MpmSmokeParams, MpmSmokeSimulationHandle } from './types';

/**
 * Controller for `Module.MpmSmokeSimulation` — the MPM-flavored Smoke
 * material (Phase 4b, see docs/mpm-smoke-sim-api.md). Deliberately does NOT
 * extend `BaseSimulationController<TParams>`, for a DIFFERENT reason than
 * `FlipSmokeSimulationController`'s (that one is grid-shaped, no particles
 * at all). This solver IS particle-shaped (positions/velocities/particle
 * count, same contract shape as water), but breaks a core assumption
 * `BaseSimulationController` bakes in: it assumes particle count only
 * changes on `reset()` and therefore only rebuilds its typed-array views
 * there. MPM smoke's `getParticleCount()` legitimately changes on EVERY
 * `step()` call (continuous emission/culling — see the doc's "buffer
 * stability contract"), so this controller re-derives the *active* view
 * length every step() instead, without ever re-fetching the underlying
 * pointers outside of reset() — mirroring the water controllers'
 * conventions (params read-modify-write, dispose() lifecycle) without
 * inheriting an interface shaped around a fixed count.
 *
 * Implementation: `positionsFull`/`velocitiesFull`/`densitiesFull`/
 * `temperaturesFull` are built ONCE per reset() at `params.maxParticles`
 * (fixed) capacity, directly from the four pointer getters — this is the
 * only place those getters are called, per the doc's explicit "you do NOT
 * need to call getPositionsPtr() etc. again just because the count changed
 * — only after an actual reset() call" guidance. Every step(), `positions`/
 * `velocities`/`densities`/`temperatures` are re-sliced from those same
 * full-capacity backing arrays via `subarray(0, n)` — a cheap view
 * descriptor over the identical underlying ArrayBuffer, NOT a copy and NOT
 * a pointer re-fetch — sized to the freshly re-read `getParticleCount()`.
 */
export class MpmSmokeSimulationController {
  readonly module: FluidsModule;
  private readonly sim: MpmSmokeSimulationHandle;

  private positionsFull: Float32Array = new Float32Array(0);
  private velocitiesFull: Float32Array = new Float32Array(0);
  private densitiesFull: Float32Array = new Float32Array(0);
  private temperaturesFull: Float32Array = new Float32Array(0);

  /** Active particle count as of the most recent step()/reset() — UNLIKE
   * every water controller's particleCount, this legitimately changes on
   * every step() (continuous emission/culling), not just reset(). */
  particleCount = 0;
  /** Views re-sliced (subarray — no copy) to `particleCount` elements every
   * step()/reset(). Safe to read directly for rendering; always reflects
   * the just-completed step's active range. */
  positions: Float32Array = new Float32Array(0);
  velocities: Float32Array = new Float32Array(0);
  densities: Float32Array = new Float32Array(0);
  temperatures: Float32Array = new Float32Array(0);

  /** From the seeded SmokeScenario (currently always makePlumeScenario()'s
   * 1.2m x 2.0m — there's only one built-in preset). Mirrors the water/FLIP
   * Smoke controllers' domainWidth/domainHeight fields so SolverManager/
   * main.ts can read camera-transform inputs uniformly. */
  domainWidth: number;
  domainHeight: number;

  constructor(module: FluidsModule) {
    this.module = module;
    this.sim = new module.MpmSmokeSimulation();
    // `new Module.MpmSmokeSimulation()` seeds a default Plume scenario
    // immediately (per docs/mpm-smoke-sim-api.md), so read its actual
    // domain size rather than assuming.
    const preset = module.makePlumeScenario();
    this.domainWidth = preset.domainWidth;
    this.domainHeight = preset.domainHeight;
    this.rebuildFullViews();
    this.refreshActiveViews();
  }

  /** Fetches all four pointers and builds full-capacity (`params
   * .maxParticles`) backing views. MUST be called after every `reset()`
   * (capacity can change) — this is the ONLY place the pointer getters are
   * called, per the doc's buffer-stability contract. */
  private rebuildFullViews(): void {
    const cap = this.sim.params.maxParticles;
    const posPtr = this.sim.getPositionsPtr();
    const velPtr = this.sim.getVelocitiesPtr();
    const densPtr = this.sim.getDensitiesPtr();
    const tempPtr = this.sim.getTemperaturesPtr();
    this.positionsFull = new Float32Array(this.module.HEAPF32.buffer, posPtr, cap * 2);
    this.velocitiesFull = new Float32Array(this.module.HEAPF32.buffer, velPtr, cap * 2);
    this.densitiesFull = new Float32Array(this.module.HEAPF32.buffer, densPtr, cap);
    this.temperaturesFull = new Float32Array(this.module.HEAPF32.buffer, tempPtr, cap);
  }

  /** Re-reads `getParticleCount()` (cheap; legitimately changes every
   * step()) and re-slices the full-capacity backing views to that active
   * length. Does NOT touch the pointers — this is the per-frame-safe half
   * of the buffer-stability contract. */
  private refreshActiveViews(): void {
    const n = this.sim.getParticleCount();
    this.particleCount = n;
    this.positions = this.positionsFull.subarray(0, n * 2);
    this.velocities = this.velocitiesFull.subarray(0, n * 2);
    this.densities = this.densitiesFull.subarray(0, n);
    this.temperatures = this.temperaturesFull.subarray(0, n);
  }

  /** Re-slice the active views without stepping or touching pointers.
   * Mirrors the water/FLIP-Smoke controllers' `refreshViews()` name for
   * SolverManager's uniform delegation, but intentionally does NOT re-fetch
   * pointers (unlike theirs) — count changing on its own is normal here and
   * doesn't imply reallocation, per the doc. Harmless/cheap to call anytime. */
  refreshViews(): void {
    this.refreshActiveViews();
  }

  step(dt: number): void {
    this.sim.step(dt);
    // getParticleCount() changes every step (emission/culling) — re-slice
    // the active views immediately so callers reading .positions etc. right
    // after step() always see the current frame's active range.
    this.refreshActiveViews();
  }

  getParams(): MpmSmokeParams {
    return this.sim.params;
  }

  /** Read-modify-write helper required by the `sim.params` gotcha: reading
   * returns a fresh copy every time, so `sim.params.x = y` silently no-ops. */
  updateParams(mutate: (p: MpmSmokeParams) => void): void {
    const p = this.sim.params;
    mutate(p);
    this.sim.params = p;
  }

  /** Reseeds the (currently only) built-in Plume preset. `MpmSmokeParams
   * .cellSize` and `.maxParticles` only take effect at `reset()` time (per
   * docs/mpm-smoke-sim-api.md) — whatever is currently set on `sim.params`
   * when this runs determines the freshly (re)allocated grid/particle-buffer
   * capacity. Callers that just changed either via `updateParams` before
   * calling this will see it applied. */
  reset(): void {
    const s = this.module.makePlumeScenario();
    this.sim.reset(s);
    this.domainWidth = s.domainWidth;
    this.domainHeight = s.domainHeight;
    this.rebuildFullViews();
    this.refreshActiveViews();
  }

  /** Frees the underlying Embind instance's WASM-heap memory. Verified
   * against the compiled glue (wasm/mpm_smoke_bindings.cpp): bound with a
   * plain `constructor<>()`, identical to every other solver class — so, per
   * the same reasoning recorded in SimulationHandle.delete's doc comment
   * (types.ts), it is never automatically garbage-collected/finalized by JS.
   * Must be called once this controller is no longer in use (e.g. when
   * SolverManager switches Solver/Material away from MPM+Smoke). */
  dispose(): void {
    this.sim.delete();
  }
}
