import type { FluidsModule, FlipSmokeParams, FlipSmokeSimulationHandle } from './types';

/**
 * Controller for `Module.FlipSmokeSimulation` — the FLIP-only Smoke
 * material (Phase 4a, see docs/flip-smoke-sim-api.md). Deliberately does
 * NOT extend `BaseSimulationController<TParams>`: that base class assumes
 * particle-shaped state (positions/velocities/particleCount), and smoke has
 * none of that at all — it's a pure grid-shaped density/temperature field.
 * This class mirrors the water controllers' *conventions* instead:
 * params read-modify-write, refreshViews()-after-reset(), dispose()
 * lifecycle — without inheriting the particle-shaped interface.
 */
export class FlipSmokeSimulationController {
  readonly module: FluidsModule;
  private readonly sim: FlipSmokeSimulationHandle;

  gridWidth = 0;
  gridHeight = 0;
  density: Float32Array = new Float32Array(0);
  temperature: Float32Array = new Float32Array(0);
  /** From the seeded SmokeScenario (currently always makePlumeScenario()'s
   * 1.2m x 2.0m — there's only one built-in preset). Mirrors the water
   * controllers' domainWidth/domainHeight fields so SolverManager/main.ts
   * can read camera-transform inputs uniformly across both render modes. */
  domainWidth: number;
  domainHeight: number;

  constructor(module: FluidsModule) {
    this.module = module;
    this.sim = new module.FlipSmokeSimulation();
    // `new Module.FlipSmokeSimulation()` seeds a default Plume scenario
    // immediately (per docs/flip-smoke-sim-api.md), so read its actual
    // domain size rather than assuming.
    const preset = module.makePlumeScenario();
    this.domainWidth = preset.domainWidth;
    this.domainHeight = preset.domainHeight;
    this.rebuildViews();
  }

  private rebuildViews(): void {
    const w = this.sim.getGridWidth();
    const h = this.sim.getGridHeight();
    const densPtr = this.sim.getDensityFieldPtr();
    const tempPtr = this.sim.getTemperatureFieldPtr();
    this.gridWidth = w;
    this.gridHeight = h;
    this.density = new Float32Array(this.module.HEAPF32.buffer, densPtr, w * h);
    this.temperature = new Float32Array(this.module.HEAPF32.buffer, tempPtr, w * h);
  }

  /** Re-fetch pointers/views. Per docs/flip-smoke-sim-api.md, pointers are
   * stable across step() calls (verified empirically in the doc), but grid
   * dimensions can change on reset() — mandatory to call this after every
   * reset(). Safe/cheap to call every frame as insurance, same discipline
   * as the water controllers' refreshViews(). */
  refreshViews(): void {
    this.rebuildViews();
  }

  step(dt: number): void {
    this.sim.step(dt);
  }

  getParams(): FlipSmokeParams {
    return this.sim.params;
  }

  /** Read-modify-write helper required by the `sim.params` gotcha: reading
   * returns a fresh copy every time, so `sim.params.x = y` silently no-ops. */
  updateParams(mutate: (p: FlipSmokeParams) => void): void {
    const p = this.sim.params;
    mutate(p);
    this.sim.params = p;
  }

  /** Reseeds the (currently only) built-in Plume preset. `FlipSmokeParams
   * .cellSize` only takes effect at `reset()` time (per
   * docs/flip-smoke-sim-api.md) — whatever cellSize is currently set on
   * `sim.params` when this runs determines the freshly rebuilt MAC grid's
   * resolution. Callers that just changed cellSize via `updateParams`
   * before calling this will see it applied. */
  reset(): void {
    const s = this.module.makePlumeScenario();
    this.sim.reset(s);
    this.domainWidth = s.domainWidth;
    this.domainHeight = s.domainHeight;
    this.rebuildViews();
  }

  /** Frees the underlying Embind instance's WASM-heap memory. Verified
   * against the compiled glue: `FlipSmokeSimulation` is bound with a plain
   * `constructor<>()` (no smart pointer), identical to `Simulation`/
   * `FlipSimulation`/`MpmSimulation` — so, per the same reasoning recorded
   * in SimulationHandle.delete's doc comment (types.ts), it is never
   * automatically garbage-collected/finalized by JS. Every constructed
   * instance must be explicitly `.delete()`d or its C++-side memory leaks
   * for the rest of the page's life. Must be called once this controller is
   * no longer in use (e.g. when SolverManager switches Solver/Material
   * away from FLIP+Smoke). */
  dispose(): void {
    this.sim.delete();
  }
}
