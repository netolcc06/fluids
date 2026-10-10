import type { FluidsModule, Scenario, SimulationHandle } from './types';

export type ScenarioName = 'damBreak' | 'blockDrop' | 'twoColumn';

/**
 * Solver-agnostic base for owning a WASM simulation instance: the
 * zero-copy typed-array views into its position/velocity buffers, and the
 * read-modify-write plumbing that `sim.params` requires (see
 * docs/sim-api.md's and docs/flip-sim-api.md's "params property gotcha",
 * identical for both solvers).
 *
 * Parameterized by `TParams` (SPHParams or FlipParams) so the same buffer
 * lifecycle / reset plumbing is shared, while each concrete subclass
 * (SPHSimulationController, FlipSimulationController) keeps its own
 * solver-specific calibration quirks — SPH's particleMass-from-spacing
 * calibration has no FLIP equivalent, and neither is generalized here.
 */
export abstract class BaseSimulationController<TParams> {
  readonly module: FluidsModule;
  protected readonly sim: SimulationHandle<TParams>;

  positions: Float32Array = new Float32Array(0);
  velocities: Float32Array = new Float32Array(0);
  particleCount = 0;
  domainWidth: number;
  domainHeight: number;
  /** Current particle spacing override (Scenario.spacing on next reset). */
  spacing: number;

  protected constructor(module: FluidsModule, sim: SimulationHandle<TParams>) {
    this.module = module;
    this.sim = sim;
    // Both `new Module.Simulation()` and `new Module.FlipSimulation()` seed
    // a default Dam Break scenario immediately (per both API docs), so read
    // its actual spacing/domain rather than assuming.
    const preset = module.makeDamBreakScenario();
    this.spacing = preset.spacing;
    this.domainWidth = preset.domainWidth;
    this.domainHeight = preset.domainHeight;
    this.rebuildViews();
  }

  private rebuildViews(): void {
    const count = this.sim.getParticleCount();
    const posPtr = this.sim.getPositionsPtr();
    const velPtr = this.sim.getVelocitiesPtr();
    this.particleCount = count;
    this.positions = new Float32Array(this.module.HEAPF32.buffer, posPtr, count * 2);
    this.velocities = new Float32Array(this.module.HEAPF32.buffer, velPtr, count * 2);
  }

  /** Re-fetch pointers/views. Cheap; safe (and recommended by both API
   * docs) to call every frame as insurance against ALLOW_MEMORY_GROWTH
   * detaching the old ArrayBuffer. Mandatory after every reset(). */
  refreshViews(): void {
    this.rebuildViews();
  }

  step(dt: number): void {
    this.sim.step(dt);
  }

  getParams(): TParams {
    return this.sim.params;
  }

  /** Read-modify-write helper required by the `sim.params` gotcha: reading
   * returns a fresh copy, so `sim.params.x = y` silently no-ops. */
  updateParams(mutate: (p: TParams) => void): void {
    const p = this.sim.params;
    mutate(p);
    this.sim.params = p;
  }

  protected scenarioFactory(name: ScenarioName): Scenario {
    switch (name) {
      case 'damBreak':
        return this.module.makeDamBreakScenario();
      case 'blockDrop':
        return this.module.makeBlockDropScenario();
      case 'twoColumn':
        return this.module.makeTwoColumnCollisionScenario();
    }
  }

  /** Resets the underlying sim to the named preset at this controller's
   * current `spacing`, then re-fetches pointers/views (hard requirement —
   * reset() can change particle count and/or reallocate buffers, per both
   * API docs). Subclasses call this from their own public `reset()` and
   * layer solver-specific param calibration around it (e.g. SPH's
   * particleMass recalibration, which must happen relative to this call;
   * FLIP's cellSize, which must be set on params *before* this call to take
   * effect — see FlipSimulationController). */
  protected resetToScenario(name: ScenarioName): Scenario {
    const s = this.scenarioFactory(name);
    s.spacing = this.spacing;
    this.sim.reset(s);
    this.domainWidth = s.domainWidth;
    this.domainHeight = s.domainHeight;
    this.rebuildViews();
    return s;
  }

  /** Frees the underlying Embind instance's WASM-heap memory (see
   * SimulationHandle.delete doc comment — Embind class instances are never
   * garbage-collected by JS). Must be called once this controller is no
   * longer in use, e.g. right before/after constructing a replacement when
   * switching solvers (see SolverManager.switchTo). Calling any other
   * method on this controller after dispose() is undefined behavior
   * (Embind throws "instance already deleted" on the underlying handle). */
  dispose(): void {
    this.sim.delete();
  }
}
