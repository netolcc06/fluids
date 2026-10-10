import type { FluidsModule, Scenario3D, SphParams3D, SphSimulation3DHandle } from './types';

export type Scenario3DName = 'damBreak3D' | 'blockDrop3D' | 'twoColumn3D';

/**
 * Owns a `Module.SphSimulation3D` instance — the first (and so far only) 3D
 * solver, per docs/sph3d-sim-api.md. Deliberately a standalone class, NOT a
 * subclass of `BaseSimulationController` (see that class's doc comment):
 * that base assumes 2D's `Scenario` shape, `ScenarioName` presets, and a
 * `count * 2` buffer stride, none of which hold here — this is a genuinely
 * separate 3D vertical slice living alongside the 2D pipeline, not threaded
 * through it.
 *
 * Same conventions as every 2D water controller, verified against
 * docs/sph3d-sim-api.md rather than assumed:
 * - Zero-copy `Float32Array` views over positions/velocities, rebuilt after
 *   every `reset()` (buffers can be reallocated when particle count
 *   changes) — but sized to `count * 3`, not `count * 2` (3D: `[x,y,z,...]`).
 * - `params` is an Embind value_object: reading returns a fresh copy every
 *   time, so mutating `sim.params.x` directly is a silent no-op — always
 *   read-modify-write via `updateParams`.
 * - `dispose()` calls `.delete()` on the underlying Embind instance. Verified
 *   against the compiled `fluids.wasm` (this build's `sph3d_bindings.cpp`
 *   registers `SphSimulation3D` with a plain `constructor<>()`, the same
 *   binding shape as every 2D solver's `SimulationHandle` — Embind never
 *   auto-finalizes a plain-`constructor<>()`-bound class, see
 *   `SimulationHandle.delete`'s doc comment in ./types.ts) — so this
 *   controller's `dispose()` is just as mandatory as every 2D controller's.
 *
 * Deliberately does NOT expose a particle-spacing slider or any
 * `particleMass` recalibration path: docs/sph3d-sim-api.md's calibration
 * section is explicit that 3D's `particleMass` correction factor (~1.468x
 * on top of the naive `restDensity * spacing^3` formula) is a *measured*
 * value tied to this build's specific `smoothingRadius / spacing` ratio, not
 * a formula safe to reapply blindly for an arbitrary spacing typed into a
 * slider. All three built-in presets already ship with a correctly
 * calibrated default `SphParams3D`, so `reset()` here just reseeds from a
 * preset and leaves `particleMass` alone — exactly the doc's recommended
 * "leave it uneditable for this first 3D slice" path.
 */
export class Sph3DSimulationController {
  readonly module: FluidsModule;
  private readonly sim: SphSimulation3DHandle;

  positions: Float32Array = new Float32Array(0);
  velocities: Float32Array = new Float32Array(0);
  particleCount = 0;
  domainWidth: number;
  domainHeight: number;
  domainDepth: number;

  constructor(module: FluidsModule) {
    this.module = module;
    this.sim = new module.SphSimulation3D();
    // `new Module.SphSimulation3D()` seeds a default Dam Break 3D scenario
    // immediately (per docs/sph3d-sim-api.md) — read its actual domain size
    // rather than hardcoding it, same defensive pattern as
    // BaseSimulationController's 2D constructor.
    const preset = module.makeDamBreak3DScenario();
    this.domainWidth = preset.domainWidth;
    this.domainHeight = preset.domainHeight;
    this.domainDepth = preset.domainDepth;
    this.rebuildViews();
  }

  private rebuildViews(): void {
    const count = this.sim.getParticleCount();
    const posPtr = this.sim.getPositionsPtr();
    const velPtr = this.sim.getVelocitiesPtr();
    this.particleCount = count;
    // 3 floats per particle, not 2 — see docs/sph3d-sim-api.md's "single
    // most important difference" callout. Getting this wrong silently drops
    // 1/3 of every particle's data instead of erroring.
    this.positions = new Float32Array(this.module.HEAPF32.buffer, posPtr, count * 3);
    this.velocities = new Float32Array(this.module.HEAPF32.buffer, velPtr, count * 3);
  }

  /** Re-fetch pointers/views. Cheap; safe to call every frame as insurance
   * against ALLOW_MEMORY_GROWTH detaching the old ArrayBuffer. Mandatory
   * after every reset(). */
  refreshViews(): void {
    this.rebuildViews();
  }

  step(dt: number): void {
    this.sim.step(dt);
  }

  getParams(): SphParams3D {
    return this.sim.params;
  }

  /** Read-modify-write helper required by the `sim.params` gotcha (see
   * class doc comment). */
  updateParams(mutate: (p: SphParams3D) => void): void {
    const p = this.sim.params;
    mutate(p);
    this.sim.params = p;
  }

  private scenarioFactory(name: Scenario3DName): Scenario3D {
    switch (name) {
      case 'damBreak3D':
        return this.module.makeDamBreak3DScenario();
      case 'blockDrop3D':
        return this.module.makeBlockDrop3DScenario();
      case 'twoColumn3D':
        return this.module.makeTwoColumnCollision3DScenario();
    }
  }

  /** Reset to a named built-in 3D preset, then re-fetch pointers/views
   * (mandatory — reset() can change particle count and reallocates both
   * buffers, per docs/sph3d-sim-api.md). No spacing override and no
   * particleMass recalibration — see class doc comment for why. */
  reset(name: Scenario3DName): void {
    const s = this.scenarioFactory(name);
    this.sim.reset(s);
    this.domainWidth = s.domainWidth;
    this.domainHeight = s.domainHeight;
    this.domainDepth = s.domainDepth;
    this.rebuildViews();
  }

  /** Frees the underlying Embind instance's WASM-heap memory. See class doc
   * comment's Embind lifecycle note — this is just as mandatory as every 2D
   * controller's `dispose()`. */
  dispose(): void {
    this.sim.delete();
  }
}
