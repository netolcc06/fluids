import type { FluidsModule, SPHParams } from './types';
import { BaseSimulationController, ScenarioName } from './SimulationControllerBase';

/**
 * SPH-specific controller. Owns the `particleMass` calibration policy
 * documented in docs/sim-api.md (`particleMass ≈ restDensity * spacing²`):
 * live-patches `particleMass` into `sim.params` immediately whenever
 * spacing or restDensity changes, rather than deferring the fix to the
 * next Reset. That way the invariant never breaks even mid-simulation, and
 * it is re-applied defensively right after every `reset()` too, since
 * neither API doc explicitly guarantees `params` survives a `reset()` call.
 */
export class SPHSimulationController extends BaseSimulationController<SPHParams> {
  constructor(module: FluidsModule) {
    super(module, new module.Simulation());
  }

  /** Recompute and apply `particleMass = restDensity * spacing^2` against
   * the live sim.params right now. */
  applyMassCalibration(restDensity: number): void {
    const spacing = this.spacing;
    this.updateParams((p) => {
      p.particleMass = restDensity * spacing * spacing;
    });
  }

  /** UI entry point for the restDensity slider: sets restDensity AND
   * recalibrates particleMass in a single read-modify-write. */
  setRestDensity(restDensity: number): void {
    const spacing = this.spacing;
    this.updateParams((p) => {
      p.restDensity = restDensity;
      p.particleMass = restDensity * spacing * spacing;
    });
  }

  /** UI entry point for the particle-spacing slider. Per the API doc,
   * `spacing` itself is a Scenario field and only takes effect by being
   * baked into the seeded regions on the next reset(); particleMass is
   * live-patched immediately here so the calibration invariant never
   * breaks even before Reset is pressed. */
  setSpacing(spacing: number, restDensity: number): void {
    this.spacing = spacing;
    this.applyMassCalibration(restDensity);
  }

  /** Reset to a named built-in preset, applying the current spacing
   * override, then re-fetch pointers/views. `particleMass` is NOT set on
   * the Scenario object passed to reset(): the Scenario field table in
   * docs/sim-api.md does not list particleMass as a Scenario field
   * (SPHParams and Scenario are separate value_objects), so calibration is
   * applied via the documented `sim.params` read-modify-write path instead,
   * right after reset() completes. */
  reset(name: ScenarioName, restDensity: number): void {
    this.resetToScenario(name);
    this.applyMassCalibration(restDensity);
  }
}
