import type { FluidsModule, FlipParams } from './types';
import { BaseSimulationController, ScenarioName } from './SimulationControllerBase';

/**
 * FLIP-specific controller. Unlike SPH, there is no mass/density
 * calibration to keep in sync with `Scenario.spacing` — FLIP's
 * incompressibility comes from grid pressure projection, not a
 * per-particle equation of state (see docs/flip-sim-api.md). `spacing`
 * only affects how many particles get seeded.
 */
export class FlipSimulationController extends BaseSimulationController<FlipParams> {
  constructor(module: FluidsModule) {
    super(module, new module.FlipSimulation());
  }

  /** Reset to a named built-in preset at the current spacing. Note:
   * `FlipParams.cellSize` only takes effect at `reset()` time (per
   * docs/flip-sim-api.md) — whatever `cellSize` is currently set on
   * `sim.params` when this runs determines the MAC grid resolution built by
   * this call. Callers that just changed `cellSize` via `updateParams`
   * before calling `reset()` will see it applied; changing it *after* this
   * call has no visible effect until the next `reset()`. */
  reset(name: ScenarioName): void {
    this.resetToScenario(name);
  }
}
