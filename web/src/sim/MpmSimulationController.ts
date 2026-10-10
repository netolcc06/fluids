import type { FluidsModule, MpmParams } from './types';
import { BaseSimulationController, ScenarioName } from './SimulationControllerBase';

/**
 * MPM-specific controller. Simpler than even FlipSimulationController: MPM
 * has no client-side mass/density calibration to do at all —
 * `particleMass = restDensity * spacing^2` is computed automatically inside
 * MPM's own C++ `reset()` from `params.restDensity`/`scenario.spacing` (see
 * docs/mpm-sim-api.md), so this controller is just scenario seeding + params
 * passthrough.
 */
export class MpmSimulationController extends BaseSimulationController<MpmParams> {
  constructor(module: FluidsModule) {
    super(module, new module.MpmSimulation());
  }

  /** Reset to a named built-in preset at the current spacing. Note:
   * `MpmParams.cellSize` and `restDensity` only take effect at `reset()`
   * time (per docs/mpm-sim-api.md, same contract as FLIP's `cellSize`) —
   * whatever is currently set on `sim.params` when this runs determines the
   * node grid resolution and calibrated `particleMass` built by this call.
   * Callers that just changed those via `updateParams` before calling
   * `reset()` will see them applied; changing them *after* this call has no
   * visible effect until the next `reset()`. */
  reset(name: ScenarioName): void {
    this.resetToScenario(name);
  }
}
