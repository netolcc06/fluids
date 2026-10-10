import { Pane } from 'tweakpane';
import type { ButtonApi } from 'tweakpane';
import { SolverManager, SolverKind, MaterialKind } from '../sim/SolverManager';
import { ScenarioName } from '../sim/SimulationControllerBase';
import { FixedTimestepLoop } from '../sim/FixedTimestepLoop';
import type {
  FlipParams,
  FlipSmokeParams,
  MpmParams,
  MpmSmokeParams,
  SPHParams,
  SphParams3D,
  SphSmokeParams,
} from '../sim/types';
import { Sph3DSimulationController, Scenario3DName } from '../sim/Sph3DSimulationController';

/** Top-level Dimension toggle. '3D' currently only supports SPH Water (see
 * docs/sph3d-sim-api.md) — the Solver/Material/Scenario controls for the 2D
 * pipeline and the "3D Scenario"/"3D SPH Params" controls below are
 * mutually exclusive in the UI (see updateVisibility) so there is no way to
 * reach an invalid (dimension, solver, material) combination. */
export type Dimension = 'twoD' | 'threeD';

export interface PanelCallbacks {
  /** Called after a 2D Reset (or a solver/material switch, which resets
   * internally) — main.ts uses this to recompute the 2D camera transform
   * and point size, and to resync the sim loop clock, since particle count
   * and/or domain size may have changed. Not used for the 3D path — see
   * `onAfter3DReset`. */
  onAfterReset: () => void;
  /** Called after a 3D Reset (or switching Dimension to '3D', which starts
   * from whatever scenario the 3D controller already has loaded) —
   * main.ts uses this to recompute the orbit camera's target (the new
   * domain's center) and resync the sim loop clock. Kept separate from
   * `onAfterReset` since the 2D and 3D paths recompute genuinely different
   * camera state (an orthographic domain-to-clip-space transform vs. an
   * orbit camera target). */
  onAfter3DReset: () => void;
  /** Called whenever the Dimension toggle changes. main.ts uses this to
   * switch which render/step path is active; the previously-active
   * pipeline's state is left untouched (not reset) so switching back
   * restores it exactly as it was. */
  onDimensionChange: (dimension: Dimension) => void;
}

const SCENARIO_OPTIONS: Record<string, ScenarioName> = {
  'Dam Break': 'damBreak',
  'Block Drop': 'blockDrop',
  'Two-Column Collision': 'twoColumn',
};

const SOLVER_OPTIONS: Record<string, SolverKind> = {
  SPH: 'sph',
  FLIP: 'flip',
  MPM: 'mpm',
};

const MATERIAL_OPTIONS: Record<string, MaterialKind> = {
  Water: 'water',
  Smoke: 'smoke',
};

const DIMENSION_OPTIONS: Record<string, Dimension> = {
  '2D': 'twoD',
  '3D': 'threeD',
};

const SCENARIO_3D_OPTIONS: Record<string, Scenario3DName> = {
  'Dam Break 3D': 'damBreak3D',
  'Block Drop 3D': 'blockDrop3D',
  'Two-Column Collision 3D': 'twoColumn3D',
};

// FlipParams defaults per docs/flip-sim-api.md's field table — hardcoded
// here (rather than spun up from a throwaway FlipSimulation instance just
// to read defaults) since the doc pins them exactly and the panel needs a
// FLIP-shaped state object to exist before FLIP is ever selected.
const FLIP_DEFAULTS: FlipParams = {
  gravity: -9.81,
  flipRatio: 0.95,
  cellSize: 0.025,
};

// MpmParams defaults per docs/mpm-sim-api.md's field table — hardcoded here
// for the same reason as FLIP_DEFAULTS: the doc pins them exactly and the
// panel needs an MPM-shaped state object to exist before MPM is ever
// selected.
const MPM_DEFAULTS: MpmParams = {
  gravity: -9.81,
  restDensity: 1000.0,
  stiffness: 20.0,
  viscosity: 0.0,
  cellSize: 0.02,
};

// FlipSmokeParams defaults per docs/flip-smoke-sim-api.md's field table —
// hardcoded here for the same reason as FLIP_DEFAULTS/MPM_DEFAULTS: the doc
// pins them exactly and the panel needs a Smoke-shaped state object to
// exist before Smoke is ever selected, its param mirror created up front
// like every other solver's.
const SMOKE_DEFAULTS: FlipSmokeParams = {
  buoyancy: 4.0,
  ambientTemperature: 0.0,
  sourceTemperature: 1.0,
  densityDissipation: 0.1,
  temperatureDissipation: 0.5,
  cellSize: 0.02,
};

// MpmSmokeParams defaults per docs/mpm-smoke-sim-api.md's field table —
// hardcoded here for the same reason as SMOKE_DEFAULTS: the doc pins them
// exactly and the panel needs an MPM-Smoke-shaped state object to exist
// before it's ever selected. Shares buoyancy/ambientTemperature/
// sourceTemperature/densityDissipation/temperatureDissipation/cellSize
// defaults with FLIP Smoke (same formulas/semantics per the doc), but adds
// `damping` (0 = disabled, same default) and `maxParticles` (fixed particle
// buffer capacity) which have no FLIP-Smoke equivalent.
const MPM_SMOKE_DEFAULTS: MpmSmokeParams = {
  buoyancy: 4.0,
  ambientTemperature: 0.0,
  sourceTemperature: 1.0,
  densityDissipation: 0.1,
  temperatureDissipation: 0.5,
  cellSize: 0.02,
  damping: 0.0,
  maxParticles: 4000,
};

// SphSmokeParams defaults per docs/sph-smoke-sim-api.md's field table —
// hardcoded here for the same reason as the other *_SMOKE_DEFAULTS: the doc
// pins them exactly and the panel needs an SPH-Smoke-shaped state object to
// exist before it's ever selected. Shares buoyancy/ambientTemperature/
// sourceTemperature/densityDissipation/temperatureDissipation/maxParticles
// defaults/semantics with MPM Smoke, but replaces cellSize/damping with
// smoothingRadius (kernel support radius, notably larger than water SPH's
// 0.045m default — see the doc's "Why smoothingRadius is larger than
// water's") and viscosity (kernel-weighted Laplacian coefficient, 75x water
// SPH's default — this solver's sole coherence mechanism, since it has no
// pressure force/EOS at all).
const SPH_SMOKE_DEFAULTS: SphSmokeParams = {
  buoyancy: 4.0,
  ambientTemperature: 0.0,
  sourceTemperature: 1.0,
  densityDissipation: 0.1,
  temperatureDissipation: 0.5,
  smoothingRadius: 0.09,
  viscosity: 6.0,
  maxParticles: 4000,
};

/** Builds the Tweakpane parameter panel and wires every control to the
 * SolverManager / FixedTimestepLoop. See docs/sim-api.md and
 * docs/flip-sim-api.md for the field ranges/defaults this is built around.
 *
 * Design note on the cellSize-needs-Reset gotcha (docs/flip-sim-api.md:
 * `cellSize` only takes effect at the next `reset()`): this panel does NOT
 * auto-reset on cellSize change — it just labels the slider so the user
 * knows a Reset click is needed to see the effect, consistent with every
 * other scenario/spacing change already requiring an explicit Reset in
 * this UI (spacing has the same "takes effect on next Reset" behavior for
 * both solvers already). Auto-reset would also be surprising mid-drag
 * (rebuilding the whole MAC grid on every slider tick).
 */
export function createPanel(
  manager: SolverManager,
  loop: FixedTimestepLoop,
  callbacks: PanelCallbacks,
  sph3d: Sph3DSimulationController
): Pane {
  const initialSphParams = manager.sph!.getParams();
  const initialSph3DParams = sph3d.getParams();

  // Persisted per-solver param mirrors: the UI's source of truth for each
  // solver's sliders, independent of which solver is currently active.
  // Kept across solver switches so comparing SPH vs FLIP back and forth
  // doesn't silently discard the user's tuning (SolverManager.switchTo
  // pushes whichever of these matches the target kind onto the freshly
  // constructed instance).
  const sphParams: SPHParams = { ...initialSphParams };
  const flipParams: FlipParams = { ...FLIP_DEFAULTS };
  const mpmParams: MpmParams = { ...MPM_DEFAULTS };
  const smokeParams: FlipSmokeParams = { ...SMOKE_DEFAULTS };
  const mpmSmokeParams: MpmSmokeParams = { ...MPM_SMOKE_DEFAULTS };
  const sphSmokeParams: SphSmokeParams = { ...SPH_SMOKE_DEFAULTS };
  // 3D SPH water's own param mirror — read from the freshly-constructed
  // Sph3DSimulationController rather than hardcoded, since its defaults
  // (per docs/sph3d-sim-api.md) are NOT the 2D defaults relabeled.
  const sph3DParams: SphParams3D = { ...initialSph3DParams };

  const state = {
    // Scenario / spacing (solver-agnostic — same Scenario struct/presets
    // work for both solvers per docs/flip-sim-api.md). Water-only; smoke
    // has no scenario preset choice or particle spacing concept.
    spacing: manager.spacing,
    scenario: 'damBreak' as ScenarioName,
    // Sim loop.
    playing: loop.playing,
    timeScale: loop.timeScale,
    // Solver/material selectors.
    solver: 'sph' as SolverKind,
    material: 'water' as MaterialKind,
    // Top-level Dimension toggle (see the `Dimension` type doc comment) —
    // defaults to '2D' so the app boots into the exact same behavior it
    // always has.
    dimension: 'twoD' as Dimension,
    scenario3D: 'damBreak3D' as Scenario3DName,
  };

  const pane = new Pane({ title: 'Fluids' });

  /** Keeps the visible params folder / scenario controls in sync with the
   * current (solver, material) selection. Smoke now exists for all three
   * solver families — FLIP (Phase 4a, grid-based), MPM (Phase 4b,
   * particle-based), SPH (Phase 4c, particle-based) — see SolverManager's
   * MaterialKind doc comment — so exactly one of the six params folders is
   * ever visible at a time (the three smoke folders are mutually exclusive
   * since only one solver is active at once), and the water-only Scenario
   * controls (preset dropdown + spacing) hide whenever Smoke is active
   * since there's only one built-in SmokeScenario preset and no particle
   * spacing concept for any smoke variant (MPM/SPH Smoke's "spacing"-like
   * knob is its own Cell size/Smoothing radius field in its own folder, not
   * the shared water Scenario spacing slider). */
  function updateVisibility(): void {
    const is3D = state.dimension === 'threeD';
    const isSmoke = state.material === 'smoke';
    const isFlipSmoke = isSmoke && state.solver === 'flip';
    const isMpmSmoke = isSmoke && state.solver === 'mpm';
    const isSphSmoke = isSmoke && state.solver === 'sph';
    // The entire 2D solver/material/scenario UI hides while in 3D — 3D
    // currently only supports one (solver, material) combination (SPH
    // Water, per docs/sph3d-sim-api.md), so rather than leaving a
    // disabled-but-visible Solver/Material dropdown sitting on screen (which
    // would still need every option grayed out except one), hiding the
    // whole 2D section and showing the dedicated "3D Scenario"/"3D SPH
    // Params" folders instead makes the only reachable combination obvious
    // and makes an invalid one structurally unreachable.
    solverFolder.hidden = is3D;
    scenarioFolder.hidden = is3D || isSmoke;
    sphFolder.hidden = is3D || isSmoke || state.solver !== 'sph';
    flipFolder.hidden = is3D || isSmoke || state.solver !== 'flip';
    mpmFolder.hidden = is3D || isSmoke || state.solver !== 'mpm';
    smokeFolder.hidden = is3D || !isFlipSmoke;
    mpmSmokeFolder.hidden = is3D || !isMpmSmoke;
    sphSmokeFolder.hidden = is3D || !isSphSmoke;
    scenario3DFolder.hidden = !is3D;
    sph3DFolder.hidden = !is3D;
    // Smoke now exists for every solver (Phase 4 complete) — Material is
    // always selectable (in 2D).
    materialBinding.disabled = false;
  }

  const dimensionBinding = pane.addBinding(state, 'dimension', {
    label: 'Dimension',
    options: DIMENSION_OPTIONS,
  });
  dimensionBinding.on('change', (ev) => {
    updateVisibility();
    callbacks.onDimensionChange(ev.value);
    loop.resync();
    if (ev.value === 'threeD') {
      callbacks.onAfter3DReset();
    } else {
      callbacks.onAfterReset();
    }
  });

  const solverFolder = pane.addFolder({ title: 'Solver' });
  const materialBinding = solverFolder.addBinding(state, 'material', {
    label: 'Material',
    options: MATERIAL_OPTIONS,
  });
  materialBinding.on('change', (ev) => {
    manager.switchTo(
      state.solver,
      ev.value,
      state.scenario,
      state.spacing,
      sphParams,
      flipParams,
      mpmParams,
      smokeParams,
      mpmSmokeParams,
      sphSmokeParams
    );
    updateVisibility();
    loop.resync();
    callbacks.onAfterReset();
  });
  solverFolder
    .addBinding(state, 'solver', {
      label: 'Solver',
      options: SOLVER_OPTIONS,
    })
    .on('change', (ev) => {
      // Smoke now exists for every solver (Phase 4 complete) — switching
      // Solver keeps whatever Material was already selected (no more
      // force-back-to-Water special case for SPH).
      manager.switchTo(
        ev.value,
        state.material,
        state.scenario,
        state.spacing,
        sphParams,
        flipParams,
        mpmParams,
        smokeParams,
        mpmSmokeParams,
        sphSmokeParams
      );
      materialBinding.refresh();
      updateVisibility();
      loop.resync();
      callbacks.onAfterReset();
    });

  const scenarioFolder = pane.addFolder({ title: 'Scenario' });
  scenarioFolder.addBinding(state, 'scenario', {
    label: 'Preset',
    options: SCENARIO_OPTIONS,
  });
  scenarioFolder
    .addBinding(state, 'spacing', {
      label: 'Particle spacing (m)',
      min: 0.015,
      max: 0.06,
      step: 0.001,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      const flip = manager.flip;
      const mpm = manager.mpm;
      if (sph) {
        sph.setSpacing(ev.value, sphParams.restDensity);
      } else if (flip) {
        // FLIP: spacing only affects initial seeding density (more
        // particles = smoother per-cell coverage), no client-side
        // calibration formula required (docs/flip-sim-api.md) — just
        // record it for the next reset()/switch.
        flip.spacing = ev.value;
      } else if (mpm) {
        // MPM: same as FLIP, but spacing also feeds the automatic
        // particleMass calibration inside MPM's own reset() (only at the
        // next reset() though — see docs/mpm-sim-api.md).
        mpm.spacing = ev.value;
      }
    });
  scenarioFolder.addButton({ title: 'Reset' }).on('click', () => {
    const sph = manager.sph;
    const flip = manager.flip;
    const mpm = manager.mpm;
    if (sph) {
      sph.reset(state.scenario, sphParams.restDensity);
    } else if (flip) {
      // cellSize (if changed since the last reset) is already live on
      // sim.params via the FLIP Params folder's read-modify-write
      // bindings below, so a plain reset() here picks it up per
      // docs/flip-sim-api.md.
      flip.reset(state.scenario);
    } else if (mpm) {
      // cellSize/restDensity (if changed since the last reset) are already
      // live on sim.params via the MPM Params folder's read-modify-write
      // bindings below, so a plain reset() here picks them up per
      // docs/mpm-sim-api.md.
      mpm.reset(state.scenario);
    }
    loop.resync();
    callbacks.onAfterReset();
  });

  // ---- SPH Params (visible when solver === 'sph') ----
  const sphFolder = pane.addFolder({ title: 'SPH Params' });
  sphFolder
    .addBinding(sphParams, 'gravity', {
      label: 'Gravity (m/s²)',
      min: -20,
      max: 5,
      step: 0.1,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.updateParams((p) => {
        p.gravity = ev.value;
      });
    });
  sphFolder
    .addBinding(sphParams, 'restDensity', {
      label: 'Rest density (kg/m²)',
      min: 200,
      max: 3000,
      step: 10,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.setRestDensity(ev.value);
    });
  sphFolder
    .addBinding(sphParams, 'gasStiffness', {
      label: 'Gas stiffness',
      min: 10,
      max: 1000,
      step: 5,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.updateParams((p) => {
        p.gasStiffness = ev.value;
      });
    });
  sphFolder
    .addBinding(sphParams, 'viscosity', {
      label: 'Viscosity',
      min: 0,
      max: 1,
      step: 0.005,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.updateParams((p) => {
        p.viscosity = ev.value;
      });
    });
  sphFolder
    .addBinding(sphParams, 'smoothingRadius', {
      label: 'Smoothing radius (m)',
      min: 0.01,
      max: 0.12,
      step: 0.001,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.updateParams((p) => {
        p.smoothingRadius = ev.value;
      });
    });
  sphFolder
    .addBinding(sphParams, 'boundaryDamping', {
      label: 'Boundary damping',
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      const sph = manager.sph;
      if (!sph) return;
      sph.updateParams((p) => {
        p.boundaryDamping = ev.value;
      });
    });

  // ---- FLIP Params (visible when solver === 'flip') ----
  const flipFolder = pane.addFolder({ title: 'FLIP Params' });
  flipFolder.hidden = true;
  flipFolder
    .addBinding(flipParams, 'gravity', {
      label: 'Gravity (m/s²)',
      min: -20,
      max: 5,
      step: 0.1,
    })
    .on('change', (ev) => {
      const flip = manager.flip;
      if (!flip) return;
      flip.updateParams((p) => {
        p.gravity = ev.value;
      });
    });
  flipFolder
    .addBinding(flipParams, 'flipRatio', {
      label: 'FLIP ratio',
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      const flip = manager.flip;
      if (!flip) return;
      flip.updateParams((p) => {
        p.flipRatio = ev.value;
      });
    });
  flipFolder
    .addBinding(flipParams, 'cellSize', {
      label: 'Cell size (m) — Reset to apply',
      // Floor per docs/flip-sim-api.md's explicit performance-cliff
      // warning: grid cell count (and the fixed 60-iteration Jacobi
      // pressure solve's per-step cost) scales as ~1/cellSize² for a fixed
      // domain. 0.01 is already ~6x the default's cell count (~30000 vs
      // ~4800 cells for the ~2x1.5m built-in scenarios) — going lower
      // (e.g. 0.005) is documented as visible-frame-drop territory in
      // single-threaded WASM, so the slider simply doesn't go there.
      // Ceiling of 0.2 keeps the grid resolution from collapsing to the
      // point of a blocky, low-detail free surface (the doc calls out
      // >0.3 on a 2m domain as already degraded).
      min: 0.01,
      max: 0.2,
      step: 0.005,
    })
    .on('change', (ev) => {
      const flip = manager.flip;
      if (!flip) return;
      flip.updateParams((p) => {
        p.cellSize = ev.value;
      });
    });

  // ---- MPM Params (visible when solver === 'mpm') ----
  const mpmFolder = pane.addFolder({ title: 'MPM Params' });
  mpmFolder.hidden = true;
  mpmFolder
    .addBinding(mpmParams, 'gravity', {
      label: 'Gravity (m/s²)',
      min: -20,
      max: 5,
      step: 0.1,
    })
    .on('change', (ev) => {
      const mpm = manager.mpm;
      if (!mpm) return;
      mpm.updateParams((p) => {
        p.gravity = ev.value;
      });
    });
  mpmFolder
    .addBinding(mpmParams, 'restDensity', {
      label: 'Rest density (kg/m²) — Reset to apply',
      min: 200,
      max: 3000,
      step: 10,
    })
    .on('change', (ev) => {
      const mpm = manager.mpm;
      if (!mpm) return;
      // Only takes effect at the next reset() (feeds the automatic
      // particleMass calibration inside MPM's own reset(), per
      // docs/mpm-sim-api.md) — still push it onto sim.params immediately so
      // it's live there when Reset is eventually clicked, matching the
      // FLIP cellSize pattern.
      mpm.updateParams((p) => {
        p.restDensity = ev.value;
      });
    });
  mpmFolder
    .addBinding(mpmParams, 'stiffness', {
      label: 'Stiffness',
      // No hard internal ceiling per docs/mpm-sim-api.md, but the doc
      // explicitly cautions that pushing stiffness well past the default
      // (40+) shrinks the stable substep further and "wasn't exhaustively
      // re-verified above stiffness = 40" — cap the slider there rather
      // than let a user drag into unverified territory at the fixed 1/480
      // substep this UI drives MPM at.
      min: 0,
      max: 40,
      step: 1,
    })
    .on('change', (ev) => {
      const mpm = manager.mpm;
      if (!mpm) return;
      mpm.updateParams((p) => {
        p.stiffness = ev.value;
      });
    });
  mpmFolder
    .addBinding(mpmParams, 'viscosity', {
      label: 'Viscosity',
      // Doc: raising this pushes the stable substep down further too — keep
      // the range modest since this UI drives a fixed 1/480 substep rather
      // than adapting it to the viscosity slider.
      min: 0,
      max: 0.5,
      step: 0.01,
    })
    .on('change', (ev) => {
      const mpm = manager.mpm;
      if (!mpm) return;
      mpm.updateParams((p) => {
        p.viscosity = ev.value;
      });
    });
  mpmFolder
    .addBinding(mpmParams, 'cellSize', {
      label: 'Cell size (m) — Reset to apply',
      // Floor per docs/mpm-sim-api.md's explicit performance-cliff warning
      // (practical floor ~0.008-0.01 for the default ~2m domains) — same
      // treatment as FLIP's cellSize slider. Ceiling of 0.1 keeps the
      // 3-cell-wide boundary margin (3*cellSize) from eating too much of
      // the domain (the doc calls out a 15%-per-side margin at 0.1).
      min: 0.008,
      max: 0.1,
      step: 0.001,
    })
    .on('change', (ev) => {
      const mpm = manager.mpm;
      if (!mpm) return;
      mpm.updateParams((p) => {
        p.cellSize = ev.value;
      });
    });

  // ---- Smoke Params (visible when material === 'smoke' && solver ===
  // 'flip'; grid-based, Phase 4a) ----
  const smokeFolder = pane.addFolder({ title: 'Smoke Params (FLIP)' });
  smokeFolder.hidden = true;
  smokeFolder
    .addBinding(smokeParams, 'buoyancy', {
      label: 'Buoyancy (m/s² per unit T)',
      // docs/flip-smoke-sim-api.md: near 0, no visible rise at all; a clear
      // plume forms above ~2-3; 10+ shoots up fast and can make the fixed
      // 60-iteration Jacobi solve's residual divergence visually noticeable.
      min: 0,
      max: 10,
      step: 0.1,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.buoyancy = ev.value;
      });
    });
  smokeFolder
    .addBinding(smokeParams, 'ambientTemperature', {
      label: 'Ambient temperature',
      // Dimensionless relative scale (docs/flip-smoke-sim-api.md) — default
      // 0 is a convention, not a requirement. Modest range around the
      // default; only the difference from sourceTemperature matters.
      min: -2,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.ambientTemperature = ev.value;
      });
    });
  smokeFolder
    .addBinding(smokeParams, 'sourceTemperature', {
      label: 'Source temperature',
      // Combined with the default buoyancy, this is what makes the plume
      // rise (source hotter than ambient) — see docs/flip-smoke-sim-api.md.
      min: 0,
      max: 5,
      step: 0.05,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.sourceTemperature = ev.value;
      });
    });
  smokeFolder
    .addBinding(smokeParams, 'densityDissipation', {
      label: 'Density dissipation (1/s)',
      // docs/flip-smoke-sim-api.md: near 0, smoke never clears (accumulates
      // toward the 1.0 clamp and stays); higher values fade the plume out
      // sooner after leaving the source.
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.densityDissipation = ev.value;
      });
    });
  smokeFolder
    .addBinding(smokeParams, 'temperatureDissipation', {
      label: 'Temperature dissipation (1/s)',
      // docs/flip-smoke-sim-api.md: low values keep accelerating upward for
      // longer (tends to hit the top and pool); higher values curl and
      // stall closer to the source.
      min: 0,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.temperatureDissipation = ev.value;
      });
    });
  smokeFolder
    .addBinding(smokeParams, 'cellSize', {
      label: 'Cell size (m) — Reset to apply',
      // Floor per docs/flip-smoke-sim-api.md's explicit performance-cliff
      // warning (same ~1/cellSize² cost shape as FLIP water's cellSize,
      // fixed 60-iteration Jacobi solve every step regardless of grid
      // size) — the doc recommends flooring around 0.01-0.015 for this
      // solver's 1.2m x 2.0m default domain. Ceiling kept modest since the
      // domain itself is small.
      min: 0.01,
      max: 0.05,
      step: 0.001,
    })
    .on('change', (ev) => {
      const smoke = manager.smoke;
      if (!smoke) return;
      smoke.updateParams((p) => {
        p.cellSize = ev.value;
      });
    });
  smokeFolder.addButton({ title: 'Reset (reseed plume)' }).on('click', () => {
    const smoke = manager.smoke;
    if (!smoke) return;
    // Only one built-in SmokeScenario preset so far (makePlumeScenario()) —
    // see docs/flip-smoke-sim-api.md — so Reset always reseeds that.
    smoke.reset();
    loop.resync();
    callbacks.onAfterReset();
  });

  // ---- Smoke Params (visible when material === 'smoke' && solver ===
  // 'mpm'; particle-based, Phase 4b — see docs/mpm-smoke-sim-api.md) ----
  const mpmSmokeFolder = pane.addFolder({ title: 'Smoke Params (MPM)' });
  mpmSmokeFolder.hidden = true;
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'buoyancy', {
      label: 'Buoyancy (m/s² per unit T)',
      // Same formula/default/visual behavior as FLIP Smoke's buoyancy (see
      // docs/mpm-smoke-sim-api.md) — same slider range reasoning applies.
      min: 0,
      max: 10,
      step: 0.1,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.buoyancy = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'ambientTemperature', {
      label: 'Ambient temperature',
      min: -2,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.ambientTemperature = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'sourceTemperature', {
      label: 'Source temperature',
      min: 0,
      max: 5,
      step: 0.05,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.sourceTemperature = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'densityDissipation', {
      label: 'Density dissipation (1/s)',
      // Also controls how long a particle survives before culling (density
      // < 0.01) — see docs/mpm-smoke-sim-api.md.
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.densityDissipation = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'temperatureDissipation', {
      label: 'Temperature dissipation (1/s)',
      min: 0,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.temperatureDissipation = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'cellSize', {
      label: 'Cell size (m) — Reset to apply',
      // MPM node grid spacing for P2G/G2P (not a rendering resolution) — same
      // default/floor reasoning as FLIP Smoke's cellSize, docs/mpm-smoke-sim-api.md.
      min: 0.01,
      max: 0.05,
      step: 0.001,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.cellSize = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'damping', {
      label: 'Damping (1/s)',
      // Optional linear velocity drag, default 0 (disabled) — per
      // docs/mpm-smoke-sim-api.md, not needed for a stable/coherent look at
      // default buoyancy/emission, but left as a knob for more chaotic
      // parameter combinations (e.g. much higher buoyancy/emissionRate).
      min: 0,
      max: 5,
      step: 0.05,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.damping = ev.value;
      });
    });
  mpmSmokeFolder
    .addBinding(mpmSmokeParams, 'maxParticles', {
      label: 'Max particles — Reset to apply',
      // Fixed particle-buffer capacity, (re)allocated only at reset() (same
      // reset()-only contract as cellSize). Default 4000 is comfortably
      // above the ~1800 steady-state count the Plume preset's defaults
      // reach in practice (docs/mpm-smoke-sim-api.md) — a safety backstop,
      // not a normally-hit ceiling, so the slider allows raising or
      // lowering it for experimentation.
      min: 500,
      max: 20000,
      step: 100,
    })
    .on('change', (ev) => {
      const mpmSmoke = manager.mpmSmoke;
      if (!mpmSmoke) return;
      mpmSmoke.updateParams((p) => {
        p.maxParticles = ev.value;
      });
    });
  mpmSmokeFolder.addButton({ title: 'Reset (reseed plume)' }).on('click', () => {
    const mpmSmoke = manager.mpmSmoke;
    if (!mpmSmoke) return;
    // Only one built-in SmokeScenario preset so far (makePlumeScenario()) —
    // see docs/mpm-smoke-sim-api.md — so Reset always reseeds that.
    mpmSmoke.reset();
    loop.resync();
    callbacks.onAfterReset();
  });

  // ---- Smoke Params (visible when material === 'smoke' && solver ===
  // 'sph'; particle-based, Phase 4c — see docs/sph-smoke-sim-api.md) ----
  const sphSmokeFolder = pane.addFolder({ title: 'Smoke Params (SPH)' });
  sphSmokeFolder.hidden = true;
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'buoyancy', {
      label: 'Buoyancy (m/s² per unit T)',
      // Same formula/default/visual behavior as the other two Smoke
      // variants' buoyancy (see docs/sph-smoke-sim-api.md) — same slider
      // range reasoning applies.
      min: 0,
      max: 10,
      step: 0.1,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.buoyancy = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'ambientTemperature', {
      label: 'Ambient temperature',
      min: -2,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.ambientTemperature = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'sourceTemperature', {
      label: 'Source temperature',
      min: 0,
      max: 5,
      step: 0.05,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.sourceTemperature = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'densityDissipation', {
      label: 'Density dissipation (1/s)',
      // Also controls how long a particle survives before culling (density
      // < 0.01) — see docs/sph-smoke-sim-api.md.
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.densityDissipation = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'temperatureDissipation', {
      label: 'Temperature dissipation (1/s)',
      min: 0,
      max: 2,
      step: 0.05,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.temperatureDissipation = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'smoothingRadius', {
      label: 'Smoothing radius (m)',
      // SPH kernel support radius — this solver's analog of the grid-based
      // smoke variants' cellSize. Unlike cellSize, changing this does NOT
      // require a Reset (docs/sph-smoke-sim-api.md — the neighbor-search
      // grid is rebuilt from scratch every step() regardless). Default
      // 0.09m is already 2x water SPH's 0.045m default (tuned for this
      // solver's much sparser particle population) — range centered on
      // that rather than water SPH's smoothingRadius slider range.
      min: 0.02,
      max: 0.2,
      step: 0.001,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.smoothingRadius = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'viscosity', {
      label: 'Viscosity',
      // Kernel-weighted Laplacian viscosity — this solver's SOLE
      // inter-particle coherence mechanism (no pressure force/EOS at all,
      // see docs/sph-smoke-sim-api.md), hence a much higher default (6.0)
      // and range than water SPH's viscosity slider (0-1). 0 degenerates to
      // ballistic buoyant particles with zero inter-particle coupling.
      min: 0,
      max: 15,
      step: 0.1,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.viscosity = ev.value;
      });
    });
  sphSmokeFolder
    .addBinding(sphSmokeParams, 'maxParticles', {
      label: 'Max particles — Reset to apply',
      // Fixed particle-buffer capacity, (re)allocated only at reset() — same
      // reset()-only contract as MPM Smoke's maxParticles. Default 4000 is
      // comfortably above the ~1800 steady-state count the Plume preset's
      // defaults reach in practice (docs/sph-smoke-sim-api.md) — a safety
      // backstop, not a normally-hit ceiling.
      min: 500,
      max: 20000,
      step: 100,
    })
    .on('change', (ev) => {
      const sphSmoke = manager.sphSmoke;
      if (!sphSmoke) return;
      sphSmoke.updateParams((p) => {
        p.maxParticles = ev.value;
      });
    });
  sphSmokeFolder.addButton({ title: 'Reset (reseed plume)' }).on('click', () => {
    const sphSmoke = manager.sphSmoke;
    if (!sphSmoke) return;
    // Only one built-in SmokeScenario preset so far (makePlumeScenario()) —
    // see docs/sph-smoke-sim-api.md — so Reset always reseeds that.
    sphSmoke.reset();
    loop.resync();
    callbacks.onAfterReset();
  });

  // ---- 3D Scenario (visible when dimension === 'threeD') ----
  const scenario3DFolder = pane.addFolder({ title: '3D Scenario' });
  scenario3DFolder.hidden = true;
  scenario3DFolder.addBinding(state, 'scenario3D', {
    label: 'Preset',
    options: SCENARIO_3D_OPTIONS,
  });
  scenario3DFolder.addButton({ title: 'Reset' }).on('click', () => {
    sph3d.reset(state.scenario3D);
    loop.resync();
    callbacks.onAfter3DReset();
  });

  // ---- 3D SPH Params (visible when dimension === 'threeD') — mirrors the
  // 2D SPH Params folder's field set, but every default/range comes from
  // docs/sph3d-sim-api.md's SphParams3D table, NOT the 2D defaults: 3D has
  // more neighbors per unit smoothing radius than 2D, so these were derived
  // independently (see the doc's "Why the defaults differ from 2D").
  // Deliberately no particle-spacing slider and no particleMass field here
  // — see Sph3DSimulationController's doc comment for why this first 3D
  // slice leaves that calibration alone rather than inventing a rescaling
  // formula for a value the doc explicitly says is measured, not derived. ----
  const sph3DFolder = pane.addFolder({ title: '3D SPH Params' });
  sph3DFolder.hidden = true;
  sph3DFolder
    .addBinding(sph3DParams, 'gravity', {
      label: 'Gravity (m/s²)',
      min: -20,
      max: 5,
      step: 0.1,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.gravity = ev.value;
      });
    });
  sph3DFolder
    .addBinding(sph3DParams, 'restDensity', {
      label: 'Rest density (kg/m³)',
      // Note: unlike the 2D panel's restDensity slider (which recalibrates
      // particleMass via `restDensity * spacing²` on every change), this
      // does NOT recalibrate particleMass. docs/sph3d-sim-api.md's
      // calibration section is explicit that 3D's correction factor
      // (~1.468x on top of the naive `restDensity * spacing³` formula) is a
      // *measured* value tied to this build's smoothingRadius/spacing
      // ratio, not a formula safe to reapply from the frontend — inventing
      // one here would risk silently decalibrating the sim exactly the way
      // the doc warns against for a spacing slider. Changing this away from
      // the preset's calibrated 1000 will visibly under/over-pressurize the
      // fluid; left tunable (per this task's "mirror the field set" ask)
      // but not auto-corrected.
      min: 200,
      max: 3000,
      step: 10,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.restDensity = ev.value;
      });
    });
  sph3DFolder
    .addBinding(sph3DParams, 'gasStiffness', {
      label: 'Gas stiffness',
      min: 10,
      max: 1000,
      step: 5,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.gasStiffness = ev.value;
      });
    });
  sph3DFolder
    .addBinding(sph3DParams, 'viscosity', {
      label: 'Viscosity',
      min: 0,
      max: 1,
      step: 0.005,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.viscosity = ev.value;
      });
    });
  sph3DFolder
    .addBinding(sph3DParams, 'smoothingRadius', {
      label: 'Smoothing radius (m)',
      // Default 0.09m is 2x the 2D default in absolute terms (deliberately
      // coarser — see docs/sph3d-sim-api.md's "Why the defaults differ from
      // 2D") — range shifted up accordingly rather than reusing 2D SPH
      // water's 0.01-0.12 range.
      min: 0.03,
      max: 0.2,
      step: 0.001,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.smoothingRadius = ev.value;
      });
    });
  sph3DFolder
    .addBinding(sph3DParams, 'boundaryDamping', {
      label: 'Boundary damping',
      min: 0,
      max: 1,
      step: 0.01,
    })
    .on('change', (ev) => {
      sph3d.updateParams((p) => {
        p.boundaryDamping = ev.value;
      });
    });

  const playbackFolder = pane.addFolder({ title: 'Playback' });
  playbackFolder.addBinding(state, 'playing', { label: 'Playing' }).on('change', (ev) => {
    loop.playing = ev.value;
    if (ev.value) loop.resync();
    stepButton.disabled = ev.value;
  });
  const stepButton: ButtonApi = playbackFolder.addButton({ title: 'Step' });
  stepButton.disabled = state.playing;
  stepButton.on('click', () => {
    if (!loop.playing) loop.stepOnce();
  });
  playbackFolder
    .addBinding(state, 'timeScale', {
      label: 'Time scale',
      min: 0,
      max: 3,
      step: 0.1,
    })
    .on('change', (ev) => {
      loop.timeScale = ev.value;
    });

  // Synchronize initial visibility/disabled state with the initial
  // (solver: 'sph', material: 'water') selection — belt-and-suspenders
  // alongside the per-folder `hidden`/`disabled` flags already set inline
  // above, so this stays correct even if that inline set ever drifts.
  updateVisibility();

  return pane;
}
