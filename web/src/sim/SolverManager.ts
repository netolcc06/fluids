import type {
  FluidsModule,
  FlipParams,
  FlipSmokeParams,
  MpmParams,
  MpmSmokeParams,
  SPHParams,
  SphSmokeParams,
} from './types';
import { ScenarioName } from './SimulationControllerBase';
import { SPHSimulationController } from './SPHSimulationController';
import { FlipSimulationController } from './FlipSimulationController';
import { MpmSimulationController } from './MpmSimulationController';
import { FlipSmokeSimulationController } from './FlipSmokeSimulationController';
import { MpmSmokeSimulationController } from './MpmSmokeSimulationController';
import { SphSmokeSimulationController } from './SphSmokeSimulationController';

export type SolverKind = 'sph' | 'flip' | 'mpm';
/** Smoke now exists for all three solver families: FLIP (Phase 4a,
 * grid-based), MPM (Phase 4b, particle-based via APIC), and SPH (Phase 4c,
 * particle-based via direct kernel-weighted neighbor forces) — this
 * completes Phase 4. Every (SolverKind, MaterialKind) combination is now
 * reachable. */
export type MaterialKind = 'water' | 'smoke';

const EMPTY_FLOAT32 = new Float32Array(0);

type WaterController = SPHSimulationController | FlipSimulationController | MpmSimulationController;
/** Controllers whose state is particle-shaped (positions/velocities/
 * particleCount) — every water controller, plus MPM Smoke and SPH Smoke
 * (unlike FLIP Smoke, which is grid-shaped and has none of this). */
type ParticleShapedController = WaterController | MpmSmokeSimulationController | SphSmokeSimulationController;
/** Controllers whose state is particle-shaped AND colored by
 * density/temperature rather than velocity (`renderMode ===
 * 'particlesDensity'`) — MPM Smoke and SPH Smoke share the exact same
 * `positions`/`velocities`/`particleCount`/`densities`/`temperatures`/
 * `getParams()` shape (both mirror `MpmSmokeSimulationController`'s
 * interface; see SphSmokeSimulationController's doc comment), so main.ts's
 * render branch can treat either uniformly through `SolverManager
 * .particleSmoke` without caring which specific solver produced it. */
type ParticleSmokeController = MpmSmokeSimulationController | SphSmokeSimulationController;

/** Fixed substep (seconds) each solver/material combo should be driven at,
 * independent of display refresh rate. SPH/FLIP water are stable at the
 * "standard" 1/120; MPM's explicit time integration with a stiff EOS is
 * noticeably less forgiving — docs/mpm-sim-api.md's empirical findings show
 * `1/240` diverges at every tested stiffness while `1/480` is stable, so MPM
 * must be driven at its own smaller substep. FLIP Smoke is its own case
 * again: docs/flip-smoke-sim-api.md recommends `1/60` — noticeably LARGER
 * (cheaper) than water's `1/120`, since Stable Fluids' semi-Lagrangian
 * advection is unconditionally stable by construction and `1/60` is purely
 * a visual-quality choice, not a stability requirement (the doc's substep
 * section explains why this doesn't transfer from either water number). */
const RECOMMENDED_SUBSTEP_WATER: Record<SolverKind, number> = {
  sph: 1 / 120,
  flip: 1 / 120,
  mpm: 1 / 480,
};
// docs/sph-smoke-sim-api.md confirms the same 1/60 recommendation as the
// other two Smoke variants (measured directly: stable across every tested
// dt, 1/30 down to 1/240 — no CFL-adaptive substepping needed, since there
// is no stiff pressure/EOS feedback loop for large dt to destabilize, same
// underlying reason as MPM Smoke's finding). `recommendedSubstep` below
// already keys purely on `material === 'smoke'`, independent of `kind`, so
// SPH Smoke automatically picks this up with no further change needed here.
const SMOKE_RECOMMENDED_SUBSTEP = 1 / 60;

/**
 * Owns whichever solver controller is currently active and handles
 * switching between them — now solver AND material aware. `main.ts`/the
 * render loop hold a single `SolverManager` instance and always go through
 * it rather than holding a direct reference to the controller, since the
 * controller identity itself changes on every solver/material switch.
 *
 * Render-mode split: FLIP Smoke has no particles at all (docs/flip-smoke-sim-api.md
 * — pure grid-shaped density/temperature field), while MPM Smoke and SPH
 * Smoke ARE particle-shaped (docs/mpm-smoke-sim-api.md /
 * docs/sph-smoke-sim-api.md — MPM reuses MPM water's APIC machinery, SPH
 * reuses SPH water's kernel/neighbor-search machinery) but colored by
 * density/temperature rather than velocity. So `renderMode` is a 3-way
 * discriminant ('particles' | 'particlesDensity' | 'field') `main.ts`
 * switches on once per frame. `.positions`/`.velocities`/`.particleCount`
 * are valid for ALL particle render modes (water and both particle-based
 * Smoke variants share the same underlying shape) and harmlessly return
 * empty/zero for 'field'; `.smoke`/`.mpmSmoke`/`.sphSmoke` are the
 * type-narrowing accessors (mirroring `.sph`/`.flip`/`.mpm`) for each smoke
 * variant's own additional state (grid field pointers vs. density/
 * temperature particle arrays); `.particleSmoke` further collapses
 * `.mpmSmoke`/`.sphSmoke` into one accessor for render code that doesn't
 * need to distinguish which particle-based Smoke solver is active.
 *
 * Embind lifecycle: every `switchTo` call constructs a brand-new instance
 * for the target solver+material and `dispose()`s the outgoing one —
 * `FlipSmokeSimulation`/`MpmSmokeSimulation`/`SphSmokeSimulation` included
 * (verified against the compiled glue: all three are bound with a plain
 * `constructor<>()`, same as every water solver, so none is ever
 * automatically garbage-collected — see FlipSmokeSimulationController/
 * MpmSmokeSimulationController/SphSmokeSimulationController's `dispose()`
 * doc comments). Skipping this would leak the outgoing instance's WASM-heap
 * memory every time the user flips the Solver/Material dropdowns, which is
 * expected normal usage, not a rare edge case.
 */
export class SolverManager {
  private readonly module: FluidsModule;
  kind: SolverKind = 'sph';
  material: MaterialKind = 'water';
  active: WaterController | FlipSmokeSimulationController | MpmSmokeSimulationController | SphSmokeSimulationController;

  constructor(module: FluidsModule) {
    this.module = module;
    this.active = new SPHSimulationController(module);
  }

  /** null when the active controller is FLIP Smoke (no particle-shaped
   * state to narrow to) — internal helper backing the particle-shaped
   * getters below. Includes MPM Smoke and SPH Smoke: both ARE
   * particle-shaped, just colored differently (see renderMode). */
  private get particleActive(): ParticleShapedController | null {
    return this.active instanceof FlipSmokeSimulationController ? null : this.active;
  }
  /** Strictly water (excludes all three smoke variants) — backs `.spacing`'s
   * water-only fallback and is kept private/internal only. */
  private get waterActive(): WaterController | null {
    return this.active instanceof FlipSmokeSimulationController ||
      this.active instanceof MpmSmokeSimulationController ||
      this.active instanceof SphSmokeSimulationController
      ? null
      : this.active;
  }
  /** The active controller when it's one of the two particle-based Smoke
   * variants (MPM or SPH) — null otherwise (water, or FLIP Smoke's
   * grid-shaped field). Both mirror the exact same interface shape (see
   * `ParticleSmokeController`'s doc comment), so main.ts's
   * `renderMode === 'particlesDensity'` branch can go through this single
   * accessor without needing to know or care which specific solver produced
   * the current frame's particles. */
  get particleSmoke(): ParticleSmokeController | null {
    return this.active instanceof MpmSmokeSimulationController || this.active instanceof SphSmokeSimulationController
      ? this.active
      : null;
  }

  /** Which renderer main.ts should call this frame. FLIP Smoke is the only
   * 'field' case; MPM Smoke and SPH Smoke are both 'particlesDensity'
   * (particle-shaped, but colored by density/temperature, not velocity);
   * everything else is the original 'particles' (velocity-colored) case. */
  get renderMode(): 'particles' | 'particlesDensity' | 'field' {
    if (this.material !== 'smoke') return 'particles';
    return this.kind === 'flip' ? 'field' : 'particlesDensity';
  }

  get positions(): Float32Array {
    return this.particleActive?.positions ?? EMPTY_FLOAT32;
  }
  get velocities(): Float32Array {
    return this.particleActive?.velocities ?? EMPTY_FLOAT32;
  }
  get particleCount(): number {
    return this.particleActive?.particleCount ?? 0;
  }
  /** Valid for every render mode: every controller (water, FLIP Smoke, MPM
   * Smoke) mirrors domainWidth/domainHeight fields specifically so
   * camera-transform code in main.ts doesn't need to branch on render mode
   * just to size the domain. */
  get domainWidth(): number {
    return this.active.domainWidth;
  }
  get domainHeight(): number {
    return this.active.domainHeight;
  }
  /** Particle spacing — meaningless for FLIP Smoke (no particles); returns 0
   * for it. For MPM Smoke (no explicit "spacing" concept either, since
   * particles are continuously emitted rather than seeded at a fixed
   * spacing), returns `params.cellSize` as a reasonable visual-scale proxy
   * for point-size computation (same role FLIP/MPM water's `cellSize` plays
   * for those controllers' particle density). SPH Smoke has no `cellSize`
   * at all (see docs/sph-smoke-sim-api.md — no grid, `smoothingRadius` is
   * its closest analog), so it uses `params.smoothingRadius` as the same
   * kind of visual-scale proxy instead. Only consumed by particle-mode-only
   * UI/rendering (e.g. point-size computation), which should be gated on
   * renderMode anyway. */
  get spacing(): number {
    if (this.active instanceof MpmSmokeSimulationController) {
      return this.active.getParams().cellSize;
    }
    if (this.active instanceof SphSmokeSimulationController) {
      return this.active.getParams().smoothingRadius;
    }
    return this.waterActive?.spacing ?? 0;
  }

  /** Fixed substep (seconds) the active solver+material must be driven at —
   * see RECOMMENDED_SUBSTEP_WATER / SMOKE_RECOMMENDED_SUBSTEP. The render
   * loop (main.ts) consults this every tick rather than being constructed
   * with a single fixed value, so switching solvers/materials live also
   * switches the substep the fixed-timestep accumulator steps by. */
  get recommendedSubstep(): number {
    return this.material === 'smoke' ? SMOKE_RECOMMENDED_SUBSTEP : RECOMMENDED_SUBSTEP_WATER[this.kind];
  }

  step(dt: number): void {
    this.active.step(dt);
  }

  refreshViews(): void {
    this.active.refreshViews();
  }

  /** Type-narrowing helpers for the UI: returns the active controller only
   * when it's actually the requested kind, so a stale event handler (e.g. a
   * Tweakpane binding for the folder that just got hidden) can no-op
   * instead of casting blindly. Water solvers additionally require
   * material === 'water' — when material is 'smoke' the active controller
   * is a *Smoke controller even though `kind` is still 'sph'/'flip'/'mpm'. */
  get sph(): SPHSimulationController | null {
    // Requires material === 'water' now that SPH Smoke exists (Phase 4c) —
    // without this check, selecting SPH+Smoke would have `.active` be a
    // SphSmokeSimulationController while `.sph` still (incorrectly) cast and
    // return it as a SPHSimulationController, since `kind` alone no longer
    // disambiguates. This was already a latent bug (the same class the MPM
    // Smoke agent found and fixed for `.mpm`) — SPH Smoke not existing yet
    // meant `kind === 'sph'` previously implied `material === 'water'`
    // unconditionally, but that implication no longer holds. Same pattern
    // `.flip`/`.mpm` already established.
    return this.kind === 'sph' && this.material === 'water' ? (this.active as SPHSimulationController) : null;
  }
  get flip(): FlipSimulationController | null {
    return this.kind === 'flip' && this.material === 'water' ? (this.active as FlipSimulationController) : null;
  }
  get mpm(): MpmSimulationController | null {
    // Requires material === 'water' now that MPM Smoke exists — without this
    // check, selecting MPM+Smoke would have `.active` be a
    // MpmSmokeSimulationController while `.mpm` still (incorrectly) cast and
    // return it as a MpmSimulationController, since `kind` alone no longer
    // disambiguates. Same pattern `.flip` already established.
    return this.kind === 'mpm' && this.material === 'water' ? (this.active as MpmSimulationController) : null;
  }
  /** FLIP Smoke (grid-shaped) narrowing accessor. */
  get smoke(): FlipSmokeSimulationController | null {
    return this.material === 'smoke' && this.kind === 'flip' ? (this.active as FlipSmokeSimulationController) : null;
  }
  /** MPM Smoke (particle-shaped) narrowing accessor. */
  get mpmSmoke(): MpmSmokeSimulationController | null {
    return this.material === 'smoke' && this.kind === 'mpm' ? (this.active as MpmSmokeSimulationController) : null;
  }
  /** SPH Smoke (particle-shaped, Phase 4c) narrowing accessor — mirrors
   * `.mpmSmoke` exactly. */
  get sphSmoke(): SphSmokeSimulationController | null {
    return this.material === 'smoke' && this.kind === 'sph' ? (this.active as SphSmokeSimulationController) : null;
  }

  /**
   * Switch to a different solver and/or material: constructs a fresh
   * instance for the resolved (kind, material) pair, seeds/configures it,
   * and disposes the outgoing instance. Used both for the Solver dropdown
   * (kind changes) and the Material dropdown (material changes, kind
   * stays whatever it already was) — Panel.ts calls this uniformly for both.
   *
   * Smoke now exists for all three solver families (Phase 4 complete) — no
   * (kind, material) combination is forced/invalid anymore, so
   * `resolvedMaterial` below is just `material` unchanged. (Historically,
   * before SPH Smoke existed, this method forced `material` back to
   * 'water' whenever `kind === 'sph'`; that defense-in-depth is no longer
   * needed now that every combination is valid, but Panel.ts is still
   * expected to keep its own dropdowns/visibility consistent.)
   *
   * Ordering notes (unchanged from the water-only version): FLIP's/MPM's
   * `cellSize` (and MPM's `restDensity`) only take effect at `reset()` time,
   * so for those the param push happens *before* `reset()`. SPH's
   * `particleMass` calibration happens *after* `reset()`. All three smoke
   * variants' reset()-gated fields (`cellSize` for FLIP/MPM Smoke,
   * `maxParticles` for MPM/SPH Smoke) follow the same "before reset()" rule
   * as FLIP/MPM water (docs/flip-smoke-sim-api.md, docs/mpm-smoke-sim-api.md,
   * docs/sph-smoke-sim-api.md).
   */
  switchTo(
    kind: SolverKind,
    material: MaterialKind,
    scenarioName: ScenarioName,
    spacing: number,
    sphParams: SPHParams,
    flipParams: FlipParams,
    mpmParams: MpmParams,
    smokeParams: FlipSmokeParams,
    mpmSmokeParams: MpmSmokeParams,
    sphSmokeParams: SphSmokeParams
  ): void {
    const resolvedMaterial: MaterialKind = material;
    if (kind === this.kind && resolvedMaterial === this.material) return;
    const outgoing = this.active;

    if (kind === 'flip' && resolvedMaterial === 'smoke') {
      const next = new FlipSmokeSimulationController(this.module);
      // Push params (including cellSize) BEFORE reset() so the freshly
      // built MAC grid uses the caller's current cellSize, same convention
      // as FLIP/MPM water's cellSize.
      next.updateParams((p) => {
        p.buoyancy = smokeParams.buoyancy;
        p.ambientTemperature = smokeParams.ambientTemperature;
        p.sourceTemperature = smokeParams.sourceTemperature;
        p.densityDissipation = smokeParams.densityDissipation;
        p.temperatureDissipation = smokeParams.temperatureDissipation;
        p.cellSize = smokeParams.cellSize;
      });
      next.reset();
      this.active = next;
    } else if (kind === 'mpm' && resolvedMaterial === 'smoke') {
      const next = new MpmSmokeSimulationController(this.module);
      // Push params (including cellSize/maxParticles) BEFORE reset() so the
      // freshly (re)allocated grid/particle-buffer capacity uses the
      // caller's current values, same convention as every other
      // reset()-only-effective field in this project.
      next.updateParams((p) => {
        p.buoyancy = mpmSmokeParams.buoyancy;
        p.ambientTemperature = mpmSmokeParams.ambientTemperature;
        p.sourceTemperature = mpmSmokeParams.sourceTemperature;
        p.densityDissipation = mpmSmokeParams.densityDissipation;
        p.temperatureDissipation = mpmSmokeParams.temperatureDissipation;
        p.cellSize = mpmSmokeParams.cellSize;
        p.damping = mpmSmokeParams.damping;
        p.maxParticles = mpmSmokeParams.maxParticles;
      });
      next.reset();
      this.active = next;
    } else if (kind === 'sph' && resolvedMaterial === 'smoke') {
      const next = new SphSmokeSimulationController(this.module);
      // Push params (including maxParticles) BEFORE reset() so the freshly
      // (re)allocated particle-buffer capacity uses the caller's current
      // value, same convention as MPM Smoke's maxParticles. `smoothingRadius`
      // doesn't require a reset() (docs/sph-smoke-sim-api.md — the neighbor
      // grid is rebuilt every step() regardless), but pushing it here too is
      // harmless and keeps this branch's shape consistent with the others.
      next.updateParams((p) => {
        p.buoyancy = sphSmokeParams.buoyancy;
        p.ambientTemperature = sphSmokeParams.ambientTemperature;
        p.sourceTemperature = sphSmokeParams.sourceTemperature;
        p.densityDissipation = sphSmokeParams.densityDissipation;
        p.temperatureDissipation = sphSmokeParams.temperatureDissipation;
        p.smoothingRadius = sphSmokeParams.smoothingRadius;
        p.viscosity = sphSmokeParams.viscosity;
        p.maxParticles = sphSmokeParams.maxParticles;
      });
      next.reset();
      this.active = next;
    } else if (kind === 'sph') {
      const next = new SPHSimulationController(this.module);
      next.spacing = spacing;
      next.reset(scenarioName, sphParams.restDensity);
      next.updateParams((p) => {
        p.gravity = sphParams.gravity;
        p.restDensity = sphParams.restDensity;
        p.gasStiffness = sphParams.gasStiffness;
        p.viscosity = sphParams.viscosity;
        p.smoothingRadius = sphParams.smoothingRadius;
        p.boundaryDamping = sphParams.boundaryDamping;
        p.particleMass = sphParams.restDensity * spacing * spacing;
      });
      this.active = next;
    } else if (kind === 'flip') {
      const next = new FlipSimulationController(this.module);
      next.spacing = spacing;
      // Push params (including cellSize) BEFORE reset() so the freshly
      // built MAC grid uses the caller's current cellSize, not the
      // constructor default (see FlipSimulationController.reset doc).
      next.updateParams((p) => {
        p.gravity = flipParams.gravity;
        p.flipRatio = flipParams.flipRatio;
        p.cellSize = flipParams.cellSize;
      });
      next.reset(scenarioName);
      this.active = next;
    } else {
      const next = new MpmSimulationController(this.module);
      next.spacing = spacing;
      // Push params (including cellSize/restDensity) BEFORE reset(), same
      // reasoning as FLIP: both only take effect at reset() time (see
      // docs/mpm-sim-api.md), and restDensity also feeds the automatic
      // particleMass calibration MPM's own C++ reset() performs.
      next.updateParams((p) => {
        p.gravity = mpmParams.gravity;
        p.restDensity = mpmParams.restDensity;
        p.stiffness = mpmParams.stiffness;
        p.viscosity = mpmParams.viscosity;
        p.cellSize = mpmParams.cellSize;
      });
      next.reset(scenarioName);
      this.active = next;
    }

    this.kind = kind;
    this.material = resolvedMaterial;
    outgoing.dispose();
  }
}
