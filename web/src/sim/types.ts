// Plain TS types mirroring the JS-facing contract documented in
// docs/sim-api.md (SPH) and docs/flip-sim-api.md (FLIP). These are
// hand-written (Embind doesn't emit .d.ts) and must be kept in sync with
// those documents if the WASM contract changes.
//
// Both `Simulation` (SPH) and `FlipSimulation` (FLIP) are deliberately
// exposed with the exact same method shape (`step`/`reset`/
// `getParticleCount`/`getPositionsPtr`/`getVelocitiesPtr`/`params`) per
// both docs, specifically so a frontend can treat either uniformly through
// the single generic `SimulationHandle<TParams>` interface below.

/** Live-tunable SPH solver parameters (`Simulation.params`). */
export interface SPHParams {
  gravity: number;
  restDensity: number;
  gasStiffness: number;
  viscosity: number;
  smoothingRadius: number;
  particleMass: number;
  boundaryDamping: number;
}

/** Live-tunable FLIP solver parameters (`FlipSimulation.params`). A
 * deliberately smaller, different field set than SPHParams — no
 * particleMass/restDensity/viscosity/gasStiffness/boundaryDamping, since
 * FLIP's incompressibility comes from grid pressure projection, not a
 * per-particle equation of state (see docs/flip-sim-api.md). */
export interface FlipParams {
  gravity: number;
  /** PIC/FLIP blend, clamped to [0,1] internally every step. Default 0.95. */
  flipRatio: number;
  /** MAC grid cell size, meters. Only takes effect at the next reset() call
   * — changing it live has no visible effect until reset() runs again. */
  cellSize: number;
}

/** Live-tunable MPM solver parameters (`MpmSimulation.params`). No
 * `particleMass`/`smoothingRadius`/`boundaryDamping` — MPM computes
 * `particleMass` automatically inside its own C++ `reset()` from
 * `restDensity`/`spacing` (no JS-side calibration needed, unlike SPH), has
 * no per-particle neighbor kernel, and uses a margin-based grid-velocity
 * boundary clamp instead of a damping coefficient (see docs/mpm-sim-api.md).
 * `restDensity` and `cellSize` only take effect at the next `reset()` call,
 * same as FLIP's `cellSize`. */
export interface MpmParams {
  gravity: number;
  restDensity: number;
  stiffness: number;
  viscosity: number;
  /** Node grid spacing, meters. Only takes effect at the next reset() call. */
  cellSize: number;
}

/** Initial-state descriptor passed to `Simulation.reset()` /
 * `FlipSimulation.reset()` / `MpmSimulation.reset()`. Shared unchanged
 * across all three solvers (see docs/flip-sim-api.md, docs/mpm-sim-api.md).
 * Every field is required — Embind's struct marshalling throws
 * `TypeError: Missing field: "<name>"` if any field is omitted, and this
 * applies to `sim.params = {...}` too. */
export interface Scenario {
  domainWidth: number;
  domainHeight: number;
  spacing: number;
  regionX: number;
  regionY: number;
  regionWidth: number;
  regionHeight: number;
  velocityX: number;
  velocityY: number;
  hasRegion2: boolean;
  region2X: number;
  region2Y: number;
  region2Width: number;
  region2Height: number;
  region2VelocityX: number;
  region2VelocityY: number;
}

/** Common shape of both `Module.Simulation` and `Module.FlipSimulation`
 * instances, parameterized by that solver's params struct type. */
export interface SimulationHandle<TParams> {
  step(dt: number): void;
  reset(scenario: Scenario): void;
  getParticleCount(): number;
  getPositionsPtr(): number;
  getVelocitiesPtr(): number;
  /** Embind value_object property: reading returns a fresh plain-object
   * COPY every time. Writing (`sim.params = p`) writes the whole struct.
   * Mutating `sim.params.x` directly is a silent no-op — always
   * read-modify-write (see docs/sim-api.md and docs/flip-sim-api.md). */
  params: TParams;
  /** Embind `ClassHandle.prototype.delete` — frees the underlying WASM-heap
   * instance. Embind class instances constructed via `new Module.X()` are
   * NOT garbage-collected by JS (verified against this build's generated
   * fluids.js: `attachFinalizer` only registers a `FinalizationRegistry`
   * callback for smart-pointer-held instances, and neither `Simulation` nor
   * `FlipSimulation` is bound with a smart pointer — plain `constructor<>()`
   * — so no automatic finalization ever happens for either). Every
   * constructed instance must be explicitly `.delete()`d or its C++-side
   * memory leaks for the rest of the page's life. */
  delete(): void;
}

/** Live-tunable FLIP-Smoke solver parameters (`FlipSmokeSimulation.params`).
 * A completely different field set from every water solver's params — no
 * gravity/particleMass/restDensity/gasStiffness/flipRatio at all. Buoyancy
 * (driven by temperature above ambient) is smoke's entire driving force;
 * see docs/flip-smoke-sim-api.md's field table for exact semantics. Like
 * `FlipParams.cellSize`/`MpmParams.cellSize`, `cellSize` here only takes
 * effect at the next `reset()` call. */
export interface FlipSmokeParams {
  buoyancy: number;
  ambientTemperature: number;
  sourceTemperature: number;
  densityDissipation: number;
  temperatureDissipation: number;
  cellSize: number;
}

/** Initial-state descriptor passed to `FlipSmokeSimulation.reset()`. A
 * deliberately SEPARATE struct from water's `Scenario` (see
 * docs/flip-smoke-sim-api.md) — smoke has no particles, so there is no
 * spacing/region/initial-velocity concept. Instead it describes a domain
 * plus a source rectangle that continuously emits density/temperature every
 * step. Every field is required, same Embind "Missing field" contract as
 * `Scenario`. */
export interface SmokeScenario {
  domainWidth: number;
  domainHeight: number;
  sourceX: number;
  sourceY: number;
  sourceWidth: number;
  sourceHeight: number;
  emissionRate: number;
}

/** Shape of a `Module.FlipSmokeSimulation` instance. Deliberately NOT
 * `SimulationHandle<FlipSmokeParams>` — there is no
 * `getParticleCount()`/`getPositionsPtr()`/`getVelocitiesPtr()` at all
 * (smoke has no particles); instead the renderable state is a grid-shaped
 * density/temperature field (see docs/flip-smoke-sim-api.md). */
export interface FlipSmokeSimulationHandle {
  step(dt: number): void;
  reset(scenario: SmokeScenario): void;
  getGridWidth(): number;
  getGridHeight(): number;
  /** Byte offset into the WASM heap. Stable across step() calls, but MUST
   * be re-fetched after every reset() (grid dimensions can change) — see
   * docs/flip-smoke-sim-api.md's buffer-stability contract. */
  getDensityFieldPtr(): number;
  getTemperatureFieldPtr(): number;
  /** Same read-modify-write gotcha as every other solver's `params`. */
  params: FlipSmokeParams;
  delete(): void;
}

/** Live-tunable MPM-Smoke solver parameters (`MpmSmokeSimulation.params`,
 * Phase 4b — see docs/mpm-smoke-sim-api.md). A different field set again
 * from `FlipSmokeParams`: shares the same buoyancy/ambientTemperature/
 * sourceTemperature/densityDissipation/temperatureDissipation/cellSize
 * semantics (same formulas/defaults), but adds two fields with no grid-smoke
 * equivalent — `damping` (optional per-particle linear drag, default 0,
 * disabled) and `maxParticles` (fixed particle-buffer capacity, backs the
 * pointer-stability contract — see MpmSmokeSimulationController). No
 * `restDensity`/`stiffness`/`viscosity`/`gravity` — this solver has no EOS
 * and buoyancy is the sole driving force, same as `FlipSmokeParams`.
 * `cellSize` AND `maxParticles` only take effect at the next `reset()` call
 * (buffer/grid (re)allocation only ever happens there); every other field is
 * read fresh by `step()` every call. */
export interface MpmSmokeParams {
  buoyancy: number;
  ambientTemperature: number;
  sourceTemperature: number;
  densityDissipation: number;
  temperatureDissipation: number;
  cellSize: number;
  damping: number;
  maxParticles: number;
}

/** Shape of a `Module.MpmSmokeSimulation` instance (Phase 4b — see
 * docs/mpm-smoke-sim-api.md). Mirrors `SimulationHandle`'s particle contract
 * (`step`/`reset`/`getParticleCount`/`getPositionsPtr`/`getVelocitiesPtr`/
 * `params`) — this is NOT `SimulationHandle<MpmSmokeParams>` only because
 * `reset()` takes a `SmokeScenario`, not water's `Scenario` — PLUS two new
 * scalar getters (`getDensitiesPtr`/`getTemperaturesPtr`) for the
 * per-particle density/temperature this solver tracks instead of water's
 * `J`. Densities/temperatures are contiguous float32 arrays of length
 * `getParticleCount()` (NOT `*2` — scalars, not vec2s).
 *
 * Buffer stability is genuinely different from every other solver here:
 * `getParticleCount()` legitimately changes EVERY `step()` call (continuous
 * emission/culling), while all four pointer getters stay stable across
 * `step()` calls exactly like every other solver — only re-fetch them after
 * `reset()`. See MpmSmokeSimulationController for how this is handled. */
export interface MpmSmokeSimulationHandle {
  step(dt: number): void;
  reset(scenario: SmokeScenario): void;
  getParticleCount(): number;
  getPositionsPtr(): number;
  getVelocitiesPtr(): number;
  getDensitiesPtr(): number;
  getTemperaturesPtr(): number;
  /** Same read-modify-write gotcha as every other solver's `params`. */
  params: MpmSmokeParams;
  delete(): void;
}

/** Live-tunable SPH-Smoke solver parameters (`SphSmokeSimulation.params`,
 * Phase 4c — see docs/sph-smoke-sim-api.md). Shares the same
 * buoyancy/ambientTemperature/sourceTemperature/densityDissipation/
 * temperatureDissipation/maxParticles field set and semantics as
 * `MpmSmokeParams`, but has a genuinely different physics field set instead
 * of `cellSize`/`damping`: `smoothingRadius` (this solver's kernel support
 * radius — the SPH analog of a grid `cellSize`, but unlike every other
 * `cellSize`/`cellSize`-equivalent in this project, changing it does NOT
 * require a `reset()` since the neighbor-search grid is rebuilt from
 * scratch every `step()` regardless) and `viscosity` (kernel-weighted
 * Laplacian viscosity — this solver's sole coherence mechanism, since it has
 * no pressure force/EOS at all, unlike water SPH's `viscosity`+`gasStiffness`
 * pair). No `gravity`/`gasStiffness`/`restDensity`/`particleMass`/
 * `cellSize`/`damping` — buoyancy is the sole driving force (same as every
 * other Smoke variant) and there is no EOS or grid at all (see the doc's
 * "Pressure/EOS decision" section). Only `maxParticles` gates behind the
 * next `reset()` call (buffer (re)allocation only ever happens there) —
 * every other field, INCLUDING `smoothingRadius`, is read fresh by `step()`
 * every call. */
export interface SphSmokeParams {
  buoyancy: number;
  ambientTemperature: number;
  sourceTemperature: number;
  densityDissipation: number;
  temperatureDissipation: number;
  smoothingRadius: number;
  viscosity: number;
  maxParticles: number;
}

/** Shape of a `Module.SphSmokeSimulation` instance (Phase 4c — see
 * docs/sph-smoke-sim-api.md). Exactly the same method/field shape as
 * `MpmSmokeSimulationHandle` (`step`/`reset`/`getParticleCount`/
 * `getPositionsPtr`/`getVelocitiesPtr`/`getDensitiesPtr`/
 * `getTemperaturesPtr`/`params`) — a deliberate consistency choice so all
 * three smoke solvers are interchangeable from the frontend's point of
 * view, differing only in `params`' field set. Same buffer-stability
 * contract as `MpmSmokeSimulationHandle`: `getParticleCount()` legitimately
 * changes every `step()` call (continuous emission/culling), while all four
 * pointer getters stay stable across `step()` calls and only need
 * re-fetching after `reset()`. See SphSmokeSimulationController for how
 * this is handled. */
export interface SphSmokeSimulationHandle {
  step(dt: number): void;
  reset(scenario: SmokeScenario): void;
  getParticleCount(): number;
  getPositionsPtr(): number;
  getVelocitiesPtr(): number;
  getDensitiesPtr(): number;
  getTemperaturesPtr(): number;
  /** Same read-modify-write gotcha as every other solver's `params`. */
  params: SphSmokeParams;
  delete(): void;
}

/** Live-tunable 3D SPH water solver parameters (`SphSimulation3D.params`,
 * see docs/sph3d-sim-api.md). Same field NAMES as 2D `SPHParams`, but these
 * are NOT the 2D defaults relabeled — every default was independently
 * empirically derived for the 3D solver (3D has more neighbors per unit
 * smoothing radius than 2D, which changes the density estimate's scale).
 * Notably `restDensity` here is genuinely volumetric (kg/m³), unlike 2D's
 * "areal" kg/m². `particleMass` must stay calibrated against `restDensity`
 * and the active `Scenario3D.spacing` via a *measured correction factor*
 * (~1.468x on top of the naive `restDensity * spacing^3` formula) — not a
 * simple formula the frontend can safely re-derive, which is why this
 * project's 3D UI (see Sph3DSimulationController / Panel.ts's "3D SPH
 * Params" folder) deliberately does not expose `particleMass` or
 * `Scenario3D.spacing` as sliders, unlike 2D's spacing-linked
 * `particleMass` recalibration. */
export interface SphParams3D {
  gravity: number;
  restDensity: number;
  gasStiffness: number;
  viscosity: number;
  smoothingRadius: number;
  particleMass: number;
  boundaryDamping: number;
}

/** Initial-state descriptor passed to `SphSimulation3D.reset()` (see
 * docs/sph3d-sim-api.md). Structurally the 3D sibling of `Scenario` above —
 * same two-box-region shape, just with a `Z`/`Depth` axis added throughout.
 * Every field is required, same Embind "Missing field" contract as every
 * other value_object in this project. */
export interface Scenario3D {
  domainWidth: number;
  domainHeight: number;
  domainDepth: number;
  spacing: number;
  regionX: number;
  regionY: number;
  regionZ: number;
  regionWidth: number;
  regionHeight: number;
  regionDepth: number;
  velocityX: number;
  velocityY: number;
  velocityZ: number;
  hasRegion2: boolean;
  region2X: number;
  region2Y: number;
  region2Z: number;
  region2Width: number;
  region2Height: number;
  region2Depth: number;
  region2VelocityX: number;
  region2VelocityY: number;
  region2VelocityZ: number;
}

/** Shape of a `Module.SphSimulation3D` instance (see docs/sph3d-sim-api.md).
 * Deliberately NOT `SimulationHandle<SphParams3D>` (that generic is 2D-only
 * — its `reset()` takes the 2D `Scenario`), but otherwise mirrors it
 * exactly: same read-modify-write `params` gotcha, same
 * `delete()`-required Embind lifecycle. The one buffer-layout difference
 * that matters most: positions/velocities are **3 floats per particle**
 * (`[x,y,z, ...]`), length `getParticleCount() * 3`, not `* 2` — see
 * Sph3DSimulationController. */
export interface SphSimulation3DHandle {
  step(dt: number): void;
  reset(scenario: Scenario3D): void;
  getParticleCount(): number;
  getPositionsPtr(): number;
  getVelocitiesPtr(): number;
  /** Same read-modify-write gotcha as every other solver's `params`:
   * reading returns a fresh copy every time. */
  params: SphParams3D;
  delete(): void;
}

export interface FluidsModule {
  HEAPF32: Float32Array;
  Simulation: new () => SimulationHandle<SPHParams>;
  FlipSimulation: new () => SimulationHandle<FlipParams>;
  MpmSimulation: new () => SimulationHandle<MpmParams>;
  FlipSmokeSimulation: new () => FlipSmokeSimulationHandle;
  MpmSmokeSimulation: new () => MpmSmokeSimulationHandle;
  SphSmokeSimulation: new () => SphSmokeSimulationHandle;
  /** The first (and so far only) 3D solver — see docs/sph3d-sim-api.md.
   * Bound in the same compiled `fluids.wasm` as every 2D solver above (one
   * WASM module, all classes registered on the same `Module`), so it lives
   * on this same flat interface rather than a separate module type, even
   * though the frontend drives it through a wholly separate controller/
   * renderer/camera path (see Sph3DSimulationController, OrbitCamera,
   * ParticleRenderer3D) — 3D has no orthographic domain-to-clip-space
   * camera, no `domainWidth`/`domainHeight`-only shape, and a genuinely
   * different (3-float-per-particle) buffer layout, so it deliberately
   * does NOT go through `SolverManager`/`BaseSimulationController`. */
  SphSimulation3D: new () => SphSimulation3DHandle;
  makeDamBreakScenario(): Scenario;
  makeBlockDropScenario(): Scenario;
  makeTwoColumnCollisionScenario(): Scenario;
  /** The only built-in `SmokeScenario` preset so far (see
   * docs/flip-smoke-sim-api.md) — seeds a chimney-like source at the bottom
   * of a tall 1.2m x 2.0m domain. Works unmodified for
   * `FlipSmokeSimulation.reset()`, `MpmSmokeSimulation.reset()`, and
   * `SphSmokeSimulation.reset()` (see docs/mpm-smoke-sim-api.md and
   * docs/sph-smoke-sim-api.md — `SmokeScenario` is reused unchanged across
   * every smoke variant, no new struct per phase). */
  makePlumeScenario(): SmokeScenario;
  /** Built-in 3D presets (see docs/sph3d-sim-api.md) — each returns a plain
   * `Scenario3D`-shaped object ready for `SphSimulation3D.reset()`. All
   * three use the default `SphParams3D` (`particleMass = 0.317`, calibrated
   * for `spacing = 0.06` / `restDensity = 1000`). */
  makeDamBreak3DScenario(): Scenario3D;
  makeBlockDrop3DScenario(): Scenario3D;
  makeTwoColumnCollision3DScenario(): Scenario3D;
}
