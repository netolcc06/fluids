# Sim API contract (Phase 4b — MPM-flavored Smoke)

This is the JS-facing contract exposed by `wasm/mpm_smoke_bindings.cpp`
(compiled from `core/`) for the Phase 4b MPM-flavored smoke solver. It's
written for the frontend agent, who works only against this document and the
compiled `web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source.

`MpmSmokeSimulation` is a **fifth, independent solver class**, added
alongside the existing Phase 1 `Simulation` (SPH water), Phase 2
`FlipSimulation` (FLIP water), Phase 3 `MpmSimulation` (MPM water), and
Phase 4a `FlipSmokeSimulation` (grid smoke) classes — it does not replace
any of them and shares no C++ inheritance relationship with them. All five
classes are exported from the same compiled module, so a page can construct
any or all of them.

**This is a genuinely different smoke solver from Phase 4a's, not a second
copy of the same algorithm.** `FlipSmokeSimulation` is a pure Eulerian
(grid-only) density/temperature field — no particles at all. `MpmSmokeSimulation`
instead reuses MPM water's actual particle/APIC machinery (the same
quadratic-B-spline P2G/G2P transfer and affine matrix `C` as
`MpmSimulation`), just with the pressure/stress force dropped and
density/temperature tracked per particle instead of a volume ratio `J`. The
result is a **particle-driven, puffy smoke** with a visibly different
character from FLIP smoke's smooth continuous field — that's the entire
point of this being a separate Phase 4b slice.

Load the module the same way as Phases 0-4a:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.MpmSmokeSimulation();   // seeds a default Plume scenario immediately
```

## Coordinate convention

Same as every other solver in this project — **not solver-specific**:
- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `SmokeScenario` below — reused unchanged from Phase 4a).
- Units are meters, seconds, EXCEPT temperature, which is a **dimensionless
  relative scale**, same convention as `FlipSmokeParams`.
- **There is no `gravity` parameter.** Buoyancy — driven by each particle's
  own temperature above ambient — is the sole driving force, same as Phase
  4a. A positive buoyancy coefficient makes hotter-than-ambient particles
  accelerate in `+y` (up).

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — same as every other solver, a rendering
concern only.

## `MpmSmokeSimulation` class

```ts
const sim = new Module.MpmSmokeSimulation(); // seeds a default Plume scenario immediately
sim.step(dt);                                // dt in seconds. See "stable substep" below.
sim.reset(scenario);                         // scenario: a plain object matching SmokeScenario's fields (reused unchanged from Phase 4a)
const n = sim.getParticleCount();            // number — changes every step, see "pointer stability" below
const posPtr = sim.getPositionsPtr();        // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr();       // number (byte offset into the WASM heap)
const densPtr = sim.getDensitiesPtr();       // number (byte offset into the WASM heap)
const tempPtr = sim.getTemperaturesPtr();    // number (byte offset into the WASM heap)
sim.params;                                  // MpmSmokeParams — see the property gotcha below
```

This deliberately mirrors `MpmSimulation`'s particle contract
(`step`/`reset`/`getParticleCount`/`getPositionsPtr`/`getVelocitiesPtr`/
`params`) PLUS two new scalar getters (`getDensitiesPtr`/
`getTemperaturesPtr`) for the per-particle density/temperature this solver
tracks instead of water's `J`.

### Reading the particle buffers (zero-copy typed arrays)

Positions/velocities are contiguous `float32` arrays laid out
`[x0, y0, x1, y1, ...]` (length = `getParticleCount() * 2`), identical
layout to every water solver. Densities/temperatures are contiguous
`float32` arrays of length `getParticleCount()` **(not `* 2` — these are
per-particle scalars, not vec2s)**:

```ts
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let densPtr = sim.getDensitiesPtr();
let tempPtr = sim.getTemperaturesPtr();

let n = sim.getParticleCount();
let positions = new Float32Array(Module.HEAPF32.buffer, posPtr, n * 2);
let velocities = new Float32Array(Module.HEAPF32.buffer, velPtr, n * 2);
let densities = new Float32Array(Module.HEAPF32.buffer, densPtr, n);
let temperatures = new Float32Array(Module.HEAPF32.buffer, tempPtr, n);
```

**Density range:** approximately `[0, 1]` — newly-emitted particles are
seeded with density in `[0.85, 1.0]` (see `SmokeScenario`/emission below),
and density only ever decays multiplicatively afterward (dissipation), so it
never exceeds its seeded value and never goes negative. Treat directly as a
per-particle alpha/opacity value for a sprite/point renderer, same
suggestion as Phase 4a's grid density.

**Temperature range:** not clamped, but bounded in practice the same way as
Phase 4a — seeded near `sourceTemperature` and relaxing toward
`ambientTemperature` every step, so with the defaults (`ambientTemperature =
0`, `sourceTemperature = 1`) it stays roughly within `[0, 1]`.

### Buffer stability contract — READ THIS CAREFULLY, it's the one genuinely new wrinkle in this contract

Every other solver in this project guarantees: buffer pointers are stable
across `step()` calls, and the particle/cell **count** only changes across
`reset()` calls. **MPM smoke breaks the second half of that pattern on
purpose** (continuous emission means the active particle count changes
*every single step*), while preserving the first half:

- **`getParticleCount()` legitimately changes value every `step()` call.**
  This is normal and expected — particles are continuously emitted inside
  the source rectangle and culled once their density fades below a small
  threshold (see "Solver notes" below). Don't treat a changing count as an
  error or a sign something needs re-fetching.
- **`getPositionsPtr()`/`getVelocitiesPtr()`/`getDensitiesPtr()`/
  `getTemperaturesPtr()` do NOT change address across `step()` calls**,
  despite the count changing. Internally, every per-particle buffer is
  preallocated at a **fixed capacity** (`params.maxParticles`) exactly once
  in `reset()`, and only the first `getParticleCount()` slots are
  meaningful at any given moment — the buffers are never reallocated
  between `reset()` calls. This was verified empirically (native Catch2
  test driving the compiled solver: fetch all four pointers once, `step()`
  600 times while confirming the particle count actually changes over that
  run, re-check all four pointers are still bit-identical to their first
  reading — see `tests/core/test_mpm_smoke_solver.cpp`'s
  `[mpm_smoke][pointer_stability]` test).
- **Practical consequence for the frontend:** fetch the four pointers once
  right after `reset()` and build your typed-array views over the buffers'
  **full capacity** (`params.maxParticles`), OR re-slice a fresh
  `Float32Array` view over the current `getParticleCount()` each frame using
  the *same* pointer (cheap — no underlying copy, `new Float32Array(...)` is
  just a view descriptor). Either way, **you do NOT need to call
  `getPositionsPtr()` etc. again just because the count changed** — only
  after an actual `reset()` call, exactly like every other solver. Example
  frame loop:

```ts
// Once, right after reset():
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let densPtr = sim.getDensitiesPtr();
let tempPtr = sim.getTemperaturesPtr();

// Every frame:
sim.step(dt);
const n = sim.getParticleCount(); // cheap, re-read every frame — this DOES change
const positions = new Float32Array(Module.HEAPF32.buffer, posPtr, n * 2); // same posPtr as above
const densities = new Float32Array(Module.HEAPF32.buffer, densPtr, n);
// render `n` particles from `positions`/`densities`/...
```

- **After `reset()`:** buffer capacity can change (`params.maxParticles` is
  only applied at `reset()` time, same contract as `cellSize` — see below).
  **Always call all four pointer getters again immediately after every
  `reset()`** and rebuild your views, same discipline as every other solver.
- No additional entry is needed in `wasm/CMakeLists.txt`'s
  `EXPORTED_RUNTIME_METHODS` — all four buffers are `float32`, so the
  existing `["HEAPF32"]` covers it.

### The `params` property gotcha (same as every other solver)

`sim.params` is an Embind `value_object` property. **Reading it returns a
brand-new plain JS object every time**:

```ts
sim.params.buoyancy = 8; // WRONG — silently mutates a throwaway copy, no effect
```

Correct pattern, read-modify-write:

```ts
const p = sim.params;
p.buoyancy = 8;
sim.params = p; // writes the whole struct back
```

`sim.params = {...}` with a missing field throws `Missing field: "<name>"` —
always supply every field, or start from `sim.params` and mutate.

`MpmSmokeParams` fields are read fresh by `step()` every call — a param
change takes effect on the very next step — **except `cellSize` and
`maxParticles`**, which only take effect on the next `reset()` call (grid
and particle-buffer (re)allocation only ever happen there).

## `MpmSmokeParams` fields

| Field | Default | Units | Meaning |
|---|---|---|---|
| `buoyancy` | `4.0` | (m/s²) per unit temperature above ambient | Applied directly to each particle's vertical velocity every step: `v.y += dt * buoyancy * (particle.temperature - ambientTemperature)`. Same formula shape and default as `FlipSmokeParams.buoyancy`, evaluated per-particle instead of per-grid-face. This is smoke's entire driving force — there is no gravity parameter. |
| `ambientTemperature` | `0.0` | relative scale | Temperature of undisturbed air. Zero point buoyancy measures against, and the value each particle's temperature relaxes toward. |
| `sourceTemperature` | `1.0` | relative scale | Temperature newly-emitted particles are seeded near (jittered to `[0.85, 1.0] * sourceTemperature`). Combined with the default buoyancy, this is what makes emitted particles rise. |
| `densityDissipation` | `0.1` | fraction/second | Each step, `density *= (1 - densityDissipation * dt)`. Same formula/default as `FlipSmokeParams`. Also controls how long a particle survives before falling below the culling threshold (`0.01`) and being removed — see "Culling" below. |
| `temperatureDissipation` | `0.5` | fraction/second | Each step, `T += (ambientTemperature - T) * min(1, temperatureDissipation * dt)`. Same formula/default as `FlipSmokeParams` — deliberately faster than `densityDissipation`, same reasoning as the grid version (a particle's buoyant "hotness" fades before its visible density does). |
| `cellSize` | `0.02` | meters | MPM node grid spacing for the P2G/G2P transfer. Grid resolution is derived from `SmokeScenario.domainWidth/domainHeight` and this value **only at `reset()` time**. |
| `damping` | `0.0` | fraction/second | Optional linear velocity drag: `v *= max(0, 1 - damping * dt)`, applied right after buoyancy each step. `0` (default, used by the Plume preset and all native tests) disables it entirely — empirically not needed for a stable/coherent-looking result at the defaults (see "Solver notes" below), but left as a knob since a different emission/buoyancy tuning might want it. |
| `maxParticles` | `4000` | count | Fixed capacity of the particle buffer, (re)allocated only at `reset()` (same reset()-only contract as `cellSize`). Backs the pointer-stability contract above. Comfortably above the steady-state particle count the default settings reach in practice (empirically ~1800, see "Solver notes" below) — a safety backstop, not a normally-hit ceiling. |

There is no `restDensity`/`stiffness`/`viscosity` the way `MpmParams`
(water) has them — this solver drops the EOS/pressure-stress force
entirely (see "Solver notes"), so there's no equation of state to
parameterize. There is no `gravity` (buoyancy is the sole driving force, per
`FlipSmokeParams`' precedent). Expect a **different slider set** per solver
if building a shared parameter panel — established pattern since Phase 2.

## `SmokeScenario` — reused unchanged, no new struct needed

**This phase needed no new scenario struct or preset.** `SmokeScenario` (the
same struct `FlipSmokeSimulation` uses, see `docs/flip-smoke-sim-api.md`)
already fits: a domain plus a rectangular source region that continuously
emits — exactly the shape MPM smoke's emission needs too.
`Module.makePlumeScenario()` works unmodified for `MpmSmokeSimulation.reset()`:

```ts
const s = Module.makePlumeScenario();
sim.reset(s);
sim.getPositionsPtr();     // MUST re-fetch after reset() (and the other 3 getters)
```

Note that `SmokeScenario` is registered as an Embind `value_object` by
`wasm/flip_smoke_bindings.cpp` and is **not** re-registered by
`wasm/mpm_smoke_bindings.cpp` (re-registering the same C++ type under a
second `EMSCRIPTEN_BINDINGS` block is an error) — same
share-the-registration pattern `wasm/mpm_bindings.cpp` already established
for water's `Scenario`. `Module.makePlumeScenario()` is likewise only
registered once, by the FLIP smoke bindings, and works for both smoke
solvers.

`SmokeScenario.emissionRate` means something slightly different for this
solver than for the grid version — see "Emission" under "Solver notes"
below for the exact translation and why it isn't literally the same
formula.

## Boundary handling

Same two-layer shape as MPM water, reused directly (see `docs/mpm-sim-api.md`):

1. **Grid-level (primary):** after P2G/grid-update (momentum → velocity, NO
   gravity — see below), any node within a 3-cell-wide margin of a domain
   wall has its outward-pointing velocity component zeroed.
2. **Particle-level (safety net):** after advection, any particle outside
   `[0,domainWidth]x[0,domainHeight]` is clamped back onto the boundary,
   with the outward velocity component zeroed.

The only difference from MPM water: grid-update has **no gravity term at
all** (buoyancy, applied per-particle after G2P, is the sole driving force —
see the per-step pipeline below), matching `FlipSmokeParams`' contract that
smoke never has a gravity field anywhere in this project.

## The stable substep — measured directly for this solver, not assumed from either prior finding

**Recommended caller `dt`: `1/60`**, same recommendation as Phase 4a (grid
smoke) and matching a plain 60fps game loop. This was **checked directly**
for this solver (`tests/core/test_mpm_smoke_solver.cpp`'s
`[mpm_smoke][stability][substep]` test), not assumed from either MPM
water's or FLIP smoke's findings — those two prior results disagree with
each other (water needs `1/480`, grid smoke is fine up to `1/30`), so
neither one transfers by default to a third, structurally different
solver.

**Finding: plain fixed-substep stepping (no internal CFL-adaptive
subdivision) is stable across every `dt` tested, `1/30` down to `1/240`**,
over a 3-simulated-second `makePlumeScenario()` run each: no NaN/Inf, and
particle speeds stayed a small fraction of a generously loose 50 m/s sanity
bound at every `dt`. Reasoning for why this solver, specifically, doesn't
need water's adaptive substepping:

- **MPM water's adaptive substepping exists because of a stiff feedback
  loop that this solver structurally cannot have.** Water's runaway mode
  (see `docs/mpm-sim-api.md`) was: a large velocity gradient across a sudden
  impact swings a particle's `J` by a large fraction in one step, which the
  EOS's `(1/J)^gamma` pressure response then amplifies into a growing
  velocity spike the *next* step, compounding. This solver has **no EOS, no
  pressure force, no `J` at all** — the P2G affine term is exactly `mass *
  C` with nothing multiplying a stiffness constant, so there is no
  mechanism for a velocity perturbation to amplify itself step over step.
- The remaining, weaker concern was whether a fast-moving emitted-particle
  stream could simply tunnel more than a grid cell per step at a large
  `dt` — a **numerical-accuracy** question, not a **stability/blowup**
  question. This was checked empirically rather than assumed away: even at
  the loosest tested `dt = 1/30`, particle speeds never approached a
  concerning fraction of a cell width per step (buoyancy's acceleration is
  modest — `4.0 * 1.0 = 4 m/s²` at the hottest, so velocity only builds up
  gradually over many steps, unlike water's discontinuous impact velocities).
- This is closer to FLIP smoke's finding (`FlipSmokeSolver`'s
  semi-Lagrangian advection is unconditionally stable "by construction")
  than to MPM water's, but for a **different underlying reason** — FLIP
  smoke's stability comes from backward-tracing/clamped-bilinear-sampling
  being unable to overshoot; this solver's stability comes from simply
  having no stiff force term to begin with. Both land on "no adaptive
  substepping needed," but don't assume the *reason* transfers to some
  future fourth smoke variant without checking.

Given this, `MpmSmokeSolver::step()` does **not** internally subdivide `dt`
the way `mpm::MpmSolver::step()` does — it runs exactly one clear/P2G/
grid-update/G2P/buoyancy/advect/dissipation/cull pass per call, after the
same defensive `dt` clamp to `1/30` every solver in this project applies
(guards a runaway caller `dt`, e.g. after a stalled tab resumes — orthogonal
to the stability finding above, not a substitute for it).

## Solver notes (background, not part of the JS contract but useful context)

Reuses MPM water's actual particle/APIC machinery
(`fluids::mpm::MpmGrid`, the same quadratic-B-spline P2G/G2P stencil and
affine matrix `C`) rather than a second copy of Phase 4a's pure-Eulerian
Stable Fluids grid solver — see `core/include/fluids/solvers/mpm_smoke/MpmSmokeSolver.h`
for the full class-level rationale. Per-particle state: position, velocity,
the APIC affine matrix `C` (2x2, same as water), `density` (~[0,1], replaces
water's `J`) and `temperature` (same semantics as `FlipSmokeSolver`'s
temperature field).

**No pressure/stress force at all.** Smoke here is a compressible buoyant
gas, not an incompressible fluid resisting compression, so P2G's affine
term is just `mass * C` (the APIC term alone) — dropping the
`-dt*4/dx^2*volume*stress` term `mpm::MpmSolver::particleToGrid` adds for
water's EOS. Per-particle mass is a **fixed internal constant** (`kParticleMass
= 1.0`, see `MpmSmokeSolver.cpp`), not calibrated from a rest density the
way water's is — with no stress force reading it, mass only ever appears as
a shared positive weight in a momentum-weighted average (P2G scatters
`mass*v`, grid-update divides accumulated momentum by accumulated mass), so
every particle sharing the same value makes it cancel out of that division
exactly. Its absolute magnitude is therefore arbitrary; there was nothing
meaningful to expose as a parameter.

**Buoyancy is applied per-particle after G2P, not via grid-resident
density/temperature fields.** This was a deliberate simplification: rather
than also transferring density/temperature onto the grid (which
`FlipSmokeSolver` needs, since its velocity lives *only* on the grid), this
solver applies `v.y += dt * buoyancy * (particle.temperature -
ambientTemperature)` directly to each particle's own velocity, using that
particle's own temperature, right after G2P. This avoids needing any
grid-side scalar fields at all — the MPM grid here only ever carries
mass/momentum, exactly like MPM water's, and adding grid-resident scalars
(with their own transfer/interpolation logic) would have been extra
machinery for no benefit given every particle already carries its own
temperature value directly.

Per-step pipeline (see `MpmSmokeSolver.cpp` for the full derivation):

1. **Emit new particles** into the active buffer, within `SmokeScenario`'s
   source rectangle (see "Emission" below).
2. **Clear grid, P2G:** scatter `mass*v + (mass*C)@dpos` (no stress term)
   onto the 3x3 node neighborhood, same quadratic-B-spline stencil as MPM
   water.
3. **Grid update:** momentum → velocity, **no gravity**, margin-based
   boundary velocity clamp (same style as MPM water).
4. **G2P:** gather velocity + reconstruct `C` from the same 9-node stencil.
5. **Buoyancy + optional damping:** per-particle, using that particle's own
   temperature (see above); `damping` (if nonzero) applies a linear drag
   right after.
6. **Advect:** `position += dt * v`.
7. **Dissipation:** `density *= (1 - densityDissipation*dt)`, `temperature
   += (ambientTemperature - temperature) * min(1, temperatureDissipation*dt)`
   — same formulas as `FlipSmokeParams`.
8. **Cull faded particles**, then a particle-level domain boundary clamp
   (belt-and-suspenders, same pattern as every other solver).

### Emission — the exact translation from `emissionRate` to a particle spawn rate, and why

`SmokeScenario.emissionRate` is defined (see `docs/flip-smoke-sim-api.md`)
as "density units added per second **within the source region**" for the
grid solver — a resolution-dependent quantity there, since it's added
per-cell per second, so the total density content added per second scales
with how many grid cells the source rectangle covers. There is no
equivalent notion of "cells" for a particle-based source, so this solver
translates `emissionRate` directly into a **particle spawn rate** instead:

```
particlesPerSecond = emissionRate * 20.0   // kParticlesPerSecondPerEmissionUnit, MpmSmokeSolver.cpp
```

Implementation detail: each step accumulates `particlesPerSecond * dt` into
a fractional counter and spawns one particle every time the counter crosses
`1.0` (a standard fractional-accumulator pattern), so the actual spawn rate
is exact regardless of the caller's `dt` — not "at most one particle per
step." Each spawned particle is placed at a uniformly-random position
within the source rectangle, given a small `±0.05 m/s` random velocity
jitter on both axes (so a burst of particles emitted the same tick doesn't
move in perfect lockstep before buoyancy/APIC take over), density jittered
to `[0.85, 1.0]`, and temperature jittered to `[0.85, 1.0] * sourceTemperature`.

`20.0` (particles/sec per unit of `emissionRate`) was chosen empirically so
that `makePlumeScenario()`'s default `emissionRate = 2.0` spawns **40
particles/second**, verified directly via a native diagnostic run of the
default Plume preset:

| Simulated time | Active particle count | Density-weighted centroid height |
|---|---|---|
| 1s | 40 | 0.64 m |
| 5s | 200 | 1.66 m |
| 10s | 400 | 1.79 m |
| 20s | 800 | 1.84 m |
| 30s | 1200 | 1.86 m |
| 45s | ~1800 | 1.87 m |
| 60s | ~1810 (steady) | 1.87 m |

The count grows linearly (exactly 40/s) until natural culling (particles
crossing the density threshold, see below) starts balancing new emissions,
reaching a **steady state around ~1800 particles by ~45 simulated
seconds** — comfortably under the default `maxParticles = 4000`, so the
capacity cap is a backstop, not something normal operation runs into (same
spirit as `mpm::MpmSolver`'s `kMaxSubsteps` cap). The centroid height rises
from the source (`sourceY + sourceHeight = 0.15`) up to ~1.87m and plateaus
there — `makePlumeScenario()`'s domain is 2.0m tall with a 3-cell (`0.06m`)
boundary margin, so the plume is rising until it's constrained by the top
wall, then pooling, a physically sensible result for a closed-top domain.

This translation deliberately does **not** scale with the source
rectangle's area the way the grid version's total flux implicitly does — a
custom `SmokeScenario` with a much larger or smaller source rectangle
changes the emitted particles' *spatial spread* but not their *count per
second*. This is a known, documented simplification (the task explicitly
left the exact translation as an implementation judgment call) rather than
an oversight; matching the grid version's area-scaling behavior exactly
would have required also depending on `cellSize`-like resolution concept
this particle solver has no natural analog for.

### Culling — why swap-compact + skip-when-full, not recycle-oldest

Each step, after dissipation, every active particle whose `density` has
decayed below `0.01` (`kDensityCullThreshold`) is removed by swapping it
with the last active particle and shrinking the active count by one
(order among active particles is not preserved or meaningful — nothing
reads particles in a specific order). Without this, the active count would
only ever grow until it pinned at `maxParticles`, silently stopping all
future emission forever — this was an explicit requirement, not an
optimization.

**When at capacity, new emission is skipped for the remainder of that tick
rather than recycling the oldest/faintest particle in place.** Two
alternatives were considered:

- **Recycle the oldest/faintest particle:** requires scanning (or
  maintaining a priority structure) every time capacity is hit just to find
  a candidate to evict, and visually **teleports** an existing (if faint)
  particle from wherever it had drifted to back to the source rectangle —
  a discontinuous jump that would be a visible artifact in a naive
  point/sprite renderer.
- **Skip new emission this tick (chosen):** simply don't spawn until a slot
  frees up naturally via the normal culling pass above. Cheaper (no scan),
  and never produces a visible teleport. The only cost is a very slightly
  bursty resumption of emission right after a long capacity-constrained
  period, which is bounded by capping how much "emission credit" can bank
  up while capped (`kMaxBankedEmission = 50` particles' worth, see
  `MpmSmokeSolver.cpp`) — without this cap, an extreme edge case (e.g. a
  user cranking `emissionRate` far above default while leaving
  `densityDissipation` near zero) could theoretically bank an unbounded
  amount of credit and then dump it all at once the moment a slot frees.

Given the steady-state measurement above (~1800 particles at the defaults,
well under the default 4000 capacity), this skip-when-full path is not
expected to engage at all during normal operation with default parameters —
it exists as a safety backstop for pathological parameter combinations
(e.g. `maxParticles` set very low, or `emissionRate` cranked very high with
`densityDissipation` near zero), the same role `mpm::MpmSolver`'s
`kMaxSubsteps` cap plays for water.

### Deviations from the task's literal description (and why)

- **No CFL-adaptive substepping**, unlike `mpm::MpmSolver`. The task
  explicitly asked to check this empirically rather than assume either
  prior finding transfers — see "The stable substep" above for the
  reasoning (no stiff EOS feedback loop exists in this solver to amplify a
  velocity perturbation) and the measurement (stable at every `dt` from
  `1/30` to `1/240`).
- **Particle mass is a fixed internal constant (`1.0`), not calibrated from
  a rest density.** Unlike `mpm::MpmParams::restDensity` (which calibrates
  water's `particleMass` and feeds its EOS), this solver has no EOS to
  calibrate for, and mass cancels out of the P2G/grid-update momentum
  average regardless of its value (see "Solver notes" above) — so there was
  nothing for a `restDensity`-equivalent field to usefully control. Not
  exposed as a parameter for that reason.
- **Buoyancy applied per-particle after G2P, with no grid-resident
  density/temperature fields**, per the task's explicitly suggested
  simpler alternative — see "Solver notes" above for why this was chosen
  over also transferring density/temperature onto the grid.
- **`damping` defaults to 0 and isn't required for a stable/coherent
  look**, at least at the default buoyancy/emission/dissipation settings
  used by `makePlumeScenario()` and all native tests — included as a knob
  per the task's "your call" framing, not because leaving it at 0 looked
  broken. A caller pushing `buoyancy` much higher or `emissionRate` much
  higher than the defaults, producing a more chaotic-looking particle gas,
  is the scenario where raising `damping` above 0 would be worth trying
  first.
- **Emission's `emissionRate`-to-particle-rate translation is a straight
  linear scale on `emissionRate` alone (not also scaled by source
  rectangle area)** — see "Emission" above for the full reasoning; this is
  a documented simplification versus the grid version's implicit
  area-dependent total flux, not an attempt to reproduce it exactly.
- **RNG is a fixed-seed `std::mt19937`, reseeded at every `reset()`,**
  rather than wall-clock/hardware seeded. No other solver in this project
  uses randomness yet, so there was no existing convention to match, and
  determinism was chosen so a given scenario/params/step sequence produces
  a reproducible particle stream (useful for tests, and costs nothing
  visually — it still looks like a random jittered stream, just the same
  one every run of the same inputs).

None of these change the class-level design decisions the orchestrator
specified (particle-based, APIC-reusing, no pressure/stress, fixed-capacity
buffer with pointer stability, per-density-threshold culling) — they're the
specific implementation judgment calls the task explicitly left open,
recorded here for whoever picks up the next phase (SPH Smoke, per the
roadmap) and wants to know what Phase 4b actually did versus what was
sketched.
