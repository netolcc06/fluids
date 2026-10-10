# Sim API contract (Phase 4a — Grid Smoke)

This is the JS-facing contract exposed by `wasm/flip_smoke_bindings.cpp`
(compiled from `core/`) for the Phase 4a Stable Fluids (Jos Stam) 2D smoke
solver. It's written for the frontend agent, who works only against this
document and the compiled `web/src/wasm-gen/fluids.{js,wasm}` — not the C++
source.

`FlipSmokeSimulation` is a **fourth, independent solver class**, added
alongside the existing Phase 1 `Simulation` (SPH water), Phase 2
`FlipSimulation` (FLIP water), and Phase 3 `MpmSimulation` (MPM water)
classes — it does not replace any of them and shares no C++ inheritance
relationship with them. All four classes are exported from the same
compiled module, so a page can construct any or all of them.

**This is a genuinely different kind of solver, not just a fourth flavor of
the same shape.** Smoke has no particles at all — no free surface, no
marker-and-cell classification from particle occupancy. It's a pure
Eulerian (grid-only) density/temperature field, advected on a MAC grid and
driven by buoyancy instead of gravity. Concretely, that means **the JS
contract shape is different**: there is no `getParticleCount()` /
`getPositionsPtr()` / `getVelocitiesPtr()` here at all. Instead you get grid
dimensions and a pointer to a renderable density field — see below.

Load the module the same way as Phases 0-3:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.FlipSmokeSimulation();   // seeds a default Plume scenario immediately
```

## Coordinate convention

Same as every other solver in this project — **not solver-specific**:
- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `SmokeScenario` below — a **different struct** from the water `Scenario`
  used by SPH/FLIP/MPM, not a reuse of it).
- Units are meters, seconds, EXCEPT temperature, which is a **dimensionless
  relative scale** (see `FlipSmokeParams` below) — there is no physical unit
  attached to it.
- **There is no `gravity` parameter for smoke at all.** Buoyancy — driven by
  temperature above ambient — is the sole driving force (see
  `FlipSmokeParams.buoyancy`). A positive buoyancy coefficient makes
  hotter-than-ambient cells accelerate in `+y` (up), consistent with `+y`
  being "up" everywhere else in this project.

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — same as every other solver, a rendering
concern only.

## `FlipSmokeSimulation` class

```ts
const sim = new Module.FlipSmokeSimulation(); // seeds a default Plume scenario immediately
sim.step(dt);                                 // dt in seconds — advance one substep. See "stable substep" below.
sim.reset(scenario);                          // scenario: a plain object matching SmokeScenario's fields (see below — NOT the water Scenario)
const w = sim.getGridWidth();                 // number — MAC grid cell columns
const h = sim.getGridHeight();                // number — MAC grid cell rows
const densPtr = sim.getDensityFieldPtr();     // number (byte offset into the WASM heap)
const tempPtr = sim.getTemperatureFieldPtr(); // number (byte offset into the WASM heap)
sim.params;                                   // FlipSmokeParams — see the property gotcha below
```

There is no `getParticleCount()`/`getPositionsPtr()`/`getVelocitiesPtr()` —
smoke has no particles. `step`/`reset`/`params` follow the same *pattern* as
every other solver (fresh-params-every-call, reset()-owns-grid-allocation,
value_object read-modify-write), but the state you read back is
grid-shaped, not particle-shaped.

### Reading the density (and temperature) field — zero-copy typed arrays

Both fields are contiguous `float32` arrays, **row-major**, one value per
cell, length `getGridWidth() * getGridHeight()`. Cell `(i, j)` (column `i`,
row `j`, both zero-based) lives at index `j * getGridWidth() + i` — i.e. row
`j=0` is the **bottom** row of the domain (matches the `+y` is up
convention), and each row is `getGridWidth()` contiguous floats before the
next row starts. This is the same row-major layout `MacGrid`'s own
cell-centered arrays use internally (pressure, cell type), just exposed
here as a plain float buffer.

```ts
let densPtr = sim.getDensityFieldPtr();
let tempPtr = sim.getTemperatureFieldPtr();
let density = new Float32Array(Module.HEAPF32.buffer, densPtr, sim.getGridWidth() * sim.getGridHeight());
let temperature = new Float32Array(Module.HEAPF32.buffer, tempPtr, sim.getGridWidth() * sim.getGridHeight());

// cell (i, j):
const value = density[j * sim.getGridWidth() + i];
```

**Density range:** approximately `[0, 1]` — `1.0` is a hard clamp applied
every step at the emission source (see `FlipSmokeParams` below); values
elsewhere are bilinear-interpolated during advection, which can never
produce a value outside the range of its four input samples, so the whole
field stays within `[0, 1]` for the life of the simulation (verified
empirically over a 5-simulated-second run in
`tests/core/test_flip_smoke_solver.cpp`'s stability test — max observed
density was `0.999`, never negative). Good to treat directly as a grayscale
alpha/opacity value: `alpha = density[idx]`.

**Temperature range:** not clamped, but bounded in practice — it's a
weighted blend toward `sourceTemperature` at the source and relaxes toward
`ambientTemperature` everywhere else (see `FlipSmokeParams` below), so with
the defaults (`ambientTemperature = 0`, `sourceTemperature = 1`) it stays
roughly within `[0, 1]` too, though nothing hard-clamps it the way density
is clamped — a custom scenario with an unusually high `sourceTemperature`
will show temperature values up around that value near the source.

### Turning the density field into a texture (worked example for `FieldRenderer`)

The array is already exactly a single-channel, row-major image buffer, so
building a WebGL2 texture from it each frame is a single call — no
transposition, no channel interleaving:

```ts
const w = sim.getGridWidth();
const h = sim.getGridHeight();
const density = new Float32Array(Module.HEAPF32.buffer, sim.getDensityFieldPtr(), w * h);

gl.bindTexture(gl.TEXTURE_2D, densityTexture);
gl.texImage2D(
  gl.TEXTURE_2D,
  0,
  gl.R32F,       // single-channel float internal format (WebGL2)
  w,
  h,
  0,
  gl.RED,
  gl.FLOAT,
  density
);
```

Row `0` of this data is the bottom of the domain; most texture/canvas
coordinate conventions put row `0` at the **top**, so either flip your UV
`v` coordinate in the sampling shader (`v_flipped = 1.0 - v`) or set
`gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true)` before upload if you're
compositing through a 2D canvas path instead — the same y-flip concern
called out in the coordinate-convention section above, just at the texture
level instead of the vertex level.

If colorizing by temperature instead of (or blended with) density, sample
`getTemperatureFieldPtr()` the same way and map it through a color ramp
(e.g. dark red at ambient rising to white/yellow near `sourceTemperature`)
before blending with density-derived alpha — density alone is enough to
render recognizable smoke, temperature is exposed in case a "hot core"
look is wanted.

### Buffer stability contract — verified empirically, and DIFFERENT (better) than expected

- **Across `step()` calls:** the pointers are stable. This was verified
  empirically against this build (Node script driving the compiled
  `fluids.js`/`fluids.wasm` directly) — and it very nearly wasn't true: an
  early implementation reassigned the internal `density`/`temperature`
  buffers via `std::move` from a freshly-allocated vector at the end of
  every `step()` (part of the semi-Lagrangian advection pass), which
  silently gave back a **different** heap address every single step. That
  was caught by exactly this kind of empirical check (fetch the pointer,
  step 600 times, fetch again, compare) and fixed to write results
  in-place into the same buffer instead — see the "Deviations" section
  below. The lesson generalizes: for this solver family, don't assume
  pointer stability across `step()` from FLIP's docs alone — verify it
  against the actual build, because a natural-looking double-buffering
  implementation can easily break it. As shipped, you can fetch
  `getDensityFieldPtr()`/`getTemperatureFieldPtr()` once after `reset()` and
  keep using the same `Float32Array` view across any number of `step()`
  calls.
- **After `reset()`:** grid dimensions can change (`getGridWidth()` /
  `getGridHeight()` depend on `SmokeScenario.domainWidth/domainHeight` and
  `FlipSmokeParams.cellSize`), which reallocates the field buffers — **always
  call `getGridWidth()`/`getGridHeight()`/`getDensityFieldPtr()`/
  `getTemperatureFieldPtr()` again immediately after every `reset()`** and
  rebuild your `Float32Array` views, same discipline as every other solver
  in this project.
- No additional entry is needed in `wasm/CMakeLists.txt`'s
  `EXPORTED_RUNTIME_METHODS` — the density/temperature fields are `float32`
  same as every other solver's buffers, so the existing `["HEAPF32"]`
  covers it.

### The `params` property gotcha (verified again for smoke)

`sim.params` is an Embind `value_object` property, registered with the
identical mechanism as every other solver's params. **Reading it returns a
brand-new plain JS object every time** — confirmed empirically against this
build:

```ts
sim.params.buoyancy = 8; // WRONG — silently mutates a throwaway copy, no effect
// re-reading sim.params.buoyancy afterward still shows the OLD value (verified: stayed at the default 4)
```

Correct pattern, read-modify-write:

```ts
const p = sim.params;
p.buoyancy = 8;
sim.params = p; // writes the whole struct back — confirmed this actually persists
```

Also confirmed empirically, same as every other solver: `sim.params = {...}`
with a missing field throws `Missing field: "<name>"` (no silent zero-fill)
— e.g. `sim.params = { buoyancy: 1 }` throws `Missing field:
"ambientTemperature"` (the first missing field alphabetically/by
registration order, not necessarily every missing field at once). Always
supply every field, or start from `sim.params` and mutate.

`FlipSmokeParams` fields are read fresh by `step()` every call — a param
change takes effect on the very next step — **except `cellSize`**, which
only takes effect on the next `reset()` call (see below), same convention
as `FlipParams.cellSize`.

## `FlipSmokeParams` fields

| Field | Default | Units | Meaning |
|---|---|---|---|
| `buoyancy` | `4.0` | (m/s²) per unit temperature above ambient | Applied to vertical (`v`) face velocities each step: `v += dt * buoyancy * (T_face - ambientTemperature)`, where `T_face` is the average of the two cell-centered temperatures adjacent to that face. This is smoke's **entire driving force** — there is no separate gravity parameter (see coordinate-convention section above). |
| `ambientTemperature` | `0.0` | relative scale | Temperature of undisturbed air. The zero-point buoyancy measures against, and the value temperature relaxes toward (see `temperatureDissipation`). |
| `sourceTemperature` | `1.0` | relative scale | Temperature blended into cells inside the scenario's source region every step. Combined with the default `buoyancy`, this is what makes the plume rise (source is hotter than ambient). |
| `densityDissipation` | `0.1` | fraction/second | Each step, `density *= (1 - densityDissipation * dt)`, clamped so density never goes negative. Without this, continuously-emitted smoke accumulates forever (there's no outflow boundary). |
| `temperatureDissipation` | `0.5` | fraction/second | Each step, `T += (ambientTemperature - T) * min(1, temperatureDissipation * dt)`. Deliberately faster than `densityDissipation` by default — a real plume's visible smoke lingers longer than its "hotness," so temperature relaxes to ambient (buoyancy fades) well before the density trail has fully dissipated. |
| `cellSize` | `0.02` | meters | MAC grid cell size. Grid resolution `(nx, ny)` is derived from `SmokeScenario.domainWidth/domainHeight` and this value **only at `reset()` time** — changing `cellSize` without calling `reset()` afterward has no effect until the next `reset()`. Default gives a 61x100 grid for `makePlumeScenario()`'s 1.2m x 2.0m domain. |

**Temperature is a dimensionless relative scale, not physical degrees** —
`ambientTemperature = 0` / `sourceTemperature = 1` is a convention, not a
requirement. Only the *difference* `(T - ambientTemperature)` ever matters
(in the buoyancy term and as the dissipation relaxation target), so nothing
stops a caller from using a different scale (e.g. real Kelvin deltas) as
long as `ambientTemperature`/`sourceTemperature` are set consistently with
whatever scale is chosen.

There is no `flipRatio` (no particles, so no PIC/FLIP blend to speak of —
velocity is entirely grid-resident and self-advected every step) and no
`restDensity`/`particleMass`/`gasStiffness`/`viscosity` (no per-particle
equation of state — incompressibility comes from the same style of grid
pressure projection FLIP water uses). Expect a **different slider set** if
building a shared parameter panel across solvers/materials — intentional,
not a gap.

### What each parameter visually does (for slider-range/labeling decisions)

- **`buoyancy`:** near `0`, density just sits in the source region and
  dissipates in place — no visible "smoke" behavior at all, just a static
  fading patch. Above roughly `2-3`, a clear rising plume forms. Much
  higher (`10`+) makes it shoot up fast and can make the fixed-iteration
  pressure solve's residual divergence more visually noticeable (faint
  turbulence-like noise) since more momentum is being injected per step for
  the same 60-iteration Jacobi budget to cancel out. Recommend a slider
  range of roughly `[0, 10]`, default `4`.
- **`densityDissipation`:** near `0`, smoke never clears — density
  accumulates toward the `1.0` clamp everywhere it's ever visited and stays
  there (a permanently "smoky" domain). Higher values make the plume look
  thinner/wispier and fade out sooner after leaving the source. Recommend
  `[0, 1]`, default `0.1`.
- **`temperatureDissipation`:** controls how far the plume rises before
  buoyancy stops driving it (once local temperature has relaxed back to
  ambient, that region of the plume stops accelerating and just carries its
  existing momentum, subject to the pressure solve). Low values (near `0`)
  give a plume that keeps accelerating upward for a long time, tending to
  hit the top boundary and pool there; higher values (`1`+) make it curl
  and stall closer to the source. Recommend `[0, 2]`, default `0.5`.
- **`cellSize`:** same performance-cliff shape as `FlipParams.cellSize` (see
  `docs/flip-sim-api.md`) — grid cell count scales `~1/cellSize²`, and the
  fixed 60-iteration Jacobi pressure solve runs over every cell every step
  regardless of grid size, so cost scales the same way. The default `0.02`
  on `makePlumeScenario()`'s 1.2m x 2.0m domain gives a 61x100 (~6100 cell)
  grid, comfortably real-time in single-threaded WASM. Halving it
  (`0.01`, ~24000 cells) is roughly 4x the work; a slider should floor
  around `0.01-0.015` for this domain scale, same guidance as FLIP water.

## `SmokeScenario` — a deliberately separate struct from water's `Scenario`

**This is NOT the same struct SPH/FLIP/MPM use, and is not registered as an
overload of it.** Water's `Scenario` describes a one-shot particle seed
(position, size, initial velocity) — smoke has no particles and no
"initial velocity" to seed. Instead, `SmokeScenario` describes a domain plus
a source rectangle that **continuously** emits density/temperature every
single step for as long as the simulation runs (think a chimney or a
candle's flame footprint, not a one-time splash).

| Field | Default (`makePlumeScenario()`) | Meaning |
|---|---|---|
| `domainWidth` | `1.2` | Domain size, meters. Box is `[0, domainWidth] x [0, domainHeight]`. |
| `domainHeight` | `2.0` | Taller than wide by default — a plume needs vertical room to rise and curl before hitting the top boundary. |
| `sourceX` | `0.45` | Bottom-left corner of the source rectangle, meters. `makePlumeScenario()` centers it horizontally: `0.5 * (domainWidth - sourceWidth)`. |
| `sourceY` | `0.0` | Sits on the floor by default (a chimney/candle base). |
| `sourceWidth` | `0.3` | Source rectangle size, meters. |
| `sourceHeight` | `0.15` | |
| `emissionRate` | `2.0` | Density units added per second within the source region, before the per-cell `[0, 1]` clamp (see `FlipSmokeParams` above — the clamp is unconditional, not a `SmokeScenario` field). |

Note `sourceTemperature` is **not** a `SmokeScenario` field — it lives on
`FlipSmokeParams` instead, since it's a live-tunable physical constant (how
hot the source is), not a shape/placement property, matching the existing
split where `FlipParams.cellSize` lives outside water's `Scenario`.
`emissionRate` stays on `SmokeScenario` because how fast a *particular*
preset's source pumps out density (a wide slow vent vs. a small fast jet)
reads more like a scenario characteristic than a global tunable — this was
a judgment call; a reasonable alternative design would put both on one
struct, but this project already has that "physical constant vs. initial
condition" split for `cellSize` and this keeps it consistent.

```ts
const s = Module.makePlumeScenario();
sim.reset(s);
sim.getGridWidth();          // MUST re-fetch after reset()
sim.getGridHeight();
sim.getDensityFieldPtr();
sim.getTemperatureFieldPtr();
```

There is currently only one built-in preset (`makePlumeScenario()`) — no
second-region concept the way water's `Scenario.hasRegion2` supports two
converging blocks, since a single continuous source region is enough to
express "a plume" and there was no second Phase 4a scenario shape called
for in scope. Nothing stops a caller from constructing a custom
`SmokeScenario` object literal directly (e.g. a wide short source near a
side wall) without new C++ — same "generic region, frontend-parameterized"
philosophy as water's scenarios.

## Boundary handling

Same two-layer "belt-and-suspenders" shape as FLIP water, adapted for the
simpler classification:

1. **Cell classification is static**, computed once in `reset()` and never
   recomputed: the one-cell-thick outer ring is `Solid`, **every other cell
   is `Fluid`** ("air that can flow"). There is no `Air` cell state for
   smoke at all — unlike water, there's no particle occupancy to determine
   which interior cells currently hold material, since density/temperature
   are continuous fields defined everywhere in the interior at all times.
2. **Face zeroing:** every face touching a `Solid` cell has its normal
   velocity forced to zero — before computing pressure-solve divergence,
   again after the solve, and once more after advection (semi-Lagrangian
   advection can reintroduce a small nonzero normal component right at a
   boundary face by sampling from just inside the domain).
3. There is no particle-level clamp (no particles), so this is the entire
   boundary story for smoke.

## The stable substep — empirically determined, and noticeably more forgiving than water

**Recommended caller `dt`: `1/60`** (matches a plain, no-substepping 60fps
game loop). `1/120` gives a marginally smoother-looking result at roughly
double the per-frame cost; there is no need to go smaller than that for
visual quality.

This is a **meaningfully different empirical finding from both FLIP water
(`1/120` recommended, particle-based PIC/FLIP transfer) and MPM water
(`1/480` required, stiff pressure-like elastic forces)** — don't assume
either number transfers over. Reasoning, and what was actually tested:

- `FlipSmokeSolver::step()` clamps its incoming `dt` to `1/30` internally
  (same defensive-clamp pattern as every other solver's `step()`, guarding
  against a runaway caller `dt` e.g. after a stalled tab resumes). This
  means **`1/30` is a hard ceiling on the effective per-`step()`-call
  substep regardless of what's passed in** — passing `dt = 1` produces the
  exact same single-step result as passing `dt = 1/30` (verified: a native
  scratch probe passing `1, 1/2, 1/5, 1/10` all produced numerically
  identical results to the `1/30`-clamped case run for the same *number* of
  calls, confirming the internal clamp — not the algorithm — is what bounds
  it).
- At that `1/30` ceiling — and at every smaller substep tested down to
  `1/240` — the solver was **completely stable** over a 5-simulated-second
  `makePlumeScenario()` run: no NaN/Inf, density stayed in `[0, 1]`, and
  (notably) the plume's total density and rise height (density-weighted
  centroid height) came out nearly identical across `dt in {1/30, 1/60,
  1/120, 1/240}` — e.g. centroid height after 5s ranged only
  ~1.28-1.30m across that whole range, not a chaotic/dt-sensitive result.
  This is the expected behavior for the classic Stable Fluids method:
  semi-Lagrangian advection is **unconditionally stable** by construction
  (backward-tracing plus clamped bilinear sampling can't diverge or
  overshoot, unlike a naive forward-Euler advection scheme), which is the
  entire reason Jos Stam's method is named "stable" — this project's
  implementation reproduces that property, not just coincidentally.
- Given that, the recommendation of `1/60` over the maximum-stable `1/30`
  is purely a **visual quality** choice (temporal resolution of the
  advected fields, not a stability requirement) — pick smaller if a
  particular scene shows visible stepping in the plume's motion, but there
  is no danger of instability at any substep up to the internal `1/30`
  clamp, unlike MPM water where `1/480` is a genuine correctness
  requirement, not a quality preference.

## Solver notes (background, not part of the JS contract but useful context)

Classic Stable Fluids (Jos Stam) on the same 2D MAC (staggered) grid data
layout FLIP water uses (`fluids::flip::MacGrid`, reused directly rather than
duplicated — see `core/include/fluids/solvers/smoke/FlipSmokeSolver.h`).
Per-step pipeline:

1. **Emit source:** within `SmokeScenario`'s source rectangle, add density
   (`density += emissionRate * dt`, clamped to `[0, 1]`) and blend
   temperature toward `sourceTemperature`
   (`T += (sourceTemperature - T) * clamp(emissionRate * dt, 0, 1)`).
2. **Buoyancy:** `v += dt * buoyancy * (T_face - ambientTemperature)` on
   every vertical face, `T_face` = average of the two adjacent cell-centered
   temperatures (falling back to `ambientTemperature` for the two faces that
   sit exactly on the domain's top/bottom edge, which get zeroed by the next
   step anyway).
3. **Pressure projection:** the same fixed-60-iteration Jacobi scheme as
   `FlipSolver`'s water projection, simplified for the Fluid/Solid-only
   classification (no `Air` state — every non-`Solid` neighbor of a `Fluid`
   cell is itself `Fluid`, collapsing the "count non-Solid neighbors for the
   diagonal, but only sum pressure from `Fluid` neighbors" split water's
   version needs). This makes it a pure-Neumann (fully walled-in) Poisson
   system rather than one with an implicit `Air` = zero-pressure Dirichlet
   boundary — solvable only up to an additive constant, but Jacobi from an
   all-zero initial guess converges to one consistent solution, and only
   pressure *differences* (used to correct face velocities) ever matter, so
   the undetermined constant cancels exactly. Solid-adjacent faces are
   zeroed both before and after, same principle as water.
4. **Self-advect velocity:** semi-Lagrangian — trace each face backward by
   `dt` through the current (post-projection) velocity field, bilinear-
   sample the OLD velocity field there.
5. **Advect density/temperature:** same semi-Lagrangian technique, tracing
   through the exact SAME post-projection velocity snapshot used in step 4
   (not the step-4 output) — both self-advection and scalar advection read
   from one shared "current" field so the two passes aren't order-dependent
   on each other for no benefit.
6. **Dissipation:** density decays multiplicatively, temperature relaxes
   toward ambient (see `FlipSmokeParams` above).
7. **Boundary:** zero solid-adjacent face velocities once more (advection
   can reintroduce a nonzero normal component right at the boundary).

### Deviations from the literal task description (and why)

- **`SmokeScenario`/`FlipSmokeParams` field split for emission.** The task
  description mentioned "emission rate for density, source temperature" as
  both a `SmokeScenario` concern and separately listed `sourceTemperature`
  under `FlipSmokeParams`. Resolved by putting `emissionRate` +
  the source rectangle's position/size on `SmokeScenario` (placement/pacing,
  an initial-condition-like concept) and `sourceTemperature` on
  `FlipSmokeParams` (a live-tunable physical constant) — see the
  `SmokeScenario` section above for the full reasoning. This does not
  change any documented default value, only which struct owns which field.
- **Pointer-stability bug found and fixed during verification, not just
  documented as a limitation.** The initial implementation of
  `FlipSmokeSolver::advect()` used `density_ = std::move(densityNew);` /
  `temperature_ = std::move(temperatureNew);` to commit each step's
  advection result — which reassigns the vector's heap allocation wholesale
  rather than mutating the existing one, silently changing
  `getDensityFieldPtr()`/`getTemperatureFieldPtr()`'s return value on
  **every single `step()` call** (confirmed via a Node script against the
  actual compiled `fluids.wasm`: the pointer moved from one address to a
  different one after 600 steps). This directly contradicts the buffer-
  stability guarantee this document makes (and that water's solvers
  provide, since their particle buffers are only ever mutated in place).
  Fixed by writing the freshly-computed values into the *existing*
  `density_`/`temperature_` buffers via `std::copy` instead of replacing the
  vectors — verified afterward with the same Node script that the pointer
  is now identical before and after 600 `step()` calls. `grid_.u`/`grid_.v`
  (the internal MAC-grid velocity, never exposed to JS) still use `std::move`
  for the same commit, since nothing depends on their address staying fixed.
- **Cell classification simplified per the task's explicit guidance** ("no
  particle-occupancy-based Fluid/Air/Solid... don't force-fit water's
  marker-and-cell classification"): implemented as a static
  Fluid-interior/Solid-ring split computed once in `reset()`, never
  recomputed in `step()` (water recomputes classification from particle
  positions every single step, since which cells hold fluid changes as
  particles move — smoke has no such notion, every interior cell always
  "can flow").
- **Jacobi over CG, same 60-iteration count as FLIP water**, reusing the
  precedent set in Phase 2 rather than re-deriving a separate iteration
  count — verified via `tests/core/test_flip_smoke_solver.cpp`'s
  buoyancy/dissipation tests (which depend on projection actually
  converging well) that this holds up for the grid sizes the default
  `cellSize` produces.
- **Temperature field exposed alongside density**, per the task's explicit
  "your call" — included because it was already a first-class internal
  field (needed for buoyancy regardless) and costs nothing extra to expose,
  giving the frontend a choice (colorize by density, temperature, or a
  blend) rather than making that call here. Density alone is sufficient for
  a recognizable smoke render if the frontend doesn't want the extra
  complexity.

None of these change the core per-step pipeline described in the task —
they're the specific judgment calls the task explicitly left open, recorded
here for whoever picks up the next Phase 4 slice (Smoke for SPH and MPM)
and wants to know what this slice actually did versus what the task
sketched.
