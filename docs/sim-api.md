# Sim API contract (Phase 1 — SPH Water)

This is the JS-facing contract exposed by `wasm/bindings.cpp` (compiled from
`core/`) for the Phase 1 SPH water solver. It's written for the frontend
agent, who works only against this document and the compiled
`web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source.

Load the module the same way as Phase 0:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.Simulation();
```

## Coordinate convention

- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `Scenario` below).
- Units are meters, seconds, kg (SPH density is a 2D "areal" density,
  kg/m², not volumetric — see `SPHParams.restDensity`).
- Gravity's default sign is **negative** (`SPHParams.gravity = -9.81`)
  because it's applied directly as `acceleration.y += gravity` and `+y` is
  up — negative pulls particles down. If you flip `gravity` positive,
  particles fall "up" (toward +y).

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — that's a rendering concern, not something
the sim does for you.

## `Simulation` class

```ts
const sim = new Module.Simulation();   // seeds a default Dam Break scenario immediately
sim.step(dt);                          // dt in seconds — advance one substep
sim.reset(scenario);                   // scenario: a plain object matching Scenario's fields (see below)
const n = sim.getParticleCount();      // number
const posPtr = sim.getPositionsPtr();  // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr(); // number (byte offset into the WASM heap)
sim.params;                            // SPHParams — see the property gotcha below
```

### Reading positions/velocities (zero-copy typed arrays)

Both buffers are contiguous `float32` arrays laid out `[x0, y0, x1, y1, ...]`
(so length = `getParticleCount() * 2`). Build views the same way Phase 0
did:

```ts
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let positions = new Float32Array(Module.HEAPF32.buffer, posPtr, sim.getParticleCount() * 2);
let velocities = new Float32Array(Module.HEAPF32.buffer, velPtr, sim.getParticleCount() * 2);
```

**Buffer stability contract:**
- Across any number of `step()` calls, the underlying buffers are **not**
  reallocated — a pointer/view taken once stays valid and up to date; you do
  not need to re-fetch the pointer every frame purely because of `step()`.
  (Re-fetching every frame anyway, as Phase 0's doc recommended, is still
  fine and is cheap insurance against `ALLOW_MEMORY_GROWTH` detaching
  `Module.HEAPF32.buffer` — do that.)
- `reset()` **can** change the particle count, which reallocates both
  buffers. **Always call `getPositionsPtr()`/`getVelocitiesPtr()` again
  immediately after every `reset()`** and rebuild your `Float32Array` views
  — the old pointer values are invalid after `reset()`.
- Velocities use the same `Module.HEAPF32` view as positions (both are
  `float32`), so no additional entry is needed in
  `wasm/CMakeLists.txt`'s `EXPORTED_RUNTIME_METHODS` beyond the existing
  `["HEAPF32"]`. If a future phase adds a differently-typed buffer (e.g.
  `int32` ids), that type's heap view (`HEAP32`, etc.) would need to be
  added to that list or `Module.HEAP32` will be `undefined`.

### The `params` property gotcha (read this before wiring up sliders)

`sim.params` is an Embind `value_object` property. **Reading it returns a
brand-new plain JS object every time** — it is a copy, not a live reference
into WASM memory. This means:

```ts
sim.params.viscosity = 0.05; // WRONG — silently mutates a throwaway copy, no effect
```

does **nothing**. The correct pattern is read-modify-write:

```ts
const p = sim.params;
p.viscosity = 0.05;
sim.params = p; // writes the whole struct back
```

This applies to every field, every time. If you're wiring a Tweakpane
slider directly, wrap each `onChange` in this get/mutate/set-back pattern,
or keep your own JS-side copy of the params object as the source of truth
and push the whole thing to `sim.params` after each edit.

`SPHParams` fields are read fresh by `step()` every call, so a param change
takes effect on the very next step — no separate "apply" call needed.

## `SPHParams` fields

All fields are plain floats (except none are bools here). Defaults shown are
what a freshly-constructed `Simulation` starts with.

| Field | Default | Units | Meaning |
|---|---|---|---|
| `gravity` | `-9.81` | m/s² along +y | Acceleration added to every particle each step. Negative = downward (see coordinate convention above). |
| `restDensity` | `1000.0` | kg/m² | Target density. Tait EOS pushes denser-than-this fluid apart; pressure is clamped to 0 when local density is below this (see note below), so it never produces attraction. |
| `gasStiffness` | `200.0` | — (Tait `k`) | Stiffness of the pressure response. Higher = less compressible / stiffer water, but needs a smaller stable `dt`. |
| `viscosity` | `0.08` | — | Laplacian viscosity coefficient. `0` = inviscid/splashy; higher smooths out relative velocity between neighbors. |
| `smoothingRadius` | `0.045` | m | Kernel support radius `h` and neighbor-search cell size. Bigger = smoother/more expensive/more damped. |
| `particleMass` | `1.2` | kg | **Must stay calibrated against `restDensity` and the scenario's `spacing`** — see below. |
| `boundaryDamping` | `0.5` | fraction, [0,1] | Velocity retained (after sign flip) when a particle bounces off a domain wall. `0` = stops dead, `1` = perfectly elastic. |

**`particleMass` calibration (important, easy to break):** density
estimation only works if a rest-packed grid of particles at the scenario's
`spacing` sums (via the Poly6 kernel) to approximately `restDensity`. The
rule of thumb used for all built-in presets is:

```
particleMass ≈ restDensity * spacing²
```

The default `particleMass = 1.2` is calibrated for `restDensity = 1000` and
the presets' `spacing = 0.035` (`1000 * 0.035² = 1.225`). **If you expose a
"particle spacing" slider that changes `Scenario.spacing`, recompute and set
`particleMass` using this formula too**, or density will never reach
`restDensity`, pressure will be permanently clamped to zero, and particles
will fall through each other with no repulsion at all (this exact bug was
caught and fixed during Phase 1 solver development — see
`core/include/fluids/solvers/sph/SPHParams.h` for the full comment). Tait
`gasStiffness` does not need rescaling when you change `spacing`/mass — it
only acts on the dimensionless ratio `density / restDensity`.

Tait EOS exponent (gamma = 7) is a fixed internal constant, not exposed —
there was no ask to tune it and it's a standard WCSPH default.

## `Scenario` — constructing and resetting

`Scenario` is also a `value_object`: pass a plain object literal to
`sim.reset(...)`, no `new Module.Scenario()`. **Every field is required** —
Embind's struct marshalling throws `TypeError: Missing field: "<name>"` if
you omit even one (verified directly against this build's generated glue;
there is no silent zero-value fallback, and this applies to `sim.params =
{...}` too). Easiest way to always supply every field: start from a
preset's returned object and override just what you need, rather than
hand-writing a literal (see below).

```ts
sim.reset({
    domainWidth: 2.0,
    domainHeight: 1.5,
    spacing: 0.035,
    regionX: 0.05, regionY: 0.05, regionWidth: 0.5, regionHeight: 1.0,
    velocityX: 0.0, velocityY: 0.0,
    hasRegion2: false,
    region2X: 0, region2Y: 0, region2Width: 0, region2Height: 0,
    region2VelocityX: 0, region2VelocityY: 0,
});
```

Recommended pattern — start from a preset and tweak:

```ts
const s = Module.makeDamBreakScenario(); // plain object, see fields above
s.spacing = 0.03;                        // denser packing...
s.particleMass = 1000 * 0.03 * 0.03;     // ...so recalibrate mass (see above)
sim.reset(s);
sim.getPositionsPtr();                   // MUST re-fetch after reset()
sim.getVelocitiesPtr();
```

### `Scenario` fields

| Field | Meaning |
|---|---|
| `domainWidth`, `domainHeight` | Simulation box size, meters. Box is `[0,domainWidth] x [0,domainHeight]`. |
| `spacing` | Particle grid spacing used to seed **both** regions, meters. Keep `particleMass` calibrated to this (see above). |
| `regionX`, `regionY`, `regionWidth`, `regionHeight` | Primary seed region: an axis-aligned rectangle filled with a regular particle grid. |
| `velocityX`, `velocityY` | Initial velocity (m/s) given to every particle in the primary region. |
| `hasRegion2` | `true` to also seed a second region (e.g. Two-Column Collision). `false` ignores all `region2*` fields. |
| `region2X`, `region2Y`, `region2Width`, `region2Height`, `region2VelocityX`, `region2VelocityY` | Same shape as the primary region, for the optional second body. |

This is a deliberate simplification of the master plan's more open-ended
"list of seeded regions": two flat regions cover every Phase 1 preset (1
region for Dam Break/Block Drop, 2 converging regions for Two-Column
Collision) plus arbitrary custom two-body setups, while keeping the
value_object contract flat (no nested arrays/vectors to marshal from JS).

## Built-in presets

Callable directly: `Module.makeDamBreakScenario()`,
`Module.makeBlockDropScenario()`, `Module.makeTwoColumnCollisionScenario()`
— each returns a plain `Scenario`-shaped object (see table above) which you
can pass straight to `sim.reset()` or mutate first.

| Preset | domain (W×H) | region 1 (x,y,w,h) | vel 1 | region 2 (x,y,w,h) | vel 2 | spacing |
|---|---|---|---|---|---|---|
| Dam Break | 2.0 × 1.5 | 0.05, 0.05, 0.5, 1.0 | (0, 0) | — | — | 0.035 |
| Block Drop | 2.0 × 2.0 | 0.75, 1.2, 0.5, 0.5 | (0, 0) | — | — | 0.035 |
| Two-Column Collision | 2.0 × 1.2 | 0.05, 0.05, 0.4, 0.6 | (+1.5, 0) | 1.55, 0.05, 0.4, 0.6 | (−1.5, 0) | 0.035 |

All three use the default `SPHParams` (`particleMass = 1.2`, calibrated for
`spacing = 0.035` / `restDensity = 1000`, per above).

Verified (native Catch2 solver test, `tests/core/test_solver.cpp`, and ad
hoc diagnostics during development): Dam Break's column visibly collapses
and spreads across the floor within ~1.5s; Block Drop falls, splashes, and
settles into a thin layer; Two-Column Collision's two blocks converge and
collide. None of the three exhibit NaN/Inf or particles escaping the domain
over several seconds of simulated time at `dt = 1/240`.

## Boundary handling

Clamped-reflect-with-damping: a particle that crosses a wall is clamped back
onto it and the wall-normal velocity component is negated and scaled by
`boundaryDamping`. This is a hard position clamp, not a soft force — if it
looks visually wrong once rendered (e.g. particles visibly sticking flush to
walls), the documented fallback (not implemented) is penalty-force
boundaries; that would be a solver-side change, not something to work
around in the frontend.

## Solver notes (background, not part of the JS contract but useful context)

- WCSPH: Poly6 kernel for density, Spiky kernel gradient for pressure force
  (symmetric/momentum-conserving form), standard Laplacian viscosity (not
  XSPH). Tait equation of state, exponent 7, negative pressure clamped to 0.
- Neighbor search: a uniform spatial hash grid keyed by `smoothingRadius`,
  rebuilt every `step()`.
- Integration: semi-implicit (symplectic) Euler.
- `step(dt)` internally clamps `dt` to at most 1/30s as a defensive measure
  against a runaway caller `dt`; drive it with your own fixed-timestep
  substep loop as planned (e.g. accumulate real frame time and call
  `step()` in fixed increments like `1/120` or `1/240`) rather than relying
  on this clamp for stability.
- **Minimum effective kernel interaction distance (post-launch bug fix, not
  part of the original Phase 1 recipe).** A resting bed of particles (Dam
  Break run long enough to settle, or the analogous end state of Block Drop
  / Two-Column Collision) used to never actually go calm: a small number of
  neighbor pairs would gradually drift to near-perfect coincidence (from
  boundary clamping + ordinary packing pressure), and each time one nudged
  fractionally apart, the poly6 density kernel's near-flat plateau close to
  `r=0` would feed an outsized density estimate into the Tait EOS's
  exponent-7 stiffness response, producing a real (bounded, not literally
  infinite, but still several-hundred-to-~2000 m/s² vs. gravity's ~10 m/s²)
  force spike — enough to launch a single particle to double-digit m/s in
  one substep, every so often, forever. `poly6()` and `spikyGradient()`
  (`core/include/fluids/solvers/sph/Kernels.h`) now floor the *effective*
  interaction distance used inside their magnitude formulas at `0.5 *
  smoothingRadius` (direction is unaffected — it still uses the true,
  unfloored `r`), which caps how disproportionate any single near-coincident
  pair's contribution can be while leaving ordinary rest-spacing/compressed
  neighbor interactions (which sit much closer to `r ~ h`) untouched. See
  the comment above `kMinEffectiveRFraction` in `Kernels.h` for the full
  derivation/reasoning and `tests/core/test_solver.cpp`'s long-run settling
  regression test for the reproduction. This is purely an internal solver
  fix — it does not change `SPHParams`, `Scenario`, or any other JS-facing
  field.

None of the above changes the JS-facing contract described earlier in this
document — they're internal solver details, listed here (and in code
comments at the relevant call sites) for whoever next touches this solver.
