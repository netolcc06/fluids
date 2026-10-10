# Sim API contract (3D SPH Water — first 3D vertical slice)

This is the JS-facing contract exposed by `wasm/sph3d_bindings.cpp` (compiled
from `core/`) for the 3D SPH water solver. It's written for the frontend
agent building the 3D renderer/UI, who works only against this document and
the compiled `web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source. It
follows the same style and the same rigor as `docs/sim-api.md` (the 2D SPH
water contract) — read that document too if anything here is ambiguous, since
every convention not called out as different here is inherited unchanged.

**If you already know the 2D contract, read this first:** the single most
important difference is that every buffer here is **3 floats per particle
(`[x,y,z]`), not 2** — `getPositionsPtr()`/`getVelocitiesPtr()` need
`count * 3`, not `count * 2`, when you build your `Float32Array` views. Get
this wrong and you'll either read garbage past the buffer or silently only
see 2/3 of your particles. Every other structural convention (zero-copy
buffers, the `params` read-modify-write gotcha, `reset()` reallocating
buffers, presets, boundary handling, dt clamping) carries over unchanged from
2D — see below for anything 3D-specific.

Load the module the same way as the 2D contract:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.SphSimulation3D();
```

## Coordinate convention

- Origin `(0, 0, 0)` is one corner of the domain box.
- `+x` points right, `+y` points **up** (same as the 2D contract).
- `+z` points **toward the viewer** (out of the screen), making this a
  **right-handed** coordinate system (`x cross y = z`) — verified directly
  against `fluids::Vec3::cross` in `tests/core/test_vec3.cpp`. This matters a
  lot for the camera: a standard right-handed camera looking down `-z` at the
  scene sees `+x` to the right and `+y` up exactly like the 2D renderer's
  screen space, with the simulation box extending **toward the camera**
  along `+z` from the far wall (`z=0`) to the near wall (`z=domainDepth`).
- The simulation box is `[0, domainWidth] x [0, domainHeight] x
  [0, domainDepth]` (see `Scenario3D` below).
- Units are meters, seconds, kg. Unlike the 2D solver (which uses a 2D
  "areal" density, kg/m², because there's no third dimension to integrate
  over), **this solver's density is genuinely volumetric, kg/m³** — see
  `SphParams3D.restDensity` below.
- Gravity's default sign is **negative** (`SphParams3D.gravity = -9.81`),
  same convention and reasoning as 2D: it's applied as `acceleration.y +=
  gravity`, and `+y` is up, so negative pulls particles down.

If your renderer uses a different up-axis or handedness convention
internally, convert at the render boundary — that's a rendering concern, not
something the sim does for you (same policy as 2D).

## `SphSimulation3D` class

```ts
const sim = new Module.SphSimulation3D(); // seeds a default Dam Break 3D scenario immediately
sim.step(dt);                             // dt in seconds — advance one substep
sim.reset(scenario);                      // scenario: a plain object matching Scenario3D's fields (see below)
const n = sim.getParticleCount();         // number
const posPtr = sim.getPositionsPtr();     // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr();    // number (byte offset into the WASM heap)
sim.params;                               // SphParams3D — see the property gotcha below
```

### Reading positions/velocities (zero-copy typed arrays) — note the `* 3`

Both buffers are contiguous `float32` arrays laid out
`[x0, y0, z0, x1, y1, z1, ...]` — **length = `getParticleCount() * 3`**, not
`* 2`:

```ts
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let positions = new Float32Array(Module.HEAPF32.buffer, posPtr, sim.getParticleCount() * 3);
let velocities = new Float32Array(Module.HEAPF32.buffer, velPtr, sim.getParticleCount() * 3);
```

**Buffer stability contract (identical to 2D, repeated here for completeness):**
- Across any number of `step()` calls, the underlying buffers are **not**
  reallocated — a pointer/view taken once stays valid and up to date.
- `reset()` **can** change the particle count, which reallocates both
  buffers. **Always call `getPositionsPtr()`/`getVelocitiesPtr()` again
  immediately after every `reset()`** and rebuild your `Float32Array` views
  (with the `* 3` stride) — the old pointer values are invalid after
  `reset()`.
- Both buffers use the same `Module.HEAPF32` view (both `float32`), so no
  new entry is needed in `wasm/CMakeLists.txt`'s `EXPORTED_RUNTIME_METHODS`
  beyond the existing `["HEAPF32"]` — confirmed against this actual build
  (verified via a native Node.js script instantiating `fluids.wasm`
  directly, not assumed from the 2D contract).

### The `params` property gotcha — verified for this build too

Same gotcha as 2D, and yes, it applies here too (verified directly, not
assumed): `sim.params` is an Embind `value_object` property. **Reading it
returns a brand-new plain JS object every time** — a copy, not a live
reference into WASM memory.

```ts
sim.params.viscosity = 0.05; // WRONG — silently mutates a throwaway copy, no effect
```

Verified against this build: after doing exactly this, `sim.params.viscosity`
still reads back the previous value. The correct pattern is read-modify-write,
identical to 2D:

```ts
const p = sim.params;
p.viscosity = 0.05;
sim.params = p; // writes the whole struct back
```

Also verified: this pattern *does* persist (`sim.params.viscosity` reads back
`0.05` afterward). `SphParams3D` fields are read fresh by `step()` every
call, so a param change takes effect on the very next step — no separate
"apply" call needed.

## `SphParams3D` fields

All fields are plain floats. Defaults shown are what a freshly-constructed
`SphSimulation3D` starts with — **these are NOT the 2D defaults with units
relabeled**, they were determined empirically for this 3D solver (see the
"Why the defaults differ from 2D" section below).

| Field | Default | Units | Meaning |
|---|---|---|---|
| `gravity` | `-9.81` | m/s² along +y | Acceleration added to every particle each step. Negative = downward (see coordinate convention above). Same as 2D. |
| `restDensity` | `1000.0` | kg/m³ (**volumetric**, unlike 2D's kg/m²) | Target density. Tait EOS pushes denser-than-this fluid apart; pressure is clamped to 0 when local density is below this, same as 2D. |
| `gasStiffness` | `200.0` | — (Tait `k`) | Stiffness of the pressure response. Same numeric default as 2D — verified empirically that it does NOT need rescaling (it acts only on the dimensionless ratio `density/restDensity`, same as 2D). |
| `viscosity` | `0.08` | — | Laplacian viscosity coefficient. Same numeric default as 2D; produced comparable damping once `particleMass`/`smoothingRadius` were recalibrated. |
| `smoothingRadius` | `0.09` | m | Kernel support radius `h` and the `UniformGrid3D` cell size. Twice the 2D default (0.045) in absolute terms — see below for why. |
| `particleMass` | `0.317` | kg | **Must stay calibrated against `restDensity` and the scenario's `spacing`** — see the calibration section below, this is the field most likely to bite you if you add a spacing slider. |
| `boundaryDamping` | `0.5` | fraction, [0,1] | Velocity retained (after sign flip) when a particle bounces off a domain wall. Same meaning and default as 2D. |

### Why the defaults differ from 2D

3D has fundamentally more neighbors within the same physical smoothing
radius than 2D (a sphere's volume grows with `h³`, a circle's area with
`h²`), which changes both the density estimate's scale and the force sums
per particle. These defaults were **not** ported from 2D by relabeling units
— they were derived and checked with a native diagnostic (see the
calibration section, and `tests/core/test_solver3d.cpp`'s calibration test):

- **`smoothingRadius` (0.09) and `spacing` (0.06, in `Scenario3D`'s presets)**
  are coarser in absolute terms than 2D's (`h=0.045`, `spacing=0.035`) —
  this is a deliberate application of the project-wide "accept coarse grids
  for performance" decision: 3D particle counts grow with the cube of
  resolution, so fewer, larger particles keep counts tractable (the default
  Dam Break 3D preset seeds 672 particles; the 2D one seeds 392 — comparable
  order of magnitude despite the extra dimension, precisely because of this
  coarsening).
- **`gasStiffness` (200) and `viscosity` (0.08)** were re-verified stable at
  these settings and the stable substep below, unchanged numerically from
  2D — they act on dimensionless ratios / already-normalized quantities, so
  they transfer without rescaling. This was *checked*, not assumed.
- **`particleMass` (0.317) needed a real correction beyond the naive formula**
  — see below, this is the one place 2D's approach ("apply the formula,
  round slightly") wasn't sufficient on its own.

### `particleMass` calibration (important, easy to break — more so than 2D)

Same failure mode as 2D if this drifts out of calibration: density
estimation only works if a rest-packed grid of particles at the scenario's
`spacing` sums (via the 3D Poly6 kernel) to approximately `restDensity`. If
it doesn't, pressure is permanently clamped to 0 (see
`SphSolver3D::computeDensityPressure`) and particles fall through each other
with no repulsion at all.

The baseline rule of thumb, dimensionally adapted from 2D's `particleMass ≈
restDensity * spacing²` — note the **cube**, not the square, because a
rest-packed 3D grid cell is a volume element, not an area element:

```
particleMass ≈ restDensity * spacing³
```

For `restDensity = 1000` and the presets' default `spacing = 0.06`, this
naive formula gives `1000 * 0.06³ = 0.216`. **Verified empirically (native
diagnostic — see `tests/core/test_solver3d.cpp`'s calibration test) that this
naive figure is NOT close enough**: summing the 3D Poly6 kernel over an
infinite rest-packed lattice at this spacing/smoothingRadius with
`particleMass = 0.216` measures a density of only **~681**, about 32% short
of `restDensity = 1000`. This is a genuine 3D-vs-2D difference, not a bug in
the formula's derivation: at this solver's `smoothingRadius / spacing = 1.5`
ratio, the kernel support only reaches the face-neighbor and edge-neighbor
shells of the lattice (19 neighbors total, including the particle itself) —
the corner-neighbor shell (8 more particles, at `sqrt(3) * spacing ≈ 1.73 *
spacing`) falls just outside the support radius and contributes nothing,
so the discrete sum under-integrates relative to the continuous kernel
integral the normalization constant assumes.

The fix applied here — and the one to reapply if you ever change
`spacing`/`smoothingRadius` — is a **measured correction factor on top of
the naive formula**, not a change to `smoothingRadius`/`spacing` themselves
(those were already chosen for the coarse-grid performance trade-off):

```
particleMass = (restDensity * spacing³) * 1.468   // ≈ 0.317 for the defaults above
```

This lands the same lattice-sum calibration within about 1% of
`restDensity` (see the tolerance in `tests/core/test_solver3d.cpp`'s
calibration test, which pins this down as a regression guard). **The 1.468
factor is itself a function of the `smoothingRadius / spacing` ratio** — it
is not a universal 3D constant. If you expose a particle-spacing slider (or
change `smoothingRadius`), don't just reapply `1.468x` blindly: recompute the
lattice-sum calibration for the new ratio the same way (sum the 3D Poly6
kernel — `fluids::sph3d::poly6` — over a rest-packed lattice at the new
spacing/radius and see how far off `restDensity` you land, then correct).

Tait EOS exponent (gamma = 7) is a fixed internal constant, same as 2D — not
exposed, no ask to tune it.

## `Scenario3D` — constructing and resetting

`Scenario3D` is also a `value_object`: pass a plain object literal to
`sim.reset(...)`, no `new Module.Scenario3D()`. **Every field is required**
— same Embind marshalling behavior as 2D's `Scenario` (throws `TypeError:
Missing field: "<name>"` if you omit one; no silent zero-value fallback).
Start from a preset's returned object and override just what you need,
same recommended pattern as 2D:

```ts
sim.reset({
    domainWidth: 1.2, domainHeight: 1.0, domainDepth: 0.6,
    spacing: 0.06,
    regionX: 0.06, regionY: 0.06, regionZ: 0.06,
    regionWidth: 0.42, regionHeight: 0.72, regionDepth: 0.48,
    velocityX: 0.0, velocityY: 0.0, velocityZ: 0.0,
    hasRegion2: false,
    region2X: 0, region2Y: 0, region2Z: 0,
    region2Width: 0, region2Height: 0, region2Depth: 0,
    region2VelocityX: 0, region2VelocityY: 0, region2VelocityZ: 0,
});
```

Recommended pattern — start from a preset and tweak:

```ts
const s = Module.makeDamBreak3DScenario(); // plain object, see fields below
s.spacing = 0.05;                          // denser packing...
// ...recalibrate particleMass for the new spacing (see the calibration
// section above — for 3D this means redoing the lattice-sum check, not
// just plugging into the naive spacing^3 formula).
sim.reset(s);
sim.getPositionsPtr();                     // MUST re-fetch after reset()
sim.getVelocitiesPtr();
```

### `Scenario3D` fields

| Field | Meaning |
|---|---|
| `domainWidth`, `domainHeight`, `domainDepth` | Simulation box size, meters. Box is `[0,domainWidth] x [0,domainHeight] x [0,domainDepth]`. |
| `spacing` | Particle grid spacing used to seed **both** regions, meters. Keep `particleMass` calibrated to this (see above). |
| `regionX`, `regionY`, `regionZ`, `regionWidth`, `regionHeight`, `regionDepth` | Primary seed region: an axis-aligned **box**, filled with a regular 3D grid of particles at `spacing`. |
| `velocityX`, `velocityY`, `velocityZ` | Initial velocity (m/s) given to every particle in the primary region. |
| `hasRegion2` | `true` to also seed a second box region (e.g. Two-Column Collision). `false` ignores all `region2*` fields. |
| `region2X`, `region2Y`, `region2Z`, `region2Width`, `region2Height`, `region2Depth`, `region2VelocityX`, `region2VelocityY`, `region2VelocityZ` | Same shape as the primary region, for the optional second body. |

Same deliberate simplification rationale as 2D's `Scenario`: two flat box
regions cover every built-in preset (1 region for Dam Break/Block Drop, 2
converging regions for Two-Column Collision) plus arbitrary two-body custom
setups from JS, while keeping the value_object contract flat.

## Built-in presets

Callable directly: `Module.makeDamBreak3DScenario()`,
`Module.makeBlockDrop3DScenario()`, `Module.makeTwoColumnCollision3DScenario()`
— each returns a plain `Scenario3D`-shaped object you can pass straight to
`sim.reset()` or mutate first.

| Preset | domain (W×H×D) | region 1 (x,y,z,w,h,d) | vel 1 | region 2 (x,y,z,w,h,d) | vel 2 | spacing | particle count |
|---|---|---|---|---|---|---|---|
| Dam Break 3D | 1.2 × 1.0 × 0.6 | 0.06, 0.06, 0.06, 0.42, 0.72, 0.48 | (0,0,0) | — | — | 0.06 | 672 |
| Block Drop 3D | 1.2 × 1.2 × 0.6 | 0.42, 0.70, 0.18, 0.36, 0.36, 0.24 | (0,0,0) | — | — | 0.06 | 144 |
| Two-Column Collision 3D | 1.4 × 0.7 × 0.5 | 0.06, 0.06, 0.06, 0.30, 0.36, 0.38 | (+1.2,0,0) | 1.04, 0.06, 0.06, 0.30, 0.36, 0.38 | (−1.2,0,0) | 0.06 | 360 |

All three use the default `SphParams3D` (`particleMass = 0.317`, calibrated
for `spacing = 0.06` / `restDensity = 1000`, per above).

**Verified (native diagnostics — a scratch program driving `SphSimulation3D`
directly, plus `tests/core/test_solver3d.cpp`'s Catch2 suite):**
- Dam Break 3D's column collapses within the first simulated second and
  spreads out along the floor across x and z; by several seconds in it
  settles into a thin resting layer with decaying particle speeds (observed
  RMS speed dropping from ~0.6 m/s at t=1s to ~0.3 m/s by t=8s in the
  diagnostic run). This qualitative "spreads thin across most of the floor
  footprint" outcome was cross-checked against the **existing, already-shipped
  2D solver** run the same way on its own Dam Break preset — the 2D reference
  shows the identical pattern (settles to a near-zero-height layer spanning
  the full domain width by ~5s). This is expected behavior for basic WCSPH
  without surface tension/cohesion at these particle counts, not a
  3D-specific problem.
- Block Drop 3D and Two-Column Collision 3D were both run for 5 simulated
  seconds in the same diagnostic: no NaN/Inf, no particles outside the
  domain bounds, and RMS speed decays monotonically after the initial
  impact/collision transient in both cases (no persistent high-speed
  outliers — the exact failure mode the 2D near-singularity bug produced).
- None of the three exhibit NaN/Inf or particles escaping the domain over
  several seconds of simulated time at `dt = 1/240`, matching the bar set by
  the 2D contract.
- **Long-run settling regression** (`tests/core/test_solver3d.cpp`, mirroring
  the 2D project's own regression test for the near-singularity bug):
  25 simulated seconds of Dam Break 3D at `dt = 1/120`, asserting every
  one-second window's peak RMS speed in the back half of the run (seconds
  13–25) stays below 2.0 m/s, and that the back half isn't more energetic
  than shortly after the initial collapse. This test was written from day
  one (not after discovering a live bug) specifically because the fix that
  prevents this failure mode (see below) was baked into the 3D kernels from
  the start.

## Stable substep

`dt = 1/120` (matching the 2D frontend's documented substep) was used for
the long-run settling regression test above and produced a calm, decaying
resting state — same substep as 2D, verified rather than assumed. `dt =
1/240` was used for the shorter stability/collapse tests, also stable. As
with 2D, `step(dt)` internally clamps `dt` to at most `1/30s` as a defensive
measure against a runaway caller `dt`; drive it with your own fixed-timestep
substep loop rather than relying on this clamp for stability.

## Boundary handling

Clamped-reflect-with-damping, extended to **6 walls** (min/max on x, y, and
z) instead of 2D's 4: a particle that crosses a wall is clamped back onto it
and the wall-normal velocity component is negated and scaled by
`boundaryDamping`. Same hard-position-clamp caveat as 2D applies (if it looks
visually wrong once rendered, the documented fallback — not implemented —
is penalty-force boundaries, a solver-side change).

## Solver notes (background, not part of the JS contract but useful context)

- WCSPH: 3D Poly6 kernel for density, 3D Spiky kernel gradient for pressure
  force (symmetric/momentum-conserving form), standard 3D Laplacian
  viscosity (not XSPH) — same algorithmic choices as 2D, with
  dimension-specific normalization constants (see
  `core/include/fluids/solvers/sph3d/Kernels3D.h`):
  - Poly6: `315 / (64 * pi * h^9) * (h^2 - r^2)^3`
  - Spiky gradient magnitude: `45 / (pi * h^6) * (h - r)^2`
  - Viscosity Laplacian: `45 / (pi * h^6) * (h - r)`
- Neighbor search: `UniformGrid3D`, the 3D sibling of the 2D `UniformGrid`
  (concrete over `Vec3`, not a generic template — same YAGNI reasoning as
  2D's own header comment), keyed by `smoothingRadius`, rebuilt every
  `step()`. A radius query scans the surrounding 3×3×3 = 27 cells instead of
  2D's 3×3 = 9.
- Integration: semi-implicit (symplectic) Euler, same as 2D.
- **The 2D SPH near-singularity bug fix was baked into this 3D solver from
  day one, not rediscovered after the fact.** The 2D project found (see
  `core/include/fluids/solvers/sph/Kernels.h` and `docs/sim-api.md`) that
  `poly6`'s near-flatness close to `r=0`, combined with `spikyGradient`'s
  force magnitude and the Tait EOS's exponent-7 response, could produce a
  near-singular repulsive force between nearly-coincident particles that
  never let a resting bed of particles go calm. The 3D kernels
  (`core/include/fluids/solvers/sph3d/Kernels3D.h`) floor the *effective*
  interaction distance used inside both `poly6`'s and `spikyGradient`'s
  magnitude formulas at `0.5 * smoothingRadius` from the start — direction
  is unaffected (still keyed to the true, unfloored `r`). See
  `kMinEffectiveRFraction` in `Kernels3D.h` for the same derivation adapted
  to the 3D normalization constants, and the long-run settling regression
  test in `tests/core/test_solver3d.cpp` (written proactively, not as a
  post-hoc reproduction) for the coverage.

None of the above changes the JS-facing contract described earlier in this
document — they're internal solver details, listed here (and in code
comments at the relevant call sites) for whoever next touches this solver.
