# Sim API contract (Phase 3 — MPM Water)

This is the JS-facing contract exposed by `wasm/mpm_bindings.cpp` (compiled
from `core/`) for the Phase 3 MLS-MPM water solver. It's written for the
frontend agent, who works only against this document and the compiled
`web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source.

`MpmSimulation` is a **third, independent solver class**, added alongside
the existing Phase 1 `Simulation` (SPH) and Phase 2 `FlipSimulation` (FLIP)
classes — it does not replace either, and shares no C++ inheritance
relationship with them (see `docs/sim-api.md` for SPH and
`docs/flip-sim-api.md` for FLIP; this document only covers what's
different/new for MPM). All three classes are exported from the same
compiled module, so a page can construct any or all of them.

Load the module the same way as Phase 0/1/2:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.MpmSimulation();   // seeds a default Dam Break scenario immediately
```

## Coordinate convention

Identical to SPH/FLIP — **not solver-specific**:
- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `Scenario`, shared unchanged across all three solvers — described in
  `docs/sim-api.md`).
- Units are meters, seconds. Like SPH, `MpmParams` has a `restDensity`
  (2D areal density, kg/m² — not volumetric), used both to calibrate
  particle mass and as the reference density in the pressure equation of
  state (see below).
- Gravity's default sign is **negative** (`MpmParams.gravity = -9.81`),
  same reasoning as SPH/FLIP: applied to grid node velocities each step,
  and `+y` is up, so negative pulls material down.

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — same as SPH/FLIP, a rendering concern
only.

## `MpmSimulation` class

```ts
const sim = new Module.MpmSimulation(); // seeds a default Dam Break scenario immediately
sim.step(dt);                           // dt in seconds — advance one substep. See "stable substep" below — MPM needs a smaller dt than SPH/FLIP.
sim.reset(scenario);                    // scenario: a plain object matching Scenario's fields (shared with SPH/FLIP)
const n = sim.getParticleCount();       // number
const posPtr = sim.getPositionsPtr();   // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr();  // number (byte offset into the WASM heap)
sim.params;                             // MpmParams — see the property gotcha below (same pattern as SPH/FLIP)
```

This is deliberately the exact same method shape as SPH's `Simulation` and
FLIP's `FlipSimulation` (`step`/`reset`/`getParticleCount`/
`getPositionsPtr`/`getVelocitiesPtr`/`params`), so a frontend
solver-switcher can treat all three uniformly.

### Reading positions/velocities (zero-copy typed arrays)

Identical pattern and contract to SPH/FLIP — both buffers are contiguous
`float32` arrays laid out `[x0, y0, x1, y1, ...]` (length =
`getParticleCount() * 2`):

```ts
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let positions = new Float32Array(Module.HEAPF32.buffer, posPtr, sim.getParticleCount() * 2);
let velocities = new Float32Array(Module.HEAPF32.buffer, velPtr, sim.getParticleCount() * 2);
```

**Buffer stability contract (same as SPH/FLIP):**
- Across any number of `step()` calls, the underlying buffers are **not**
  reallocated — a pointer/view taken once stays valid and up to date.
- `reset()` **can** change the particle count AND **always**
  (re)allocates the internal node grid (grid resolution depends on
  `params.cellSize`, and per-particle mass is recalibrated from
  `params.restDensity` and `scenario.spacing` — both only applied at
  `reset()` time, see `MpmParams` below). Either way, **always call
  `getPositionsPtr()`/`getVelocitiesPtr()` again immediately after every
  `reset()`** and rebuild your `Float32Array` views.
- No additional entry is needed in `wasm/CMakeLists.txt`'s
  `EXPORTED_RUNTIME_METHODS` for MPM — positions/velocities are `float32`
  same as SPH/FLIP, so the existing `["HEAPF32"]` covers it. (MPM's
  internal per-particle affine matrix `C`, scalar `J`, and node grid are
  never exposed to JS at all — nothing new is grid- or matrix-shaped in
  this contract.)

### The `params` property gotcha (verified again for MPM)

`sim.params` is an Embind `value_object` property, registered with the
identical Embind mechanism as `SPHParams`/`FlipParams`. **Reading it
returns a brand-new plain JS object every time** — verified empirically
against this build (Node script driving the compiled `fluids.js`/
`fluids.wasm` directly, mirroring the checks done for SPH/FLIP):

```ts
sim.params.stiffness = 40; // WRONG — silently mutates a throwaway copy, no effect
// re-reading sim.params.stiffness afterward still shows the OLD value
```

Correct pattern, read-modify-write (same as SPH/FLIP):

```ts
const p = sim.params;
p.stiffness = 40;
sim.params = p; // writes the whole struct back — confirmed this actually persists
```

Also confirmed empirically: `sim.params = {...}` with a missing field
throws `Missing field: "<name>"` (no silent zero-fill) — e.g.
`sim.params = { gravity: -9.81 }` throws because `restDensity`,
`stiffness`, `viscosity`, and `cellSize` are all missing. Always supply
every field, or start from `sim.params` and mutate.

`MpmParams` fields are read fresh by `step()` every call — a param change
takes effect on the very next step — **except `cellSize` and
`restDensity`**, which only take effect on the next `reset()` call (see
below; both feed grid/particle-mass calibration that only happens in
`reset()`).

## `MpmParams` fields

| Field | Default | Units | Range | Meaning |
|---|---|---|---|---|
| `gravity` | `-9.81` | m/s² along +y | any | Acceleration added to grid node velocities each step. Negative = downward (see coordinate convention). Same convention and default as `SPHParams.gravity`/`FlipParams.gravity`. |
| `restDensity` | `1000.0` | kg/m² | `> 0` | Reference density: `pressure(J) = stiffness * ((1/J)^7 - 1)`, and `1/J == density/restDensity` for a particle whose current volume is `J` times its rest volume. Also feeds the `particleMass` calibration at `reset()` — see below. Only takes effect at the next `reset()`, same as `cellSize`. |
| `stiffness` | `20.0` | — (EOS "k") | `>= 0` | Weakly-compressible EOS stiffness — this solver's analog of `SPHParams.gasStiffness`. Higher = less compressible / stiffer water, but needs a smaller stable substep (see below — MPM's stable substep is noticeably smaller than SPH/FLIP's even at this default). `0` disables the pressure force entirely (material free-falls with no repulsion — useful for isolating the P2G/G2P transfer itself, see the native transfer test). |
| `viscosity` | `0.0` | — | `>= 0` | Optional viscous stress coefficient: adds `viscosity * (C + Cᵀ)` to the per-particle stress, where `C` is the APIC affine velocity matrix (a cheap local velocity-gradient estimate, already computed every step). `0` (the default, used by all built-in presets) disables this term — pressure-only stress is the Phase 3 baseline. Raising it smooths relative velocity between nearby material like `SPHParams.viscosity` does, but pushes the stable substep down further — reduce your substep if you raise this a lot. |
| `cellSize` | `0.02` | meters | practical floor ~`0.008`-`0.01` for the default ~2m domains (see performance note below) | Node grid spacing. Grid resolution is derived from `Scenario.domainWidth/domainHeight` and this value **only at `reset()` time** — changing `cellSize` without calling `reset()` afterward has no effect until the next `reset()` (same contract as `FlipParams.cellSize`). |

There is no `smoothingRadius` or `boundaryDamping` for MPM — deliberately.
MPM has no per-particle neighbor kernel (the grid transfer stencil is a
fixed 3x3 quadratic B-spline, not a tunable radius), and its boundary
handling is a margin-based grid-velocity clamp near domain walls, not a
per-particle bounce (see "Boundary handling" below) — there's no damping
coefficient to expose for it. If you're building a shared parameter panel,
expect a **different slider set per solver**, same as FLIP already
established.

### `particleMass` calibration (same rule as SPH, applied internally)

MPM doesn't expose `particleMass` as a settable field — it's derived
automatically at every `reset()` from `params.restDensity` and
`scenario.spacing`:

```
particleMass = restDensity * spacing²
```

This is the exact same calibration rule as `SPHParams.particleMass` (mass
of one rest-packed grid cell's worth of area), and it matters for the same
reason: the EOS pressure response is driven by `J` (current volume /
rest volume), and rest volume is derived from `spacing²` too
(`restVolume = spacing²`, so `particleMass / restVolume == restDensity`
exactly by construction). Because this is computed automatically inside
`reset()` rather than left for the caller to get wrong, **there is no way
to desync it from `Scenario.spacing` the way SPH's manually-set
`particleMass` field can be** — if you expose a "particle spacing" slider
that changes `Scenario.spacing`, just call `reset()` with the new
scenario and the mass recalibrates itself automatically. (If you don't
call `reset()` after changing `spacing`, nothing bad happens either —
`spacing` only affects seeding, so it simply has no effect until the next
`reset()`, same as `cellSize`.)

The EOS exponent (`gamma = 7`, matching SPH's Tait exponent) is a fixed
internal constant, not exposed — same reasoning as SPH: no ask surfaced to
tune it, and it's a standard weakly-compressible default.

### What `stiffness` visually does (for slider-range/labeling decisions)

- Low values (e.g. `5`-`10`): water compresses more visibly under its own
  weight/impact before pressure pushes back — a shallow resting layer can
  settle noticeably below its nominal rest height (see the "settled
  layer" behavior in the empirical substep section below). Splashes look
  softer/mushier.
- Around the default (`20`): a reasonable balance for the built-in ~2x1.5m
  presets — visible splash and collapse dynamics without needing an
  impractically small substep.
- Higher values (`40`+): stiffer/less compressible water, but the stable
  substep shrinks further (see below) — a slider that allows large
  `stiffness` increases should be paired with either a smaller fixed
  substep or a warning that quality/performance trades off.

There is no hard internal ceiling on `stiffness`, but very high values
combined with a substep that's only stable for lower stiffness **will**
blow up (see the empirical substep findings below — this was directly
observed: `dt = 1/240` diverges at every stiffness tested, `1/480` and
`1/960` do not).

## The stable substep — `step(dt)` now subdivides internally; `1/480` is still the recommended caller dt

MPM's explicit time integration with a stiff EOS is noticeably less
forgiving than SPH's particle-pressure forces or FLIP's implicit-ish
pressure projection. This was checked directly (native diagnostic driving
`MpmSolver` for a 3-second Dam Break run at several `(stiffness, dt)`
combinations, `tests/core/test_mpm_solver.cpp`'s development process):

| `dt` | Result at `stiffness` in `{5, 10, 20, 40}` |
|---|---|
| `1/240` | **Diverges at every stiffness tested** — velocities exceed 50 m/s and `J` pins to its clamp bounds within the 3-second run. |
| `1/480` | Stable at every stiffness tested (`5`-`40`) **for a gradual scenario like Dam Break**. This is the **recommended default substep** — used by all of this phase's native tests and matching the default `MpmParams.stiffness = 20`. |
| `1/960` | Also stable, marginally tighter `J` distribution than `1/480`, at roughly double the per-frame cost. |

**This table was originally taken as "the" stable substep, but it was only
ever validated against Dam Break's gradual, progressive collapse.** The
Block Drop preset (`Module.makeBlockDropScenario()`) seeds a square block
well above the floor, so it free-falls as a single coherent mass and hits
the floor all at once at roughly `sqrt(2*9.81*1.2) ~= 4.9 m/s` — a much
larger, more sudden velocity discontinuity across the impact interface than
Dam Break ever produces. At a fixed `dt = 1/480`, this discontinuity was
large enough to swing a particle's `J` to the `kJMin = 0.1` clamp within a
single step at the moment of impact, which the stiff EOS then amplified
into a hundreds-to-thousands-of-m/s runaway within a handful of further
steps — a real bug (particles scattering across the whole domain at max
render speed within a couple of simulated seconds), not a rendering
artifact. Confirmed via a native trace of max particle speed and `J`
min/max around the impact frame.

**Fix: `MpmSolver::step(params, dt)` now internally subdivides the
caller's `dt` into one or more CFL-safe sub-steps**, re-deriving the safe
sub-dt from the *current* max particle speed before each one:

```
safeDt = kCflFraction * cellSize / max(maxParticleSpeed, epsilon)
```

with `kCflFraction = 0.1` (see `MpmSolver.cpp`, next to `kCflFraction`'s
definition, for the full derivation of why this fraction — in short,
`dt * maxSpeed / cellSize` is a rough bound on the largest fractional
change in `J` a single step's APIC transfer can produce, and keeping that
fraction small keeps the EOS's `(1/J)^gamma` pressure response from ever
spiking hard enough in one step to become unrecoverable). The loop is
capped at 512 sub-steps per `step()` call as a safety net against a
pathological infinite loop (not expected to be hit in normal operation —
even a 1000 m/s particle at the default `cellSize` only needs a few hundred
sub-steps to cover a `1/30`s caller `dt`); if the cap is exhausted, the
remaining time is advanced in one final step so `step()` always fully
advances by the requested `dt`.

**This is entirely internal — the JS-facing contract is unchanged.**
`MpmSimulation::step(dt)` still means "advance by `dt` seconds"; it does
not need a different `dt`, a different call pattern, or any frontend
change. During slow/settled regimes (Dam Break at rest, a settled layer,
Two-Column Collision once it stops sloshing) `maxParticleSpeed` is small
enough that `safeDt >= dt` and the loop runs exactly one sub-step — no
behavior or performance change from before. The extra sub-steps only kick
in during genuinely fast motion (Block Drop's impact, Two-Column
Collision's initial closing speed), where they're needed and cheap relative
to the alternative of re-tuning `dt` globally downward for every scenario.

**Keep driving `MpmSimulation.step()` with `dt = 1/480`** (not `1/120` or
`1/240` like SPH/FLIP) — this is still the recommended per-call `dt` for
the built-in presets and default `stiffness`; the adaptive sub-stepping
above is what makes it safe for Block Drop too, not a reason to pick a
different caller-facing value. If you expose a `stiffness` slider that lets
users push it meaningfully higher than `20`, the adaptive sub-stepping
should absorb most of the added stiffness automatically, but this wasn't
exhaustively re-verified above `stiffness = 40` — watch for a much higher
per-frame sub-step count (i.e. a performance cliff) as an early warning
sign rather than an outright explosion.

Note that `step(dt)` still clamps the *caller's* `dt` to at most `1/30` as
a defensive measure against a runaway caller `dt` (same pattern as SPH/
FLIP), before the adaptive sub-stepping described above ever sees it — this
guards against pauses/stalls, and is orthogonal to (not a substitute for)
the internal CFL sub-stepping that now makes any resulting `dt` safe.

### What `J` does and why it's not exposed to JS

Each particle tracks a scalar `J` (determinant of the deformation
gradient — current volume / rest volume) internally, used to compute EOS
pressure. It's clamped to `[0.1, 10]` as a defensive backstop (see
`MpmSolver.cpp`) against a runaway particle reaching a degenerate volume.
For a settled configuration under the default `stiffness = 20`, `J`
empirically stays in roughly `0.5`-`1.0` (the weakly-compressible EOS lets
a resting hydrostatic load compress the material somewhat before pressure
balances gravity — the same qualitative behavior WCSPH shows at moderate
stiffness) — see `tests/core/test_mpm_solver.cpp`'s bounds test. `J` isn't
part of the JS contract (nothing in SPH or FLIP's per-particle state is
exposed beyond position/velocity either) — it's purely an internal solver
quantity.

## `Scenario` — constructing and resetting

**Unchanged from SPH/FLIP — the exact same struct, exact same fields,
reused as-is.** See `docs/sim-api.md`'s `Scenario` section for the full
field table and the built-in presets (`Module.makeDamBreakScenario()`,
`Module.makeBlockDropScenario()`, `Module.makeTwoColumnCollisionScenario()`)
— all three work unmodified for `MpmSimulation.reset()`, exactly as for
SPH's and FLIP's `reset()`. There is no MPM-specific scenario field.

```ts
const s = Module.makeDamBreakScenario(); // same preset, works for all three solvers
sim.reset(s);
sim.getPositionsPtr(); // MUST re-fetch after reset()
sim.getVelocitiesPtr();
```

Note one interaction to be aware of, similar to FLIP's note on `spacing`:
`Scenario.spacing` controls both initial particle seeding density AND (via
the automatic `particleMass = restDensity * spacing²` calibration
described above) the EOS calibration — but unlike SPH, there's no separate
mass field for a caller to forget to update, since MPM recomputes it
automatically inside `reset()`. Changing `spacing` and calling `reset()`
is always safe; the only thing to know is that a denser `spacing` means
more particles (more P2G/G2P work per step), same performance
consideration as FLIP.

## Boundary handling

Simpler than both SPH (per-particle damped bounce) and FLIP (grid
Fluid/Air/Solid classification + pressure-solve boundary condition), by
design — MPM's EOS stress already resists local compression, so no
pressure-solve-style boundary condition is needed, just enough of a
velocity clamp to keep material from crossing a wall:

1. **Grid-level (primary):** after computing node velocities each step
   (momentum → velocity, then gravity), any node within a
   3-cell-wide margin of a domain wall has its **outward-pointing**
   velocity component zeroed (e.g. a node near `x=0` with `vx < 0` gets
   `vx = 0`; a node near `x=domainWidth` with `vx > 0` gets `vx = 0`, and
   the same for `y`/gravity's axis). This is a grid-velocity clamp, not a
   pressure boundary condition — it doesn't attempt to conserve
   incompressibility globally the way FLIP's Solid ring does, it just
   stops the bulk flow of material through a wall.
2. **Particle-level (safety net):** after advection (G2P), any particle
   whose position ends up outside `[0,domainWidth]x[0,domainHeight]` is
   clamped back onto the boundary, and the velocity component that would
   have carried it further out is zeroed — identical in shape to FLIP's
   particle-level safety net (not a damped bounce like SPH's).

## Performance scaling with `cellSize`

Like FLIP, the node grid's cell count scales as `~1/cellSize²` for a fixed
domain, and P2G/G2P both touch a fixed 9-node stencil per particle
regardless of grid resolution — so the grid-side cost of a step scales
roughly `~1/cellSize²` too (there's no per-cell iterative solve like
FLIP's Jacobi pressure projection, so MPM's grid pass is cheaper per-cell,
but the scaling trend with `cellSize` is the same shape). For the default
~2x1.5m scenarios:

- `cellSize = 0.02` (default): grid roughly 100x75+padding cells.
  Comfortably real-time in single-threaded WASM at the recommended `1/480`
  substep.
- `cellSize = 0.01`: ~4x the grid cells. Likely still fine but getting
  closer to a frame budget on slower machines, same qualitative curve as
  FLIP's `cellSize` note.
- `cellSize < 0.008` or so: getting into "performance cliff" territory for
  the default domain sizes, similar to FLIP's guidance — a slider should
  floor `cellSize` around `0.008`-`0.01` rather than let a user drag it to
  zero.
- Large `cellSize` (e.g. `> 0.1` on a 2m domain) doesn't crash, but the
  3-node-per-axis stencil becomes coarse relative to particle spacing,
  giving blocky, low-detail behavior and a less accurate local volume
  (`J`) estimate.

Unlike `FlipParams.cellSize`, `MpmParams.cellSize` doesn't create a
one-cell "inset" boundary ring the way FLIP's Solid classification does —
MPM's boundary margin (3 cells, see above) is defined directly in world
units (`3 * cellSize`), so a larger `cellSize` does shrink the usable
interior area, proportionally more than FLIP's one-cell inset, since the
margin here is 3 cells deep rather than 1. At the default `cellSize =
0.02` this is a 3cm margin per side on a 2m domain (1.5% — negligible); at
`cellSize = 0.1` it becomes a 30cm margin per side (15%), visibly shrinking
the usable area.

## Solver notes (background, not part of the JS contract but useful context)

Standard MLS-MPM (Moving Least Squares Material Point Method) with APIC
(Affine Particle-In-Cell) transfer — the "88-line MLS-MPM" fluid recipe
(Hu et al. / Taichi), adapted to standalone 2D C++. Per-particle state:
position, velocity, an APIC affine velocity matrix `C` (2x2), and a scalar
`J` (only `J`, not a full deformation gradient — a fluid has no shear
elasticity, so isotropic volume change is all the constitutive model
needs). `MpmSolver::step()` wraps the pipeline below in a CFL-adaptive
sub-stepping loop (see "The stable substep" above) — everything below
describes one sub-step; `step()` runs as many of them as needed to safely
cover the caller's requested `dt`. Per-sub-step pipeline (see
`core/src/solvers/mpm/MpmSolver.cpp` for the full derivation):

1. **Clear grid:** zero every node's mass and momentum accumulator.
2. **P2G:** for each particle, compute the standard MLS-MPM stencil
   (`base = floor(p/dx - 0.5)`, quadratic B-spline weights) and scatter
   mass, momentum (`mass*v + affine·dpos`), where `affine` folds in both
   the APIC term (`mass*C`) and the EOS-pressure-derived stress force
   term, onto the surrounding 3x3 node neighborhood.
3. **Grid update:** convert each node's accumulated momentum to velocity
   (divide by mass), apply gravity, then the margin-based boundary clamp
   described above.
4. **G2P:** for each particle, gather the new velocity and reconstruct the
   APIC affine matrix `C` from the same 9-node stencil, update `J` from
   `trace(C)` (clamped to `[0.1, 10]`), advect position by `dt * v`, then
   apply the particle-level boundary clamp.

### Deviations from the literal task description (and why)

- **Viscosity implemented but off by default.** The optional
  `viscosity * (C + Cᵀ)` stress term described in the task was
  implemented (see `MpmParams.viscosity` and `MpmSolver::particleToGrid`)
  rather than dropped — it was straightforward to add and didn't
  destabilize anything in testing at the default `stiffness`/substep. It
  defaults to `0` (matching every built-in preset and all native tests),
  so the pressure-only baseline is what ships; a caller opts into the
  viscous term explicitly.
- **`J` bounds tests use a settled shallow layer, not the Dam Break
  preset.** A freshly-released Dam Break column is still actively
  splashing/spreading at the 3-second mark (confirmed via a native trace:
  height spread actually grows again around `t=1.0`-`1.5s` from a
  wall-impact sloshback wave before flattening by `t=2`-`3s`), which is a
  much noisier regime for a per-particle `J` check than an
  already-at-rest configuration. The dedicated `[mpm][stability]` test
  instead seeds a wide, shallow, already-resting layer and checks `J`
  after it reaches quiescence — the Dam Break preset is still used (and
  does settle to a flat, low profile with no NaN/escapes) by the separate
  `[mpm][dambreak]` settling test, matching the pattern used for SPH and
  FLIP.
- **Grid padding for negative stencil indices.** The task's stencil
  formula (`base = floor(p/dx - 0.5)`) can legitimately produce a base
  index of `-1` for a particle sitting exactly at `x=0` or `y=0` (after
  the particle-level boundary clamp lands it exactly on the wall) — an
  out-of-range negative array index if taken literally. The grid is
  allocated with a small constant index padding (`kGridPadding = 2` in
  `MpmSolver.cpp`) so every physical array index used is
  `logicalIndex + kGridPadding`, and generously oversized on the far side
  too so the 3-node stencil never reads out of bounds for a particle at
  the domain's opposite edge either. This is purely an internal indexing
  detail — world positions/velocities exposed to JS are unaffected.
- **Boundary margin (3 cells) is world-unit-based, not a MacGrid-style
  Solid cell ring.** Per the task's explicit instruction not to force
  MPM onto FLIP's MacGrid layout, the boundary condition here is a
  velocity clamp keyed directly off world-space distance to each wall
  (`3 * cellSize`), not a classified ring of grid cells. Functionally
  similar in spirit to FLIP's Solid ring, but simpler to reason about for
  a node-centered grid — see the "Boundary handling" section above.

- **CFL-adaptive internal sub-stepping (post-launch bug fix, not part of
  the original Phase 3 recipe).** The task's literal per-step pipeline
  (clear grid, P2G, grid update, G2P) runs once per *sub-step*, not once
  per caller `step(dt)` call — `MpmSolver::step()` now computes a CFL-safe
  sub-dt from the current max particle speed and loops until the full
  requested `dt` is covered (capped at 512 iterations as a safety net; see
  "The stable substep" above for the full writeup and why it was needed —
  Block Drop's sudden floor impact could swing `J` to its clamp within one
  step of the original fixed-`dt` scheme, which the stiff EOS then
  amplified into a runaway). This only adds sub-steps during genuinely fast
  motion; slow/settled regimes still run exactly one sub-step per `step()`
  call, so it's not a general performance regression.

None of these change the JS-facing contract above — they're internal
solver details, listed here (and in code comments at the relevant call
sites in `MpmSolver.cpp`) for whoever picks up Phase 4 (Smoke) and wants
to know what Phase 3 actually did versus what the roadmap sketched.
