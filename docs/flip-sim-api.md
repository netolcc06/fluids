# Sim API contract (Phase 2 — FLIP Water)

This is the JS-facing contract exposed by `wasm/flip_bindings.cpp` (compiled
from `core/`) for the Phase 2 PIC/FLIP water solver. It's written for the
frontend agent, who works only against this document and the compiled
`web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source.

`FlipSimulation` is a **second, independent solver class**, added alongside
the existing Phase 1 `Simulation` (SPH) class — it does not replace it, and
the two share no C++ inheritance relationship (see `docs/sim-api.md` for
SPH; this document only covers what's different/new for FLIP). Both classes
are exported from the same compiled module, so a page can construct either
or both.

Load the module the same way as Phase 0/1:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.FlipSimulation();   // seeds a default Dam Break scenario immediately
```

## Coordinate convention

Identical to SPH — **not solver-specific**:
- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `Scenario`, shared unchanged with SPH — described in `docs/sim-api.md`).
- Units are meters, seconds. Unlike SPH, FLIP has no areal-density unit to
  worry about at all (see `FlipParams` below) — there is no `restDensity`
  or `particleMass`.
- Gravity's default sign is **negative** (`FlipParams.gravity = -9.81`),
  same reasoning as SPH: applied as `+y` grid-velocity change each step,
  and `+y` is up, so negative pulls the fluid down.

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — same as SPH, a rendering concern only.

## `FlipSimulation` class

```ts
const sim = new Module.FlipSimulation(); // seeds a default Dam Break scenario immediately
sim.step(dt);                            // dt in seconds — advance one substep
sim.reset(scenario);                     // scenario: a plain object matching Scenario's fields (shared with SPH)
const n = sim.getParticleCount();        // number
const posPtr = sim.getPositionsPtr();    // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr();   // number (byte offset into the WASM heap)
sim.params;                              // FlipParams — see the property gotcha below (same pattern as SPH)
```

This is deliberately the exact same method shape as SPH's `Simulation`
class (`step`/`reset`/`getParticleCount`/`getPositionsPtr`/
`getVelocitiesPtr`/`params`), so a frontend solver-switcher can treat both
uniformly — e.g. `const sim = solverName === "flip" ? new
Module.FlipSimulation() : new Module.Simulation();` and drive either one
identically from that point on.

### Reading positions/velocities (zero-copy typed arrays)

Identical pattern and contract to SPH — both buffers are contiguous
`float32` arrays laid out `[x0, y0, x1, y1, ...]` (length =
`getParticleCount() * 2`):

```ts
let posPtr = sim.getPositionsPtr();
let velPtr = sim.getVelocitiesPtr();
let positions = new Float32Array(Module.HEAPF32.buffer, posPtr, sim.getParticleCount() * 2);
let velocities = new Float32Array(Module.HEAPF32.buffer, velPtr, sim.getParticleCount() * 2);
```

**Buffer stability contract (same as SPH):**
- Across any number of `step()` calls, the underlying buffers are **not**
  reallocated — a pointer/view taken once stays valid and up to date.
- `reset()` **can** change the particle count (e.g. switching from Dam
  Break to Block Drop) AND **always** reallocates the MAC grid internally
  (grid resolution depends on `params.cellSize`, only applied at `reset()`
  time — see `FlipParams.cellSize` below). Either way, **always call
  `getPositionsPtr()`/`getVelocitiesPtr()` again immediately after every
  `reset()`** and rebuild your `Float32Array` views. Verified empirically
  against this build: a `reset()` that changes particle count did not
  actually move the returned pointer value in one observed run, but that's
  incidental (allocator reuse), not a guarantee — keep refetching.
- No additional entry is needed in `wasm/CMakeLists.txt`'s
  `EXPORTED_RUNTIME_METHODS` for FLIP — positions/velocities are `float32`
  same as SPH, so the existing `["HEAPF32"]` covers it. (FLIP's internal
  MAC grid — cell types, pressure, face velocities — is never exposed to
  JS at all; there is nothing grid-shaped in this contract to view.)

### The `params` property gotcha (same as SPH — verified again for FLIP)

`sim.params` is an Embind `value_object` property, registered with the
identical Embind mechanism as SPH's `SPHParams`. **Reading it returns a
brand-new plain JS object every time** — confirmed empirically against this
build (Node script driving the compiled `fluids.js`/`fluids.wasm` directly):

```ts
sim.params.flipRatio = 0.5; // WRONG — silently mutates a throwaway copy, no effect
// re-reading sim.params.flipRatio afterward still shows the OLD value
```

Correct pattern, read-modify-write:

```ts
const p = sim.params;
p.flipRatio = 0.5;
sim.params = p; // writes the whole struct back — confirmed this actually persists
```

Also confirmed empirically, same as SPH: `sim.params = {...}` with a
missing field throws `Missing field: "<name>"` (no silent zero-fill) — e.g.
`sim.params = { gravity: -9.81 }` throws because `flipRatio` and
`cellSize` are missing. Always supply every field, or start from
`sim.params` and mutate.

`FlipParams` fields are read fresh by `step()` every call — a param change
takes effect on the very next step — **except `cellSize`**, which only
takes effect on the next `reset()` call (see below).

## `FlipParams` fields

| Field | Default | Units | Range | Meaning |
|---|---|---|---|---|
| `gravity` | `-9.81` | m/s² along +y | any | Acceleration added to grid velocities each step. Negative = downward (see coordinate convention). Same convention and default as `SPHParams.gravity`. |
| `flipRatio` | `0.95` | fraction, clamped to [0, 1] internally every step | `[0, 1]` | PIC/FLIP blend. `0` = pure PIC (very damped/stable, "thick"-looking water — each step fully replaces particle velocity with the interpolated grid velocity). `1` = pure FLIP (energetic/splashy — particles keep their own velocity and only add the grid's *change* since the last transfer, so nothing damps small solver noise). `0.95` is the standard default: mostly FLIP, with a sliver of PIC to suppress visible jitter. |
| `cellSize` | `0.025` | meters | practical floor ~`0.01` for the default ~2m domains (see performance note below); no hard ceiling but very large values shrink the usable fluid area (see below) | MAC grid cell size. Grid resolution `(nx, ny)` is derived from `Scenario.domainWidth/domainHeight` and this value **only at `reset()` time** — changing `cellSize` without calling `reset()` afterward has no visible effect (verified: `step()` keeps running fine against the old grid). |

There is no `particleMass`, `restDensity`, `gasStiffness`, `viscosity`, or
`boundaryDamping` for FLIP — deliberately. FLIP's incompressibility comes
from the grid pressure projection (a global linear solve), not a
per-particle equation of state, so there's no particle-mass/rest-density
calibration to keep in sync with `Scenario.spacing` the way SPH's
`particleMass` formula requires. If you're building a shared parameter
panel, expect a **different slider set per solver** — this was called out
in the task and is intentional, not a gap.

### What `flipRatio` visually does (for slider-range/labeling decisions)

- Near `0`: very stable, but looks noticeably viscous/thick — small
  splashes get smoothed away almost immediately, water tends to slump
  rather than splash. Good "safe" end of a slider.
- Near `1`: energetic, splashy, more visual detail in waves and impacts —
  but with only a 60-iteration fixed Jacobi pressure solve (see below),
  small divergence residuals aren't fully damped by PIC blending, so very
  high `flipRatio` (esp. at `1.0` exactly) can show faint persistent
  jitter/noise in a settled pool that a slightly lower value (`0.9-0.95`)
  hides. Recommend a slider range of `[0, 1]` with the default at `0.95`,
  and don't be surprised if the top 1-2% of the range looks slightly
  noisier than the rest — that's expected given the fixed iteration count,
  not a bug.

### What happens if `cellSize` is set absurdly small (performance cliff)

Grid cell count scales as `~1/cellSize²` for a fixed domain, and the
pressure solve's fixed 60 Jacobi iterations run over every cell every
step regardless of grid size — so cost scales roughly `~1/cellSize²` too
(no adaptive iteration count). For the default ~2x1.5m scenarios:

- `cellSize = 0.025` (default): ~80x60 = 4800 cells. Comfortably real-time
  in single-threaded WASM.
- `cellSize = 0.01`: ~200x150 = 30000 cells (~6x the work). Likely still
  fine but getting close to a frame budget on slower machines.
- `cellSize = 0.005` or smaller: ~400x300+ = 120000+ cells (~25x the
  default). Expect visible frame drops in single-threaded WASM — this is
  the "performance cliff" territory. There's no internal safety clamp on
  how small `cellSize` can go, so a slider should cap it (e.g. floor
  around `0.01-0.015` for the built-in domain sizes) rather than let a
  user drag it to zero.
- Extremely large `cellSize` (e.g. `> 0.3` on a 2m domain) doesn't crash
  either, but degrades quality: the grid resolution constructor clamps
  `nx`/`ny` to a minimum of 3 (so the solver never has zero interior
  cells), but a coarse grid gives a blocky, low-detail free surface and
  more visible pressure-solve error.

### The one-cell Solid-boundary inset (worth knowing, not a bug)

The MAC grid classifies the entire outer ring of cells (`i=0`, `i=nx-1`,
`j=0`, `j=ny-1`) as `Solid` — there are no interior obstacles yet, so this
ring stands in for the domain walls. Every face touching a `Solid` cell has
its normal velocity forced to zero, both before and after the pressure
solve. Practical effect: the fluid-carrying region is effectively inset by
one `cellSize` from the nominal `[0,domainWidth]x[0,domainHeight]` box on
every side. At the default `cellSize = 0.025` on a 2m-wide domain that's a
1.25%-per-side inset — invisible in practice. If a slider pushes
`cellSize` up toward, say, `0.15` on the same domain, the inset (~7.5% per
side) becomes visually noticeable as fluid never quite reaching the walls.
Particle positions themselves are still clamped to the full nominal domain
(`clampToDomain()`, belt-and-suspenders per the master plan) independent of
this grid inset, so particles can sit inside the nominal
`[0,domainWidth]` box slightly beyond the "true" grid-solid wall without
anything breaking — they just won't be pushed there by the fluid itself.

## `Scenario` — constructing and resetting

**Unchanged from SPH — the exact same struct, exact same fields, reused
as-is.** See `docs/sim-api.md`'s `Scenario` section for the full field
table and the built-in presets (`Module.makeDamBreakScenario()`,
`Module.makeBlockDropScenario()`, `Module.makeTwoColumnCollisionScenario()`)
— all three work unmodified for `FlipSimulation.reset()`, exactly as for
SPH's `Simulation.reset()`. There is no FLIP-specific scenario field (no
"cellSize" in `Scenario` — that lives on `FlipParams` instead, since it's a
solver property, not an initial-condition property).

```ts
const s = Module.makeDamBreakScenario(); // same preset, works for either solver
sim.reset(s);
sim.getPositionsPtr(); // MUST re-fetch after reset()
sim.getVelocitiesPtr();
```

Note one interaction to be aware of: `Scenario.spacing` controls initial
particle seeding density for **both** solvers, but only SPH needs it kept
in sync with a mass/density parameter. For FLIP, `spacing` only affects how
many particles get seeded (denser spacing = more particles = smoother
grid coverage per cell, generally desirable, but no calibration formula is
required — just more particles = more P2G/G2P work per step).

## Boundary handling

Two layers, matching the master plan's "belt-and-suspenders" instruction:

1. **Grid-level (primary):** the one-cell Solid boundary ring (see above)
   zeroes wall-normal face velocities both before computing divergence and
   again after the pressure solve, so fluid physically cannot flow through
   a wall.
2. **Particle-level (safety net):** after advection, any particle whose
   position ends up outside `[0,domainWidth]x[0,domainHeight]` is clamped
   back onto the boundary, and the velocity component that would have
   carried it further out is zeroed (not reflected/bounced — unlike SPH's
   damped-bounce boundary, FLIP's particle-level clamp is a rare-case
   safety net, not the primary boundary mechanism, so there's no
   "boundaryDamping"-style tunable for it).

## Solver notes (background, not part of the JS contract but useful context)

Standard Bridson-style PIC/FLIP on a 2D MAC (staggered) grid. Per-step
pipeline (see `core/include/fluids/solvers/flip/FlipSolver.h` for the full
breakdown):

1. **P2G:** bilinear-weighted scatter of particle velocities onto grid
   faces (`u` on vertical faces, `v` on horizontal faces), normalized by
   accumulated weight per face.
2. **Classify cells:** `Fluid` (>=1 particle), `Air` (none), `Solid`
   (boundary ring) — redone from scratch every step.
3. **Gravity:** applied directly to `v` (vertical) face velocities.
4. **Pressure projection:** Jacobi iteration (**60 fixed iterations**, not
   a full PCG solve — chosen deliberately for Phase 2 per the master
   plan's "correctness over performance" guidance; a Jacobi solve is
   simpler to reason about and debug than a preconditioned CG, at the cost
   of needing more iterations to converge as fully). `Air` cells act as an
   implicit zero-pressure Dirichlet boundary; `Solid`-adjacent face
   velocities are explicitly zeroed both before computing divergence
   (so a wall never looks like a source/sink) and again after the solve
   (belt-and-suspenders, per the master plan's explicit warning that this
   step is easy to skip).
5. **Velocity extrapolation:** a 3-cell-deep flood-fill/BFS-style average
   fills `Air`-cell velocities adjacent to the fluid region, run on
   **both** the post-projection grid and the pre-projection P2G snapshot
   (same validity mask for both) — see the "Deviation" note below.
6. **G2P with PIC/FLIP blend:** per particle, `v_pic` = bilinear sample of
   the extrapolated post-projection grid; `v_flip` = particle's existing
   velocity + (extrapolated new grid velocity − extrapolated old snapshot)
   sampled at the particle's position; final velocity =
   `flipRatio * v_flip + (1 - flipRatio) * v_pic`.
7. **Advection:** forward Euler.
8. **Boundary clamp:** see above.

### Deviations from the literal task description (and why)

- **Extrapolating the pre-projection snapshot too, not just the
  post-projection field.** The task describes extrapolating only the
  post-projection grid before G2P. Implemented that, but additionally
  extrapolate the pre-projection (`uOld`/`vOld`, captured right after P2G)
  snapshot with the identical validity mask before computing the FLIP
  delta (`v_grid_new - v_grid_old`). Reasoning: without this, a particle
  sampling a cell that had zero P2G weight this step (e.g. a newly-wetted
  cell right at the advancing free surface) would see its "old" value as a
  hard zero (never extrapolated), making `v_grid_new - v_grid_old`
  spuriously equal to the *entire* new value rather than the actual
  physical change — which would spike the FLIP contribution exactly at the
  free surface, the opposite of what the extrapolation step is meant to
  prevent. Extrapolating both snapshots with the same mask keeps the delta
  well-defined everywhere the new field is well-defined, at negligible
  extra cost (grid sizes are small; two extra flood-fill passes per step).
- **Jacobi over CG:** used Jacobi with a fixed 60-iteration count, per the
  plan's explicitly offered option ("Jacobi iteration (simplest to get
  correct and debug)... CG is acceptable too... your call, but correctness
  over performance for Phase 2"). Verified via
  `tests/core/test_flip_solver.cpp`'s divergence test that 60 iterations
  converges interior fluid-cell divergence to well under `0.1` (in
  practice much tighter) for grids in the size range the default
  `cellSize` produces — see that test for the exact tolerance used.
- **`pressureIterations` not exposed as a `FlipParams` field:** kept as an
  internal constant (`kPressureIterations = 60` in `FlipSolver.cpp`)
  rather than a tunable, per the plan's explicit "a fixed internal
  constant is also fine and arguably better for Phase 2 simplicity" — no
  concrete need for frontend tuning surfaced during implementation.
- **P2G/G2P weighting:** used the standard bilinear/hat-function scheme
  described in the plan (not quadratic B-splines) — no deviation here,
  called out only because it's the one part of the pipeline where a
  different scheme would have been a plausible alternate choice.

None of these change the JS-facing contract above — they're internal
solver details, listed here (and in code comments at the relevant call
sites in `FlipSolver.cpp`) for whoever picks up Phase 3 (MPM) and wants to
know what Phase 2 actually did versus what the roadmap sketched.
