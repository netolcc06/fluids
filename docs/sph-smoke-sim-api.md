# Sim API contract (Phase 4c — SPH-flavored Smoke)

This is the JS-facing contract exposed by `wasm/sph_smoke_bindings.cpp`
(compiled from `core/`) for the Phase 4c SPH smoke solver — the third and
final Smoke variant, completing Phase 4 (Smoke added to all three solver
families: FLIP/grid, MPM/APIC, and now SPH/kernel-particle). It's written for
the frontend agent, who works only against this document and the compiled
`web/src/wasm-gen/fluids.{js,wasm}` — not the C++ source.

`SphSmokeSimulation` is a **sixth, independent solver class**, added
alongside the existing Phase 1 `Simulation` (SPH water), Phase 2
`FlipSimulation` (FLIP water), Phase 3 `MpmSimulation` (MPM water), Phase 4a
`FlipSmokeSimulation` (grid smoke), and Phase 4b `MpmSmokeSimulation` (APIC
particle smoke) classes — it does not replace any of them and shares no C++
inheritance relationship with them. All six classes are exported from the
same compiled module, so a page can construct any or all of them.

**This is a genuinely different smoke solver from both prior ones, not a
third copy of an existing algorithm.** `FlipSmokeSimulation` is a pure
Eulerian (grid-only) field with no particles. `MpmSmokeSimulation` carries
particles but routes all inter-particle interaction through an MPM node
grid (APIC P2G/G2P transfer). `SphSmokeSimulation` instead reuses SPH
water's actual kernel/neighbor-search machinery — a `UniformGrid` spatial
hash purely for neighbor lookups (not a velocity-carrying grid the way
FLIP/MPM's are) plus the `poly6` and `viscosityLaplacian` kernels from
`fluids/solvers/sph/Kernels.h`, completely unmodified — so every particle's
motion is influenced directly by kernel-weighted neighbor averaging, with no
grid mediating it at all. The result is a third, again visibly different
character: a buoyant particle gas whose only coherence comes from direct
local viscosity diffusion, no pressure force, no grid.

Load the module the same way as Phases 0-4b:

```ts
import createFluidsModule from "./wasm-gen/fluids.js";
const Module = await createFluidsModule();
const sim = new Module.SphSmokeSimulation();   // seeds a default Plume scenario immediately
```

## Coordinate convention

Same as every other solver in this project — not solver-specific:
- Origin `(0, 0)` is the domain's **bottom-left** corner.
- `+x` points right, `+y` points **up**.
- The simulation box is `[0, domainWidth] x [0, domainHeight]` (see
  `SmokeScenario` below — reused unchanged from Phase 4a/4b).
- Units are meters, seconds, EXCEPT temperature, which is a **dimensionless
  relative scale**, same convention as `FlipSmokeParams`/`MpmSmokeParams`.
- **There is no `gravity` parameter.** Buoyancy — driven by each particle's
  own temperature above ambient — is the sole driving force, same as the
  other two smoke variants.

If your renderer uses screen-space (+y down), flip y when mapping sim
coordinates to clip/pixel space — same as every other solver, a rendering
concern only.

## `SphSmokeSimulation` class

```ts
const sim = new Module.SphSmokeSimulation(); // seeds a default Plume scenario immediately
sim.step(dt);                                // dt in seconds. See "stable substep" below.
sim.reset(scenario);                         // scenario: a plain object matching SmokeScenario's fields (reused unchanged)
const n = sim.getParticleCount();            // number — changes every step, see "pointer stability" below
const posPtr = sim.getPositionsPtr();        // number (byte offset into the WASM heap)
const velPtr = sim.getVelocitiesPtr();       // number (byte offset into the WASM heap)
const densPtr = sim.getDensitiesPtr();       // number (byte offset into the WASM heap)
const tempPtr = sim.getTemperaturesPtr();    // number (byte offset into the WASM heap)
sim.params;                                  // SphSmokeParams — see the property gotcha below
```

This is **exactly the same method/field shape as `MpmSmokeSimulation`**
(`step`/`reset`/`getParticleCount`/`getPositionsPtr`/`getVelocitiesPtr`/
`getDensitiesPtr`/`getTemperaturesPtr`/`params`) — a deliberate consistency
choice: all three smoke solvers should be interchangeable from the
frontend's point of view (same shape of calls, different `params` field
sets), even though their internal transfer schemes are unrelated.

### Reading the particle buffers (zero-copy typed arrays)

Positions/velocities are contiguous `float32` arrays laid out
`[x0, y0, x1, y1, ...]` (length = `getParticleCount() * 2`), identical
layout to every other solver in this project. Densities/temperatures are
contiguous `float32` arrays of length `getParticleCount()` **(not `* 2` —
these are per-particle scalars, not vec2s)**:

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
seeded with density in `[0.85, 1.0]` (same jitter range as
`MpmSmokeSimulation`), and density only ever decays multiplicatively
afterward (dissipation), so it never exceeds its seeded value and never
goes negative. Treat directly as a per-particle alpha/opacity value for a
sprite/point renderer, same suggestion as the other two smoke variants.

**IMPORTANT — this is a *different* "density" than the SPH kernel-density
estimate this solver computes internally.** `getDensitiesPtr()` exposes the
per-particle **visual smoke-density state variable** (seeded near 1,
dissipates over time, drives culling) — the same semantic role
`MpmSmokeSimulation.getDensitiesPtr()`/`FlipSmokeSimulation`'s density field
play. It is **not** the kernel-weighted local-packing density
(`sum_j mass * poly6(r_ij, h)`) the solver separately computes each step
purely to normalize its internal viscosity force. That kernel-density
estimate is not exposed via Embind at all — see "Pressure/EOS decision"
below for why the two are kept conceptually and numerically separate.

**Temperature range:** not clamped, but bounded in practice the same way as
the other two smoke variants — seeded near `sourceTemperature` and relaxing
toward `ambientTemperature` every step, so with the defaults
(`ambientTemperature = 0`, `sourceTemperature = 1`) it stays roughly within
`[0, 1]`.

### Buffer stability contract — identical to `MpmSmokeSimulation`'s

Same fixed-capacity/pointer-stability contract as `MpmSmokeSimulation` (see
`docs/mpm-smoke-sim-api.md` for the full writeup this section mirrors):

- **`getParticleCount()` legitimately changes value every `step()` call.**
  Particles are continuously emitted inside the source rectangle and culled
  once their density fades below a small threshold. Don't treat a changing
  count as an error.
- **`getPositionsPtr()`/`getVelocitiesPtr()`/`getDensitiesPtr()`/
  `getTemperaturesPtr()` do NOT change address across `step()` calls**,
  despite the count changing. Every per-particle buffer is preallocated at
  a fixed capacity (`params.maxParticles`) exactly once in `reset()`, and
  only the first `getParticleCount()` slots are meaningful at any given
  moment. Verified empirically (native Catch2 test driving the compiled
  solver: fetch all four pointers once, `step()` 600 times while confirming
  the particle count actually changes over that run, re-check all four
  pointers are still bit-identical to their first reading — see
  `tests/core/test_sph_smoke_solver.cpp`'s `[sph_smoke][pointer_stability]`
  test).
- **Practical consequence for the frontend:** identical to
  `MpmSmokeSimulation` — fetch the four pointers once right after `reset()`,
  build views over the full capacity (`params.maxParticles`) or re-slice a
  fresh `Float32Array` view each frame using the *same* pointer and the
  current `getParticleCount()`. You do NOT need to call the pointer getters
  again just because the count changed — only after an actual `reset()`
  call.
- **After `reset()`:** buffer capacity can change (`params.maxParticles` is
  only applied at `reset()` time). **Always call all four pointer getters
  again immediately after every `reset()`** and rebuild your views.
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

`SphSmokeParams` fields are read fresh by `step()` every call — a param
change takes effect on the very next step — **except `maxParticles`**,
which only takes effect on the next `reset()` call (buffer (re)allocation
only ever happens there). Note this is a shorter exception list than
`MpmSmokeParams`' (which also gates `cellSize` behind `reset()`) —
`smoothingRadius` here does NOT require a `reset()`; see the field table
below for why.

## `SphSmokeParams` fields

| Field | Default | Units | Meaning |
|---|---|---|---|
| `buoyancy` | `4.0` | (m/s²) per unit temperature above ambient | Applied directly to each particle's vertical acceleration every step: `accel.y += buoyancy * (particle.temperature - ambientTemperature)`. Same formula shape and default as the other two smoke variants — smoke's entire driving force; there is no gravity parameter. |
| `ambientTemperature` | `0.0` | relative scale | Temperature of undisturbed air. Zero point buoyancy measures against, and the value each particle's temperature relaxes toward. |
| `sourceTemperature` | `1.0` | relative scale | Temperature newly-emitted particles are seeded near (jittered to `[0.85, 1.0] * sourceTemperature`). Combined with the default buoyancy, this is what makes emitted particles rise. |
| `densityDissipation` | `0.1` | fraction/second | Each step, `density *= (1 - densityDissipation * dt)`. Same formula/default as the other two smoke variants. Also controls how long a particle survives before falling below the culling threshold (`0.01`) and being removed. |
| `temperatureDissipation` | `0.5` | fraction/second | Each step, `T += (ambientTemperature - T) * min(1, temperatureDissipation * dt)`. Same formula/default as the other two smoke variants. |
| `smoothingRadius` | `0.09` | meters | SPH kernel support radius `h` — this solver's analog of `cellSize` in the grid/APIC solvers. Sets both the `UniformGrid` neighbor-search cell size and the `poly6`/`viscosityLaplacian` kernel support. **Notably larger than water SPH's default (0.045m)** — see "Why `smoothingRadius` is larger than water's" below. Unlike `cellSize` elsewhere in this project, changing this does **not** require a `reset()` — the neighbor-search grid is rebuilt from scratch every `step()` regardless (same as water SPH), so a live change is picked up on the very next step. |
| `viscosity` | `6.0` | — | Kernel-weighted Laplacian viscosity coefficient (reuses `fluids::sph::viscosityLaplacian` unchanged from water SPH). This is the **sole source of inter-particle coherence** in this solver (there is no pressure force — see "Pressure/EOS decision" below), so it is doing significantly more visual work than water SPH's `viscosity` (default 0.08) — see "Why `viscosity` is much higher than water's" below for the empirical justification. `0` degenerates to ballistic buoyant particles with zero inter-particle coupling. |
| `maxParticles` | `4000` | count | Fixed capacity of the particle buffer, (re)allocated only at `reset()`. Same default and role as `MpmSmokeParams::maxParticles` — comfortably above this solver's empirically measured steady-state population at default settings (~1800, see "Emission and steady-state population" below). |

There is no `gasStiffness`/`restDensity` the way `sph::SPHParams` (water)
has them — see "Pressure/EOS decision" immediately below. There is no
`gravity` (buoyancy is the sole driving force, per every other smoke
variant's precedent). There is no `particleMass`-equivalent exposed either:
per-particle mass for the internal kernel-density estimate is a fixed
internal constant (`kParticleMass = 1.0`, see `SphSmokeSolver.cpp`), not a
tunable — see the "Why `particleMass` isn't exposed" subsection below for
the derivation of why its absolute value doesn't matter.

### Pressure/EOS decision — omitted entirely, and what was observed

**This solver has no pressure force and no equation of state at all.** The
only force terms are buoyancy (a direct per-particle body force) and the
kernel-weighted Laplacian viscosity described above. This mirrors
`MpmSmokeSolver`'s decision to drop water's stress/pressure term entirely,
generalized to SPH's own pressure mechanism (Tait EOS + Spiky-kernel
gradient force) instead of MPM's (stress tensor in the P2G affine term).

**Why, reasoned through before implementing (per the task's explicit ask to
investigate empirically rather than assume):**

Water's WCSPH calibrates a stiff Tait EOS (`gasStiffness=200`, exponent 7)
against a **roughly constant** local particle density: particles are
seeded rest-packed at a fixed `spacing`, and `particleMass` is calibrated
(`particleMass ≈ restDensity * spacing²`, see `docs/sim-api.md`) so a
rest-packed neighborhood's kernel-summed density lands right at
`restDensity`. The EOS then produces near-zero pressure at rest and pushes
back hard only when compressed — a well-behaved, self-correcting regime
because the density estimate always sits in a narrow band around a known
target.

Smoke's particle population has no such target. Particles are
continuously emitted into a small source rectangle, then spread out by
buoyancy and (previously) their own pressure as they rise — their local
kernel-density estimate is not tracking any "rest" state, it's just
whatever the current, ever-changing local packing happens to be. Feeding
that into a stiff exponent-7 EOS the way water does would mean: near the
(comparatively dense) emission source, particles are packed close together
by construction (spawned at random points inside a small ~0.045 m² box at
40/s), producing a locally high density estimate relative to any
arbitrarily-chosen `restDensity`, and thus artificially strong pressure
right at the point where the plume needs to look like a soft continuous
source, not an exploding cluster — exactly the "sparse/thinning population
misbehaving against a water-shaped EOS" risk the task called out.

**What was actually checked, concretely, before finalizing this decision:**

1. **The self-inclusion property of `UniformGrid::queryNeighbors` rules out
   one entire failure mode.** Every particle's own position is included in
   its own neighbor query at `r=0` (confirmed directly —
   `tests/core/test_uniform_grid.cpp`'s "query includes the point itself at
   r=0" test, and re-verified for this solver's own grid usage). This means
   the kernel-density estimate `sum_j mass * poly6(r_ij, h)` is **always
   at least `mass * poly6(0, h) > 0`**, strictly, for every active
   particle, no matter how isolated it is — there is no way to reach a
   near-zero density estimate the way a naive "smoke is sparse" intuition
   might suggest. This matters for the viscosity term (see below), which
   *does* divide by this estimate, and rules out a divide-by-near-zero
   blowup there without needing any additional floor/clamp.
2. **The pressure-free result was built and tested directly** (native
   Catch2 suite, `tests/core/test_sph_smoke_solver.cpp`), not merely
   assumed safe from (1) above. Every stability/regression test passes:
   no NaN/Inf over a 90-simulated-second run, no persistent high-speed
   outliers (windowed peak-RMS-speed regression test, see "Verification
   against the water SPH bug" below), particles stay within domain bounds,
   and the density-weighted centroid visibly rises and plateaus (see the
   measured table below) — i.e. it looks like rising, coherent smoke, not
   a scattered ballistic dust cloud, using **viscosity alone** for
   coherence.
3. **A soft/linear pressure alternative was considered analytically but not
   implemented**, once (2) confirmed the pressure-free result already met
   every stability and coherence criterion with no observed shortfall.
   Adding a pressure term back — even a soft, linear-in-density-excess one
   — would reintroduce an EOS calibration surface (some notion of "rest"
   density for smoke, which has no natural definition here, see above)
   purely to solve a problem the tests show doesn't exist. Per the task's
   own framing ("investigate empirically... before deciding you need a
   pressure term at all"), the empirical answer was: no.

**Consequence for the water SPH near-singularity bug specifically**
(`fluids::sph::Kernels.h`'s `kMinEffectiveRFraction` fix,
`docs/sim-api.md`'s "Minimum effective kernel interaction distance"
section): that bug's mechanism was specifically the Tait EOS's exponent-7
amplification of an inflated `poly6` density estimate near `r=0`, funneled
through `spikyGradient`'s pressure force. This solver calls **neither**
`spikyGradient` **nor any EOS** — `poly6` is used only for the
viscosity-normalizing density estimate (a smooth, bounded, non-exponentiated
quantity), and `viscosityLaplacian` is itself a plain linear-in-`r` function
with a small finite maximum at `r=0` (nothing resembling an exponent-7
amplifier). There is structurally no mechanism here for a near-coincident
pair to produce a disproportionate force the way water's pressure force
could. This is verified directly, not just argued, by
`tests/core/test_sph_smoke_solver.cpp`'s
`[sph_smoke][stability][regression]` windowed-peak-RMS-speed test (adapted
from `test_solver.cpp`'s water regression test — see "Verification against
the water SPH bug" below for why the adaptation was necessary).

### Why the internal viscosity-density normalization never divides by
near-zero (the sparsity concern generalized beyond pressure)

The task specifically also flagged risk beyond pressure: "a `density`
scalar that's a *visual* smoke-density state variable... not necessarily
tied 1:1 to local SPH particle-packing density," which could make **any**
kernel-density-normalized force term (not just an EOS) misbehave in sparse
regions. This solver's viscosity term does divide by the kernel-density
estimate (`kParticleMass * lap(r,h) / rho_j`, then again by `rho_i`,
mirroring water's own viscosity formula shape) — but per the self-inclusion
argument above, that estimate is always bounded below by
`kParticleMass * poly6(0, h)`, a fixed positive constant independent of how
sparse the surrounding neighborhood is. No additional floor/clamp was
needed (unlike, say, `Kernels.h`'s `kMinEffectiveRFraction` fix, which
exists specifically because poly6's near-`r=0` plateau feeds an
*exponentiated* EOS — there is no exponentiation anywhere in this solver's
viscosity term for a similar near-coincidence effect to get amplified
into).

### Why `particleMass` isn't exposed

Per-particle mass for the kernel-density estimate is a fixed internal
constant (`kParticleMass = 1.0`, `SphSmokeSolver.cpp`), same pattern as
`MpmSmokeParams` not exposing a particle mass. The derivation differs
slightly from MPM smoke's (where mass cancels completely out of a
momentum-weighted average): here, the viscosity acceleration works out to
`viscosity * (1/rho_i) * sum_j (v_j - v_i) * mass * lap(r_ij,h) / rho_j`,
and since `rho_i = mass * D_i` / `rho_j = mass * D_j` (where `D` is the
mass-independent kernel sum), one factor of `mass` cancels between the
numerator's `mass * lap` and `rho_j`'s own `mass` factor, but the outer
`1/rho_i = 1/(mass * D_i)` leaves an overall `1/mass` scaling — i.e.
`kParticleMass`'s absolute value is **not fully arbitrary** the way MPM
smoke's is, but it *is* fully redundant with the live-tunable `viscosity`
parameter (any change to `kParticleMass` is exactly equivalent to an
inverse rescaling of `viscosity`), so there's nothing a separate exposed
parameter would add — same practical conclusion (fixed internal constant,
tuned against the exposed `viscosity` default) via a slightly different
argument.

### Why `smoothingRadius` is larger than water's

Water SPH's default `smoothingRadius = 0.045m` is tuned against a densely
rest-packed particle bed (`spacing = 0.035m`, so neighbors sit close to
`r ≈ h`, ~10-15 neighbors typical). This solver's particles are much
sparser: at the default emission rate (~40 particles/second, see below)
spread across a domain up to ~2.4 m² once risen, the steady-state areal
density is roughly two orders of magnitude sparser than water's. A
water-sized `smoothingRadius` would leave most particles with zero or one
neighbor, making the viscosity term nearly inert. `0.09m` (2x water's
default) was chosen empirically — confirmed via
`tests/core/test_sph_smoke_solver.cpp`'s `[sph_smoke][viscosity]` test,
which measures each particle's velocity residual against its own
kernel-neighborhood's mean velocity (a direct "how locally coherent is the
motion" signal): at the default `smoothingRadius`/`viscosity`, this
residual averages **~0.011 m/s**, vs. **~0.045 m/s** with `viscosity = 0`
(same `smoothingRadius`) — roughly a 4x tightening of local velocity
coherence, a clearly measurable, non-trivial effect, not a knob that merely
looks like it does something.

### Why `viscosity` is much higher than water's

With no pressure force to lend the simulation any of its own structure,
viscosity here is solely responsible for making a rising particle cloud
read as one coherent puff of smoke rather than a scatter of independent
ballistic points. `viscosity = 6.0` (75x water's `0.08` default) was tuned
empirically against the `smoothingRadius = 0.09` default until the
neighbor-velocity-residual measurement above showed a clear, stable
coherence effect without over-damping the plume's natural rise/spread
(over-damping would show up as the centroid height plateauing too low or
too slowly relative to the other two smoke variants' comparable
measurements — it doesn't; see the table below).

## `SmokeScenario` — reused unchanged, no new scenario struct needed

Same as Phase 4b: **this phase needed no new scenario struct or preset.**
`SmokeScenario` (the same struct all three smoke solvers use, see
`docs/flip-smoke-sim-api.md`) already fits. `Module.makePlumeScenario()`
works unmodified for `SphSmokeSimulation.reset()`:

```ts
const s = Module.makePlumeScenario();
sim.reset(s);
sim.getPositionsPtr();     // MUST re-fetch after reset() (and the other 3 getters)
```

`SmokeScenario` is registered as an Embind `value_object` by
`wasm/flip_smoke_bindings.cpp` and is **not** re-registered by
`wasm/sph_smoke_bindings.cpp` (re-registering the same C++ type under a
second `EMSCRIPTEN_BINDINGS` block is an error) — same share-the-
registration pattern `wasm/mpm_smoke_bindings.cpp` and `wasm/mpm_bindings.cpp`
already established. `Module.makePlumeScenario()` is likewise only
registered once and works for all three smoke solvers.

### Emission — same translation as MPM smoke, reused deliberately unchanged

`SmokeScenario.emissionRate` is translated directly into a particle spawn
rate, identical formula and identical constant to `MpmSmokeSolver`:

```
particlesPerSecond = emissionRate * 20.0   // kParticlesPerSecondPerEmissionUnit, SphSmokeSolver.cpp
```

implemented with the same fractional-accumulator pattern (exact spawn rate
regardless of caller `dt`, not "at most one particle per step") and the
same emission-credit-banking cap (`kMaxBankedEmission = 50`) for the same
"don't recycle, don't burst unboundedly" reasoning as `MpmSmokeSolver`. Each
spawned particle gets a uniformly-random position within the source
rectangle, a small `±0.05 m/s` velocity jitter, density jittered to
`[0.85, 1.0]`, and temperature jittered to `[0.85, 1.0] * sourceTemperature`
— all identical to `MpmSmokeSolver`'s emission.

**This constant was deliberately reused unchanged rather than re-derived**,
for two reasons: (1) it keeps the two particle-based smoke solvers'
particle counts directly comparable for the same `SmokeScenario` (useful
for a frontend that lets users switch solver families live and expects a
similar visual "amount of smoke"), and (2) empirical testing (see the table
below) confirmed it gives this solver's `smoothingRadius` enough local
particle density for the viscosity kernel to have a clearly measurable
effect (see "Why `smoothingRadius` is larger than water's" above) — there
was no observed need for a different rate.

### Emission and steady-state population (measured)

Native diagnostic run of the default `makePlumeScenario()` (same style of
table as `docs/mpm-smoke-sim-api.md`'s, gathered the same way — a manual
instrumented run, not a committed test):

| Simulated time | Active particle count | Density-weighted centroid height | Max particle speed | RMS speed |
|---|---|---|---|---|
| 1s | 40 | 0.64 m | 2.87 m/s | 1.78 m/s |
| 5s | 200 | 1.67 m | 2.88 m/s | 0.83 m/s |
| 10s | 400 | 1.80 m | 3.19 m/s | 0.63 m/s |
| 20s | 800 | 1.85 m | 3.04 m/s | 0.49 m/s |
| 30s | 1200 | 1.86 m | 2.99 m/s | 0.36 m/s |
| 45s | 1795 | 1.87 m | 3.12 m/s | 0.32 m/s |
| 60s | 1804 | 1.87 m | 3.08 m/s | 0.30 m/s |
| 90s | 1808 | 1.87 m | 3.09 m/s | 0.26 m/s |

The population grows (exactly 40/s, same as MPM smoke's identical emission
rate) until natural culling balances new emissions, reaching a **steady
state around ~1800 particles by ~45 simulated seconds** — essentially
identical population dynamics to `MpmSmokeSimulation` under the same
scenario/emission settings (expected, since both reuse the same
translation constant), comfortably under the default `maxParticles = 4000`.
The centroid height rises from the source (`sourceY + sourceHeight =
0.15m`) up to ~1.87m and plateaus — `makePlumeScenario()`'s domain is 2.0m
tall, so the plume is rising until constrained by the top boundary, then
pooling, the same physically-sensible closed-top-domain result the other
two smoke variants show. Notably, **RMS speed steadily *decreases* over the
run** (1.78 → 0.26 m/s) even as the population keeps growing toward steady
state — the opposite of what a persistent instability would look like
(RMS speed staying elevated or growing) — consistent with the system
settling into a calm, buoyancy-vs-viscosity-vs-dissipation equilibrium
rather than accumulating energy.

### Culling — identical design to MPM smoke, same reasoning

Each step, after dissipation, every active particle whose `density` has
decayed below `0.01` (`kDensityCullThreshold`, same value as
`MpmSmokeSolver`) is removed by swapping it with the last active particle
and shrinking the active count by one (order among active particles is not
preserved or meaningful). Without this, active count would only ever grow
until pinned at `maxParticles`, silently stopping all future emission.

**When at capacity, new emission is skipped for the remainder of that tick**
rather than recycling the oldest/faintest particle — identical reasoning to
`MpmSmokeSolver::emit` (avoids a scan, avoids a visible teleport; bounded
by the same `kMaxBankedEmission = 50` banking cap). Given the measured
~1800-particle steady state, well under the default 4000 capacity, this
path is a safety backstop, not something normal operation with default
parameters engages.

**This solver's emission/culling design is identical in shape to
`MpmSmokeSolver`'s** — same fixed-capacity buffer, same swap-and-shrink
culling, same skip-when-full-at-capacity emission policy, same banking cap,
same jitter ranges, same RNG approach (fixed-seed `std::mt19937`, reseeded
at `reset()`, for reproducible tests). The only genuine difference is the
emission rate's *interpretation once spawned* doesn't change (same particle
seeding), only what happens to particles *after* spawning (SPH kernel
forces instead of APIC P2G/G2P) differs.

## Boundary handling

**Particle-level only** — this solver has no grid-level boundary step,
unlike both other smoke variants (`FlipSmokeSolver`/`MpmSmokeSolver` each
have a margin-based node/face velocity clamp their respective grids apply
*before* the particle-level clamp). This is not an oversight: this
solver's `UniformGrid` is purely a neighbor-search spatial hash, rebuilt
fresh from the current active particle positions every step — it carries no
velocity/mass state of its own the way a MAC grid or MPM node grid does, so
there is nothing analogous to "clamp the grid's outward-facing velocity" to
apply. This matches **water SPH's own boundary approach** exactly (see
`docs/sim-api.md`'s "Boundary handling" section) — water SPH also has only
a single, particle-level boundary layer, for the identical reason (no
velocity-carrying grid exists there either).

Unlike water SPH's damped-bounce (reflect + scale by `boundaryDamping`),
this solver **zeroes the outward velocity component instead of reflecting
it** — same "just stop at the wall, don't bounce" convention the other two
smoke variants use (there is no `boundaryDamping`-equivalent parameter
here, matching their precedent: a gas venting into a wall shouldn't bounce
back elastically the way a water droplet plausibly could).

## The stable substep — measured directly for this solver

**Recommended caller `dt`: `1/60`**, same recommendation as both other
smoke variants and a plain 60fps game loop. Checked directly
(`tests/core/test_sph_smoke_solver.cpp`'s `[sph_smoke][stability][substep]`
test): **plain fixed-substep stepping (no internal CFL-adaptive
subdivision) is stable across every `dt` tested, `1/30` down to `1/240`**,
over a 3-simulated-second `makePlumeScenario()` run each — no NaN/Inf, and
particle speeds stayed a small fraction of a generously loose 50 m/s sanity
bound at every `dt`.

This finding's underlying reason is the closest of the three smoke variants
to `MpmSmokeSolver`'s: **there is no stiff feedback loop for large `dt` to
destabilize, because there is no pressure/EOS term at all** (see "Pressure/
EOS decision" above) — the one mechanism that forces water SPH and MPM
water's adaptive substepping (a stiff EOS/stress response amplifying a
velocity or volume-ratio perturbation into a growing spike the next step)
is structurally absent here, for the same reason it's absent in MPM smoke.
The only force terms — buoyancy (a small, smoothly-varying acceleration:
`4.0 * 1.0 = 4 m/s²` at the hottest) and kernel-weighted viscosity (bounded,
non-exponentiated, see above) — have no mechanism to compound a
perturbation across steps. Given this, `SphSmokeSolver::step()` does
**not** internally subdivide `dt` — it runs exactly one
emit/grid-build/density/forces/integrate/dissipate/cull/clamp pass per
call, after the same defensive `dt` clamp to `1/30` every solver in this
project applies.

## Solver notes (background, not part of the JS contract but useful context)

Reuses SPH water's actual `UniformGrid` (`fluids::UniformGrid`) and kernel
functions (`fluids::sph::poly6`, `fluids::sph::viscosityLaplacian` from
`Kernels.h`) completely unmodified — see
`core/include/fluids/solvers/sph_smoke/SphSmokeSolver.h` for the full
class-level rationale. Per-particle state: position, velocity, `density`
(~[0,1], the visual smoke-density state variable) and `temperature` (same
semantics as the other two smoke variants). There is deliberately **no**
APIC-style affine matrix (`C`) the way MPM smoke has — SPH has no analogous
per-particle transfer state to carry.

Per-step pipeline (see `SphSmokeSolver.cpp` for the full derivation):

1. **Emit new particles** into the active buffer, within `SmokeScenario`'s
   source rectangle (see "Emission" above).
2. **Copy the active position prefix into a scratch buffer** and build a
   fresh `UniformGrid` over it, keyed by `params.smoothingRadius` — a
   necessary extra step this solver has that neither other smoke variant
   needs: because particles live in a **fixed-capacity** buffer where only
   the first `activeCount` entries are live (see the pointer-stability
   contract above), the neighbor-search structure cannot be built directly
   over the raw buffer (it would see stale/inactive entries beyond
   `activeCount`) the way water SPH's fixed-size, non-culled particle array
   allows. The scratch copy is a plain prefix copy, so neighbor-query
   results index directly into the real buffers with no translation.
3. **Compute the internal SPH kernel-density estimate** (`poly6` sum, per
   particle) — used only to normalize the viscosity term in step 4, not
   exposed to JS (see "Pressure/EOS decision" above for why this is kept
   separate from the particle's own `density` state variable).
4. **Compute forces:** buoyancy (direct body force) plus kernel-weighted
   Laplacian viscosity, normalized by the density estimate from step 3 (see
   "Why the internal viscosity-density normalization never divides by
   near-zero" above for why this is safe).
5. **Integrate:** symplectic (semi-implicit) Euler, same scheme as water
   SPH: `v += a*dt; p += v*dt`.
6. **Dissipation:** `density *= (1 - densityDissipation*dt)`, `temperature
   += (ambientTemperature - temperature) * min(1, temperatureDissipation*dt)`
   — same formulas as the other two smoke variants.
7. **Cull faded particles**, then a particle-level domain boundary clamp
   (see "Boundary handling" above for why there is no grid-level layer
   here).

## Comparison across all three smoke solvers (for whoever reads all three docs)

| | `FlipSmokeSimulation` (Phase 4a) | `MpmSmokeSimulation` (Phase 4b) | `SphSmokeSimulation` (Phase 4c) |
|---|---|---|---|
| Representation | Pure Eulerian field — no particles at all | Particles, transferred through an MPM node grid (APIC P2G/G2P) | Particles, interacting directly via kernel-weighted neighbor pairs — no grid mediates it |
| Spatial structure | MAC grid (velocity lives only here) | MPM node grid (mass/momentum only, no scalar fields) | `UniformGrid` spatial hash for neighbor search ONLY — carries no velocity/mass state itself |
| Coherence mechanism | Semi-Lagrangian self-advection of the whole field | APIC affine transfer (`C` matrix) reconstructing local velocity gradients through the grid | Direct kernel-weighted Laplacian viscosity between neighbor pairs — the sole coherence source, since... |
| Pressure/EOS | N/A (no particles/stress concept) | Dropped entirely — no stress term in P2G | ...also dropped entirely — no EOS, no pressure force at all (see "Pressure/EOS decision" above) |
| Buoyancy application | Per grid-face, using face-averaged cell temperatures | Per-particle, directly on that particle's own velocity | Per-particle, directly on that particle's own acceleration (same shape as MPM smoke) |
| Emission | Continuous cell-based injection (density/temperature added directly to grid cells in the source region) | Continuous particle spawning, rate derived from `emissionRate * 20/s` | Continuous particle spawning, **same** `emissionRate * 20/s` translation, reused unchanged from MPM smoke |
| Particle buffer | N/A | Fixed-capacity (`maxParticles`), pointer-stable across `step()` | Fixed-capacity (`maxParticles`), pointer-stable across `step()` — identical contract |
| Stable substep | Any tested `dt` up to `1/30` (unconditionally stable by construction — semi-Lagrangian sampling can't overshoot) | Any tested `dt`, `1/30` to `1/240` (no stiff feedback loop to amplify) | Any tested `dt`, `1/30` to `1/240` (no stiff feedback loop, for the same underlying reason as MPM smoke) |
| Visual character | Smooth, continuous, blurry-at-low-resolution field | Puffy, particle-driven, grid-quantized motion | Puffy, particle-driven, motion driven by direct local averaging rather than any grid quantization — the most "raw N-body-with-local-smoothing" feeling of the three |

The throughline across all three: **buoyancy is universally the sole
driving force (no `gravity` field anywhere in Smoke), and every variant
that has a stress/pressure-analogous term available to it (MPM smoke, SPH
smoke) was found not to need one** — Phase 4's smoke material consistently
turns out to be better modeled as a "compressible buoyant gas with some
diffusive coherence mechanism" than as "the same incompressible-fluid
machinery as water, just with different forcing," across every one of the
three very different discretizations this project implements it with.

## Verification against the water SPH bug

Because this solver reuses the exact kernel functions responsible for the
water SPH near-coincidence bug (`poly6`, and — even though this solver
never calls it — the same `Kernels.h` that also contains `spikyGradient`),
this was checked directly rather than assumed safe from the "no EOS at all"
argument alone:
`tests/core/test_sph_smoke_solver.cpp`'s
`[sph_smoke][stability][regression]` test runs the default `makePlumeScenario()`
for 30 simulated seconds and tracks **windowed peak RMS speed** (the same
metric `test_solver.cpp`'s water regression test uses) in one-second
windows, asserting every window in the back half of the run stays below a
generous bound and that the back half isn't meaningfully more energetic
than the early (post-transient) run. This metric was chosen specifically
because, unlike water's fixed population, this solver's active particle
count changes every step (continuous emission/culling reuses particle
indices via swap-on-cull), so tracking "is this one particular particle
still calm" across steps isn't meaningful the way it is for water's fixed
population — RMS speed is already population-size-normalized, so it stays
a well-defined, comparable signal regardless of how many particles are
currently active. All windows passed comfortably under bound in testing
(see "Emission and steady-state population" above for the actual measured
RMS-speed trend, which decreases over the run rather than showing any
recurring spike pattern).

## Deviations from the task's literal description (and why)

- **No `gasStiffness`-equivalent field at all** — pressure/EOS omitted
  entirely rather than made "soft." See "Pressure/EOS decision" above for
  the full empirical reasoning (self-inclusion rules out the divide-by-
  near-zero concern structurally; the pressure-free result already passed
  every stability/coherence test; a soft pressure term would reintroduce an
  EOS calibration surface with no evidenced benefit).
- **No `restDensity`-equivalent field.** Nothing in this solver needs a
  target density to calibrate against — the internal kernel-density
  estimate is used only to normalize viscosity, and its absolute scale is
  fully redundant with the exposed `viscosity` parameter (see "Why
  `particleMass` isn't exposed" above).
- **Emission's `emissionRate`-to-particle-rate constant (`20.0`) reused
  verbatim from `MpmSmokeSolver`** rather than independently re-derived —
  see "Emission" above for why this was a deliberate consistency choice,
  validated empirically rather than assumed to transfer.
- **No CFL-adaptive substepping**, matching MPM smoke's finding (for a
  related but not identical reason — see "The stable substep" above) —
  checked empirically across `1/30` to `1/240`, not assumed.
- **`smoothingRadius` and `viscosity` defaults chosen well outside water
  SPH's range** (2x and 75x respectively) — both empirically tuned and
  validated via the neighbor-velocity-residual coherence measurement (see
  "Why `smoothingRadius`/`viscosity`..." above), not carried over from
  water's defaults, which were calibrated for a completely different
  (dense, rest-packed) particle regime.
- **No grid-level boundary handling layer**, unlike both other smoke
  variants — this matches water SPH's own precedent exactly (no
  velocity-carrying grid exists in either solver to apply such a layer to),
  not a simplification or omission relative to this solver's own
  architecture.
- **RNG is a fixed-seed `std::mt19937`, reseeded at every `reset()`**,
  identical approach and seed value (`1337`) to `MpmSmokeSolver`, for the
  same reproducibility reasoning.

None of these change the class-level design decisions the orchestrator
specified (particle-based, SPH-kernel-reusing, fixed-capacity buffer with
pointer stability, per-density-threshold culling, `SmokeScenario` reused
unchanged) — they're the specific implementation judgment calls the task
explicitly left open, recorded here for whoever next touches Phase 4 or
adds a fourth material/solver combination.
