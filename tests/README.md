# tests/

Native Catch2 unit tests for `fluids_core` (Phase 1, SPH). Built and run
without Emscripten — see the top-level `README.md`'s "Native build + tests"
section, or directly:

```powershell
cmake -S . -B build\native
cmake --build build\native --config Release
ctest --test-dir build\native -C Release --output-on-failure
```

Catch2 is fetched automatically via CMake `FetchContent` (see
`tests/CMakeLists.txt`) — no separate install needed, just network access on
first configure.

## Coverage (`tests/core/`)

- `test_vec2.cpp` — `Vec2` arithmetic (add/sub/scale/dot/length).
- `test_kernels.cpp` — Poly6/Spiky/viscosity kernel sanity at r=0, r=h, r>h
  (maximal/zero/monotonic behavior).
- `test_uniform_grid.cpp` — `UniformGrid` neighbor queries matched against
  brute-force search for random point sets, plus edge cases (empty grid,
  negative coordinates, self-inclusion at r=0).
- `test_solver.cpp` — solver stability smoke test (no NaN/Inf, particles
  stay within domain bounds over several seconds) and a Dam Break scenario
  test asserting the column visibly collapses (average height drops) and
  spreads (max x increases) without exploding (bounded velocities).
