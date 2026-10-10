# Fluids

2D fluid simulation sandbox (SPH -> FLIP -> MPM, eventually Water and Smoke
materials). Physics core is C++17 compiled to WebAssembly via Emscripten;
rendering is WebGL2 with a plain TypeScript frontend (Vite).

**Status**: three independent 2D water solvers are implemented and
switchable live from the UI — SPH (Weakly-Compressible SPH: Poly6/Spiky
kernels, Tait EOS), FLIP (MAC grid, Jacobi pressure projection, PIC/FLIP
blend), and MPM (MLS-MPM with APIC transfer, weakly-compressible EOS via a
tracked Jacobian). All three share the same `Scenario` seeding struct and
three built-in presets (Dam Break, Block Drop, Two-Column Collision). See
`docs/sim-api.md` (SPH), `docs/flip-sim-api.md` (FLIP), and
`docs/mpm-sim-api.md` (MPM) for the full JS-facing contracts. `tests/` has
a native Catch2 suite (21 tests: kernels, neighbor grid, grid transfer,
pressure projection, per-solver stability/settling checks) that builds and
runs without Emscripten. The frontend (`web/`) renders particles via
WebGL2 (colored by velocity) and exposes a Tweakpane panel: solver
switcher, scenario switcher + particle spacing, per-solver parameter
sliders, and play/pause/step/time-scale controls. Smoke material (for all
three solvers) is the next phase of work.

## Repository layout

```
core/     Portable C++17 simulation library (fluids_core). No Emscripten
          headers/macros — compiles with the system's default compiler.
wasm/     Embind bindings + CMake target that links fluids_core into a
          WASM module. Only configurable via emcmake (Emscripten toolchain).
web/      Vite + TypeScript frontend. Loads the generated ES6 glue module
          from web/src/wasm-gen/ and renders via WebGL2.
tests/    Catch2 native unit tests (kernels, neighbor grid, solver
          stability) for fluids_core — build/run without Emscripten.
scripts/  build_wasm.ps1: emcmake/emmake wrapper.
```

## Prerequisites

- **CMake** and **git** (already required to work in this repo at all).
- **Node.js** (LTS). If missing on Windows:
  ```powershell
  winget install --id OpenJS.NodeJS.LTS -e
  ```
  If `node`/`npm` aren't found immediately after install in the same
  PowerShell session, refresh `PATH` in that session:
  ```powershell
  $env:Path = [System.Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [System.Environment]::GetEnvironmentVariable('Path','User')
  ```
- **Emscripten SDK (emsdk)** — installed *locally inside this repo* at
  `.emsdk/`, not system-wide:
  ```powershell
  git clone https://github.com/emscripten-core/emsdk.git .emsdk
  .\.emsdk\emsdk.ps1 install latest
  .\.emsdk\emsdk.ps1 activate latest
  ```
- **Ninja** (CMake generator) — `emcmake` needs a Makefile/Ninja-style
  generator, and this project assumes no system-wide Visual Studio or MinGW
  make. Rather than installing Ninja system-wide, it's vendored locally
  using emsdk's own bundled Python:
  ```powershell
  & .\.emsdk\python\3.13.3_64bit\python.exe -m pip install --target .\.emsdk\ninja-pkg ninja
  ```
  (adjust the python version folder name if a different emsdk version was
  installed). `scripts/build_wasm.ps1` automatically prepends
  `.emsdk\ninja-pkg\bin` to `PATH` if it exists.

## Activating emsdk in a fresh shell

`scripts/build_wasm.ps1` does this automatically, but to get `emcc` on
`PATH` manually in a new PowerShell session:

```powershell
. .\.emsdk\emsdk_env.ps1
emcc --version
```

## Building the WASM module

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build_wasm.ps1
```

This activates emsdk, runs `emcmake cmake -S wasm -B build\wasm`, then
`emmake cmake --build build\wasm`, producing `fluids.js` and `fluids.wasm`
directly in `web/src/wasm-gen/` (MODULARIZE=1, EXPORT_ES6=1 — a plain ES
module the frontend `import()`s at runtime). These generated files are not
committed (see `.gitignore`) — re-run this script whenever `core/` or
`wasm/` change.

## Running the frontend

```powershell
cd web
npm install
npm run dev
```

Open the printed local URL (default `http://localhost:5173`). You should
see a Dam Break scenario collapsing under SPH, with a parameter panel on
the right — use the Solver dropdown to switch between SPH/FLIP/MPM live,
and the Scenario dropdown to switch initial layouts.

If PowerShell refuses to run `npm` ("execution of scripts is disabled"),
either call `npm.cmd run dev` directly, or run
`Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned`
once to fix it for your account permanently.

`npm run build` (in `web/`) type-checks and produces a production bundle in
`web/dist/`; it expects `web/src/wasm-gen/fluids.{js,wasm}` to already exist
(run `scripts/build_wasm.ps1` first).

## Native build + tests (`core/` and `tests/`)

`core/` is plain C++17 and configures/builds independently of Emscripten.
The top-level `CMakeLists.txt` builds `fluids_core` plus the native Catch2
test suite (fetched via CMake `FetchContent`, no separate install needed):

```powershell
cmake -S . -B build\native
cmake --build build\native --config Release
ctest --test-dir build\native -C Release --output-on-failure
```

On this project's dev machine that resolves to the Visual Studio 2022 (MSVC)
generator automatically; any of MSVC/clang/gcc works. If no system compiler
is available, the clang++ bundled inside `.emsdk/upstream/bin/` also works
as a native (non-Emscripten) compiler in a pinch, e.g.:
`cmake -S . -B build\native -G Ninja -DCMAKE_CXX_COMPILER="%CD%\.emsdk\upstream\bin\clang++.exe"`.

Pass `-DFLUIDS_BUILD_TESTS=OFF` to configure `fluids_core` alone without the
test suite. `wasm/CMakeLists.txt` remains a fully separate, emcmake-only
build graph (see `scripts/build_wasm.ps1`) — building natively from the
top-level `CMakeLists.txt` never touches it.
