cmake_minimum_required(VERSION 3.15)
project(fluids CXX)

# Top-level entry point for the NATIVE build: fluids_core + its Catch2 test
# suite. This is deliberately separate from wasm/CMakeLists.txt, which
# configures its own independent project (add_subdirectory'ing ../core
# itself) and only makes sense under `emcmake` — see scripts/build_wasm.ps1.
# Building from here never touches wasm/, so it can't break that path.
#
# Usage:
#   cmake -S . -B build
#   cmake --build build
#   ctest --test-dir build --output-on-failure

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_subdirectory(core)

option(FLUIDS_BUILD_TESTS "Build the native Catch2 test suite" ON)
if(FLUIDS_BUILD_TESTS)
    add_subdirectory(tests)
endif()
