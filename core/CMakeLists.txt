cmake_minimum_required(VERSION 3.15)
project(fluids_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Plain, portable C++17 — no Emscripten-specific headers or macros belong in
# this target. It must compile with the system's default compiler (MSVC,
# clang, gcc) so Phase 1 native Catch2 tests can link against it directly,
# and it must also compile cleanly under emcmake for the WASM build.
add_library(fluids_core STATIC
    src/Simulation.cpp
    src/FlipSimulation.cpp
    src/MpmSimulation.cpp
    src/FlipSmokeSimulation.cpp
    src/MpmSmokeSimulation.cpp
    src/SphSmokeSimulation.cpp
    src/SphSimulation3D.cpp
    src/common/UniformGrid.cpp
    src/common/UniformGrid3D.cpp
    src/scenario/Scenario.cpp
    src/scenario/SmokeScenario.cpp
    src/scenario/Scenario3D.cpp
    src/solvers/sph/SPHSolver.cpp
    src/solvers/flip/FlipSolver.cpp
    src/solvers/mpm/MpmSolver.cpp
    src/solvers/smoke/FlipSmokeSolver.cpp
    src/solvers/mpm_smoke/MpmSmokeSolver.cpp
    src/solvers/sph_smoke/SphSmokeSolver.cpp
    src/solvers/sph3d/SphSolver3D.cpp
)

target_include_directories(fluids_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(fluids_core PRIVATE /W4)
else()
    target_compile_options(fluids_core PRIVATE -Wall -Wextra)
endif()
