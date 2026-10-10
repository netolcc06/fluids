cmake_minimum_required(VERSION 3.15)
project(fluids_wasm CXX)

# This target only makes sense when configured via `emcmake cmake` (it uses
# Emscripten-specific link flags below). It is not meant to be built as part
# of a normal native configure of the top-level project.
if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "wasm/ must be configured with emcmake (Emscripten toolchain). Use scripts/build_wasm.ps1.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pull in fluids_core (plain native C++17 lib) from ../core.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core ${CMAKE_CURRENT_BINARY_DIR}/core)

add_executable(fluids_wasm bindings.cpp flip_bindings.cpp mpm_bindings.cpp flip_smoke_bindings.cpp mpm_smoke_bindings.cpp sph_smoke_bindings.cpp sph3d_bindings.cpp)
target_link_libraries(fluids_wasm PRIVATE fluids_core)

set_target_properties(fluids_wasm PROPERTIES OUTPUT_NAME "fluids")

target_link_options(fluids_wasm PRIVATE
    "--bind"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=createFluidsModule"
    "-sENVIRONMENT=web"
    "-sALLOW_MEMORY_GROWTH=1"
    # Emscripten does not attach heap typed-array views (HEAPF32 etc.) to the
    # returned Module object by default — the JS side needs HEAPF32 to build
    # a zero-copy Float32Array view into WASM memory for particle data.
    "-sEXPORTED_RUNTIME_METHODS=[\"HEAPF32\"]"
)

# Emit the generated glue .js / .wasm into web/src/wasm-gen so Vite treats
# them as normal source modules. Files under web/public/ are served as-is
# and Vite's dev server REFUSES to serve them through the ES module import
# pipeline ("this file is in /public ... should not be imported from source
# code") — dynamic import() of a public/ file 500s in `npm run dev` even
# though `vite build` happens to work (prod just copies public/ as-is and
# the browser does a native import with no Vite transform involved). Living
# under src/ makes fluids.js a real module in both dev and build, and Vite's
# built-in `new URL('fluids.wasm', import.meta.url)` asset handling (which
# the Emscripten glue already uses) takes care of the .wasm binary.
set_target_properties(fluids_wasm PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../web/src/wasm-gen
)
