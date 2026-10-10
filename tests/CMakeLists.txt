include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1
)
FetchContent_MakeAvailable(Catch2)

add_executable(fluids_tests
    core/test_vec2.cpp
    core/test_kernels.cpp
    core/test_uniform_grid.cpp
    core/test_solver.cpp
    core/test_flip_solver.cpp
    core/test_mpm_solver.cpp
    core/test_flip_smoke_solver.cpp
    core/test_mpm_smoke_solver.cpp
    core/test_sph_smoke_solver.cpp
    core/test_vec3.cpp
    core/test_kernels3d.cpp
    core/test_uniform_grid3d.cpp
    core/test_solver3d.cpp
)

target_link_libraries(fluids_tests PRIVATE fluids_core Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(fluids_tests)
