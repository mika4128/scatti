cmake_minimum_required(VERSION 3.15)
project(scatti VERSION 0.2.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(BUILD_TESTS "Build test programs" ON)
option(BUILD_BENCHMARK "Build benchmark program" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_WRAPPER_TESTS "Register Python/Rust README smoke tests with ctest" ON)

# ============================================================================
# Portability / embedding options
# ============================================================================
#   SCATTI_PROVIDE_FAST_CBRT  — build the musl-style scatti_cbrt() helper.
#                               Useful when embedding in an environment without
#                               libm (e.g. LinuxCNC kernel module). You must
#                               also #define SCATTI_CBRT(x) scatti_cbrt(x) in
#                               your integration header for it to be used.
#   SCATTI_ENABLE_EXTRA_SOLVERS — compile optional reference solver variants
#                               (currently: time_none_smooth in pos3 step2)
#                               that are not wired into the dispatcher.
option(SCATTI_PROVIDE_FAST_CBRT    "Compile scatti_cbrt() no-libm cube root" OFF)
option(SCATTI_ENABLE_EXTRA_SOLVERS "Compile extra reference solvers"         OFF)

# ============================================================================
# Performance Optimization Level (0, 1, 2)
#
#   0 = Default   : -O2, portable, no special flags
#   1 = Optimized : -O3 + LTO (link-time optimization for cross-file inlining)
#   2 = Aggressive: -O3 + LTO + -march=native + -ffast-math
#
# Usage:
#   cmake .. -DSCATTI_OPT_LEVEL=2
# ============================================================================
set(SCATTI_OPT_LEVEL "0" CACHE STRING "Optimization level: 0=default, 1=LTO+O3, 2=aggressive")

# Library sources
set(SCATTI_SOURCES
    src/roots.c
    src/brake.c
    src/profile.c
    src/block.c
    src/position_first_step1.c
    src/position_first_step2.c
    src/position_second_step1.c
    src/position_second_step2.c
    src/position_third_step1.c
    src/position_third_step2.c
    src/velocity_second_step1.c
    src/velocity_second_step2.c
    src/velocity_third_step1.c
    src/velocity_third_step2.c
    src/trajectory.c
    src/calculator.c
    src/input_parameter.c
    src/output_parameter.c
    src/scatti.c
)

if(SCATTI_PROVIDE_FAST_CBRT)
    list(APPEND SCATTI_SOURCES src/scatti_cbrt.c)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Static library
add_library(scatti STATIC ${SCATTI_SOURCES})
target_include_directories(scatti PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(scatti PRIVATE m)

# Shared library
add_library(scatti_shared SHARED ${SCATTI_SOURCES})
set_target_properties(scatti_shared PROPERTIES
    OUTPUT_NAME scatti
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
target_include_directories(scatti_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(scatti_shared PRIVATE m)

# ============================================================================
# Compiler flags based on optimization level
# ============================================================================
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")

    # Base flags (all levels)
    set(SCATTI_BASE_FLAGS -Wall -Wextra -DNDEBUG)

    if(SCATTI_OPT_LEVEL STREQUAL "0")
        # Level 0: Default portable build
        set(SCATTI_OPT_FLAGS -O2)
        message(STATUS "scatti optimization: Level 0 (default, -O2)")

    elseif(SCATTI_OPT_LEVEL STREQUAL "1")
        # Level 1: LTO + O3
        set(SCATTI_OPT_FLAGS -O3 -flto)
        message(STATUS "scatti optimization: Level 1 (LTO + O3)")

    elseif(SCATTI_OPT_LEVEL STREQUAL "2")
        # Level 2: Aggressive - LTO + O3 + native + safe fast-math subset
        # Note: -ffast-math breaks isinf() which is used to detect infinite jerk/accel.
        # Instead we use individual safe flags that don't assume no-infinities.
        set(SCATTI_OPT_FLAGS -O3 -flto -march=native
            -fno-math-errno -funsafe-math-optimizations -fno-trapping-math
            -fassociative-math -freciprocal-math)
        message(STATUS "scatti optimization: Level 2 (LTO + O3 + native + tuned-math)")

    else()
        message(FATAL_ERROR "Invalid SCATTI_OPT_LEVEL: ${SCATTI_OPT_LEVEL}. Must be 0, 1, or 2.")
    endif()

    # Apply to static library
    target_compile_options(scatti PRIVATE ${SCATTI_BASE_FLAGS} ${SCATTI_OPT_FLAGS})

    # Apply to shared library (add -fPIC)
    target_compile_options(scatti_shared PRIVATE ${SCATTI_BASE_FLAGS} ${SCATTI_OPT_FLAGS} -fPIC)

    # LTO requires linker flags too
    if(SCATTI_OPT_LEVEL GREATER 0)
        target_link_options(scatti PRIVATE -flto)
        target_link_options(scatti_shared PRIVATE -flto)
    endif()

endif()

# Pass optimization level as a compile definition
target_compile_definitions(scatti PRIVATE SCATTI_OPT_LEVEL=${SCATTI_OPT_LEVEL})
target_compile_definitions(scatti_shared PRIVATE SCATTI_OPT_LEVEL=${SCATTI_OPT_LEVEL})

# Feature toggles (propagated to both library targets)
if(SCATTI_PROVIDE_FAST_CBRT)
    target_compile_definitions(scatti        PUBLIC SCATTI_PROVIDE_FAST_CBRT)
    target_compile_definitions(scatti_shared PUBLIC SCATTI_PROVIDE_FAST_CBRT)
endif()
if(SCATTI_ENABLE_EXTRA_SOLVERS)
    target_compile_definitions(scatti        PRIVATE SCATTI_ENABLE_EXTRA_SOLVERS)
    target_compile_definitions(scatti_shared PRIVATE SCATTI_ENABLE_EXTRA_SOLVERS)
endif()

# ============================================================================
# Tests
# ============================================================================
if(BUILD_TESTS)
    enable_testing()

    add_executable(scatti_test test/test_main.c)
    target_link_libraries(scatti_test PRIVATE scatti m)

    add_executable(scatti_test_random test/test_random.c)
    target_link_libraries(scatti_test_random PRIVATE scatti m)

    add_executable(scatti_test_readme test/test_readme_example.c)
    target_link_libraries(scatti_test_readme PRIVATE scatti m)
    add_test(NAME readme_example COMMAND scatti_test_readme)

    if(BUILD_WRAPPER_TESTS)
        find_program(Python3_EXECUTABLE NAMES python3)
        find_program(CARGO_EXECUTABLE NAMES cargo)

        if(Python3_EXECUTABLE)
            add_test(
                NAME readme_python
                COMMAND ${CMAKE_COMMAND} -E env
                    "SCATTI_LIB=$<TARGET_FILE:scatti_shared>"
                    "PYTHONPATH=${CMAKE_SOURCE_DIR}/wrapper/python"
                    ${Python3_EXECUTABLE}
                    ${CMAKE_SOURCE_DIR}/wrapper/python/examples/test_readme_python.py
            )
            set_tests_properties(readme_python PROPERTIES LABELS "wrapper;python")
        else()
            message(STATUS "python3 not found — skipping readme_python ctest")
        endif()

        if(CARGO_EXECUTABLE)
            add_test(
                NAME readme_rust
                COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/wrapper/rust
                    ${CARGO_EXECUTABLE} test --test readme_example --quiet
            )
            set_tests_properties(readme_rust PROPERTIES LABELS "wrapper;rust")
        else()
            message(STATUS "cargo not found — skipping readme_rust ctest")
        endif()
    endif()

    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        if(SCATTI_OPT_LEVEL GREATER 0)
            target_compile_options(scatti_test PRIVATE -O3 -flto)
            target_link_options(scatti_test PRIVATE -flto)
            target_compile_options(scatti_test_random PRIVATE -O3 -flto)
            target_link_options(scatti_test_random PRIVATE -flto)
        endif()
    endif()

    # Cross-validation test: compare scatti vs C++ Ruckig (optional)
    set(RUCKIG_DIR "" CACHE PATH "Path to Ruckig C++ source tree (for cross-validation test)")
    if(RUCKIG_DIR AND EXISTS "${RUCKIG_DIR}/include/ruckig/ruckig.hpp")
        enable_language(CXX)
        set(CMAKE_CXX_STANDARD 17)
        add_executable(scatti_test_comparison test/test_comparison.cpp)
        target_include_directories(scatti_test_comparison PRIVATE
            ${RUCKIG_DIR}/include
        )
        target_link_libraries(scatti_test_comparison PRIVATE
            scatti m
            ${RUCKIG_DIR}/build/libruckig.so
        )
        target_compile_options(scatti_test_comparison PRIVATE -O2)
        message(STATUS "Cross-validation test enabled (Ruckig at ${RUCKIG_DIR})")
    endif()
endif()

# Benchmark
if(BUILD_BENCHMARK)
    add_executable(scatti_benchmark test/benchmark.c)
    target_link_libraries(scatti_benchmark PRIVATE scatti m)

    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        if(SCATTI_OPT_LEVEL GREATER 0)
            target_compile_options(scatti_benchmark PRIVATE -O3 -flto)
            target_link_options(scatti_benchmark PRIVATE -flto)
        endif()
    endif()
endif()

# Examples
if(BUILD_EXAMPLES)
    add_executable(scatti_example_position examples/01_position.c)
    target_link_libraries(scatti_example_position PRIVATE scatti m)
endif()

# ============================================================================
# Install rules (with component separation: runtime vs development)
# ============================================================================

# --- Runtime component: shared library only ---
install(TARGETS scatti_shared
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT development
)

# --- Development component: static lib, headers, cmake config ---
install(TARGETS scatti
    EXPORT scatti-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT development
)

install(DIRECTORY include/scatti
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT development
)

# CMake config files for find_package(scatti)
set(SCATTI_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(EXPORT scatti-targets
    DESTINATION ${SCATTI_INSTALL_CONFIGDIR}
    NAMESPACE scatti::
    COMPONENT development
)

configure_package_config_file(
    cmake/scatti-config.cmake.in
    scatti-config.cmake
    INSTALL_DESTINATION ${SCATTI_INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
    scatti-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/scatti-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/scatti-config-version.cmake"
    DESTINATION ${SCATTI_INSTALL_CONFIGDIR}
    COMPONENT development
)

# ============================================================================
# CPack / DEB packaging  (two packages: libscatti + libscatti-dev)
# ============================================================================
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "scatti")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Enable component-based packaging: one .deb per component
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_COMPONENTS_ALL runtime development)

# ---------- Component metadata ----------
set(CPACK_COMPONENT_RUNTIME_DISPLAY_NAME "scatti runtime library")
set(CPACK_COMPONENT_RUNTIME_DESCRIPTION
    "Shared library for scatti jerk-limited trajectory generation.")

set(CPACK_COMPONENT_DEVELOPMENT_DISPLAY_NAME "scatti development files")
set(CPACK_COMPONENT_DEVELOPMENT_DESCRIPTION
    "Headers, static library, and CMake config for scatti.")
set(CPACK_COMPONENT_DEVELOPMENT_DEPENDS runtime)

# ---------- Common DEB fields ----------
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "scatti maintainers")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

# ---------- Per-component DEB fields ----------
# runtime  →  libscatti_0.2.0_amd64.deb
set(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "libscatti")
set(CPACK_DEBIAN_RUNTIME_FILE_NAME "libscatti_${PROJECT_VERSION}_${CMAKE_HOST_SYSTEM_PROCESSOR}.deb")
set(CPACK_DEBIAN_RUNTIME_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_RUNTIME_PACKAGE_DEPENDS "libc6 (>= 2.17)")
set(CPACK_DEBIAN_RUNTIME_PACKAGE_DESCRIPTION
"Pure C jerk-limited real-time trajectory generation (runtime)
 scatti generates time-optimal, jerk-limited trajectories for robots
 and machines.  This package contains the shared library.")

# development  →  libscatti-dev_0.2.0_amd64.deb
set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "libscatti-dev")
set(CPACK_DEBIAN_DEVELOPMENT_FILE_NAME "libscatti-dev_${PROJECT_VERSION}_${CMAKE_HOST_SYSTEM_PROCESSOR}.deb")
set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_SECTION "libdevel")
set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DEPENDS "libscatti (= ${PROJECT_VERSION})")
set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DESCRIPTION
"Pure C jerk-limited real-time trajectory generation (development)
 Headers, static library (.a), and CMake find_package() config files
 for building against scatti.")

include(CPack)

# Print summary
message(STATUS "scatti v${PROJECT_VERSION} | OPT_LEVEL=${SCATTI_OPT_LEVEL}")
