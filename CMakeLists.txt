cmake_minimum_required(VERSION 3.28)

# Now declare project
project(tentacle-tree
        VERSION 0.0.0
        DESCRIPTION "Tentacle Tree: A C++ library for incremental octree"
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(benchmark CONFIG REQUIRED)
find_package(doctest CONFIG REQUIRED)
find_package(rerun_sdk CONFIG QUIET)

# If rerun_sdk was found, we'll link it into tests/benchmarks and define RERUN_ENABLED
if (rerun_sdk_FOUND)
        message(STATUS "Found rerun_sdk: enabling RERUN_ENABLED")
else()
        message(STATUS "rerun_sdk not found: building without rerun support")
endif()

include(CTest)

set(COMPILE_OPTIONS
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
)

add_library(tentacle-tree
        src/tentacle_tree.cpp
        include/tt/impl/bounded_priority_queue.hpp
)

target_include_directories(tentacle-tree
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(tentacle-tree
        PRIVATE
        ${COMPILE_OPTIONS}
)

include(doctest)
enable_testing()
add_executable(tentacle-tree-test
        tests/tests.cpp
        tests/indexed_view.cpp
        tests/impl.cpp
        tests/point.h
        tests/rerun.h
)
target_link_libraries(tentacle-tree-test
        PRIVATE
        tentacle-tree
        doctest::doctest
        $<$<BOOL:${rerun_sdk_FOUND}>:rerun_sdk>
)
target_compile_options(tentacle-tree-test
        PRIVATE
        ${COMPILE_OPTIONS}
)
doctest_discover_tests(tentacle-tree-test)

add_executable(tentacle-tree-benchmark
        tests/benchmark.cpp
)
target_link_libraries(tentacle-tree-benchmark
        PRIVATE
        tentacle-tree
        benchmark::benchmark
        benchmark::benchmark_main
        $<$<BOOL:${rerun_sdk_FOUND}>:rerun_sdk>
)

if (rerun_sdk_FOUND)
    target_compile_definitions(tentacle-tree-test PRIVATE RERUN_ENABLED)
    target_compile_definitions(tentacle-tree-benchmark PRIVATE RERUN_ENABLED)
endif()