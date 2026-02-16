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
find_package(rerun_sdk CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)

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
        rerun_sdk
        Boost::boost
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
        rerun_sdk
        Boost::boost
)