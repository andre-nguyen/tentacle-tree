cmake_minimum_required(VERSION 3.24)

include(cmake/CPM.cmake)

# Fetch cpp-library before project()
# Check https://github.com/stlab/cpp-library/releases for the latest version
CPMAddPackage("gh:stlab/cpp-library@5.2.0")
include(${cpp-library_SOURCE_DIR}/cpp-library.cmake)

# Enable dependency tracking before project()
cpp_library_enable_dependency_tracking()

# Now declare project
project(tentacle-tree)

# Enable CTest infrastructure (required for tests/examples to work)
include(CTest)

# Setup library
cpp_library_setup(
    DESCRIPTION "A reimplementation of an incremental octree"
    NAMESPACE tt
    HEADERS tentacle-tree.hpp
    SOURCES tentacle-tree.cpp
    EXAMPLES example.cpp
    TESTS tests.cpp
)
