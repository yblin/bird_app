cmake_minimum_required(VERSION 3.16)
project(codelibrary-tests LANGUAGES CXX)

# Find glog and fmt (using codelibrary's own testing framework)
find_package(glog REQUIRED)
find_package(fmt REQUIRED)

# Read all_tests.h and extract include lines
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/all_tests.h" ALL_LINES)

# Generate a cpp file for each include
set(GENERATED_TEST_FILES)
foreach(LINE ${ALL_LINES})
    # Skip lines that don't contain #include
    if(NOT LINE MATCHES "#include")
        continue()
    endif()

    # Skip commented lines (lines starting with optional whitespace then //)
    if(LINE MATCHES "^[ \t]*//")
        continue()
    endif()

    # Extract the header file path from the include line
    string(REGEX REPLACE ".*#include \"([^\"]+)\".*" "\\1" HEADER_PATH ${LINE})

    # Create a unique cpp filename based on the header path
    string(REPLACE "/" "_" CPP_FILENAME ${HEADER_PATH})
    string(REPLACE ".h" ".cpp" CPP_FILENAME ${CPP_FILENAME})

    # Write the generated cpp file only if it doesn't exist
    set(GENERATED_CPP_FILE "${CMAKE_CURRENT_BINARY_DIR}/generated/${CPP_FILENAME}")
    if(NOT EXISTS ${GENERATED_CPP_FILE})
        file(WRITE ${GENERATED_CPP_FILE} "#include \"${HEADER_PATH}\"\n")
    endif()
    list(APPEND GENERATED_TEST_FILES ${GENERATED_CPP_FILE})

    message(STATUS "added test ${HEADER_PATH}")
endforeach()

# Create the main test executable
# Create a test runner with main function using codelibrary's testing framework
set(TEST_RUNNER_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_runner.cpp")
file(WRITE ${TEST_RUNNER_FILE} "#include \"codelibrary/base/testing.h\"\n\n
    int main(int argc, char** argv) {\n
        return RUN_ALL_TESTS_WITH_ARGS(argc, argv);\n
    }\n")

add_executable(codelibrary_tests ${TEST_RUNNER_FILE} ${GENERATED_TEST_FILES})

# Set C++ standard
target_compile_features(codelibrary_tests PRIVATE cxx_std_17)

# Include directories
target_include_directories(codelibrary_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Link with glog and fmt
target_link_libraries(codelibrary_tests
    glog::glog
    fmt::fmt
)

# Enable testing
enable_testing()
add_test(NAME CodelibaryTests COMMAND codelibrary_tests)
