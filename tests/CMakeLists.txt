# Tests CMakeLists.txt

# Find Google Test (Gtest) if available
find_package(GTest QUIET)

if(NOT GTEST_FOUND)
    # Download and build googletest as a fallback
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.11.0
    )
    # Prevent googletest from installing itself as a system library
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Include directories for tests
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create test executables
add_executable(ai_tests ai_tests.cpp)
target_link_libraries(ai_tests neural_racer_lib gtest gtest_main pthread)

add_executable(hardware_tests hardware_tests.cpp)
target_link_libraries(hardware_tests neural_racer_lib gtest gtest_main pthread)

add_executable(physics_tests physics_tests.cpp)
target_link_libraries(physics_tests neural_racer_lib gtest gtest_main pthread)

add_executable(simulation_tests simulation_tests.cpp)
target_link_libraries(simulation_tests neural_racer_lib gtest gtest_main pthread)

# Add tests to CTest
add_test(NAME AITests COMMAND ai_tests)
add_test(NAME HardwareTests COMMAND hardware_tests)
add_test(NAME PhysicsTests COMMAND physics_tests)
add_test(NAME SimulationTests COMMAND simulation_tests)

# Display message about running tests
message(STATUS "Test configuration complete. Run tests with 'ctest' or 'make test'.")