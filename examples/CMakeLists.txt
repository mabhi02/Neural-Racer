# Examples CMakeLists.txt
cmake_minimum_required(VERSION 3.15)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create example executables
add_executable(basic_race basic_race.cpp)
target_link_libraries(basic_race PRIVATE neural_racer_lib)

add_executable(hardware_monitor hardware_monitor.cpp)
target_link_libraries(hardware_monitor PRIVATE neural_racer_lib)

# Display message about examples
message(STATUS "Example configuration complete. Run examples with './basic_race' or './hardware_monitor'")