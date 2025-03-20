cmake_minimum_required(VERSION 3.15)
project(neural_racer VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
endif()

# Enable warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Threads REQUIRED)

# Platform-specific definitions
if(WIN32)
    add_definitions(-D_WIN32_WINNT=0x0601)
elseif(APPLE)
    add_definitions(-DMACOSX)
elseif(UNIX)
    add_definitions(-DLINUX)
endif()

# Include directories
include_directories(include)

# Source files
file(GLOB_RECURSE HEADERS "include/neural_racer/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Remove main.cpp from library sources
list(FILTER SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# Create library
add_library(neural_racer_lib ${SOURCES} ${HEADERS})
target_link_libraries(neural_racer_lib PRIVATE Threads::Threads)

# Main executable
add_executable(neural_racer src/main.cpp)
target_link_libraries(neural_racer PRIVATE neural_racer_lib)

# Examples
add_subdirectory(examples)

# Tests
option(BUILD_TESTS "Build the tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
install(TARGETS neural_racer neural_racer_lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Package configuration
set(CPACK_PACKAGE_NAME "neural_racer")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Neural Racer - AI-powered racing simulation")
include(CPack)

# Print configuration summary
message(STATUS "Configuration:")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build tests: ${BUILD_TESTS}")