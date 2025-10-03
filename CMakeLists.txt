cmake_minimum_required(VERSION 3.16)

# Main project
project(Lifelong_SMART VERSION 1.0.0)

# Set global C++ standard (use C++17 as minimum to satisfy all components)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release by default if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Global compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -pthread")

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")

# Global options
option(BUILD_SERVER "Build the ADG server component" ON)
option(BUILD_CLIENT "Build the ARGoS client component" ON)
option(BUILD_ALL "Build all components" ON)

# Find common dependencies
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)

# Add subdirectories based on options
if(BUILD_ALL OR BUILD_SERVER)
    message(STATUS "Adding server component...")
    add_subdirectory(server)
endif()

if(BUILD_ALL OR BUILD_CLIENT)
    message(STATUS "Adding client component...")
    # Check if ARGoS is available before building client
    find_package(ARGoS QUIET)
    if(ARGoS_FOUND)
        add_subdirectory(client)
        message(STATUS "ARGoS found - client component will be built")
    else()
        message(WARNING "ARGoS not found - skipping client component. Install ARGoS3 to build client.")
    endif()
endif()

# Create a custom target to build all executables
add_custom_target(all_executables
    COMMENT "Building all executables in the project"
)

# Add dependencies for the custom target
if(TARGET ADG_server)
    add_dependencies(all_executables ADG_server)
endif()

if(TARGET mpga_phototaxis)
    add_dependencies(all_executables mpga_phototaxis)
endif()

if(TARGET galib_phototaxis)
    add_dependencies(all_executables galib_phototaxis)
endif()

if(TARGET lns)
    add_dependencies(all_executables lns)
endif()

if(TARGET pbs)
    add_dependencies(all_executables pbs)
endif()

if(TARGET benchmark_pbs)
    add_dependencies(all_executables benchmark_pbs)
endif()

# Print summary
message(STATUS "")
message(STATUS "=== Lifelong-SMART Build Configuration ===")
message(STATUS "Server component:   ${BUILD_SERVER}")
message(STATUS "Client component:   ${BUILD_CLIENT}")
message(STATUS "Build type:         ${CMAKE_BUILD_TYPE}")
message(STATUS "==========================================")