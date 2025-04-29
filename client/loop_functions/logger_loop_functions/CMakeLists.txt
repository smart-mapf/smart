cmake_minimum_required(VERSION 3.0)
project(LoggerLoopFunctions)
include_directories(/usr/include/lua5.3)

# find_package(PkgConfig)
# pkg_check_modules(ARGOS REQUIRED argos3_simulator)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_library(logger_loop_functions SHARED LoggerLoopFunctions.cpp)
target_link_libraries(logger_loop_functions ${ARGOS_LDFLAGS} Qt6::Widgets)
