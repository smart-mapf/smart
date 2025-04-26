cmake_minimum_required(VERSION 3.10)
project(external_visualizer)

include_directories(/usr/include/lua5.3)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})


find_library(ARGOS_FOOTBOT_LIB
NAMES argos3plugin_simulator_footbot
PATHS ${ARGOS_LIBRARY_DIRS}
)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED ExternalVisualizer.cpp)

target_link_libraries(${PROJECT_NAME} ${ARGOS_LIBRARIES} ${ARGOS_FOOTBOT_LIB} Qt6::Widgets)