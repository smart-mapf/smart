cmake_minimum_required(VERSION 3.10)
project(external_visualizer)

find_package(fmt)
find_package(ARGoS REQUIRED)

include_directories(/usr/include/lua5.3)
include_directories(/usr/include/argos3)
link_directories(/usr/lib/argos3)
find_library(ARGOS_FOOTBOT_LIB
    NAMES argos3plugin_simulator_footbot
    PATHS /usr/lib/argos3/
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED ExternalVisualizer.cpp)

target_link_libraries(${PROJECT_NAME} fmt::fmt ${ARGOS_LIBRARIES} ${ARGOS_LDFLAGS} ${ARGOS_FOOTBOT_LIB} Qt5::Widgets)
