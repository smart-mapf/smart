add_library(mpga SHARED
  mpga.h mpga.cpp
  mpga_loop_functions.h mpga_loop_functions.cpp)
target_link_libraries(mpga
  argos3core_simulator)
if(NOT APPLE)
target_link_libraries(mpga rt)
endif(NOT APPLE)


add_library(mpga_phototaxis_loop_functions SHARED
  mpga_phototaxis_loop_functions.h mpga_phototaxis_loop_functions.cpp)

target_link_libraries(mpga_phototaxis_loop_functions
  footbot_nn
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  mpga)
