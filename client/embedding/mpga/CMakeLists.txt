add_executable(mpga_phototaxis main.cpp)
target_link_libraries(mpga_phototaxis
  footbot_nn
  mpga_phototaxis_loop_functions
  argos3core_simulator)
