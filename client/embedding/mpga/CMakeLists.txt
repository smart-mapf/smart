add_executable(mpga_phototaxis main.cpp)
target_link_libraries(mpga_phototaxis
  mpga_phototaxis_loop_functions
  argos3core_simulator)
