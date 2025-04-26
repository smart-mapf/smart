add_executable(galib_phototaxis main.cpp)

target_link_libraries(galib_phototaxis
  ${GALIB_LIBRARIES}
  footbot_nn
  galib_phototaxis_loop_functions
  argos3core_simulator)
