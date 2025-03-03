include_directories(${CMAKE_SOURCE_DIR}/controllers)
include_directories(${CMAKE_SOURCE_DIR}/client/controllers)

add_subdirectory(footbot_diffusion)
add_subdirectory(footbot_nn)
add_subdirectory(footbot_manualcontrol)
