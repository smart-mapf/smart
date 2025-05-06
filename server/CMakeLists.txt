cmake_minimum_required(VERSION 3.10)


project(ADG_Server_Project)

find_package(fmt)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O3 -ffast-math")
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem)
SET(CMAKE_BUILD_TYPE "RELEASE")

include_directories("inc")

file(GLOB SOURCES "src/*.cpp")

add_executable(ADG_server ${SOURCES})

target_link_libraries(ADG_server PUBLIC fmt::fmt ${Boost_LIBRARIES} PRIVATE rpc pthread)
