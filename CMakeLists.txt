cmake_minimum_required(VERSION 3.5)
project(lgsvl_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Boost REQUIRED system)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)

ament_auto_find_build_dependencies()

include_directories(${Boost_INCLUDE_DIRS})

add_executable(lgsvl_bridge
  src/bridge.cc
  src/client.cc
  src/clients.cc
  src/node.cc
  src/server.cc
  src/types.cc
)

target_link_libraries(lgsvl_bridge ${Boost_LIBRARIES} ${CMAKE_DL_LIBS})

ament_target_dependencies(lgsvl_bridge rcutils rcl)

install(TARGETS lgsvl_bridge DESTINATION bin)

ament_auto_package()
