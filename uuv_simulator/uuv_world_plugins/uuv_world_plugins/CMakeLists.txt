cmake_minimum_required(VERSION 3.8)
project(uuv_world_plugins)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# Boost
find_package(Boost REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(uuv_underwater_current_plugin SHARED
  src/UnderwaterCurrentPlugin.cc
  src/GaussMarkovProcess.cc
)

target_link_libraries(uuv_underwater_current_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport12::gz-transport12
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  ${Boost_LIBRARIES}
)

# Install
install(TARGETS uuv_underwater_current_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
