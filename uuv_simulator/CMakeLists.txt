cmake_minimum_required(VERSION 3.8)
project(uuv_simulator)

find_package(ament_cmake REQUIRED)

# Find all subdirectories that are ROS 2 packages
file(GLOB SUBDIRS "uuv_*")

foreach(subdir ${SUBDIRS})
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
    # Check if it's a ROS 2 package by looking for package.xml
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/package.xml)
      add_subdirectory(${subdir})
    endif()
  endif()
endforeach()

ament_package()
