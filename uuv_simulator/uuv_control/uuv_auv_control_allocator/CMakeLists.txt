cmake_minimum_required(VERSION 3.8)

project(uuv_auv_control_allocator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/AUVCommand.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_python_install_package(
    uuv_auv_actuator_interface
    PACKAGE_DIR src/uuv_auv_actuator_interface
)

install(
    PROGRAMS
      scripts/control_allocator

    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch

    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    rosidl_default_runtime
)

ament_package()
