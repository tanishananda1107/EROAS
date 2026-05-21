cmake_minimum_required(VERSION 3.8)
project(uuv_assistants)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Build C++ node
add_executable(uuv_message_to_tf src/message_to_tf.cc)
target_link_libraries(uuv_message_to_tf
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)
target_include_directories(uuv_message_to_tf PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Install C++ executable
install(TARGETS
  uuv_message_to_tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  scripts/create_new_robot_model
  scripts/create_thruster_manager_configuration
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch" EXCLUDE
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.py"
)

ament_package()
