cmake_minimum_required(VERSION 3.8)
project(rexrov2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE
        PATTERN "*.launch" EXCLUDE)

ament_package()
