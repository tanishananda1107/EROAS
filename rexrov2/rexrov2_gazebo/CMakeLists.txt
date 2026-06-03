cmake_minimum_required(VERSION 3.8)
project(rexrov2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        launch
        worlds
        models
        urdf
        xacro
        meshes
        config
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL
        PATTERN "*~" EXCLUDE
        PATTERN "__pycache__" EXCLUDE)

ament_package()
