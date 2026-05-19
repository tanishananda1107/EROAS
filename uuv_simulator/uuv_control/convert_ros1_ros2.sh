#!/bin/bash

ROOT="$HOME/eroas_ws/src/EROAS/uuv_simulator/uuv_control"


find "$ROOT" -type f \( \
-name "*.py" -o \
-name "CMakeLists.txt" -o \
-name "package.xml" -o \
-name "setup.py" \
\) | while read FILE
do
    echo "Processing: $FILE"

    TMP=$(mktemp)

    cat <<EOF | ollama run llama3 > "$TMP"


Rules:

ROS:

rospy -> rclpy

tf -> tf2_ros

catkin -> ament_cmake

catkin_python_setup() remove

catkin_install_python ->
install(PROGRAMS ...)

CATKIN_PACKAGE_BIN_DESTINATION ->
lib/\${PROJECT_NAME}

CATKIN_PACKAGE_SHARE_DESTINATION ->
share/\${PROJECT_NAME}

package.xml:

buildtool_depend catkin ->
ament_cmake

remove rosbuild

replace rospy dependency with rclpy

Python:


rospy.Publisher

to:

self.create_publisher()


rospy.Subscriber

to:

self.create_subscription()

rospy.get_param ->
declare_parameter

rospy.Time.now ->
node.get_clock().now()

rospy.get_time ->
clock.nanoseconds

Service migration:

rospy.Service ->
create_service()

Return ONLY converted code.

FILE:

$(cat "$FILE")

EOF

    mv "$TMP" "$FILE"

done

echo "Conversion finished"
