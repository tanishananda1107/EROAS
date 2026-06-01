
# ROS2
source install/setup.bash
# ROS2
ros2 topic pub --rate 10 /rexrov2/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 run navigator_auv contour_heading.py
ros2 launch rexrov2_gz start_demo_pid_controller.launch.py teleop_on:=true
pkill -f "gz sim"
ros2 topic echo /rexrov2/blueview_p900/sonar_image_raw
ros2 run ros_gz_bridge parameter_bridge \
  /rexrov2/blueview_p900/sonar_image_raw@sensor_msgs/msg/Image[gz.msgs.Image

