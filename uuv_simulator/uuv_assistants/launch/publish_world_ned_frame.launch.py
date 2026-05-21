world ned frame.launch · PY
#!/usr/bin/env python3
"""
ROS2 conversion of publish_world_ned_frame.launch
 
ROS1 original:
  <node pkg="tf2_ros" type="static_transform_publisher" name="world_ned_frame_publisher"
    args="0 0 0 1.5707963267948966 0 3.141592653589793 world world_ned" />
 
ROS1 args order : x y z  yaw               pitch  roll               parent  child
                  0 0 0  1.5707963267948966 0      3.141592653589793  world   world_ned
 
ROS2 static_transform_publisher CLI flags (humble+):
  --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
 
NOTE: In ROS1 the arg order is  x y z YAW PITCH ROLL  (intrinsic, ZYX).
      Mapping to ROS2 named flags:
        --roll  = 3.141592653589793   (was 3rd rotation value in ROS1)
        --pitch = 0                   (was 2nd rotation value in ROS1)
        --yaw   = 1.5707963267948966  (was 1st rotation value in ROS1)
"""
 
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_ned_frame_publisher',
            output='screen',
            arguments=[
                '--x',              '0',
                '--y',              '0',
                '--z',              '0',
                '--yaw',            '1.5707963267948966',
                '--pitch',          '0',
                '--roll',           '3.141592653589793',
                '--frame-id',       'world',
                '--child-frame-id', 'world_ned',
            ],
        ),
    ])
