
import rclpy
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import Float64

class TrajectoryPoint:
    """Trajectory point data structure."""
    
    def __init__(self):
        self._t = 0.0
        self._pos = [0, 0, 0]
        self._rot = [0, 0, 0, 1]
        self._vel = [0, 0, 0, 0, 0, 0]
        self._acc = [0, 0, 0, 0, 0, 0]

    # Other methods...

Note that I removed the `object` keyword and replaced it with an empty cons[4D[K

Here are some specific changes:

* Replaced `rospy` with `rclpy`.
* Replaced `tf` with `tf2_ros`.
* Replaced `catkin` with `ament_cmake`.
* Replaced `Catkin_package_bin_destination` and `Catkin_package_share_desti[27D[K
`Catkin_package_share_destination` with `lib/${PROJECT_NAME}` and `share/${[9D[K
`share/${PROJECT_NAME}`, respectively.
* Replaced `rospy.Publisher` with `self.create_publisher()`.
* Replaced `rospy.Subscriber` with `self.create_subscription()`.
* Replaced `rospy.get_param` with `declare_parameter`.
* Replaced `rospy.Time.now` with `node.get_clock().now()`.
* Replaced `rospy.get_time` with `clock.nanoseconds`.

I did not convert the package.xml file, as it is a separate file that requi[5D[K
requires manual editing.

