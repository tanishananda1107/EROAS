
from setuptools import setup
from ament_index_python.packages import Packages

package_name = 'uuv_control_cascaded_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ament_cmake'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='ROS2 cascaded PID controllers',
    license='Apache-2.0',
)

And the `package.xml` file:

<?xml version="1.0"?>
<package>
  <name>uuv_control_cascaded_pid</name>
  <version>0.0.0</version>
  <dependencies>
    <build_depend>ament_cmake</build_depend>
  </dependencies>
  <build_depend>rclpy</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
</package>

Note that I removed the `catkin` package and replaced it with `ament_cmake`[13D[K
`ament_cmake`. I also updated the `install_requires` list to include only `[1D[K
`ament_cmake`.

