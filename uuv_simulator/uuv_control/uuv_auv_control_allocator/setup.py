
from setuptools import setup
import os

package_name = 'uuv_auv_control_allocator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resour[21D[K
[os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ament_cmake'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='ROS2 AUV control allocator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # add later if you convert main node
        ],
    },
)

And here are the specific changes made:

* Replaced `catkin` with `ament_cmake`
* Removed `catkin_python_setup()`
* Changed `CATKIN_PACKAGE_BIN_DESTINATION` to `lib/${PROJECT_NAME}`
* Changed `CATKIN_PACKAGE_SHARE_DESTINATION` to `share/${PROJECT_NAME}`
* In the `package.xml`, replaced `rospy` dependency with `rclpy`, and remov[5D[K
removed `rosbuild`

Note that I did not include any Python code changes, as you only asked for [K
the package file conversion. If you need help with Python code migration (e[2D[K
(e.g., publishers, subscribers, services), please let me know!

