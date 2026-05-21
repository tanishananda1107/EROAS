from setuptools import setup
import os

package_name = 'uuv_auv_control_allocator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='AIRLab IISc',
    maintainer_email='airlab@iisc.ac.in',
    description='ROS2 AUV control allocator',
    license='Apache-2.0',
)
