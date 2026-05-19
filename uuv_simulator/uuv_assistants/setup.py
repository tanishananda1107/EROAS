from setuptools import setup

package_name = 'uuv_assistants'

setup(
    name=package_name,
    version='0.0.0',
    packages=['tf_quaternion'],

    package_dir={'': 'src'},

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='ROS2 migration',
    maintainer_email='ros2@todo.com',

    description='UUV Assistants - quaternion utilities',

    license='Apache-2.0',
)
