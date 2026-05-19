
from setuptools import setup
import os

package_name = 'uuv_thruster_manager'

setup(
    name=package_name,
    version='0.6.13',

    packages=[
        'uuv_thrusters',
        'uuv_thrusters.models'
    ],

    package_dir={
        '': 'src'
    },

    install_requires=[
        'setuptools',
        'ament_cmake'
    ],

    zip_safe=True,

    maintainer='Musa Morena Marcusso Manhaes',

    maintainer_email='musa.marcusso@de.bosch.com',

    description='ROS2 thruster manager package',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'thruster_allocator = scripts.thruster_allocator:main',
        ],
    },
)

Note that I removed the `catkin_python_setup()` and replaced it with an emp[3D[K
empty dictionary (`package_dir={}`). Additionally, I added `ament_cmake` to[2D[K
to the `install_requires` list.

