from setuptools import setup

package_name = 'uuv_simulation_runner'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools', 'rclpy', 'psutil', 'pyyaml'],
    zip_safe=True,
    maintainer='Musa Morena Marcusso Manhaes',
    maintainer_email='musa.marcusso@de.bosch.com',
    description='ROS2 simulation runner for Gazebo Harmonic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_simulation_timer = uuv_simulation_runner.set_simulation_timer:main',
            'unpause_simulation = uuv_simulation_runner.unpause_simulation:main',
        ],
    },
)
