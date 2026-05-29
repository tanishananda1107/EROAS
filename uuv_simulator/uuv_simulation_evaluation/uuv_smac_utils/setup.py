# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS2 / Gazebo Harmonic (gz-sim 8) migration notes:
#
#   ROS1 used catkin_pkg.python_setup.generate_distutils_setup() to hook
#   into catkin's build system.  In ROS2 with ament_cmake_python the
#   CMakeLists.txt drives installation via ament_python_install_package(),
#   so this setup.py is kept only for:
#     (a) `pip install -e .` / editable installs during development, and
#     (b) tools that introspect package metadata (e.g. colcon, rosdep).
#
#   Key changes:
#     - Removed: `from distutils.core import setup`
#                `from catkin_pkg.python_setup import generate_distutils_setup`
#                (distutils is deprecated in Python 3.10+ and removed in 3.12)
#     - Added:   setuptools.setup()   (the modern standard)
#     - Removed: requires=['rospy']   → install_requires=['rclpy']
#     - package_dir keeps {'': 'src'} so colcon finds src/uuv_smac_utils/
#     - Added entry_points so `ros2 run uuv_smac_utils <script>` works
#       (ament_cmake_python handles this automatically via CMakeLists.txt,
#        but declaring them here also makes them available via pip installs)

from setuptools import setup, find_packages

package_name = 'uuv_smac_utils'

setup(
    name=package_name,
    version='0.5.0',
    # Discover the uuv_smac_utils package inside src/
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    # ---------------------------------------------------------------------------
    # Runtime dependencies
    # When installed via colcon/ament these come from package.xml exec_depend.
    # When installed via pip these install_requires entries are used instead.
    # ---------------------------------------------------------------------------
    install_requires=[
        'rclpy',
        'pyyaml',
        'matplotlib',
        'simplejson',
        'numpy',
    ],
    # ---------------------------------------------------------------------------
    # Console scripts → `ros2 run uuv_smac_utils <name>` entry points
    # These mirror the scripts/ listed in CMakeLists.txt install(PROGRAMS ...).
    # Each entry point must point to a callable in the installed Python package.
    # Adjust the module.function targets to match your actual script internals.
    # ---------------------------------------------------------------------------
    entry_points={
        'console_scripts': [
            'smac_wrapper              = uuv_smac_utils.smac_wrapper:main',
            'run_smac                  = uuv_smac_utils.run_smac:main',
            'create_smac_config_files  = uuv_smac_utils.create_smac_config_files:main',
            'evaluate_smac_best_results= uuv_smac_utils.evaluate_smac_best_results:main',
            'smac                      = uuv_smac_utils.smac:main',
            'create_results_folder     = uuv_smac_utils.create_results_folder:main',
            'generate_motion_primitives= uuv_smac_utils.generate_motion_primitives:main',
            'sync_smac_files           = uuv_smac_utils.sync_smac_files:main',
        ],
    },
    # ---------------------------------------------------------------------------
    # Package metadata
    # ---------------------------------------------------------------------------
    author='Musa Morena Marcusso Manhaes',
    author_email='musa.marcusso@de.bosch.com',
    maintainer='Musa Morena Marcusso Manhaes',
    maintainer_email='musa.marcusso@de.bosch.com',
    description='Utility scripts and configuration for the SMAC optimization',
    license='Apache-2.0',
    # zip_safe must be False for ament resource index discovery to work
    zip_safe=False,
    python_requires='>=3.8',
)
