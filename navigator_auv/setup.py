from setuptools import setup
from glob import glob
import os

package_name = 'navigator_auv'

setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    data_files=[

        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'models/rexrov2'
            ),
            glob('models/rexrov2/*')
        ),

        (
            os.path.join(
                'lib',
                package_name
            ),
            glob('scripts/*.py')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='airl-user',

    maintainer_email='airl@todo.todo',

    description='EROAS Navigation Stack',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [

            'grid_detection.py = scripts.grid_detection:main',
            'sonar_reconstruction.py = scripts.sonar_reconstruction:main',
            'contour_heading.py = scripts.contour_heading:main',
            'cbf_implementation.py = scripts.cbf_implementation:main',
            'pose_plotter.py = scripts.pose_plotter:main',
            'obstacle_draw.py = scripts.obstacle_draw:main',

        ],
    },
)
