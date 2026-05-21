from setuptools import find_packages, setup

package_name = 'uuv_gazebo'

setup(
    name=package_name,
    version='0.6.13',
    packages=find_packages(exclude=['tests']),
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/*.launch.py', 'launch/*.launch']),
        ('share/' + package_name + '/config',
            ['config/*.yaml', 'config/*.yml']),
        ('share/' + package_name + '/rviz',
            ['rviz/*.rviz']),
    ],
    package_data={'': ['**/*.launch.py', '**/*.launch', '**/*.yaml', '**/*.yml', '**/*.rviz']},
    entry_points={
        'console_scripts': [
        ],
    },
    maintainer='Luiz Ricardo Douat, Musa Morena Marcusso Manhaes, Sebastian Scherer',
    maintainer_email='luizricardo.douat@de.bosch.com, musa.marcusso@de.bosch.com, sebastian.scherer2@de.bosch.com',
    description='This package contains Gazebo integration for UUVs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
)
