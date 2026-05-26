from setuptools import setup

package_name = "uuv_trajectory_control"

setup(
    name=package_name,

    version="0.6.13",

    packages=[
        "uuv_control_interfaces",
        "uuv_trajectory_generator",
        "uuv_waypoints",
        "uuv_trajectory_generator.path_generator"
    ],

    package_dir={
        "": "src"
    },

    install_requires=[
        "setuptools",
        "numpy",
        "scipy",
        "pyyaml",
        "matplotlib"
    ],

    zip_safe=True,

    maintainer="Luiz Ricardo Douat",

    description="ROS2 UUV trajectory control package",

    license="Apache-2.0",

    tests_require=[
        "pytest"
    ],

    entry_points={
        "console_scripts": []
    }
)
