from setuptools import setup

package_name = "uuv_thrusters"

setup(
    name=package_name,

    version="0.6.13",

    packages=[
        "uuv_thrusters",
        "uuv_thrusters.models"
    ],

    package_dir={
        "": "src"
    },

    install_requires=[
        "setuptools",
        "numpy",
        "PyYAML"
    ],

    zip_safe=True,

    maintainer="UUV Simulator Authors",

    maintainer_email="musa.marcusso@de.bosch.com",

    description="ROS2 UUV thrusters package",

    license="Apache-2.0",

    tests_require=["pytest"],

    entry_points={
        "console_scripts": [
            "thruster_allocator = scripts.thruster_allocator:main"
        ]
    }
)
