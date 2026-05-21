from setuptools import setup

package_name = "uuv_control_cascaded_pid"

setup(

    name=package_name,

    version="0.6.13",

    packages=[
        "PID"
    ],

    package_dir={
        "": "src"
    },

    install_requires=[
        "setuptools",
        "numpy"
    ],

    zip_safe=True,

    maintainer=
    "Musa Morena Marcusso Manhaes",

    description=
    "Cascade PID control package",

    license="Apache-2.0",

    entry_points={

        "console_scripts":[

            "AccelerationControl = AccelerationControl:main",

            "VelocityControl = VelocityControl:main",

            "PositionControl = PositionControl:main",

            "PositionControlUnderactuated = PositionControlUnderactuated:main"
        ]
    }
)
