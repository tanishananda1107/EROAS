from setuptools import setup

package_name='uuv_assistants'

setup(

    name=package_name,

    version='0.6.13',

    packages=['tf_quaternion'],

    package_dir={'':'src'},

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    entry_points={

        'console_scripts':[

            'publish_footprints=scripts.publish_footprints:main',

            'publish_vehicle_footprint=scripts.publish_vehicle_footprint:main',

            'publish_world_models=scripts.publish_world_models:main',

            'set_simulation_timer=scripts.set_simulation_timer:main',

            'unpause_simulation=scripts.unpause_simulation:main'
        ]
    }
)
