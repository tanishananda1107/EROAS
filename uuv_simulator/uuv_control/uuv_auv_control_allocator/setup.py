from setuptools import setup

package_name = 'uuv_auv_control_allocator'

setup(

    name=package_name,

    version='0.6.13',

    packages=[
        'uuv_auv_actuator_interface'
    ],

    package_dir={
        '':'src'
    },

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    data_files=[

        (
            'share/ament_index/resource_index/packages',

            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,

            ['package.xml']
        )
    ],

    entry_points={

        'console_scripts':[

            'control_allocator=scripts.control_allocator:main'
        ]
    }
)
