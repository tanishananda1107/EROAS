from setuptools import setup, find_packages

package_name = 'uuv_simulation_evaluation'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},

    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'pyyaml'
    ],

    zip_safe=True,

    maintainer='Musa Morena Marcusso Manhaes',
    maintainer_email='musa.marcusso@de.bosch.com',

    description='ROS2 bag evaluation and cost function computation tools',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'evaluate_bag = uuv_bag_evaluation.evaluate_bag:main',
            'run_best_worst_comparison = uuv_bag_evaluation.run_best_worst_comparison:main',
        ],
    },
)
