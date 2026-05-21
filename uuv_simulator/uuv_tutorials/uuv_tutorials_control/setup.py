from setuptools import setup

package_name = 'uuv_tutorials_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIRLab IISc',
    maintainer_email='airlab@iisc.ac.in',
    description='Control tutorial package for UUV Simulator',
    license='Apache-2.0',
    tests_require=['pytest'],
)
