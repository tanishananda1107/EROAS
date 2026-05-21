from setuptools import setup

package_name = 'uuv_assistants'

setup(
    name=package_name,
    version='0.0.0',
    packages=['tf_quaternion'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='AIRLab IISc',
    maintainer_email='airlab@iisc.ac.in',
    description='UUV Assistants - quaternion utilities, TF helpers, simulation timer',
    license='Apache-2.0',
)
