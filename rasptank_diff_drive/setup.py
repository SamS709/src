import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rasptank_diff_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='RaspTank differential drive robot package with ros2_control integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = rasptank_diff_drive.motor_controller:main',
        ],
    },
)
