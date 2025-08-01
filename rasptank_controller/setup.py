from setuptools import find_packages, setup

package_name = 'rasptank_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "make_circles = rasptank_controller.make_circles:main",
            "camera_controller = rasptank_controller.camera_controller:main",
            "ultrasonic_sensor_controller = rasptank_controller.ultrasonic_sensor_controller:main",

        ],
    },
)
