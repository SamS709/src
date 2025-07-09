from setuptools import find_packages, setup

package_name = 'my_cv_package'

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
    maintainer_email='sami.leroux75@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detecter = my_cv_package.color_detection:main',
            'face_detecter = my_cv_package.face_detection:main',
            'face_detecter_pi = my_cv_package.face_detection_pi:main',
        ],
    },
)
