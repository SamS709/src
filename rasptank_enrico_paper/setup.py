from setuptools import find_packages, setup

package_name = 'rasptank_enrico_paper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sami',
    maintainer_email='sami@example.com',
    description='Enrico paper example node adapted for rasptank',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rasptank_enrico_paper_node = rasptank_enrico_paper.enrico_paper_node:main',
            'rasptank_enrico_paper_turtle_node = rasptank_enrico_paper.enrico_paper_node_turtle:main',
            'rasptank_enrico_paper_final_node = rasptank_enrico_paper.enrico_paper_final_node:main'
        ],
    },
)
