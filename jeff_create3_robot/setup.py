from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jeff_create3_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),   glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'puzzles'), glob(os.path.join('puzzles', '*.html'))),
        (os.path.join('share', package_name, 'puzzles'), glob(os.path.join('puzzles', '*.css'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create3-nuc',
    maintainer_email='braydon.webster1011@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer_node = jeff_create3_robot.explorer_node:main',
            'hazard_depth_filter = jeff_create3_robot.hazard_depth_filter:main',
            'hazard_to_pointcloud = jeff_create3_robot.hazard_to_pointcloud:main',
            'twist_stamper = jeff_create3_robot.twist_stamper:main',
            'artifact_inspector_node = jeff_create3_robot.artifact_inspector_node:main',
            'puzzle_server_node = jeff_create3_robot.puzzle_server_node:main',
        ],
    },
)
