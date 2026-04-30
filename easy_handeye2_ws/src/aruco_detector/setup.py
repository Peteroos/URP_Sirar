import os
from glob import glob
from setuptools import setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BenreillyZ',
    maintainer_email='benreillyz@github.com',
    description='ArUco marker detection and pose estimation ROS2 node for hand-eye calibration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_vis = aruco_detector.detect_vis_node:main',
        ],
    },
)
