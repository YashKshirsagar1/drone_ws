import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_bringup'


def tree(directory):
    """Install `directory` under share/<pkg>, keeping its subdirectory layout."""
    return [
        (os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
        for root, _, files in os.walk(directory)
        if files
    ]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        *tree('worlds'),
        *tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yash Kshirsagar',
    maintainer_email='camsagar@gmail.com',
    description='Quadcopter simulation in Gazebo Sim, bridged to ROS 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'takeoff = drone_bringup.takeoff:main',
            'teleop_key = drone_bringup.teleop_key:main',
            'mission = drone_bringup.mission:main',
        ],
    },
)
