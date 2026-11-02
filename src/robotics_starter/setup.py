from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robotics_starter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adiseshareddy',
    maintainer_email='adiseshareddy1221@gmail.com',
    description='ROS 2 starter package: pub/sub, service, teleop and shape follower.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = robotics_starter.talker:main',
            'listener = robotics_starter.listener:main',
            'add_service = robotics_starter.add_service:main',
            'add_client = robotics_starter.add_client:main',
            'teleop_keyboard = robotics_starter.teleop_keyboard:main',
            'shape_follower = robotics_starter.shape_follower:main',
        ],
    },
)
