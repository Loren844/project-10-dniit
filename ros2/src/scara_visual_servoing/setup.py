from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'scara_visual_servoing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stage M1',
    maintainer_email='stage@dut-danang.edu.vn',
    description='Nœuds ROS2 PBVS pour SCARA 4-DOF',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vs_node          = scara_visual_servoing.vs_node:main',
            'sim_target_node  = scara_visual_servoing.sim_target_node:main',
            'gazebo_bridge    = scara_visual_servoing.gazebo_bridge:main',
            'vision_node = scara_visual_servoing.vision_node:main',
        ],
    },
)
