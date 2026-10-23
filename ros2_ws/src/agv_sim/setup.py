from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*sdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishmika',
    maintainer_email='rish2001wijewardhana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'sim_node = agv_sim.sim_node:main',
        'ultrasonic_sim = agv_sim.ultrasonic_sim:main',
        'gazebo_mts160 = agv_sim.gazebo_mts160:main',
        ],
    },
)
