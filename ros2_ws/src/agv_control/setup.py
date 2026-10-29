from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'agv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishmika',
    maintainer_email='rish2001wijewardhana@gmail.com',
    description='Shared tape following, obstacle stopping, and mission planning',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'line_follower = agv_control.line_follower:main',
        'obstacle_stop = agv_control.obstacle_stop:main',
        'wheel_odometry =agv_control.wheel_odometry:main',
    ]},
)
