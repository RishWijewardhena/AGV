from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'agv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pymodbus>=3.6,<3.7'],
    zip_safe=True,
    maintainer='rishmika',
    maintainer_email='rish2001wijewardhana@gmail.com',
    description='Real-robot startup scaffold and hardware configuration',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'motor_node = agv_bringup.motor_node:main',
    ]},
)
