import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agv_description'
share = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
    ] + [
        (os.path.join(share, folder), glob(f'{folder}/*.{ext}'))
        for folder, ext in [('config', 'yaml'), ('urdf', 'xacro'), ('meshes', 'dae'),
                            ('launch', 'launch.py'), ('rviz', 'rviz')]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishmika',
    maintainer_email='rish2001wijewardhana@gmail.com',
    description='URDF, meshes and geometry of the AGV',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
)
