import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duyroscube',
    maintainer_email='duyroscube@todo.todo',
    description='IvastBot bringup: robot model, motor control, lidar, scan filter',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_filter_node = bringup.scan_filter_node:main',
        ],
    },
)
