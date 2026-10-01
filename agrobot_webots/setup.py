import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agrobot_webots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'protos'), glob('protos/*')),
        (os.path.join('share', package_name, 'meshes'),
            ['../agrobot_description/meshes/chassis.stl']),
        (os.path.join('share', package_name, 'controllers', 'agrobot_driver'),
            glob('controllers/agrobot_driver/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkauesm',
    maintainer_email='kkauesm@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
