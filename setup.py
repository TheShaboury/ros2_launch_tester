from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_launch_tester'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'rich',
        'argparse',
    ],
    zip_safe=True,
    maintainer='Ahmed Shaboury',
    maintainer_email='ahmedshaboury000@gmail.com',
    description='ROS2 Launch Tester - Automatically test launch files',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2-launch-test = ros2_launch_tester.ros2_launch_test:main',
        ],
    },
)

