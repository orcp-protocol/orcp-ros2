import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'orcp_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ORCP Contributors',
    maintainer_email='jon@controlbits.com',
    description='ROS 2 driver for ORCP-compliant motor controllers.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orcp_driver = orcp_ros2.driver:main',
        ],
    },
)
