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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    # ⚠️ orcp>=0.2.0 IS A CORRECTNESS FLOOR, not a feature preference.
    # 0.1.x used serial.readline(), which returns a PARTIAL line when its
    # timeout expires, so a telemetry push split mid-line and the tail was
    # handed to a waiting command as its response. This driver streams
    # continuously and polls STATUS on a timer, i.e. it runs permanently in the
    # exact condition that triggers it — and the failure gets worse with
    # latency, so worst over WiFi on a real robot.
    install_requires=['setuptools', 'orcp>=0.2.0'],
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
