from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'capytown_esan'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capytown',
    maintainer_email='capytown@ue.edu.pe',
    description='CapyTown RC-2 lane following — ESAN 2026-I',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub        = capytown_esan.cam_pub:main',
            'lane_detector  = capytown_esan.lane_detector:main',
            'lane_controller= capytown_esan.lane_controller:main',
            'hsv_tuner      = capytown_esan.hsv_tuner:main',
            'camera_stream  = capytown_esan.camera_stream:main',
        ],
    },
)
