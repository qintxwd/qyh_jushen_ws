from setuptools import setup
import os
from glob import glob

package_name = 'qyh_vr_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qyh',
    maintainer_email='dev@example.com',
    description='VR Calibration management and interface nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_calibration_node = qyh_vr_calibration.vr_calibration_node:main',
            'vr_interface_node = qyh_vr_calibration.vr_interface_node:main',
            'vr_simulator_node = qyh_vr_calibration.vr_simulator_node:main',
            'calibration_tool = qyh_vr_calibration.calibration_tool:main',
        ],
    },
)
