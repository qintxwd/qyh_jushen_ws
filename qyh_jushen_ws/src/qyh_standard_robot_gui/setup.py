from setuptools import setup
import os
from glob import glob

package_name = 'qyh_standard_robot_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='qyh',
    maintainer_email='jsqinyinghao@live.com',
    description='PyQt5 GUI for monitoring Standard robot status',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'standard_robot_monitor = qyh_standard_robot_gui.standard_robot_monitor:main',
        ],
    },
)
