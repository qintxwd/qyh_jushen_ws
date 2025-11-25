from setuptools import setup
import os
from glob import glob

package_name = 'qyh_head_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.sdk'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qintxwd',
    maintainer_email='qintxwd@example.com',
    description='HTD-85H servo controller for robot head',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_servo_node = qyh_head_control.head_servo_node:main',
            'set_servo_id = qyh_head_control.set_servo_id:main',
            'test_head_servo = scripts.test_head_servo:main',
        ],
    },
)
