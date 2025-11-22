from setuptools import setup

package_name = 'qyh_gripper_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='PyQt5 GUI for JODELL EPG gripper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_gui = qyh_gripper_gui.gripper_gui:main',
        ],
    },
)
