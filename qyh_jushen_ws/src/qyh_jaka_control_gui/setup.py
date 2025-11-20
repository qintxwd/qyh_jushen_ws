from setuptools import setup

package_name = 'qyh_jaka_control_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qyh',
    maintainer_email='dev@example.com',
    description='GUI for JAKA servo commands',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_gui = qyh_jaka_control_gui.servo_gui:main',
        ],
    },
)