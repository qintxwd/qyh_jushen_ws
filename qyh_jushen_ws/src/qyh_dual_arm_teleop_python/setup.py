from setuptools import find_packages, setup

package_name = 'qyh_dual_arm_teleop_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='qyh',
    maintainer_email='qyh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qyh_teleop_left = qyh_dual_arm_teleop_python.qyh_teleop_left:main',
            'qyh_teleop_right = qyh_dual_arm_teleop_python.qyh_teleop_right:main',
        ],
    },
)
