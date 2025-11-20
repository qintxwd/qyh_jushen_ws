from setuptools import setup

package_name = 'qyh_vr_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qyh',
    maintainer_email='dev@example.com',
    description='VR Calibration management node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_calibration_node = qyh_vr_calibration.vr_calibration_node:main',
        ],
    },
)
