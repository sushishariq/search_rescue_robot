from glob import glob

from setuptools import setup

package_name = 'sar_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shariq',
    maintainer_email='shariqjamil260@gmail.com',
    description='ArUco victim detection and camera-to-map TF projection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector = sar_perception.aruco_detector:main',
        ],
    },
)
