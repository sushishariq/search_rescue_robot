from glob import glob

from setuptools import setup

package_name = 'sar_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shariq',
    maintainer_email='shariqjamil260@gmail.com',
    description='Waypoint patrol and deduplicated victim registry service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victim_registry = sar_mission.victim_registry:main',
            'patrol = sar_mission.patrol:main',
        ],
    },
)
