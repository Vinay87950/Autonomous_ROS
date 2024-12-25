import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'task3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wolfgang',
    maintainer_email='b566@oth-aw.de',
    description='Task 3 for AURE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning = task3.path_planning:main'
        ],
    },
)
