from setuptools import find_packages, setup

package_name = 'keyboard_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolfgang Weigl',
    maintainer_email='w.weigl@oth-aw.de',
    description='Task 1 for AURE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable_name = package_name.python_file:function
            'control_executable = keyboard_control.keyboard_control:main'
        ],
    },
)
