from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_admin',
    maintainer_email='ros_admin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rover_controller_node = rover_controller.controller_node:main",
            "serial_message_generator = rover_controller.serial_message_generator:main",
            "serial_passer_node = rover_controller.serial_passer:main"
        ],
    },
)
