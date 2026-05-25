from setuptools import setup
import os
from glob import glob

package_name = 'cyberdog_controller'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={package_name: '.'},
    packages=[
        package_name,
        package_name + '.camera',
        package_name + '.FSM',
        package_name + '.FSM.states',
        package_name + '.locomotion',
        package_name + '.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Cyberdog Race 2026 Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = cyberdog_controller.FSM.fsm:main'
        ],
    },
)
