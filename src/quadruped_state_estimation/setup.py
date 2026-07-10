import os
from glob import glob

from setuptools import setup

package_name = 'quadruped_state_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surivona Robotics',
    maintainer_email='dev@example.com',
    description='Quadruped state estimation scaffolding.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_state_estimator = quadruped_state_estimation.base_state_estimator_node:main',
        ],
    },
)
