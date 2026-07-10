import os
from glob import glob

from setuptools import setup

package_name = 'quadruped_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surivona Robotics',
    maintainer_email='dev@example.com',
    description='Robot-agnostic low-level quadruped controllers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_pd_controller = quadruped_control.joint_pd_controller_node:main',
            'cartesian_foot_controller = quadruped_control.cartesian_foot_controller_node:main',
            'gravity_compensation = quadruped_control.gravity_compensation_node:main',
            'go2w_cmd_vel_to_wheels = quadruped_control.go2w_cmd_vel_to_wheels:main',
            'go2w_keyboard_cmd_vel = quadruped_control.go2w_keyboard_cmd_vel:main',
            'go2w_keyboard_teleop = quadruped_control.go2w_keyboard_teleop:main',
        ],
    },
)
