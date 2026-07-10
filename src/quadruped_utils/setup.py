from setuptools import setup

package_name = 'quadruped_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surivona Robotics',
    maintainer_email='dev@example.com',
    description='Shared utilities for quadruped packages.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'utils_sanity = quadruped_utils.utils_sanity_node:main',
        ],
    },
)
