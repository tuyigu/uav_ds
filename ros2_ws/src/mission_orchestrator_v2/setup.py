from setuptools import setup
import os
from glob import glob

package_name = 'mission_orchestrator_v2'

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
    maintainer='User',
    maintainer_email='user@example.com',
    description='V2 Mission Orchestrator with gRPC and Intent Engine',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'orchestrator_node_v2 = mission_orchestrator_v2.orchestrator_node_v2:main',
        ],
    },
)
