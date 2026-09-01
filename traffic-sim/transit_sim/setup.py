import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'transit_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anbit',
    maintainer_email='anbitadhi123@gmail.com',
    description='Top-down visualisation of the learning-period city. Subscribes to '
    'VehicleState and SignalState and draws them as markers for Foxglove or RViz2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transit_sim = transit_sim.sim_node:main',
        ],
    },
)
