from glob import glob

from setuptools import find_packages, setup

package_name = 'segmentation_learning_period'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/model', glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eirisak',
    maintainer_email='eirisak@ntnu.no',
    description='ROS 2 package that provides a YOLO-based instance segmentation node (yolo_seg_node) for real-time segmentation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_seg_node = segmentation_learning_period.yolo_seg_node:main',
        ],
    },
)
