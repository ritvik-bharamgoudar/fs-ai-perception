from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cone_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/DetectedCone3D.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ritvik',
    maintainer_email='ritvik@todo.todo',
    description='Cone detector with 3D message publishing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detector_node = cone_detector.cone_detector_node:main',
        ],
    },
)

