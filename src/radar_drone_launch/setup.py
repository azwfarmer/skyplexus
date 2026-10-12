from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'radar_drone_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch', 'mavros'),
         glob(os.path.join('launch', 'mavros', '*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaron',
    maintainer_email='aaronzzwang@gmail.com',
    description='launch radar_drone packages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
