from setuptools import find_packages, setup

package_name = 'py_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaron',
    maintainer_email='aaronzzwang@gmail.com',
    description='Python scripts for perception',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fast_depth = py_perception.fast_depth:main',
            'map_gen = py_perception.map_gen:main',
            'map_gen_old = py_perception.map_gen_old:main'
        ],
    },
)
