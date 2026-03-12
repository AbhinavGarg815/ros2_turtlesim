from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='abhinavgrg815@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_autonomy = turtle_autonomy.autonomy_node:main',
            'obstacle_server = turtle_autonomy.obstacle_server_node:main',
            'spawn_turtles = turtle_autonomy.spawn_turtles_node:main',
        ],
    },
)
